import socket
import json
import threading
import time
import collections
import config  # config.py をインポート
import params as params

from typing import Dict, Any
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from sim_robot import SimulatedRobot
    from sim_ball import SimulatedBall
    from simulator import Simulator


class SimulatorUDP:
    def __init__(self, robots_dict: Dict[str, 'SimulatedRobot'], ball_sim: 'SimulatedBall', simulator_instance_ref: 'Simulator'):
        self.robots = robots_dict  # simulator.py で実際に有効化されたロボットの辞書
        self.ball = ball_sim
        self.running = True
        self.simulator_instance = simulator_instance_ref

        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Visionデータやセンサーデータの送信先は config.CONTROLLER_IP
        self.controller_dest_ip = config.CONTROLLER_IP
        if self.controller_dest_ip == "0.0.0.0":  # "0.0.0.0" は送信先として不適切
            self.controller_dest_ip = "127.0.0.1"
            print("警告: config.CONTROLLER_IP が '0.0.0.0' でした。送信先を '127.0.0.1' に変更します。")

        self.cmd_sockets: Dict[str, socket.socket] = {}
        self.cmd_threads: Dict[str, threading.Thread] = {}

        self.vision_data_buffer = collections.deque()
        self.robot_sensor_buffers: Dict[str, collections.deque] = collections.defaultdict(
            collections.deque)

        self.vision_data_delay_s = params.VISION_DATA_DELAY_S
        self.robot_sensor_delay_s = params.ROBOT_SENSOR_DELAY_S

        # コマンド受信用ソケットの初期化
        # config.py の設定に基づいて、有効なロボットのソケットを作成
        all_robot_configs = [('yellow', conf) for conf in config.YELLOW_ROBOTS_CONFIG] + \
            [('blue', conf) for conf in config.BLUE_ROBOTS_CONFIG]

        for team_color, robot_conf in all_robot_configs:
            if robot_conf["enabled"]:
                robot_id_str = f"{team_color}_{robot_conf['id']}"
                # この robot_id_str が self.robots のキーと一致するか確認
                if robot_id_str not in self.robots:
                    print(
                        f"警告: 設定ファイルで有効なロボット {robot_id_str} が Simulator インスタンスに存在しません。")
                    continue

                try:
                    listen_ip = robot_conf["ip_for_command_listen"]
                    port = robot_conf["command_listen_port"]

                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock.bind((listen_ip, port))
                    sock.settimeout(0.1)
                    self.cmd_sockets[robot_id_str] = sock
                    print(
                        f"シミュレータはロボット {robot_id_str} のコマンドを {listen_ip}:{port} で待機します")

                    thread = threading.Thread(
                        target=self._receive_commands, args=(robot_id_str, sock), daemon=True)
                    self.cmd_threads[robot_id_str] = thread
                    thread.start()

                except OSError as e:
                    print(
                        f"エラー: ロボット {robot_id_str} のコマンド用 {listen_ip}:{port} にバインドできませんでした: {e}")
                except KeyError as e:
                    print(
                        f"エラー: ロボット {robot_id_str} の設定に {e} が見つかりません。config.py を確認してください。")

    def _receive_commands(self, robot_id: str, sock: socket.socket):
        while self.running:
            try:
                data, addr = sock.recvfrom(config.BUFFER_SIZE)  # addr を受け取る
                command_dict = json.loads(data.decode('utf-8'))
                if robot_id in self.robots:
                    # print(f"受信: {robot_id} from {addr} data: {command_dict}") # デバッグ用
                    self.robots[robot_id].set_command(command_dict)
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                print(f"シミュレータは {robot_id} 用に無効なJSONコマンドを受信しました")
            except Exception as e:
                if self.running:
                    print(f"{robot_id} のコマンド受信中にエラーが発生しました: {e}")
                break

    def send_vision_data(self):
        current_sim_time = time.time()

        raw_vision_payload: Dict[str, Any] = {
            "timestamp": current_sim_time,
            "fps": round(self.simulator_instance.clock.get_fps(), 2),
            "is_calibrated": True,
            "yellow_robots": {},
            "blue_robots": {},
            "orange_balls": [self.ball.get_vision_data()] if self.ball else []
        }

        # self.robots には simulator.py で有効化されたロボットのみが含まれる
        for robot_id_key, robot_instance in self.robots.items():
            try:
                team_color, id_num_str = robot_id_key.split('_')
                if team_color == "yellow":
                    raw_vision_payload["yellow_robots"][id_num_str] = robot_instance.get_vision_data(
                    )
                elif team_color == "blue":
                    raw_vision_payload["blue_robots"][id_num_str] = robot_instance.get_vision_data(
                    )
            except ValueError:
                print(
                    f"警告: ロボットID '{robot_id_key}' の形式が不正です。Visionデータに含められません。")

        self.vision_data_buffer.append(raw_vision_payload)

        data_to_send = None
        current_send_attempt_time = time.time()
        while self.vision_data_buffer:
            if current_send_attempt_time - self.vision_data_buffer[0]["timestamp"] >= self.vision_data_delay_s:
                data_to_send = self.vision_data_buffer.popleft()
            else:
                break

        if data_to_send:
            try:
                msg = json.dumps(data_to_send).encode('utf-8')
                self.send_socket.sendto(
                    msg, (self.controller_dest_ip, config.VISION_LISTEN_PORT))
            except Exception as e:
                print(f"遅延Visionデータの送信中にエラーが発生しました: {e}")

    def send_sensor_data(self):
        current_sim_time = time.time()
        current_send_attempt_time = time.time()

        # config.py の設定からロボットごとのセンサー送信ポートを取得する
        # まず、色とIDから送信ポートを引けるようにマップを作成
        sensor_ports_map = {}
        for team_color_prefix, robot_configs_list in [("yellow", config.YELLOW_ROBOTS_CONFIG), ("blue", config.BLUE_ROBOTS_CONFIG)]:
            for robot_conf_item in robot_configs_list:
                if robot_conf_item["enabled"]:
                    try:
                        robot_id_str_map = f"{team_color_prefix}_{robot_conf_item['id']}"
                        sensor_ports_map[robot_id_str_map] = robot_conf_item['sensor_send_port']
                    except KeyError:
                        print(
                            f"警告: {team_color_prefix} robot id {robot_conf_item.get('id', 'N/A')} の設定に 'sensor_send_port' または 'id' がありません。")

        # self.robots には実際に有効化されたロボットインスタンスが入っている
        for robot_id_key, robot_instance in self.robots.items():
            if robot_id_key not in sensor_ports_map:
                # print(f"デバッグ: ロボット {robot_id_key} のセンサーデータ送信ポートが設定で見つかりません。")
                continue

            destination_port = sensor_ports_map[robot_id_key]

            raw_sensor_payload = {
                "timestamp": current_sim_time,
                "data": robot_instance.get_sensor_data(self.ball)
            }
            self.robot_sensor_buffers[robot_id_key].append(raw_sensor_payload)

            data_to_send = None
            buffer_for_robot = self.robot_sensor_buffers[robot_id_key]
            while buffer_for_robot:
                if current_send_attempt_time - buffer_for_robot[0]["timestamp"] >= self.robot_sensor_delay_s:
                    data_to_send = buffer_for_robot.popleft()["data"]
                else:
                    break

            if data_to_send:
                try:
                    msg = json.dumps(data_to_send).encode('utf-8')
                    self.send_socket.sendto(
                        msg, (self.controller_dest_ip, destination_port))
                except Exception as e:
                    print(f"遅延センサーデータ ({robot_id_key}) 送信中にエラー: {e}")

    def stop(self):
        self.running = False
        for robot_id, thread in self.cmd_threads.items():
            if thread.is_alive():
                print(f"コマンド受信スレッド ({robot_id}) を停止中...")
                thread.join(timeout=0.2)
        for sock in self.cmd_sockets.values():
            sock.close()
        if self.send_socket:
            self.send_socket.close()
        print("UDPハンドラが停止しました。")
