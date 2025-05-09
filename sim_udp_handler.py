import socket
import json
import threading
import time  # time をインポート
import collections  # collections をインポート
import config
import sim_params as params  # 調整可能なパラメータをインポート

# 型ヒント用
from typing import Dict, Any
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from sim_robot import SimulatedRobot
    from sim_ball import SimulatedBall
    from simulator import Simulator


class SimulatorUDP:
    def __init__(self, robots_dict: Dict[str, 'SimulatedRobot'], ball_sim: 'SimulatedBall', simulator_instance_ref: 'Simulator'):
        self.robots = robots_dict
        self.ball = ball_sim
        self.running = True
        self.simulator_instance = simulator_instance_ref

        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.controller_listen_ip = config.LISTEN_IP
        if self.controller_listen_ip == "0.0.0.0":
            self.controller_listen_ip = "127.0.0.1"

        self.cmd_sockets: Dict[str, socket.socket] = {}
        self.cmd_threads: Dict[str, threading.Thread] = {}

        # データ遅延用バッファ
        self.vision_data_buffer = collections.deque()
        self.yellow_robot_0_sensor_buffer = collections.deque()
        self.yellow_robot_1_sensor_buffer = collections.deque()

        # 遅延時間設定
        self.vision_data_delay_s = params.VISION_DATA_DELAY_S
        self.robot_sensor_delay_s = params.ROBOT_SENSOR_DELAY_S

        if config.ENABLE_YELLOW_ROBOT_0 and "yellow_0" in self.robots:
            try:
                ip, port = config.ROBOT_LOCAL_IP, config.YELLOW_ROBOT_0_SEND_PORT
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((ip, port))
                sock.settimeout(0.1)
                self.cmd_sockets["yellow_0"] = sock
                print(f"シミュレータは黄色ロボットのコマンドを {ip}:{port} で待機します")
            except OSError as e:
                print(f"エラー: 黄色ロボットのコマンド用 {ip}:{port} にバインドできませんでした: {e}")

        if config.ENABLE_YELLOW_ROBOT_1 and "yellow_1" in self.robots:
            try:
                ip, port = config.ROBOT_LOCAL_IP, config.YELLOW_ROBOT_1_SEND_PORT
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((ip, port))
                sock.settimeout(0.1)
                self.cmd_sockets["yellow_1"] = sock
                print(f"シミュレータは黄色ロボットのコマンドを {ip}:{port} で待機します")
            except OSError as e:
                print(f"エラー: 黄色ロボットのコマンド用 {ip}:{port} にバインドできませんでした: {e}")

        for color, sock in self.cmd_sockets.items():
            thread = threading.Thread(
                target=self._receive_commands, args=(color, sock), daemon=True)
            self.cmd_threads[color] = thread
            thread.start()

    def _receive_commands(self, robot_color: str, sock: socket.socket):
        while self.running:
            try:
                data, _ = sock.recvfrom(config.BUFFER_SIZE)
                command_dict = json.loads(data.decode('utf-8'))
                if robot_color in self.robots:
                    self.robots[robot_color].set_command(command_dict)
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                print(f"シミュレータは {robot_color} 用に無効なJSONコマンドを受信しました")
            except Exception as e:
                if self.running:
                    print(f"{robot_color} のコマンド受信中にエラーが発生しました: {e}")
                break

    def send_vision_data(self):
        current_sim_time = time.time()  # データ生成時刻の基準として使用

        raw_vision_payload: Dict[str, Any] = {
            "timestamp": current_sim_time,  # このデータが生成されたシミュレーション時刻
            "fps": round(self.simulator_instance.clock.get_fps(), 2),
            "is_calibrated": True,
            "yellow_robots": {},
            "blue_robots": {},
            "orange_balls": [self.ball.get_vision_data()] if self.ball else []
        }

        if "yellow_0" in self.robots and config.ENABLE_YELLOW_ROBOT_0:
            raw_vision_payload["yellow_robots"]["0"] = self.robots["yellow_0"].get_vision_data(
            )
        if "yellow_1" in self.robots and config.ENABLE_YELLOW_ROBOT_1:
            raw_vision_payload["yellow_robots"]["1"] = self.robots["yellow_1"].get_vision_data(
            )
        if "blue_0" in self.robots and config.ENABLE_BLUE_ROBOT_0:
            raw_vision_payload["blue_robots"]["0"] = self.robots["blue_0"].get_vision_data(
            )
        if "blue_1" in self.robots and config.ENABLE_BLUE_ROBOT_1:
            raw_vision_payload["blue_robots"]["1"] = self.robots["blue_1"].get_vision_data(
            )

        self.vision_data_buffer.append(raw_vision_payload)

        data_to_send = None
        # バッファの先頭から、送信時刻（現在時刻）に対して遅延時間を満たすものを探す
        # current_time はこの関数が呼ばれた時刻 (送信試行時刻)
        current_send_attempt_time = time.time()
        while self.vision_data_buffer:
            if current_send_attempt_time - self.vision_data_buffer[0]["timestamp"] >= self.vision_data_delay_s:
                data_to_send = self.vision_data_buffer.popleft()
                # 複数のデータが遅延時間を満たしていても、最新の1つだけを送る（あるいは全て送るか仕様による）
                # ここでは最後にpopleftされたものが送信される
            else:
                break  # バッファの先頭がまだ早ければ、それ以降も早いので抜ける

        if data_to_send:
            # 送信するデータからタイムスタンプを削除してもよい
            # del data_to_send["timestamp"]
            try:
                msg = json.dumps(data_to_send).encode('utf-8')
                self.send_socket.sendto(
                    msg, (self.controller_listen_ip, config.VISION_LISTEN_PORT))
            except Exception as e:
                print(f"遅延Visionデータの送信中にエラーが発生しました: {e}")

    def send_sensor_data(self):
        current_sim_time = time.time()  # データ生成時刻の基準
        current_send_attempt_time = time.time()  # 送信試行時刻

        if "yellow_0" in self.robots and config.ENABLE_YELLOW_ROBOT_0:
            # get_sensor_dataは生の(遅延なし)データを返す
            raw_yellow_robot_0_sensor_payload = {
                "timestamp": current_sim_time,
                "data": self.robots["yellow_0"].get_sensor_data(self.ball)
            }
            self.yellow_robot_0_sensor_buffer.append(
                raw_yellow_robot_0_sensor_payload)

            data_to_send = None
            while self.yellow_robot_0_sensor_buffer:
                if current_send_attempt_time - self.yellow_robot_0_sensor_buffer[0]["timestamp"] >= self.robot_sensor_delay_s:
                    data_to_send = self.yellow_robot_0_sensor_buffer.popleft()[
                        "data"]
                else:
                    break
            if data_to_send:
                try:
                    msg = json.dumps(data_to_send).encode('utf-8')
                    self.send_socket.sendto(
                        msg, (self.controller_listen_ip, config.ROBOT_0_SENSOR_LISTEN_PORT))
                except Exception as e:
                    print(f"遅延黄色センサーデータ送信中にエラー: {e}")

        if "yellow_1" in self.robots and config.ENABLE_YELLOW_ROBOT_1:
            raw_yellow_robot_1_sensor_payload = {
                "timestamp": current_sim_time,
                "data": self.robots["yellow_1"].get_sensor_data(self.ball)
            }
            self.yellow_robot_1_sensor_buffer.append(
                raw_yellow_robot_1_sensor_payload)

            data_to_send = None
            while self.yellow_robot_1_sensor_buffer:
                if current_send_attempt_time - self.yellow_robot_1_sensor_buffer[0]["timestamp"] >= self.robot_sensor_delay_s:
                    data_to_send = self.yellow_robot_1_sensor_buffer.popleft()[
                        "data"]
                else:
                    break
            if data_to_send:
                try:
                    msg = json.dumps(data_to_send).encode('utf-8')
                    self.send_socket.sendto(
                        msg, (self.controller_listen_ip, config.ROBOT_1_SENSOR_LISTEN_PORT))
                except Exception as e:
                    print(f"遅延青色センサーデータ送信中にエラー: {e}")

    def stop(self):
        self.running = False
        for color, thread in self.cmd_threads.items():
            if thread.is_alive():
                thread.join(timeout=0.2)
        for sock in self.cmd_sockets.values():
            sock.close()
        if self.send_socket:
            self.send_socket.close()
