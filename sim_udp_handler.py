import socket
import json
import threading
import config

# 型ヒント用 (基本的な機能にはオプション、Pythonは動的型付け)
from typing import Dict, Any
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from sim_robot import SimulatedRobot
    from sim_ball import SimulatedBall
    from simulator import Simulator  # メインシミュレータクラス


class SimulatorUDP:
    def __init__(self, robots_dict: Dict[str, 'SimulatedRobot'], ball_sim: 'SimulatedBall', simulator_instance_ref: 'Simulator'):
        self.robots = robots_dict  # 制御対象のロボット辞書
        self.ball = ball_sim  # ボールオブジェクト
        self.running = True  # UDPハンドラ動作フラグ
        self.simulator_instance = simulator_instance_ref  # Simulatorインスタンスへの参照

        self.send_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)  # 送信用ソケット
        self.controller_listen_ip = config.LISTEN_IP
        if self.controller_listen_ip == "0.0.0.0":  # 全インターフェースで待機する場合、ローカル送信には127.0.0.1を使用
            self.controller_listen_ip = "127.0.0.1"

        self.cmd_sockets: Dict[str, socket.socket] = {}  # コマンド受信用ソケット辞書 (色別)
        self.cmd_threads: Dict[str, threading.Thread] = {}  # コマンド受信スレッド辞書

        # 黄色ロボットのコマンド受信用ソケット設定
        if config.ENABLE_YELLOW_ROBOT and "yellow" in self.robots:
            try:
                ip, port = config.ROBOT_LOCAL_IP, config.YELLOW_SEND_PORT
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((ip, port))
                sock.settimeout(0.1)  # ノンブロッキング受信
                self.cmd_sockets["yellow"] = sock
                print(f"シミュレータは黄色ロボットのコマンドを {ip}:{port} で待機します")
            except OSError as e:
                print(f"エラー: 黄色ロボットのコマンド用 {ip}:{port} にバインドできませんでした: {e}")

        # 青色ロボットのコマンド受信用ソケット設定
        if config.ENABLE_BLUE_ROBOT and "blue" in self.robots:
            try:
                ip, port = config.ROBOT_LOCAL_IP, config.BLUE_SEND_PORT
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((ip, port))
                sock.settimeout(0.1)  # ノンブロッキング受信
                self.cmd_sockets["blue"] = sock
                print(f"シミュレータは青色ロボットのコマンドを {ip}:{port} で待機します")
            except OSError as e:
                print(f"エラー: 青色ロボットのコマンド用 {ip}:{port} にバインドできませんでした: {e}")

        # 各ロボットのコマンド受信スレッドを開始
        for color, sock in self.cmd_sockets.items():
            thread = threading.Thread(
                target=self._receive_commands, args=(color, sock), daemon=True)
            self.cmd_threads[color] = thread
            thread.start()

    def _receive_commands(self, robot_color: str, sock: socket.socket):
        """指定されたロボットの色に対応するコマンドを受信する"""
        while self.running:
            try:
                data, _ = sock.recvfrom(config.BUFFER_SIZE)
                command_dict = json.loads(data.decode('utf-8'))
                if robot_color in self.robots:
                    self.robots[robot_color].set_command(command_dict)
            except socket.timeout:
                continue  # タイムアウトは正常なので継続
            except json.JSONDecodeError:
                print(f"シミュレータは {robot_color} 用に無効なJSONコマンドを受信しました")
            except Exception as e:
                if self.running:  # シャットダウン中のエラーメッセージを回避
                    print(f"{robot_color} のコマンド受信中にエラーが発生しました: {e}")
                break  # その他のエラーの場合はスレッドを終了

    def send_vision_data(self):
        """Visionデータをコントローラに送信する"""
        vision_payload: Dict[str, Any] = {
            "fps": round(self.simulator_instance.clock.get_fps(), 2),
            "is_calibrated": True,  # シミュレータは常にキャリブレーション済みとする
            "yellow_robots": [],
            "blue_robots": [],
            "orange_balls": [self.ball.get_vision_data()] if self.ball else []
        }

        if "yellow" in self.robots and config.ENABLE_YELLOW_ROBOT:
            vision_payload["yellow_robots"].append(
                self.robots["yellow"].get_vision_data())
        if "blue" in self.robots and config.ENABLE_BLUE_ROBOT:
            vision_payload["blue_robots"].append(
                self.robots["blue"].get_vision_data())

        try:
            msg = json.dumps(vision_payload).encode('utf-8')
            self.send_socket.sendto(
                msg, (self.controller_listen_ip, config.VISION_LISTEN_PORT))
        except Exception as e:
            print(f"Visionデータの送信中にエラーが発生しました: {e}")

    def send_sensor_data(self):
        """センサーデータをコントローラに送信する"""
        if "yellow" in self.robots and config.ENABLE_YELLOW_ROBOT:
            sensor_payload = self.robots["yellow"].get_sensor_data(self.ball)
            try:
                msg = json.dumps(sensor_payload).encode('utf-8')
                self.send_socket.sendto(
                    msg, (self.controller_listen_ip, config.YELLOW_SENSOR_LISTEN_PORT))
            except Exception as e:
                print(f"黄色ロボットのセンサーデータ送信中にエラーが発生しました: {e}")

        if "blue" in self.robots and config.ENABLE_BLUE_ROBOT:
            sensor_payload = self.robots["blue"].get_sensor_data(self.ball)
            try:
                msg = json.dumps(sensor_payload).encode('utf-8')
                self.send_socket.sendto(
                    msg, (self.controller_listen_ip, config.BLUE_SENSOR_LISTEN_PORT))
            except Exception as e:
                print(f"青色ロボットのセンサーデータ送信中にエラーが発生しました: {e}")

    def stop(self):
        """UDPハンドラを停止し、リソースを解放する"""
        self.running = False
        for color, thread in self.cmd_threads.items():
            if thread.is_alive():
                thread.join(timeout=0.2)  # スレッド終了を待機
        for sock in self.cmd_sockets.values():
            sock.close()  # ソケットを閉じる
        if self.send_socket:
            self.send_socket.close()  # 送信用ソケットを閉じる
