# --- UDP 通信設定 ---
# Vision からの受信ポート (シミュレータがリッスン)
VISION_LISTEN_PORT = 50007

# 待ち受けIPアドレス: 通常は '0.0.0.0' (すべてのインターフェースで待機)
# これはシミュレータがコマンドやVisionデータを受信する際にバインドするIP
SIMULATOR_LISTEN_IP = "0.0.0.0"

# Visionデータやセンサーデータの送信先IP (コントローラが動作しているマシンのIP)
# "127.0.0.1" はローカルホスト (同一マシンでコントローラを実行する場合)
CONTROLLER_IP = "127.0.0.1"

# ロボット設定
# "id" はロボットの識別番号 (チーム内でユニーク)
# "ip" はロボットコントローラがコマンドを送信してくるIP (シミュレータ側で受信用ソケットをバインドする際のIP)
#     通常、シミュレータは SIMULATOR_LISTEN_IP (例: "0.0.0.0") で待機し、
#     特定のロボットコントローラからのコマンドをそのロボットの "command_listen_port" で受け付けます。
#     この "ip" フィールドは、将来的にシミュレータからロボットへ直接何かを送信する場合や、
#     セキュリティ目的で送信元IPを制限する場合に役立つかもしれません。
#     現状のコマンド受信ロジックでは、SIMULATOR_LISTEN_IP が主に使われます。
# "command_listen_port" はシミュレータがこのロボットのコマンドを受信するポート
# "sensor_send_port" はシミュレータがこのロボットのセンサーデータを送信するポート (コントローラ側がリッスン)
# "enabled" はこのロボットをシミュレーションで有効にするかどうか


# COURT_WIDTH_M = 5.2  # コートの幅 (メートル) - 白線間の距離
# COURT_HEIGHT_M = 3.7  # コートの高さ (メートル) - 白線間の距離
COURT_WIDTH_M = 1.5  # コートの幅 (メートル) - 白線間の距離
COURT_HEIGHT_M = 1  # コートの高さ (メートル) - 白線間の距離

# ロボットごとの設定
INITIAI_YELLOW_ROBOT_PORT = 50010
NUM_YELLOW_ROBOTS = 2  # ロボットの数

# ロボットごとの設定
YELLOW_ROBOTS_CONFIG = []

for i in range(NUM_YELLOW_ROBOTS):
    robot_config = {
        "id": i,
        "ip_for_command_listen": SIMULATOR_LISTEN_IP,
        "command_listen_port": INITIAI_YELLOW_ROBOT_PORT + i * 2,
        "sensor_send_port": INITIAI_YELLOW_ROBOT_PORT + i * 2 + 1,
        "enabled": True,
        "initial_pos_x_m": -0.4 - i * 0.2,  # コート幅の1/4 左
        "initial_pos_y_m": 0.0,
        "initial_angle_deg": 0.0
    }
    YELLOW_ROBOTS_CONFIG.append(robot_config)

# ロボットごとの設定
INITIAI_BLUE_ROBOT_PORT = 50030
NUM_BLUE_ROBOTS = 2  # ロボットの数

# ロボットごとの設定
BLUE_ROBOTS_CONFIG = []

for i in range(NUM_YELLOW_ROBOTS):
    robot_config = {
        "id": i,
        "ip_for_command_listen": SIMULATOR_LISTEN_IP,
        "command_listen_port": INITIAI_BLUE_ROBOT_PORT + i * 2,
        "sensor_send_port": INITIAI_BLUE_ROBOT_PORT + i * 2 + 1,
        "enabled": False,
        "initial_pos_x_m": -0.4 - i * 0.2,  # コート幅の1/4 左
        "initial_pos_y_m": 0.0,
        "initial_angle_deg": 0.0
    }
    BLUE_ROBOTS_CONFIG.append(robot_config)

# 受信バッファサイズ
BUFFER_SIZE = 65536

# 制御ループのポーリング間隔 (秒) - シミュレータ内のデータ送信周期の目安
CONTROL_LOOP_INTERVAL = 0.001  # 約60Hz (16ms)


# --- 初期描画定数 (ウィンドウサイズ変更時に更新) ---
INITIAL_PIXELS_PER_METER = 300
INITIAL_SCREEN_PADDING_PX = 10
INITIAL_SCREEN_WIDTH_PX = int(
    COURT_WIDTH_M * INITIAL_PIXELS_PER_METER) + 2 * INITIAL_SCREEN_PADDING_PX
INITIAL_SCREEN_HEIGHT_PX = int(COURT_HEIGHT_M * INITIAL_PIXELS_PER_METER) + \
    2 * INITIAL_SCREEN_PADDING_PX

FIELD_MARKING_WIDTH_PX = 2
ROBOT_OUTLINE_WIDTH_PX = 2
WALL_LINE_WIDTH_PX = 4

COLOR_BACKGROUND = (0, 50, 0)
COLOR_WALLS = (0, 0, 0)
COLOR_FIELD_LINES = (255, 255, 255)
COLOR_YELLOW_ROBOT = (255, 255, 0)
COLOR_BLUE_ROBOT = (0, 0, 255)
COLOR_BALL = (255, 165, 0)
COLOR_TEXT = (200, 200, 200)
COLOR_ROBOT_FRONT = (200, 0, 0)
COLOR_DEBUG_VECTOR = (0, 255, 0)

FPS = 60
