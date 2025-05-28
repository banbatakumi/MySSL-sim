import params

# --- UDP 通信設定 ---
VISION_LISTEN_PORT = 50007

SIMULATOR_LISTEN_IP = "0.0.0.0"

CONTROLLER_IP = "127.0.0.1"

# 受信バッファサイズ
BUFFER_SIZE = 65536


# ロボットごとの設定
INITIAI_YELLOW_ROBOT_PORT = 50010
NUM_YELLOW_ROBOTS = 11  # ロボットの数

# ロボットごとの設定
YELLOW_ROBOTS_CONFIG = []

for i in range(NUM_YELLOW_ROBOTS):
    robot_config = {
        "id": i,
        "ip_for_command_listen": SIMULATOR_LISTEN_IP,
        "command_listen_port": INITIAI_YELLOW_ROBOT_PORT + i * 2,
        "sensor_send_port": INITIAI_YELLOW_ROBOT_PORT + i * 2 + 1,
        "enabled": True,
        "initial_pos_x_m": -0.4 - i * 0.2,
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


# 制御ループのポーリング間隔 (秒) - シミュレータ内のデータ送信周期の目安
CONTROL_LOOP_INTERVAL = 0.001  # 約60Hz (16ms)
FPS = 60

# --- フィールドのパラメータ ---
FIELD_MARKING_WIDTH_PX = 2
ROBOT_OUTLINE_WIDTH_PX = 2
WALL_LINE_WIDTH_PX = 4


# --- 初期描画定数 (ウィンドウサイズ変更時に更新) ---
INITIAL_PIXELS_PER_METER = 300
INITIAL_SCREEN_PADDING_PX = 10
INITIAL_SCREEN_WIDTH_PX = int(
    params.COURT_WIDTH_M * INITIAL_PIXELS_PER_METER) + 2 * INITIAL_SCREEN_PADDING_PX
INITIAL_SCREEN_HEIGHT_PX = int(params.COURT_HEIGHT_M * INITIAL_PIXELS_PER_METER) + \
    2 * INITIAL_SCREEN_PADDING_PX


COLOR_BACKGROUND = (0, 50, 0)
COLOR_WALLS = (0, 0, 0)
COLOR_GOAL = (0, 0, 0)
COLOR_FIELD_LINES = (255, 255, 255)
COLOR_YELLOW_ROBOT = (255, 255, 0)
COLOR_BLUE_ROBOT = (0, 0, 255)
COLOR_BALL = (255, 165, 0)
COLOR_TEXT = (200, 200, 200)
COLOR_ROBOT_FRONT = (200, 0, 0)
COLOR_DEBUG_VECTOR = (0, 255, 255)
