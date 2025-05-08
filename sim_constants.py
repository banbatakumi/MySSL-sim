import math
import config  # COURT_WIDTH, COURT_HEIGHT のため

# --- シミュレータ定数 ---
ROBOT_DIAMETER_M = 0.18  # ロボットの直径 (メートル)
ROBOT_RADIUS_M = ROBOT_DIAMETER_M / 2.0  # ロボットの半径 (メートル)
COURT_WIDTH_M = config.COURT_WIDTH / 100.0  # コートの幅 (メートル)
COURT_HEIGHT_M = config.COURT_HEIGHT / 100.0  # コートの高さ (メートル)
ROBOT_MAX_SPEED_MPS = 1.5  # ロボットの最大速度 (メートル/秒)
ROBOT_MAX_ACCE_MPSS = 5  # ロボットの最大加速度 (メートル/秒^2)
ROBOT_MAX_ANGULAR_SPEED_RADPS = 6 * math.pi  # ロボットの最大角速度 (ラジアン/秒) - 3 RPS

BALL_RADIUS_M = 0.0215  # ボールの半径 (メートル)
BALL_FRICTION_COEFF = 0.4  # ボールの摩擦係数
GRAVITY_MPSS = 9.81  # 重力加速度 (メートル/秒^2)

SENSOR_FOV_HALF_ANGLE_RAD = math.radians(40)  # センサーの視野角の半分 (ラジアン)

KICK_POWER_TO_SPEED_MPS = 3.0 / 100.0  # キックパワーから速度への変換係数 (メートル/秒)
DRIBBLE_PULL_FACTOR = 8.0  # ドリブル時のボール引き寄せ係数

# --- 初期描画定数 (ウィンドウサイズ変更時に更新) ---
INITIAL_PIXELS_PER_METER = 300  # 初期スケール (ピクセル/メートル)
INITIAL_SCREEN_PADDING_PX = 50  # 初期画面の余白 (ピクセル)
INITIAL_SCREEN_WIDTH_PX = int(
    # 初期画面幅 (ピクセル)
    COURT_WIDTH_M * INITIAL_PIXELS_PER_METER) + 2 * INITIAL_SCREEN_PADDING_PX
INITIAL_SCREEN_HEIGHT_PX = int(COURT_HEIGHT_M * INITIAL_PIXELS_PER_METER) + \
    2 * INITIAL_SCREEN_PADDING_PX  # 初期画面高さ (ピクセル)

FIELD_MARKING_WIDTH_PX = 2  # フィールドラインの幅 (ピクセル)
ROBOT_OUTLINE_WIDTH_PX = 2  # ロボットの輪郭線の幅 (ピクセル)

COLOR_BACKGROUND = (0, 50, 0)  # 背景色
COLOR_FIELD_LINES = (255, 255, 255)  # フィールドラインの色
COLOR_YELLOW_ROBOT = (255, 255, 0)  # 黄色ロボットの色
COLOR_BLUE_ROBOT = (0, 0, 255)  # 青色ロボットの色
COLOR_BALL = (255, 165, 0)  # ボールの色
COLOR_TEXT = (200, 200, 200)  # テキストの色
COLOR_ROBOT_FRONT = (200, 0, 0)  # ロボットの前面インジケータの色
COLOR_DEBUG_VECTOR = (0, 255, 0)  # デバッグ用ベクトルの色 (緑)

SIMULATION_TIMESTEP = 0.01  # シミュレーションのタイムステップ (秒)
FPS = 60  # 目標フレームレート
