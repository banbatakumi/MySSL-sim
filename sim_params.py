import math
# シミュレーションの調整可能な物理パラメータや遅延設定

# --- ロボット物理パラメータ ---
ROBOT_MASS_KG = 1.6
OMNI_WHEEL_FRICTION_COEFF = 0.5
NUM_OMNI_WHEELS = 4  # ホイール数 (現在は直接使われていないが、将来的な詳細モデル用)
ROBOT_DIAMETER_M = 0.18  # ロボットの直径 (メートル)
ROBOT_RADIUS_M = ROBOT_DIAMETER_M / 2.0  # ロボットの半径 (メートル)
ROBOT_MAX_SPEED_MPS = 1.5  # ロボットの最大速度 (メートル/秒)
ROBOT_MAX_ANGULAR_SPEED_RADPS = 6 * math.pi  # ロボットの最大角速度 (ラジアン/秒) - 3 RPS
KICK_POWER_TO_SPEED_MPS = 3.0 / 100.0  # キックパワーから速度への変換係数 (メートル/秒)
DRIBBLE_PULL_FACTOR = 8.0  # ドリブル時のボール引き寄せ係数


# --- センサー遅延設定 ---
VISION_DATA_DELAY_S = 0.2  # Visionデータの遅延時間 (秒), 例: 50ms
ROBOT_SENSOR_DELAY_S = 0.1  # ロボット搭載センサーの遅延時間 (秒), 例: 25ms


BALL_RADIUS_M = 0.0215  # ボールの半径 (メートル)
BALL_FRICTION_COEFF = 0.4  # ボールの摩擦係数
GRAVITY_MPSS = 9.81  # 重力加速度 (メートル/秒^2)

SENSOR_FOV_HALF_ANGLE_RAD = math.radians(40)  # センサーの視野角の半分 (ラジアン)
