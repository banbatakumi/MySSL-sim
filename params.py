import math

# --- DivAのパラメータ ---
COURT_WIDTH_M = 12  # コートの幅 (メートル) - 白線間の距離
COURT_HEIGHT_M = 9  # コートの高さ (メートル) - 白線間の距離
CENTEWR_CIRCLE_RADIUS_M = 0.5  # センターサークルの半径 (メートル)
GOAL_AREA_WIDTH_M = 3.6  # ゴールエリアの幅 (メートル)
GOAL_AREA_HEIGHT_M = 1.8  # ゴールエリアの高さ (メートル)
GOAL_WIDTH = 1.8
GOAL_HEIGHT = 0.2


# --- DivBのパラメータ ---
# COURT_WIDTH_M = 9  # コートの幅 (メートル) - 白線間の距離
# COURT_HEIGHT_M = 6  # コートの高さ (メートル) - 白線間の距離
# CENTEWR_CIRCLE_RADIUS_M = 0.5  # センターサークルの半径 (メートル)
# GOAL_AREA_WIDTH_M = 2  # ゴールエリアの幅 (メートル)
# GOAL_AREA_HEIGHT_M = 1  # ゴールエリアの高さ (メートル)
# GOAL_WIDTH = 1
# GOAL_HEIGHT = 0.2

# --- ロボット物理パラメータ ---
ROBOT_MASS_KG = 1.6  # ロボットの質量 (kg)
OMNI_WHEEL_FRICTION_COEFF = 0.5  # オムニホイールの摩擦係数
NUM_OMNI_WHEELS = 4  # ホイール数
ROBOT_DIAMETER_M = 0.18  # ロボットの直径 (メートル)
ROBOT_RADIUS_M = ROBOT_DIAMETER_M * 0.5  # ロボットの半径 (メートル)
ROBOT_MAX_SPEED_MPS = 1.5  # ロボットの最大速度 (メートル/秒)
ROBOT_MAX_ACCE_MPSS = 5.0  # ロボットの最大加速度 (メートル/秒^2)
ROBOT_MAX_ANGULAR_SPEED_RADPS = 2.5 * math.pi  # ロボットの最大角速度 (ラジアン/秒) - 約3 RPS
KICK_POWER_TO_SPEED_MPS = 2.5 * 0.01  # キックパワーから速度への変換係数 (メートル/秒)
KICK_ANGLE_RANDOMNESS_DEG = 5  # キック角度のランダム性の幅 (度、ロボット正面から左右それぞれ)
DRIBBLE_PULL_FACTOR = 7.0  # ドリブル時のボール引き寄せ係数


# --- センサー遅延設定 ---
VISION_DATA_DELAY_S = 0.1  # Visionデータの遅延時間 (秒), 例: 100ms
ROBOT_SENSOR_DELAY_S = 0.1  # ロボット搭載センサーの遅延時間 (秒), 例: 100ms


BALL_RADIUS_M = 0.021  # ボールの半径 (メートル)
BALL_FRICTION_COEFF = 0.1  # ボールの摩擦係数
GRAVITY_MPSS = 9.81  # 重力加速度 (メートル/秒^2)
BALL_MASS_KG = 0.043  # ボールの質量 (kg) (SSL規格では40-45g)

SENSOR_FOV_HALF_ANGLE_RAD = math.radians(20)  # センサーの視野角の半分 (ラジアン)

# --- コートと壁のパラメータ ---
WALL_OFFSET_M = 0.30  # コートの白線から壁までのオフセット距離 (メートル)
BALL_WALL_RESTITUTION_COEFF = 0.5  # ボールと壁の反発係数
ROBOT_WALL_RESTITUTION_COEFF = 0.4  # ロボットと壁の反発係数
ROBOT_ROBOT_RESTITUTION_COEFF = 0.5  # ロボット同士の反発係数

# --- ボールとロボットの衝突パラメータ ---
# 高速パスとしてロボットがボールを弾く場合の反発係数
BALL_ROBOT_FAST_PASS_RESTITUTION_COEFF = 0.25
# ボールがロボットにこの法線方向相対速度以上で接近した場合、高速パスとみなしドリブルせずに弾く閾値 (m/s)
ROBOT_BALL_FAST_PASS_THRESHOLD_MPS = 2  # 値は調整が必要
# 上記以外（ドリブルにも至らない）の通常衝突時のボールとロボットの反発係数
BALL_ROBOT_NORMAL_RESTITUTION_COEFF = 0.5
