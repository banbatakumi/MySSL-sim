# --- UDP 通信設定 ---
# Vision からの受信ポート
VISION_LISTEN_PORT = 50007

# ESP32 への送信ポート
YELLOW_SEND_PORT = 50008
BLUE_SEND_PORT = 50012

# ESP32 からの受信ポート
YELLOW_SENSOR_LISTEN_PORT = 50009  # 黄ロボット用のセンサー受信ポート
BLUE_SENSOR_LISTEN_PORT = 50010  # 青ロボット用のセンサー受信ポート

# ゲームコントローラーからのコマンド受信ポート
GAME_COMMAND_LISTEN_PORT = 50011

# 待ち受けIPアドレス: 通常は '0.0.0.0'
LISTEN_IP = "0.0.0.0"

# ロボットのローカル IP アドレス
YELLOW_ROBOT_IP = "127.0.0.1"
BLUE_ROBOT_IP = "127.0.0.1"

# 有効にするロボットの設定 (True/False で切り替え)
ENABLE_YELLOW_ROBOT = True
ENABLE_BLUE_ROBOT = False

# 受信バッファサイズ
BUFFER_SIZE = 65536

# 制御ループのポーリング間隔 (秒)
CONTROL_LOOP_INTERVAL = 0.01  # 10ms

# コート中心への移動時の閾値とゲイン
MAX_SPEED = 1        # 最大線形速度 m/s
PLACEMENT_R = 10  # cm

COURT_WIDTH = 150  # cm
COURT_HEIGHT = 100  # cm

ROBOT_R = 9
