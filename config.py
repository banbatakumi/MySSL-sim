# --- UDP 通信設定 ---
# Vision からの受信ポート (シミュレータがリッスン)
VISION_LISTEN_PORT = 50007

# ロボットコマンドの受信ポート (シミュレータがリッスン)
YELLOW_SEND_PORT = 50008  # 歴史的経緯でSENDだが、シミュレータ側は受信
BLUE_SEND_PORT = 50012   # 歴史的経緯でSENDだが、シミュレータ側は受信

# ロボットセンサーデータの送信ポート (シミュレータが送信、コントローラがリッスン)
YELLOW_SENSOR_LISTEN_PORT = 50009
BLUE_SENSOR_LISTEN_PORT = 50010

# 待ち受けIPアドレス: 通常は '0.0.0.0' (すべてのインターフェースで待機)
LISTEN_IP = "0.0.0.0"

# コマンド送信元 (コントローラ等) のIPアドレス (シミュレータがコマンドを受信する際の相手)
# 通常、シミュレータは LISTEN_IP で待機し、任意のIPからのコマンドを受け付ける。
# 送信先IP (Visionデータやセンサーデータ) は SimulatorUDP 内で決定される。
ROBOT_LOCAL_IP = "127.0.0.1"  # これはコマンド送信元の想定だが、現状あまり使われていない

# 有効にするロボットの設定 (True/False で切り替え)
ENABLE_YELLOW_ROBOT = True
ENABLE_BLUE_ROBOT = False

# 受信バッファサイズ
BUFFER_SIZE = 65536

# 制御ループのポーリング間隔 (秒) - シミュレータ内のデータ送信周期の目安
CONTROL_LOOP_INTERVAL = 0.016  # 約60Hz (16ms)

COURT_WIDTH_M = 1.5  # コートの幅 (メートル) - 白線間の距離
COURT_HEIGHT_M = 1.0  # コートの高さ (メートル) - 白線間の距離


# --- 初期描画定数 (ウィンドウサイズ変更時に更新) ---
INITIAL_PIXELS_PER_METER = 300  # 初期スケール (ピクセル/メートル)
INITIAL_SCREEN_PADDING_PX = 50  # 初期画面の余白 (ピクセル)
INITIAL_SCREEN_WIDTH_PX = int(
    COURT_WIDTH_M * INITIAL_PIXELS_PER_METER) + 2 * INITIAL_SCREEN_PADDING_PX
INITIAL_SCREEN_HEIGHT_PX = int(COURT_HEIGHT_M * INITIAL_PIXELS_PER_METER) + \
    2 * INITIAL_SCREEN_PADDING_PX

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

# SIMULATION_TIMESTEP は simulator.py の dt で動的に計算されるため不要
FPS = 60  # 目標フレームレート (描画と物理演算の更新頻度)
