import config  # COURT_WIDTH, COURT_HEIGHT のため


COURT_WIDTH_M = config.COURT_WIDTH / 100.0  # コートの幅 (メートル)
COURT_HEIGHT_M = config.COURT_HEIGHT / 100.0  # コートの高さ (メートル)


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
FPS = 30  # 目標フレームレート
