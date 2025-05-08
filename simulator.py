from sim_udp_handler import SimulatorUDP
from sim_robot import SimulatedRobot
from sim_ball import SimulatedBall
from sim_constants import (
    COURT_WIDTH_M, COURT_HEIGHT_M, BALL_RADIUS_M,  # BALL_RADIUS_M はマウスクリックのクランプ処理用
    INITIAL_SCREEN_WIDTH_PX, INITIAL_SCREEN_HEIGHT_PX,
    INITIAL_SCREEN_PADDING_PX, INITIAL_PIXELS_PER_METER,
    FIELD_MARKING_WIDTH_PX, COLOR_BACKGROUND, COLOR_FIELD_LINES, COLOR_TEXT,
    SIMULATION_TIMESTEP, FPS
)
import pygame
import time
import math
import sys

# プロジェクトルートを sys.path に追加して config と lib をインポート可能にする
sys.path.append('.')
try:
    import config
except ImportError as e:
    print(f"プロジェクトモジュールのインポート中にエラーが発生しました: {e}")
    print("simulator.py が main.py, config.py および lib/ フォルダと同じディレクトリにあることを確認してください。")
    sys.exit(1)

# sim_utils.normalize_angle_rad は sim_robot で使用され、ここでは直接使用されない


class Simulator:
    def __init__(self):
        pygame.init()
        pygame.font.init()

        self.current_screen_width_px = INITIAL_SCREEN_WIDTH_PX
        self.current_screen_height_px = INITIAL_SCREEN_HEIGHT_PX
        self.current_screen_padding_px = INITIAL_SCREEN_PADDING_PX
        self.current_pixels_per_meter = INITIAL_PIXELS_PER_METER

        self.screen: pygame.Surface = pygame.display.set_mode(
            (self.current_screen_width_px, self.current_screen_height_px),
            pygame.RESIZABLE  # ウィンドウサイズ変更可能
        )
        self._update_drawing_parameters()  # screen作成後に呼び出す

        pygame.display.set_caption("ロボットサッカーシミュレータ")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 16)  # HUD用フォント
        self.running = True  # メインループ実行フラグ
        self.show_debug_vectors = False  # ロボットのデバッグベクトル表示フラグ

        self.robots: dict[str, SimulatedRobot] = {}  # ロボットオブジェクトの辞書
        if config.ENABLE_YELLOW_ROBOT:
            self.robots["yellow"] = SimulatedRobot(
                0, "yellow", -COURT_WIDTH_M/4, 0, 0)
        if config.ENABLE_BLUE_ROBOT:
            blue_initial_x = COURT_WIDTH_M/4 if config.ENABLE_YELLOW_ROBOT else 0
            self.robots["blue"] = SimulatedRobot(
                0, "blue", blue_initial_x, 0, 180)

        self.ball: SimulatedBall = SimulatedBall(0, 0)  # ボールオブジェクト
        self.udp_handler: SimulatorUDP = SimulatorUDP(
            self.robots, self.ball, self)  # UDP通信ハンドラ

        self.last_vision_send_time = 0  # 最後にVisionデータを送信した時刻
        self.last_sensor_send_time = 0  # 最後にセンサーデータを送信した時刻
        # データ送信間隔 (configから)
        self.data_send_interval = config.CONTROL_LOOP_INTERVAL

    def world_to_screen_pos(self, x_m: float, y_m: float) -> tuple[int, int]:
        """ワールド座標 (メートル) をスクリーン座標 (ピクセル) に変換する"""
        center_x_px = self.current_screen_width_px / 2
        center_y_px = self.current_screen_height_px / 2
        screen_x = center_x_px + x_m * self.current_pixels_per_meter
        screen_y = center_y_px - y_m * self.current_pixels_per_meter  # ワールド座標系ではY軸が上
        return int(screen_x), int(screen_y)

    def screen_to_world_pos(self, screen_x_px: int, screen_y_px: int) -> tuple[float, float]:
        """スクリーン座標 (ピクセル) をワールド座標 (メートル) に変換する"""
        center_x_px = self.current_screen_width_px / 2
        center_y_px = self.current_screen_height_px / 2
        if self.current_pixels_per_meter == 0:
            return 0.0, 0.0  # ゼロ除算を避ける
        world_x_m = (screen_x_px - center_x_px) / self.current_pixels_per_meter
        world_y_m = (center_y_px - screen_y_px) / \
            self.current_pixels_per_meter  # スクリーン座標系ではY軸が下
        return world_x_m, world_y_m

    def _update_drawing_parameters(self):
        """描画パラメータ (スケールなど) を現在のウィンドウサイズに合わせて更新する"""
        # リサイズによって変更された可能性のある実際のスクリーン寸法を使用
        self.current_screen_width_px = self.screen.get_width()
        self.current_screen_height_px = self.screen.get_height()

        effective_width = self.current_screen_width_px - \
            2 * self.current_screen_padding_px
        effective_height = self.current_screen_height_px - \
            2 * self.current_screen_padding_px

        if COURT_WIDTH_M > 0 and COURT_HEIGHT_M > 0 and effective_width > 0 and effective_height > 0:
            ppm_w = effective_width / COURT_WIDTH_M  # 幅基準のスケール
            ppm_h = effective_height / COURT_HEIGHT_M  # 高さ基準のスケール
            self.current_pixels_per_meter = min(
                ppm_w, ppm_h)  # より小さいスケールを採用して全体を表示
        else:
            self.current_pixels_per_meter = 1.0  # フォールバック

        if self.current_pixels_per_meter <= 0:  # 正の値を保証
            self.current_pixels_per_meter = 1.0

    def run(self):
        """シミュレータのメインループを実行する"""
        while self.running:
            dt = SIMULATION_TIMESTEP  # 固定タイムステップ

            # イベント処理
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.VIDEORESIZE:  # ウィンドウサイズ変更
                    self.screen = pygame.display.set_mode(
                        (event.w, event.h), pygame.RESIZABLE)
                    self._update_drawing_parameters()  # 描画パラメータを更新
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # 左クリック
                        wx, wy = self.screen_to_world_pos(*event.pos)

                        # ボール位置をコート境界内にクランプ (ボール半径を考慮)
                        h_court_w = COURT_WIDTH_M / 2.0 - self.ball.radius_m  # self.ball.radius_m を使用
                        h_court_h = COURT_HEIGHT_M / 2.0 - self.ball.radius_m

                        clamped_wx = max(-h_court_w, min(wx, h_court_w))
                        clamped_wy = max(-h_court_h, min(wy, h_court_h))

                        if self.ball.is_dribbled_by:
                            self.ball.stop_dribble()  # ドリブル中なら解除
                        self.ball.x_m, self.ball.y_m = clamped_wx, clamped_wy
                        self.ball.vx_mps, self.ball.vy_mps = 0.0, 0.0  # 速度もリセット
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:  # 'r'キーでボールリセット
                        if self.ball.is_dribbled_by:
                            self.ball.stop_dribble()
                        self.ball = SimulatedBall(0, 0)  # デフォルト半径で再生成
                        self.udp_handler.ball = self.ball  # UDPハンドラのボール参照も更新
                    elif event.key == pygame.K_m:  # 'm'キーでデバッグベクトル表示切り替え
                        self.show_debug_vectors = not self.show_debug_vectors
                    elif event.key == pygame.K_ESCAPE:  # ESCキーで終了
                        self.running = False

            if dt <= 0:  # 固定SIMULATION_TIMESTEPでは発生しないはず
                time.sleep(0.01)
                continue

            # 物理演算の更新
            for robot in self.robots.values():
                robot.update_physics(dt, self.ball)
            self.ball.update_physics(dt)

            # データ送信
            current_time = time.time()
            if current_time - self.last_vision_send_time >= self.data_send_interval:
                self.udp_handler.send_vision_data()
                self.last_vision_send_time = current_time
            if current_time - self.last_sensor_send_time >= self.data_send_interval:
                self.udp_handler.send_sensor_data()
                self.last_sensor_send_time = current_time

            self.draw()  # 描画
            self.clock.tick(FPS)  # FPS制御

        self.cleanup()  # 終了処理

    def draw_field(self):
        """フィールドを描画する"""
        self.screen.fill(COLOR_BACKGROUND)
        hw_m, hh_m = COURT_WIDTH_M / 2.0, COURT_HEIGHT_M / 2.0  # コートの半分の幅と高さ

        # フィールドの左上隅 (スクリーン座標)
        tl_sx, tl_sy = self.world_to_screen_pos(-hw_m, hh_m)
        field_w_px = max(
            1, int(COURT_WIDTH_M * self.current_pixels_per_meter))  # 最小1ピクセル
        field_h_px = max(
            1, int(COURT_HEIGHT_M * self.current_pixels_per_meter))

        field_rect = pygame.Rect(tl_sx, tl_sy, field_w_px, field_h_px)
        pygame.draw.rect(self.screen, COLOR_FIELD_LINES,
                         field_rect, FIELD_MARKING_WIDTH_PX)  # 外枠

        # センターサークル
        cx_s, cy_s = self.world_to_screen_pos(0, 0)
        center_circle_radius_m = 0.25  # 元のコードでの暗黙的な値
        cc_r_px = int(center_circle_radius_m * self.current_pixels_per_meter)
        if cc_r_px > 0:
            pygame.draw.circle(self.screen, COLOR_FIELD_LINES,
                               (cx_s, cy_s), cc_r_px, FIELD_MARKING_WIDTH_PX)

        # センターライン
        cl_top_s = self.world_to_screen_pos(0, hh_m)
        cl_bot_s = self.world_to_screen_pos(0, -hh_m)
        pygame.draw.line(self.screen, COLOR_FIELD_LINES,
                         cl_top_s, cl_bot_s, FIELD_MARKING_WIDTH_PX)

    def draw_hud(self):
        """HUD (FPS情報など) を描画する"""
        y_offset = 10
        sim_fps_text = f"Sim FPS: {self.clock.get_fps():.1f}"
        surf_fps = self.font.render(sim_fps_text, True, COLOR_TEXT)
        # 現在のスクリーン幅に対して相対的な位置に調整
        self.screen.blit(
            surf_fps, (self.current_screen_width_px - surf_fps.get_width() - 10, y_offset))

    def draw(self):
        """全体の描画処理を行う"""
        self.draw_field()
        for robot in self.robots.values():
            robot.draw(self.screen, self)  # self (Simulatorインスタンス) を渡す
        self.ball.draw(self.screen, self)   # self (Simulatorインスタンス) を渡す
        self.draw_hud()
        pygame.display.flip()  # 画面更新

    def cleanup(self):
        """シミュレータ終了時のクリーンアップ処理"""
        print("シミュレータをシャットダウンしています...")
        self.udp_handler.stop()
        pygame.quit()
        print("シミュレータが終了しました。")


if __name__ == "__main__":
    print("ロボットサッカーシミュレータを開始しています...")
    if not (config.ENABLE_YELLOW_ROBOT or config.ENABLE_BLUE_ROBOT):
        print("警告: config.py で有効なロボットがありません。シミュレータは実行されますが、ロボットは制御されません。")

    sim = Simulator()
    sim.run()
