from sim_udp_handler import SimulatorUDP
from sim_robot import SimulatedRobot
from sim_ball import SimulatedBall
import sim_constants as const  # 定数モジュールをインポート
import pygame
import time
import math
import sys

sys.path.append('.')
try:
    import config
except ImportError as e:
    print(f"プロジェクトモジュールのインポート中にエラーが発生しました: {e}")
    print("simulator.py が main.py, config.py および lib/ フォルダと同じディレクトリにあることを確認してください。")
    sys.exit(1)

# sim_config_params は sim_robot や sim_udp_handler で直接インポートされる


class Simulator:
    def __init__(self):
        pygame.init()
        pygame.font.init()

        self.current_screen_width_px = const.INITIAL_SCREEN_WIDTH_PX
        self.current_screen_height_px = const.INITIAL_SCREEN_HEIGHT_PX
        self.current_screen_padding_px = const.INITIAL_SCREEN_PADDING_PX
        self.current_pixels_per_meter = const.INITIAL_PIXELS_PER_METER

        self.screen: pygame.Surface = pygame.display.set_mode(
            (self.current_screen_width_px, self.current_screen_height_px),
            pygame.RESIZABLE
        )
        self._update_drawing_parameters()

        pygame.display.set_caption("ロボットサッカーシミュレータ")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 16)
        self.running = True
        self.show_debug_vectors = False

        self.robots: dict[str, SimulatedRobot] = {}
        if config.ENABLE_YELLOW_ROBOT:
            self.robots["yellow"] = SimulatedRobot(
                0, "yellow", -const.COURT_WIDTH_M/4, 0, 0)
        if config.ENABLE_BLUE_ROBOT:
            blue_initial_x = const.COURT_WIDTH_M/4 if config.ENABLE_YELLOW_ROBOT else 0
            self.robots["blue"] = SimulatedRobot(
                0, "blue", blue_initial_x, 0, 180)

        self.ball: SimulatedBall = SimulatedBall(0, 0)
        self.udp_handler: SimulatorUDP = SimulatorUDP(
            self.robots, self.ball, self)

        self.last_vision_send_time = 0
        self.last_sensor_send_time = 0
        self.data_send_interval = config.CONTROL_LOOP_INTERVAL

    def world_to_screen_pos(self, x_m: float, y_m: float) -> tuple[int, int]:
        center_x_px = self.current_screen_width_px / 2
        center_y_px = self.current_screen_height_px / 2
        screen_x = center_x_px + x_m * self.current_pixels_per_meter
        screen_y = center_y_px - y_m * self.current_pixels_per_meter
        return int(screen_x), int(screen_y)

    def screen_to_world_pos(self, screen_x_px: int, screen_y_px: int) -> tuple[float, float]:
        center_x_px = self.current_screen_width_px / 2
        center_y_px = self.current_screen_height_px / 2
        if self.current_pixels_per_meter == 0:
            return 0.0, 0.0
        world_x_m = (screen_x_px - center_x_px) / self.current_pixels_per_meter
        world_y_m = (center_y_px - screen_y_px) / self.current_pixels_per_meter
        return world_x_m, world_y_m

    def _update_drawing_parameters(self):
        self.current_screen_width_px = self.screen.get_width()
        self.current_screen_height_px = self.screen.get_height()

        effective_width = self.current_screen_width_px - \
            2 * self.current_screen_padding_px
        effective_height = self.current_screen_height_px - \
            2 * self.current_screen_padding_px

        if const.COURT_WIDTH_M > 0 and const.COURT_HEIGHT_M > 0 and effective_width > 0 and effective_height > 0:
            ppm_w = effective_width / const.COURT_WIDTH_M
            ppm_h = effective_height / const.COURT_HEIGHT_M
            self.current_pixels_per_meter = min(ppm_w, ppm_h)
        else:
            self.current_pixels_per_meter = 1.0

        if self.current_pixels_per_meter <= 0:
            self.current_pixels_per_meter = 1.0

    def run(self):
        accumulated_time = 0.0  # シミュレーションの累積時間
        last_time = time.time()
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.VIDEORESIZE:
                    self.screen = pygame.display.set_mode(
                        (event.w, event.h), pygame.RESIZABLE)
                    self._update_drawing_parameters()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        wx, wy = self.screen_to_world_pos(*event.pos)

                        h_court_w = const.COURT_WIDTH_M / 2.0 - self.ball.radius_m
                        h_court_h = const.COURT_HEIGHT_M / 2.0 - self.ball.radius_m

                        clamped_wx = max(-h_court_w, min(wx, h_court_w))
                        clamped_wy = max(-h_court_h, min(wy, h_court_h))

                        if self.ball.is_dribbled_by:
                            self.ball.stop_dribble()
                        self.ball.x_m, self.ball.y_m = clamped_wx, clamped_wy
                        self.ball.vx_mps, self.ball.vy_mps = 0.0, 0.0
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        if self.ball.is_dribbled_by:
                            self.ball.stop_dribble()
                        self.ball = SimulatedBall(0, 0)
                        self.udp_handler.ball = self.ball
                    elif event.key == pygame.K_m:
                        self.show_debug_vectors = not self.show_debug_vectors
                    elif event.key == pygame.K_ESCAPE:
                        self.running = False

            if dt <= 0:
                time.sleep(0.01)
                continue

            for robot in self.robots.values():
                robot.update_physics(dt, self.ball)
            self.ball.update_physics(dt)

            current_time = time.time()
            # データ送信はudp_handler内部のロジックで遅延制御されるので、
            # ここでは一定間隔で送信関数を呼び出すだけで良い
            if current_time - self.last_vision_send_time >= self.data_send_interval:
                self.udp_handler.send_vision_data()
                self.last_vision_send_time = current_time
            if current_time - self.last_sensor_send_time >= self.data_send_interval:
                self.udp_handler.send_sensor_data()
                self.last_sensor_send_time = current_time

            self.draw()
            self.clock.tick(const.FPS)

        self.cleanup()

    def draw_field(self):
        self.screen.fill(const.COLOR_BACKGROUND)
        hw_m, hh_m = const.COURT_WIDTH_M / 2.0, const.COURT_HEIGHT_M / 2.0

        tl_sx, tl_sy = self.world_to_screen_pos(-hw_m, hh_m)
        field_w_px = max(1, int(const.COURT_WIDTH_M *
                         self.current_pixels_per_meter))
        field_h_px = max(1, int(const.COURT_HEIGHT_M *
                         self.current_pixels_per_meter))

        field_rect = pygame.Rect(tl_sx, tl_sy, field_w_px, field_h_px)
        pygame.draw.rect(self.screen, const.COLOR_FIELD_LINES,
                         field_rect, const.FIELD_MARKING_WIDTH_PX)

        cx_s, cy_s = self.world_to_screen_pos(0, 0)
        center_circle_radius_m = 0.25
        cc_r_px = int(center_circle_radius_m * self.current_pixels_per_meter)
        if cc_r_px > 0:
            pygame.draw.circle(self.screen, const.COLOR_FIELD_LINES,
                               (cx_s, cy_s), cc_r_px, const.FIELD_MARKING_WIDTH_PX)

        cl_top_s = self.world_to_screen_pos(0, hh_m)
        cl_bot_s = self.world_to_screen_pos(0, -hh_m)
        pygame.draw.line(self.screen, const.COLOR_FIELD_LINES,
                         cl_top_s, cl_bot_s, const.FIELD_MARKING_WIDTH_PX)

    def draw_hud(self):
        y_offset = 10
        sim_fps_text = f"Sim FPS: {self.clock.get_fps():.1f}"
        surf_fps = self.font.render(sim_fps_text, True, const.COLOR_TEXT)
        self.screen.blit(
            surf_fps, (self.current_screen_width_px - surf_fps.get_width() - 10, y_offset))

    def draw(self):
        self.draw_field()
        for robot in self.robots.values():
            robot.draw(self.screen, self)
        self.ball.draw(self.screen, self)
        self.draw_hud()
        pygame.display.flip()

    def cleanup(self):
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
