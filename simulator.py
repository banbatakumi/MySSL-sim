from sim_udp_handler import SimulatorUDP
from sim_robot import SimulatedRobot
from sim_ball import SimulatedBall
import config
import sim_params as params
import pygame
import time
import sys


class Simulator:
    def __init__(self):
        pygame.init()
        pygame.font.init()

        self.current_screen_width_px = config.INITIAL_SCREEN_WIDTH_PX
        self.current_screen_height_px = config.INITIAL_SCREEN_HEIGHT_PX
        self.current_screen_padding_px = config.INITIAL_SCREEN_PADDING_PX

        self.screen: pygame.Surface = pygame.display.set_mode(
            (self.current_screen_width_px, self.current_screen_height_px),
            pygame.RESIZABLE
        )
        self._update_drawing_parameters()

        pygame.display.set_caption("SSLシミュレータ")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 16)
        self.running = True
        self.show_debug_vectors = False

        self.robots: dict[str, SimulatedRobot] = {}

        # 黄色ロボットの初期化
        for robot_conf in config.YELLOW_ROBOTS_CONFIG:
            if robot_conf["enabled"]:
                robot_id_str = f"yellow_{robot_conf['id']}"
                self.robots[robot_id_str] = SimulatedRobot(
                    "yellow",
                    robot_conf["initial_pos_x_m"],
                    robot_conf["initial_pos_y_m"],
                    robot_conf["initial_angle_deg"]
                )
                print(
                    f"ロボット {robot_id_str} を初期位置 ({robot_conf['initial_pos_x_m']:.2f}, {robot_conf['initial_pos_y_m']:.2f}) で有効化")

        # 青色ロボットの初期化
        for robot_conf in config.BLUE_ROBOTS_CONFIG:
            if robot_conf["enabled"]:
                robot_id_str = f"blue_{robot_conf['id']}"
                self.robots[robot_id_str] = SimulatedRobot(
                    "blue",
                    robot_conf["initial_pos_x_m"],
                    robot_conf["initial_pos_y_m"],
                    robot_conf["initial_angle_deg"]
                )
                print(
                    f"ロボット {robot_id_str} を初期位置 ({robot_conf['initial_pos_x_m']:.2f}, {robot_conf['initial_pos_y_m']:.2f}) で有効化")

        self.ball: SimulatedBall = SimulatedBall(0, 0)
        # UDPハンドラに渡すのは、実際にインスタンス化されたロボットの辞書
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

        effective_width_px = self.current_screen_width_px - \
            2 * self.current_screen_padding_px
        effective_height_px = self.current_screen_height_px - \
            2 * self.current_screen_padding_px

        world_display_width_m = config.COURT_WIDTH_M + 2 * params.WALL_OFFSET_M
        world_display_height_m = config.COURT_HEIGHT_M + 2 * params.WALL_OFFSET_M

        if world_display_width_m > 0 and world_display_height_m > 0 and effective_width_px > 0 and effective_height_px > 0:
            ppm_w = effective_width_px / world_display_width_m
            ppm_h = effective_height_px / world_display_height_m
            self.current_pixels_per_meter = min(ppm_w, ppm_h)
        else:
            self.current_pixels_per_meter = 1.0

        if self.current_pixels_per_meter <= 0:
            self.current_pixels_per_meter = 1.0

    def run(self):
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

                        h_wall_boundary_w = config.COURT_WIDTH_M / 2.0 + \
                            params.WALL_OFFSET_M - self.ball.radius_m
                        h_wall_boundary_h = config.COURT_HEIGHT_M / 2.0 + \
                            params.WALL_OFFSET_M - self.ball.radius_m

                        clamped_wx = max(-h_wall_boundary_w,
                                         min(wx, h_wall_boundary_w))
                        clamped_wy = max(-h_wall_boundary_h,
                                         min(wy, h_wall_boundary_h))

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
                time.sleep(0.001)
                continue

            for robot in self.robots.values():
                robot.update_physics(dt, self.ball)
            self.ball.update_physics(dt)

            current_time_for_send = time.time()
            if current_time_for_send - self.last_vision_send_time >= self.data_send_interval:
                self.udp_handler.send_vision_data()
                self.last_vision_send_time = current_time_for_send
            if current_time_for_send - self.last_sensor_send_time >= self.data_send_interval:
                self.udp_handler.send_sensor_data()
                self.last_sensor_send_time = current_time_for_send

            self.draw()
            self.clock.tick(config.FPS)

        self.cleanup()

    def draw_field(self):
        self.screen.fill(config.COLOR_BACKGROUND)
        hw_m, hh_m = config.COURT_WIDTH_M / 2.0, config.COURT_HEIGHT_M / 2.0
        tl_lines_sx, tl_lines_sy = self.world_to_screen_pos(-hw_m, hh_m)
        field_lines_w_px = max(
            1, int(config.COURT_WIDTH_M * self.current_pixels_per_meter))
        field_lines_h_px = max(
            1, int(config.COURT_HEIGHT_M * self.current_pixels_per_meter))

        if field_lines_w_px > 0 and field_lines_h_px > 0:
            pygame.draw.rect(self.screen, config.COLOR_FIELD_LINES,
                             (tl_lines_sx, tl_lines_sy,
                              field_lines_w_px, field_lines_h_px),
                             config.FIELD_MARKING_WIDTH_PX)

        cx_s, cy_s = self.world_to_screen_pos(0, 0)
        center_circle_radius_m = 0.25
        cc_r_px = int(center_circle_radius_m * self.current_pixels_per_meter)
        if cc_r_px >= config.FIELD_MARKING_WIDTH_PX:
            pygame.draw.circle(self.screen, config.COLOR_FIELD_LINES,
                               (cx_s, cy_s), cc_r_px, config.FIELD_MARKING_WIDTH_PX)
        elif cc_r_px > 0:
            pygame.draw.circle(
                self.screen, config.COLOR_FIELD_LINES, (cx_s, cy_s), cc_r_px)

        cl_top_s = self.world_to_screen_pos(0, hh_m)
        cl_bot_s = self.world_to_screen_pos(0, -hh_m)
        pygame.draw.line(self.screen, config.COLOR_FIELD_LINES,
                         cl_top_s, cl_bot_s, config.FIELD_MARKING_WIDTH_PX)

        wall_line_thickness_px = config.WALL_LINE_WIDTH_PX
        wall_top_y_m = config.COURT_HEIGHT_M / 2.0 + params.WALL_OFFSET_M
        wall_bottom_y_m = - (config.COURT_HEIGHT_M /
                             2.0 + params.WALL_OFFSET_M)
        wall_left_x_m = - (config.COURT_WIDTH_M / 2.0 + params.WALL_OFFSET_M)
        wall_right_x_m = config.COURT_WIDTH_M / 2.0 + params.WALL_OFFSET_M
        wall_top_left_s = self.world_to_screen_pos(wall_left_x_m, wall_top_y_m)
        wall_top_right_s = self.world_to_screen_pos(
            wall_right_x_m, wall_top_y_m)
        wall_bottom_left_s = self.world_to_screen_pos(
            wall_left_x_m, wall_bottom_y_m)
        wall_bottom_right_s = self.world_to_screen_pos(
            wall_right_x_m, wall_bottom_y_m)

        pygame.draw.line(self.screen, config.COLOR_WALLS,
                         wall_top_left_s, wall_top_right_s, wall_line_thickness_px)
        pygame.draw.line(self.screen, config.COLOR_WALLS, wall_bottom_left_s,
                         wall_bottom_right_s, wall_line_thickness_px)
        pygame.draw.line(self.screen, config.COLOR_WALLS,
                         wall_top_left_s, wall_bottom_left_s, wall_line_thickness_px)
        pygame.draw.line(self.screen, config.COLOR_WALLS, wall_top_right_s,
                         wall_bottom_right_s, wall_line_thickness_px)

    def draw_hud(self):
        y_offset = 10
        sim_fps_text = f"Sim FPS: {self.clock.get_fps():.1f}"
        surf_fps = self.font.render(sim_fps_text, True, config.COLOR_TEXT)
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

    # 有効なロボットがいるかどうかのチェック
    num_enabled_robots = 0
    for conf in config.YELLOW_ROBOTS_CONFIG:
        if conf["enabled"]:
            num_enabled_robots += 1
    for conf in config.BLUE_ROBOTS_CONFIG:
        if conf["enabled"]:
            num_enabled_robots += 1

    if num_enabled_robots == 0:
        print("警告: config.py で有効なロボットがありません。シミュレータは実行されますが、ロボットは制御されません。")

    sim = Simulator()
    sim.run()
