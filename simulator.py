from sim_udp_handler import SimulatorUDP
from sim_robot import SimulatedRobot
from sim_ball import SimulatedBall
import config
import params as params
import pygame
import time
import sys
import math  # math.hypot, math.sqrt のために追加


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
        self._update_drawing_parameters()  # 描画パラメータを更新

        pygame.display.set_caption("SSLシミュレータ")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 16)
        self.running = True
        self.show_debug_vectors = False  # デバッグベクトルの表示フラグ

        self.robots: dict[str, SimulatedRobot] = {}

        # 黄色ロボットの初期化
        for robot_conf in config.YELLOW_ROBOTS_CONFIG:
            if robot_conf["enabled"]:
                robot_id_str = f"yellow_{robot_conf['id']}"
                self.robots[robot_id_str] = SimulatedRobot(
                    "yellow",
                    robot_conf['id'],  # robot_id_num を渡す
                    robot_conf["initial_pos_x_m"],
                    robot_conf["initial_pos_y_m"],
                    robot_conf["initial_angle_deg"]  # configの角度もCW正と解釈
                )
                print(
                    f"ロボット {robot_id_str} を初期位置 ({robot_conf['initial_pos_x_m']:.2f}, {robot_conf['initial_pos_y_m']:.2f}) で有効化")

        # 青色ロボットの初期化
        for robot_conf in config.BLUE_ROBOTS_CONFIG:
            if robot_conf["enabled"]:
                robot_id_str = f"blue_{robot_conf['id']}"
                self.robots[robot_id_str] = SimulatedRobot(
                    "blue",
                    robot_conf['id'],  # robot_id_num を渡す
                    robot_conf["initial_pos_x_m"],
                    robot_conf["initial_pos_y_m"],
                    robot_conf["initial_angle_deg"]  # configの角度もCW正と解釈
                )
                print(
                    f"ロボット {robot_id_str} を初期位置 ({robot_conf['initial_pos_x_m']:.2f}, {robot_conf['initial_pos_y_m']:.2f}) で有効化")

        self.ball: SimulatedBall = SimulatedBall(0, 0)
        # UDPハンドラに渡すのは、実際にインスタンス化されたロボットの辞書
        self.udp_handler: SimulatorUDP = SimulatorUDP(
            self.robots, self.ball, self)

        self.last_vision_send_time = 0  # 最後にVisionデータを送信した時刻
        self.last_sensor_send_time = 0  # 最後にセンサーデータを送信した時刻
        self.data_send_interval = config.CONTROL_LOOP_INTERVAL  # データ送信間隔

    def world_to_screen_pos(self, x_m: float, y_m: float) -> tuple[int, int]:
        # ワールド座標(Y軸上向き)をスクリーン座標(Y軸下向き)に変換
        center_x_px = self.current_screen_width_px / 2
        center_y_px = self.current_screen_height_px / 2
        screen_x = center_x_px + x_m * self.current_pixels_per_meter
        screen_y = center_y_px - y_m * self.current_pixels_per_meter  # Y軸反転
        return int(screen_x), int(screen_y)

    def screen_to_world_pos(self, screen_x_px: int, screen_y_px: int) -> tuple[float, float]:
        # スクリーン座標(Y軸下向き)をワールド座標(Y軸上向き)に変換
        center_x_px = self.current_screen_width_px / 2
        center_y_px = self.current_screen_height_px / 2
        if self.current_pixels_per_meter == 0:
            return 0.0, 0.0
        world_x_m = (screen_x_px - center_x_px) / self.current_pixels_per_meter
        world_y_m = (center_y_px - screen_y_px) / \
            self.current_pixels_per_meter  # Y軸反転
        return world_x_m, world_y_m

    def _update_drawing_parameters(self):
        # スクリーンサイズ変更時に描画関連のパラメータを更新
        self.current_screen_width_px = self.screen.get_width()
        self.current_screen_height_px = self.screen.get_height()

        effective_width_px = self.current_screen_width_px - \
            2 * self.current_screen_padding_px
        effective_height_px = self.current_screen_height_px - \
            2 * self.current_screen_padding_px

        world_display_width_m = params.COURT_WIDTH_M + 2 * params.WALL_OFFSET_M
        world_display_height_m = params.COURT_HEIGHT_M + 2 * params.WALL_OFFSET_M

        if world_display_width_m > 0 and world_display_height_m > 0 and effective_width_px > 0 and effective_height_px > 0:
            ppm_w = effective_width_px / world_display_width_m
            ppm_h = effective_height_px / world_display_height_m
            self.current_pixels_per_meter = min(ppm_w, ppm_h)
        else:
            self.current_pixels_per_meter = 1.0  # フォールバック

        if self.current_pixels_per_meter <= 0:
            self.current_pixels_per_meter = 1.0  # フォールバック

    def resolve_robot_collisions(self, dt: float, num_iterations=5):
        # ロボット同士の衝突解決処理
        robots_list = list(self.robots.values())
        num_robots = len(robots_list)
        restitution_coeff = params.ROBOT_ROBOT_RESTITUTION_COEFF  # 反発係数

        for _ in range(num_iterations):  # 安定性のために複数回イテレーション
            collision_occurred_in_iteration = False
            for i in range(num_robots):
                robot1 = robots_list[i]
                for j in range(i + 1, num_robots):
                    robot2 = robots_list[j]

                    dx = robot2.x_m - robot1.x_m
                    dy = robot2.y_m - robot1.y_m
                    dist_sq = dx*dx + dy*dy

                    sum_radii = robot1.radius_m + robot2.radius_m

                    # 衝突チェック (dist_sq > 0 は完全に一致している場合の問題を避けるため)
                    if dist_sq < sum_radii * sum_radii and dist_sq > 1e-9:  # 衝突発生
                        collision_occurred_in_iteration = True
                        dist = math.sqrt(dist_sq)

                        # 1. オーバーラップの解決
                        overlap = sum_radii - dist

                        # 正規化された衝突ベクトル (robot1 から robot2 へ)
                        nx = dx / dist
                        ny = dy / dist

                        # 質量に応じて移動量を分配
                        # m1*d1 + m2*d2 = 0, d2 - d1 = overlap_vec
                        # d1 = -overlap * m2 / (m1+m2)
                        # d2 = overlap * m1 / (m1+m2)
                        total_mass = robot1.mass_kg + robot2.mass_kg
                        if total_mass > 1e-9:
                            move_dist1 = -overlap * \
                                (robot2.mass_kg / total_mass)
                            move_dist2 = overlap * \
                                (robot1.mass_kg / total_mass)
                        else:  # ゼロ質量なら半分ずつ
                            move_dist1 = -overlap / 2.0
                            move_dist2 = overlap / 2.0

                        robot1.x_m += nx * move_dist1
                        robot1.y_m += ny * move_dist1
                        robot2.x_m += nx * move_dist2
                        robot2.y_m += ny * move_dist2

                        # 2. 衝突応答 (速度)
                        # 相対速度
                        rvx = robot2.vx_mps - robot1.vx_mps
                        rvy = robot2.vy_mps - robot1.vy_mps

                        # 法線方向の速度成分 (ベクトル r1->r2)
                        vel_along_normal = rvx * nx + rvy * ny

                        # 速度が離れる方向の場合は解決しない (vel_along_normal > 0)
                        if vel_along_normal > 0:
                            continue

                        m1 = robot1.mass_kg
                        m2 = robot2.mass_kg

                        if (1/m1 + 1/m2) <= 1e-9:  # 有効質量の分母ゼロ回避
                            impulse_j = 0
                        else:
                            impulse_j = -(1 + restitution_coeff) * \
                                vel_along_normal
                            impulse_j /= (1/m1 + 1/m2)  # 有効質量

                        # 力積を適用
                        impulse_x = impulse_j * nx
                        impulse_y = impulse_j * ny

                        robot1.vx_mps -= impulse_x / m1
                        robot1.vy_mps -= impulse_y / m1
                        robot2.vx_mps += impulse_x / m2
                        robot2.vy_mps += impulse_y / m2

            if not collision_occurred_in_iteration:
                break  # このステップでの全ての衝突が解決された

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
                    if event.button == 1:  # 左クリック
                        wx, wy = self.screen_to_world_pos(*event.pos)

                        # ボール配置境界 (ボールの壁衝突境界と同じ)
                        h_wall_boundary_w = params.COURT_WIDTH_M / 2.0 + \
                            params.WALL_OFFSET_M - self.ball.radius_m
                        h_wall_boundary_h = params.COURT_HEIGHT_M / 2.0 + \
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
                    if event.key == pygame.K_r:  # ボールリセット
                        if self.ball.is_dribbled_by:
                            self.ball.stop_dribble()
                        self.ball = SimulatedBall(0, 0)
                        self.udp_handler.ball = self.ball  # UDPハンドラのボール参照を更新
                    elif event.key == pygame.K_m:  # デバッグベクトルの表示切り替え
                        self.show_debug_vectors = not self.show_debug_vectors
                    elif event.key == pygame.K_ESCAPE:
                        self.running = False

            if dt <= 0:  # dtが0以下の場合の問題を回避
                time.sleep(0.001)  # dtが0の場合のビジーループを防ぐために小スリープ
                continue

            # 1. ロボットの内部状態を更新 (コマンドからの運動、ボールとの相互作用「試行」)
            for robot in self.robots.values():
                robot.update_physics(dt, self.ball)

            # 2. ロボット同士の衝突を解決 (位置と速度を調整)
            if len(self.robots) > 1:  # ロボットが複数いる場合のみ
                self.resolve_robot_collisions(dt)

            # 3. ロボットにコート境界を適用 (壁に対する最終的な位置/速度調整)
            #    ボールとの衝突より後、ボールの物理更新より前が良いか、あるいは全物理更新の後か。
            #    ここではボール更新の前にロボットの位置を確定させる。
            for robot in self.robots.values():
                robot.apply_court_boundaries()

            # 4. ボールの物理演算を更新 (最終的なロボット位置と相互作用、壁との衝突)
            #    self.robots.values() をリストにして渡す
            self.ball.update_physics(dt, list(self.robots.values()))

            # 5. (オプション) ボール更新後に再度ロボット境界を適用する場合もあるが、通常は1回で十分
            # for robot in self.robots.values():
            #    robot.apply_court_boundaries()

            # 設定された間隔でデータを送信
            current_time_for_send = time.time()
            if current_time_for_send - self.last_vision_send_time >= self.data_send_interval:
                self.udp_handler.send_vision_data()
                self.last_vision_send_time = current_time_for_send
            if current_time_for_send - self.last_sensor_send_time >= self.data_send_interval:
                self.udp_handler.send_sensor_data()
                self.last_sensor_send_time = current_time_for_send

            self.draw()  # 描画処理
            self.clock.tick(config.FPS)  # FPS制御

        self.cleanup()  # 終了処理

    def draw_field(self):
        self.screen.fill(config.COLOR_BACKGROUND)  # 背景色で塗りつぶし
        hw_m, hh_m = params.COURT_WIDTH_M / 2.0, params.COURT_HEIGHT_M / 2.0  # コート半幅、半高
        # フィールドラインの左上スクリーン座標
        tl_lines_sx, tl_lines_sy = self.world_to_screen_pos(-hw_m, hh_m)
        # フィールドラインの幅と高さ (ピクセル)
        field_lines_w_px = max(
            1, int(params.COURT_WIDTH_M * self.current_pixels_per_meter))
        field_lines_h_px = max(
            1, int(params.COURT_HEIGHT_M * self.current_pixels_per_meter))

        if field_lines_w_px > 0 and field_lines_h_px > 0:
            pygame.draw.rect(self.screen, config.COLOR_FIELD_LINES,
                             (tl_lines_sx, tl_lines_sy,
                              field_lines_w_px, field_lines_h_px),
                             config.FIELD_MARKING_WIDTH_PX)  # フィールド外枠

        # センターサークル
        cx_s, cy_s = self.world_to_screen_pos(0, 0)  # 中心点
        cc_r_px = int(params.CENTEWR_CIRCLE_RADIUS_M *
                      self.current_pixels_per_meter)
        if cc_r_px >= config.FIELD_MARKING_WIDTH_PX:
            pygame.draw.circle(self.screen, config.COLOR_FIELD_LINES,
                               (cx_s, cy_s), cc_r_px, config.FIELD_MARKING_WIDTH_PX)
        elif cc_r_px > 0:  # 線幅が太すぎる場合は塗りつぶし
            pygame.draw.circle(
                self.screen, config.COLOR_FIELD_LINES, (cx_s, cy_s), cc_r_px)

        # センターライン
        cl_top_s = self.world_to_screen_pos(0, hh_m)
        cl_bot_s = self.world_to_screen_pos(0, -hh_m)
        pygame.draw.line(self.screen, config.COLOR_FIELD_LINES,
                         cl_top_s, cl_bot_s, config.FIELD_MARKING_WIDTH_PX)
        cl_left_s = self.world_to_screen_pos(hw_m, 0)
        cl_right_s = self.world_to_screen_pos(-hw_m, 0)
        pygame.draw.line(self.screen, config.COLOR_FIELD_LINES,
                         cl_left_s, cl_right_s, config.FIELD_MARKING_WIDTH_PX)

        # ゴールエリア（白色）
        goal_area_w = params.GOAL_AREA_WIDTH_M
        goal_area_h = params.GOAL_AREA_HEIGHT_M

        # 左ゴールエリア
        ga_left_x1 = -params.COURT_WIDTH_M / 2
        ga_left_x2 = ga_left_x1 + goal_area_h
        ga_y1 = goal_area_w / 2
        ga_y2 = -goal_area_w / 2
        ga_left_rect_topleft = self.world_to_screen_pos(ga_left_x1, ga_y1)
        ga_left_rect_bottomright = self.world_to_screen_pos(ga_left_x2, ga_y2)
        pygame.draw.rect(
            self.screen,
            config.COLOR_FIELD_LINES,
            pygame.Rect(
                min(ga_left_rect_topleft[0], ga_left_rect_bottomright[0]),
                min(ga_left_rect_topleft[1], ga_left_rect_bottomright[1]),
                abs(ga_left_rect_bottomright[0] - ga_left_rect_topleft[0]),
                abs(ga_left_rect_bottomright[1] - ga_left_rect_topleft[1])
            ),
            config.FIELD_MARKING_WIDTH_PX
        )

        # 右ゴールエリア
        ga_right_x2 = params.COURT_WIDTH_M / 2
        ga_right_x1 = ga_right_x2 - goal_area_h
        ga_right_rect_topleft = self.world_to_screen_pos(ga_right_x1, ga_y1)
        ga_right_rect_bottomright = self.world_to_screen_pos(
            ga_right_x2, ga_y2)
        pygame.draw.rect(
            self.screen,
            config.COLOR_FIELD_LINES,
            pygame.Rect(
                min(ga_right_rect_topleft[0], ga_right_rect_bottomright[0]),
                min(ga_right_rect_topleft[1], ga_right_rect_bottomright[1]),
                abs(ga_right_rect_bottomright[0] - ga_right_rect_topleft[0]),
                abs(ga_right_rect_bottomright[1] - ga_right_rect_topleft[1])
            ),
            config.FIELD_MARKING_WIDTH_PX
        )

        # ゴール（黒色）
        goal_w = params.GOAL_WIDTH   # ゴールの幅（Y方向）
        goal_h = params.GOAL_HEIGHT  # ゴールの厚み（X方向）

        # 左ゴール
        left_goal_center_x = -params.COURT_WIDTH_M / 2 - goal_h / 2
        left_goal_rect_topleft = self.world_to_screen_pos(
            left_goal_center_x - goal_h / 2,  goal_w / 2)
        left_goal_rect_bottomright = self.world_to_screen_pos(
            left_goal_center_x + goal_h / 2, -goal_w / 2)
        pygame.draw.rect(
            self.screen,
            config.COLOR_GOAL,
            pygame.Rect(
                min(left_goal_rect_topleft[0], left_goal_rect_bottomright[0]),
                min(left_goal_rect_topleft[1], left_goal_rect_bottomright[1]),
                abs(left_goal_rect_bottomright[0] - left_goal_rect_topleft[0]),
                abs(left_goal_rect_bottomright[1] - left_goal_rect_topleft[1])
            )
        )

        # 右ゴール
        right_goal_center_x = params.COURT_WIDTH_M / 2 + goal_h / 2
        right_goal_rect_topleft = self.world_to_screen_pos(
            right_goal_center_x - goal_h / 2,  goal_w / 2)
        right_goal_rect_bottomright = self.world_to_screen_pos(
            right_goal_center_x + goal_h / 2, -goal_w / 2)
        pygame.draw.rect(
            self.screen,
            config.COLOR_GOAL,
            pygame.Rect(
                min(right_goal_rect_topleft[0],
                    right_goal_rect_bottomright[0]),
                min(right_goal_rect_topleft[1],
                    right_goal_rect_bottomright[1]),
                abs(right_goal_rect_bottomright[0] -
                    right_goal_rect_topleft[0]),
                abs(right_goal_rect_bottomright[1] -
                    right_goal_rect_topleft[1])
            )
        )

        # 壁 (フィールドラインから WALL_OFFSET_M 外側)
        wall_line_thickness_px = config.WALL_LINE_WIDTH_PX
        wall_top_y_m = params.COURT_HEIGHT_M / 2.0 + params.WALL_OFFSET_M
        wall_bottom_y_m = - (params.COURT_HEIGHT_M /
                             2.0 + params.WALL_OFFSET_M)
        wall_left_x_m = - (params.COURT_WIDTH_M / 2.0 + params.WALL_OFFSET_M)
        wall_right_x_m = params.COURT_WIDTH_M / 2.0 + params.WALL_OFFSET_M
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
        # HUD (Heads-Up Display) の描画 (FPSなど)
        y_offset = 10
        sim_fps_text = f"Sim FPS: {self.clock.get_fps():.1f}"
        surf_fps = self.font.render(sim_fps_text, True, config.COLOR_TEXT)
        self.screen.blit(
            surf_fps, (self.current_screen_width_px - surf_fps.get_width() - 10, y_offset))

    def draw(self):
        # 全描画処理の呼び出し
        self.draw_field()
        for robot in self.robots.values():
            robot.draw(self.screen, self)
        self.ball.draw(self.screen, self)
        self.draw_hud()
        pygame.display.flip()  # 画面更新

    def cleanup(self):
        # 終了時のクリーンアップ処理
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
