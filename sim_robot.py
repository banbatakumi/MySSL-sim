import pygame
import math
import time

import sim_constants as const
import sim_params as params  # 調整可能なパラメータをインポート
from sim_utils import normalize_angle_rad

# 循環インポートを避けるための前方型ヒント
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from sim_ball import SimulatedBall
    from simulator import Simulator  # メインシミュレータクラス


class SimulatedRobot:
    def __init__(self, robot_id_num, color_name, initial_x_m, initial_y_m, initial_angle_deg, radius_m=params.ROBOT_RADIUS_M):
        self.color_name = color_name  # "yellow" または "blue"
        self.x_m = initial_x_m  # X座標 (メートル)
        self.y_m = initial_y_m  # Y座標 (メートル)
        # 向き (ラジアン、ワールド座標系: +Xから反時計回り、Y軸上向き)
        self.angle_rad = math.radians(initial_angle_deg)
        self.radius_m = radius_m  # 半径 (メートル)

        self.vx_mps = 0.0  # X方向の速度 (メートル/秒)
        self.vy_mps = 0.0  # Y方向の速度 (メートル/秒)
        self.omega_radps = 0.0  # 角速度 (ラジアン/秒)

        self.current_command = None  # 現在のコマンド辞書
        self.last_command_time = 0  # 最後にコマンドを受信した時刻

        # デバッグ用: 実効移動方向 (グローバル、ラジアン)
        self.debug_effective_move_angle_global_rad = 0.0
        self.debug_move_speed_mps = 0.0  # デバッグ用: 移動速度 (メートル/秒)

        # 物理パラメータ (調整可能)
        self.mass_kg = params.ROBOT_MASS_KG
        self.wheel_friction_coeff = params.OMNI_WHEEL_FRICTION_COEFF

    def set_command(self, command_dict):
        """ロボットへのコマンドを設定する"""
        self.current_command = command_dict.get("cmd")
        self.last_command_time = time.time()

    def update_physics(self, dt, ball: 'SimulatedBall'):
        """ロボットの物理演算を更新する"""
        self.debug_effective_move_angle_global_rad = self.angle_rad
        self.debug_move_speed_mps = 0.0

        if self.current_command is None or (time.time() - self.last_command_time > 0.5):
            self.vx_mps *= 0.9
            self.vy_mps *= 0.9
            self.omega_radps *= 0.9
            if abs(self.vx_mps) < 0.01:
                self.vx_mps = 0
            if abs(self.vy_mps) < 0.01:
                self.vy_mps = 0
            if abs(self.omega_radps) < 0.01:
                self.omega_radps = 0
        else:
            cmd = self.current_command
            if cmd.get("stop", False):
                self.vx_mps = 0.0
                self.vy_mps = 0.0
                self.omega_radps = 0.0
                if ball.is_dribbled_by == self:
                    ball.stop_dribble()
            else:
                move_speed_mps = cmd.get("move_speed", 0.0)
                move_angle_relative_deg = cmd.get("move_angle", 0.0)
                move_angle_relative_rad = math.radians(move_angle_relative_deg)
                effective_move_angle_global_rad = normalize_angle_rad(
                    self.angle_rad + move_angle_relative_rad)

                self.debug_effective_move_angle_global_rad = effective_move_angle_global_rad
                self.debug_move_speed_mps = move_speed_mps

                target_vx_global = move_speed_mps * \
                    math.cos(effective_move_angle_global_rad)
                target_vy_global = move_speed_mps * - \
                    math.sin(effective_move_angle_global_rad)

                # --- スリップモデルの導入 ---
                target_ax_global = (target_vx_global -
                                    self.vx_mps) / dt if dt > 0 else 0
                target_ay_global = (target_vy_global -
                                    self.vy_mps) / dt if dt > 0 else 0

                target_force_x_N = self.mass_kg * target_ax_global
                target_force_y_N = self.mass_kg * target_ay_global
                target_force_magnitude_N = math.hypot(
                    target_force_x_N, target_force_y_N)

                # ロボット全体が発揮できる最大推進力 (単純化モデル)
                max_robot_propulsion_force_N = self.mass_kg * \
                    params.GRAVITY_MPSS * self.wheel_friction_coeff * params.NUM_OMNI_WHEELS

                actual_force_x_N = target_force_x_N
                actual_force_y_N = target_force_y_N

                if target_force_magnitude_N > max_robot_propulsion_force_N and target_force_magnitude_N > 0:
                    scale_factor = max_robot_propulsion_force_N / target_force_magnitude_N
                    actual_force_x_N *= scale_factor
                    actual_force_y_N *= scale_factor
                    print(
                        f"スリップ検知! 目標力: {target_force_magnitude_N:.2f}N, 最大: {max_robot_propulsion_force_N:.2f}N")

                actual_ax_global = actual_force_x_N / self.mass_kg if self.mass_kg > 0 else 0
                actual_ay_global = actual_force_y_N / self.mass_kg if self.mass_kg > 0 else 0
                # --- スリップモデルここまで ---

                # 元の加速度制限 (ROBOT_MAX_ACCE_MPSS) は、スリップ後の加速度に適用するか検討。
                # スリップモデルが物理的限界を表すため、ここではスリップ後の加速度をそのまま使用。
                max_accel_this_command = cmd.get(
                    "move_acce", params.ROBOT_MAX_ACCE_MPSS)
                current_actual_accel_magnitude = math.hypot(
                    actual_ax_global, actual_ay_global)
                if current_actual_accel_magnitude > max_accel_this_command:
                    scale = max_accel_this_command / current_actual_accel_magnitude
                    actual_ax_global *= scale
                    actual_ay_global *= scale

                self.vx_mps += actual_ax_global * dt
                self.vy_mps += actual_ay_global * dt

                # 速度制限 (ROBOT_MAX_SPEED_MPS) はスリップモデル適用後にかける
                current_speed_magnitude = math.hypot(self.vx_mps, self.vy_mps)
                if current_speed_magnitude > params.ROBOT_MAX_SPEED_MPS:
                    scale = params.ROBOT_MAX_SPEED_MPS / current_speed_magnitude
                    self.vx_mps *= scale
                    self.vy_mps *= scale

                target_face_angle_deg = cmd.get(
                    "face_angle", math.degrees(self.angle_rad))
                target_face_angle_rad = math.radians(target_face_angle_deg)
                face_speed_limit_radps = cmd.get(
                    "face_speed", params.ROBOT_MAX_ANGULAR_SPEED_RADPS)
                if cmd.get("face_axis") == 0:
                    face_speed_limit_radps = params.ROBOT_MAX_ANGULAR_SPEED_RADPS

                angle_diff_rad = normalize_angle_rad(
                    target_face_angle_rad - self.angle_rad)

                if dt > 0:
                    if abs(angle_diff_rad) < abs(face_speed_limit_radps * dt):
                        self.omega_radps = angle_diff_rad / dt
                    else:
                        self.omega_radps = math.copysign(
                            face_speed_limit_radps, angle_diff_rad)
                else:
                    self.omega_radps = 0

                if abs(self.omega_radps) > params.ROBOT_MAX_ANGULAR_SPEED_RADPS:
                    self.omega_radps = math.copysign(
                        params.ROBOT_MAX_ANGULAR_SPEED_RADPS, self.omega_radps)

            kick_power = cmd.get("kick", 0)
            dribble_power = cmd.get("dribble", 0)
            kicked_this_step = False

            if isinstance(kick_power, (int, float)) and kick_power > 0:
                dist_to_ball_center = math.hypot(
                    ball.x_m - self.x_m, ball.y_m - self.y_m)
                max_kick_dist = self.radius_m + params.BALL_RADIUS_M * 0.5
                if dist_to_ball_center < max_kick_dist:
                    angle_to_ball_global_cw = - \
                        math.atan2(ball.y_m - self.y_m, ball.x_m - self.x_m)
                    relative_angle_to_ball = normalize_angle_rad(
                        angle_to_ball_global_cw - self.angle_rad)
                    if abs(relative_angle_to_ball) < math.radians(30):
                        if ball.is_dribbled_by == self:
                            ball.stop_dribble()
                        ball.kick(self, kick_power)
                        kicked_this_step = True

            if not kicked_this_step:
                if isinstance(dribble_power, (int, float)) and dribble_power > 0:
                    dist_center_to_center = math.hypot(
                        ball.x_m - self.x_m, ball.y_m - self.y_m)
                    touch_distance = self.radius_m + params.BALL_RADIUS_M - 0.005
                    initiate_dribble_max_dist = self.radius_m + params.BALL_RADIUS_M + 0.02
                    if dist_center_to_center < initiate_dribble_max_dist:
                        angle_to_ball_global_cw = - \
                            math.atan2(ball.y_m - self.y_m,
                                       ball.x_m - self.x_m)
                        relative_angle_to_ball = normalize_angle_rad(
                            angle_to_ball_global_cw - self.angle_rad)
                        if abs(relative_angle_to_ball) < math.radians(70):
                            if dist_center_to_center < touch_distance or ball.is_dribbled_by == self:
                                ball.start_dribble(self)
                else:
                    if ball.is_dribbled_by == self:
                        ball.stop_dribble()

        self.x_m += self.vx_mps * dt
        self.y_m += self.vy_mps * dt
        self.angle_rad = normalize_angle_rad(
            self.angle_rad + self.omega_radps * dt)

        half_court_width = const.COURT_WIDTH_M / 2.0
        half_court_height = const.COURT_HEIGHT_M / 2.0
        self.x_m = max(-half_court_width + self.radius_m,
                       min(self.x_m, half_court_width - self.radius_m))
        self.y_m = max(-half_court_height + self.radius_m,
                       min(self.y_m, half_court_height - self.radius_m))

    def get_vision_data(self):
        return {
            "type": self.color_name,
            "angle": round(math.degrees(self.angle_rad), 2),
            "pos": (round(self.x_m * 100.0, 2), round(self.y_m * 100.0, 2))
        }

    def get_sensor_data(self, ball: 'SimulatedBall'):
        """ロボットの生のセンサーデータを取得する (ボール検知、遅延なし)"""
        photo_front = False
        photo_back = False
        dx = ball.x_m - self.x_m
        dy = ball.y_m - self.y_m
        dist_center_to_center = math.hypot(dx, dy)
        overlap_depth_m = params.BALL_RADIUS_M * 0.2  # params.BALL_RADIUS_M
        dribbler_offset_m = self.radius_m - overlap_depth_m

        if dist_center_to_center <= dribbler_offset_m + 0.005:
            angle_to_ball_global_cw = -math.atan2(dy, dx)
            relative_angle_to_ball_cw = normalize_angle_rad(
                angle_to_ball_global_cw - self.angle_rad)
            if abs(relative_angle_to_ball_cw) < params.SENSOR_FOV_HALF_ANGLE_RAD:
                photo_front = True
            angle_to_back_sensor_cw = normalize_angle_rad(
                relative_angle_to_ball_cw - math.pi)
            if abs(angle_to_back_sensor_cw) < params.SENSOR_FOV_HALF_ANGLE_RAD:
                photo_back = True

        return {"type": "sensor_data", "photo": {"front": photo_front, "back": photo_back}}

    def draw(self, screen: pygame.Surface, simulator: 'Simulator'):
        robot_color_rgb = const.COLOR_YELLOW_ROBOT if self.color_name == "yellow" else const.COLOR_BLUE_ROBOT
        screen_x, screen_y = simulator.world_to_screen_pos(self.x_m, self.y_m)
        robot_screen_radius = max(
            1, int(self.radius_m * simulator.current_pixels_per_meter))

        pygame.draw.circle(screen, robot_color_rgb,
                           (screen_x, screen_y), robot_screen_radius)
        pygame.draw.circle(screen, (0, 0, 0), (screen_x, screen_y),
                           robot_screen_radius, const.ROBOT_OUTLINE_WIDTH_PX)

        front_indicator_len_screen = self.radius_m * \
            0.8 * simulator.current_pixels_per_meter

        front_x_indicator = screen_x + \
            front_indicator_len_screen * math.cos(self.angle_rad)
        front_y_indicator = screen_y + \
            front_indicator_len_screen * math.sin(self.angle_rad)
        pygame.draw.line(screen, const.COLOR_ROBOT_FRONT, (screen_x, screen_y), (int(
            front_x_indicator), int(front_y_indicator)), 3)

        if simulator.show_debug_vectors and self.current_command and not self.current_command.get("stop", False) and self.debug_move_speed_mps > 0:
            vec_len_world = 0.3 * \
                (self.debug_move_speed_mps / params.ROBOT_MAX_SPEED_MPS)

            vec_end_x_world = self.x_m + vec_len_world * \
                math.cos(self.debug_effective_move_angle_global_rad)
            vec_end_y_world = self.y_m + vec_len_world * - \
                math.sin(self.debug_effective_move_angle_global_rad)

            vec_end_screen_x, vec_end_screen_y = simulator.world_to_screen_pos(
                vec_end_x_world, vec_end_y_world)
            pygame.draw.line(screen, const.COLOR_DEBUG_VECTOR, (screen_x,
                             screen_y), (vec_end_screen_x, vec_end_screen_y), 2)
