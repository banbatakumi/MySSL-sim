import pygame
import math
import time

import config
import params as params  # 調整可能なパラメータをインポート
from sim_utils import normalize_angle_rad  # 角度正規化関数

# 循環インポートを避けるための前方型ヒント
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from sim_ball import SimulatedBall
    from simulator import Simulator


class SimulatedRobot:
    def __init__(self, color_name: str, robot_id_num: int, initial_x_m: float, initial_y_m: float, initial_angle_deg: float, radius_m: float = params.ROBOT_RADIUS_M):
        self.color_name = color_name  # "yellow" または "blue"
        self.robot_id_num = robot_id_num  # ロボットのID (config.py由来)
        self.x_m = initial_x_m  # X座標 (メートル)
        self.y_m = initial_y_m  # Y座標 (メートル)
        # 角度 (ラジアン、ワールド座標系+X軸から時計回り(CW)が正)
        self.angle_rad = math.radians(initial_angle_deg)  # configの角度もCW正と解釈
        self.radius_m = radius_m  # 半径 (メートル)

        self.vx_mps = 0.0  # X方向の速度 (メートル/秒)
        self.vy_mps = 0.0  # Y方向の速度 (メートル/秒)
        self.omega_radps = 0.0  # 角速度 (ラジアン/秒、CWが正)

        self.current_command = None  # 現在のコマンド辞書
        self.last_command_time = 0  # 最後にコマンドを受信した時刻

        # デバッグ用: 実効移動方向 (グローバル、ラジアン、CWが正)
        self.debug_effective_move_angle_global_rad = 0.0
        self.debug_move_speed_mps = 0.0  # デバッグ用: 移動速度 (メートル/秒)

        self.mass_kg = params.ROBOT_MASS_KG  # 質量 (kg)
        self.wheel_friction_coeff = params.OMNI_WHEEL_FRICTION_COEFF  # ホイール摩擦係数

    def set_command(self, command_dict):
        """ロボットへのコマンドを設定する"""
        self.current_command = command_dict.get("cmd")
        self.last_command_time = time.time()

    def _update_velocities_from_command(self, dt: float, cmd: dict):
        """コマンドに基づいて self.vx_mps, self.vy_mps, self.omega_radps を計算・更新する"""
        self.debug_effective_move_angle_global_rad = self.angle_rad  # 移動がない場合のデフォルト
        self.debug_move_speed_mps = 0.0

        if cmd.get("stop", False):
            self.vx_mps = 0.0
            self.vy_mps = 0.0
            self.omega_radps = 0.0
        else:
            move_speed_mps = cmd.get("move_speed", 0.0)
            # move_angle_relative_deg: ロボット正面0度、時計回り(CW)が正。例: 90度は左。
            move_angle_relative_deg = cmd.get("move_angle", 0.0)
            move_angle_relative_rad = math.radians(move_angle_relative_deg)

            # effective_move_angle_global_rad: ワールド+X軸からCWが正
            effective_move_angle_global_rad = normalize_angle_rad(
                self.angle_rad + move_angle_relative_rad)

            self.debug_effective_move_angle_global_rad = effective_move_angle_global_rad
            self.debug_move_speed_mps = move_speed_mps

            target_vx_global = move_speed_mps * \
                math.cos(effective_move_angle_global_rad)
            # ワールドY軸は上向き正、角度はCWなので、sinの符号を反転
            target_vy_global = move_speed_mps * - \
                math.sin(effective_move_angle_global_rad)

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
                # print(f"スリップ検知! 目標力: {target_force_magnitude_N:.2f}N, 最大: {max_robot_propulsion_force_N:.2f}N")

            actual_ax_global = actual_force_x_N / self.mass_kg if self.mass_kg > 0 else 0
            actual_ay_global = actual_force_y_N / self.mass_kg if self.mass_kg > 0 else 0

            max_accel_this_command = cmd.get(
                "move_acce", params.ROBOT_MAX_ACCE_MPSS)
            if (max_accel_this_command == 0):
                max_accel_this_command = params.ROBOT_MAX_ACCE_MPSS
            current_actual_accel_magnitude = math.hypot(
                actual_ax_global, actual_ay_global)
            if current_actual_accel_magnitude > max_accel_this_command and current_actual_accel_magnitude > 0:
                scale = max_accel_this_command / current_actual_accel_magnitude
                actual_ax_global *= scale
                actual_ay_global *= scale

            self.vx_mps += actual_ax_global * dt
            self.vy_mps += actual_ay_global * dt

            current_speed_magnitude = math.hypot(self.vx_mps, self.vy_mps)
            if current_speed_magnitude > params.ROBOT_MAX_SPEED_MPS and current_speed_magnitude > 0:
                scale = params.ROBOT_MAX_SPEED_MPS / current_speed_magnitude
                self.vx_mps *= scale
                self.vy_mps *= scale

            # face_angle: ワールド+X軸からCWが正 (度単位)
            target_face_angle_deg = cmd.get(
                "face_angle", math.degrees(self.angle_rad))
            target_face_angle_rad = normalize_angle_rad(
                math.radians(target_face_angle_deg))

            face_speed_limit_radps = cmd.get(
                "face_speed", params.ROBOT_MAX_ANGULAR_SPEED_RADPS)
            if cmd.get("face_axis", 0) == 0:
                face_speed_limit_radps = params.ROBOT_MAX_ANGULAR_SPEED_RADPS  # 中心回転時は最大角速度

            angle_diff_rad = normalize_angle_rad(
                target_face_angle_rad - self.angle_rad)

            if dt > 0:
                max_rotation_this_step = abs(face_speed_limit_radps * dt)
                if abs(angle_diff_rad) <= max_rotation_this_step:
                    self.omega_radps = angle_diff_rad / dt  # CWが正
                else:
                    self.omega_radps = math.copysign(
                        face_speed_limit_radps, angle_diff_rad)  # CWが正
            else:
                self.omega_radps = 0.0

            if abs(self.omega_radps) > params.ROBOT_MAX_ANGULAR_SPEED_RADPS:
                self.omega_radps = math.copysign(
                    params.ROBOT_MAX_ANGULAR_SPEED_RADPS, self.omega_radps)

    def _update_position_from_velocities(self, dt: float, cmd: dict):
        """現在の速度に基づいて self.x_m, self.y_m, self.angle_rad を更新する"""
        x_translation_this_step = self.vx_mps * dt
        y_translation_this_step = self.vy_mps * dt
        angle_rotation_this_step = self.omega_radps * dt  # CWの回転量

        initial_x_m_this_step = self.x_m
        initial_y_m_this_step = self.y_m
        initial_angle_rad_this_step = self.angle_rad  # CW

        if cmd and cmd.get("face_axis", 0) == 1:  # 前方軸回転
            pivot_offset_m = self.radius_m

            # 回転軸のワールド座標 (initial_angle_rad_this_step はCW)
            pivot_x_relative_to_center = pivot_offset_m * \
                math.cos(initial_angle_rad_this_step)
            pivot_y_relative_to_center = pivot_offset_m * - \
                math.sin(initial_angle_rad_this_step)  # Y軸上向き, CW角度

            pivot_x_world_at_step_start = initial_x_m_this_step + pivot_x_relative_to_center
            pivot_y_world_at_step_start = initial_y_m_this_step + pivot_y_relative_to_center

            translated_pivot_x_world = pivot_x_world_at_step_start + x_translation_this_step
            translated_pivot_y_world = pivot_y_world_at_step_start + y_translation_this_step

            # 回転軸からロボット中心へのベクトル (回転前)
            dx_pivot_to_center = -pivot_x_relative_to_center
            dy_pivot_to_center = -pivot_y_relative_to_center

            # angle_rotation_this_step はCW
            cos_rot = math.cos(angle_rotation_this_step)
            sin_rot = math.sin(angle_rotation_this_step)

            # CW回転行列を適用:
            # x' = x*cos(theta_cw) + y*sin(theta_cw)
            # y' = -x*sin(theta_cw) + y*cos(theta_cw)
            rotated_dx_pivot_to_center = dx_pivot_to_center * \
                cos_rot + dy_pivot_to_center * sin_rot
            rotated_dy_pivot_to_center = -dx_pivot_to_center * \
                sin_rot + dy_pivot_to_center * cos_rot

            self.x_m = translated_pivot_x_world + rotated_dx_pivot_to_center
            self.y_m = translated_pivot_y_world + rotated_dy_pivot_to_center
            self.angle_rad = normalize_angle_rad(
                initial_angle_rad_this_step + angle_rotation_this_step)  # CW
        else:  # 中心軸回転
            self.x_m = initial_x_m_this_step + x_translation_this_step
            self.y_m = initial_y_m_this_step + y_translation_this_step
            self.angle_rad = normalize_angle_rad(
                initial_angle_rad_this_step + angle_rotation_this_step)  # CW

    def _handle_ball_interactions(self, dt: float, ball: 'SimulatedBall', cmd: dict):
        """現在の状態とコマンドに基づいてキックとドリブルを「試行」する"""
        if not cmd:
            return

        kick_power = cmd.get("kick", 0)
        dribble_power = cmd.get("dribble", 0)

        # キック試行
        if isinstance(kick_power, (int, float)) and kick_power > 0:
            dist_to_ball_center = math.hypot(
                ball.x_m - self.x_m, ball.y_m - self.y_m)
            # キック可能距離: ロボット半径 + ボール半径の半分程度 (ボールにめり込んでいる想定)
            max_kick_dist = self.radius_m + params.BALL_RADIUS_M * 0.75
            if dist_to_ball_center < max_kick_dist:
                world_angle_to_ball_rad_cw = - \
                    math.atan2(ball.y_m - self.y_m, ball.x_m - self.x_m)
                relative_angle_to_ball_rad = normalize_angle_rad(
                    world_angle_to_ball_rad_cw - self.angle_rad)

                if abs(relative_angle_to_ball_rad) < math.radians(30):  # キッカー視野角: +/- 30度
                    # ボール側の衝突判定で高速パスとして弾かれる可能性もあるが、ここではキックを試みる
                    # ドリブル中なら解除してからキック
                    if ball.is_dribbled_by == self:
                        ball.stop_dribble()
                    ball.kick(self, kick_power)
                    return  # キックしたらその後のドリブル試行はしない

        # ドリブル試行 (キックしなかった場合)
        if isinstance(dribble_power, (int, float)) and dribble_power > 0:
            dist_center_to_center = math.hypot(
                ball.x_m - self.x_m, ball.y_m - self.y_m)
            # ドリブル開始可能距離: ロボット半径 + ボール半径 + 少しの余裕
            initiate_dribble_max_dist = self.radius_m + params.BALL_RADIUS_M + 0.02

            if dist_center_to_center < initiate_dribble_max_dist:
                world_angle_to_ball_rad_cw = - \
                    math.atan2(ball.y_m - self.y_m, ball.x_m - self.x_m)
                relative_angle_to_ball_rad = normalize_angle_rad(
                    world_angle_to_ball_rad_cw - self.angle_rad)

                # ドリブラー視野角: +/- 70度程度
                if abs(relative_angle_to_ball_rad) < math.radians(30):
                    # ボール側の衝突判定で高速パスとして弾かれる可能性もあるが、ここではドリブルを試みる
                    # ドリブルセンサーに触れる程度の距離、または既に自分がドリブル中なら維持しようとする
                    touch_distance = self.radius_m + params.BALL_RADIUS_M + 0.002
                    if dist_center_to_center < touch_distance or ball.is_dribbled_by == self:
                        ball.start_dribble(self)
        else:  # ドリブルパワーが0または無効
            if ball.is_dribbled_by == self:
                ball.stop_dribble()

    def update_physics(self, dt: float, ball: 'SimulatedBall'):
        """シミュレータから呼び出される主要な物理更新。運動とボール相互作用を扱う。"""
        cmd_to_process = self.current_command
        is_cmd_timed_out = (time.time() - self.last_command_time > 0.5)

        if cmd_to_process is None or is_cmd_timed_out:
            # コマンドがないかタイムアウトした場合、摩擦/減衰を適用
            self.vx_mps *= 0.9
            self.vy_mps *= 0.9
            self.omega_radps *= 0.9  # 角速度の減衰
            if abs(self.vx_mps) < 0.01:
                self.vx_mps = 0.0
            if abs(self.vy_mps) < 0.01:
                self.vy_mps = 0.0
            if abs(self.omega_radps) < 0.01:
                self.omega_radps = 0.0

            if is_cmd_timed_out and ball.is_dribbled_by == self:
                ball.stop_dribble()  # コマンドタイムアウトでドリブル停止

            active_cmd = {"stop": True}  # 運動更新用に "stop" コマンドを使用
        else:
            active_cmd = cmd_to_process

        # 1. コマンドに基づいて速度 (vx, vy, omega) を更新 (または停止コマンド)
        self._update_velocities_from_command(dt, active_cmd)

        # 2. 新しい速度に基づいて位置 (x, y, angle) を更新
        self._update_position_from_velocities(dt, active_cmd)

        # 3. ボール相互作用 (キック、ドリブル) を元のコマンドで処理
        #    active_cmdが{"stop": True}の場合、_handle_ball_interactionsに渡すcmdはNone相当が良い。
        effective_ball_interaction_cmd = active_cmd if not active_cmd.get(
            "stop") else None
        self._handle_ball_interactions(
            dt, ball, effective_ball_interaction_cmd)

    def apply_court_boundaries(self):
        """self.x_m, self.y_m にコート境界を適用し、速度を反射させる"""
        half_court_w = params.COURT_WIDTH_M / 2.0
        half_court_h = params.COURT_HEIGHT_M / 2.0
        restitution = params.ROBOT_WALL_RESTITUTION_COEFF  # 壁との反発係数

        # 壁は +/- (コート次元/2 + WALL_OFFSET_M) の位置にある
        # ロボット中心はこの壁から radius_m 離れている必要がある
        limit_pos_x = half_court_w + params.WALL_OFFSET_M - self.radius_m
        limit_neg_x = -half_court_w - params.WALL_OFFSET_M + self.radius_m
        limit_pos_y = half_court_h + params.WALL_OFFSET_M - self.radius_m
        limit_neg_y = -half_court_h - params.WALL_OFFSET_M + self.radius_m

        if self.x_m > limit_pos_x:
            self.x_m = limit_pos_x
            self.vx_mps *= -restitution
        elif self.x_m < limit_neg_x:
            self.x_m = limit_neg_x
            self.vx_mps *= -restitution

        if self.y_m > limit_pos_y:
            self.y_m = limit_pos_y
            self.vy_mps *= -restitution
        elif self.y_m < limit_neg_y:
            self.y_m = limit_neg_y
            self.vy_mps *= -restitution

    def get_vision_data(self):
        # Visionシステム用のロボットデータを取得
        return {
            "angle": round(math.degrees(self.angle_rad), 2),  # 度単位、+X軸からCW
            "pos": (round(self.x_m, 3), round(self.y_m, 3))  # m単位
        }

    def get_sensor_data(self, ball: 'SimulatedBall'):
        """ロボットの生センサーデータを取得 (ボール検知、遅延なし)"""
        photo_front = False
        photo_back = False
        dx_ball = ball.x_m - self.x_m
        dy_ball = ball.y_m - self.y_m
        dist_center_to_center = math.hypot(dx_ball, dy_ball)

        # センサーがロボットの端より少し内側にあり、ボール表面を検出すると仮定
        detection_distance_threshold = self.radius_m + params.BALL_RADIUS_M - 0.005

        if dist_center_to_center <= detection_distance_threshold:
            # ロボットからボールへの角度 (ワールド座標系、+X軸からCW)
            angle_to_ball_world_rad_cw = -math.atan2(dy_ball, dx_ball)
            # ロボット正面に対するボールの相対角度 (CW)
            relative_angle_to_ball_rad = normalize_angle_rad(
                angle_to_ball_world_rad_cw - self.angle_rad)

            # 前方センサー
            if abs(relative_angle_to_ball_rad) < params.SENSOR_FOV_HALF_ANGLE_RAD:
                photo_front = True

            # 後方センサー (前方から180度)
            # ボールへの角度をロボット後方からの相対角度に変換
            relative_angle_to_ball_for_back_sensor_rad = normalize_angle_rad(
                relative_angle_to_ball_rad - math.pi)  # または +math.pi
            if abs(relative_angle_to_ball_for_back_sensor_rad) < params.SENSOR_FOV_HALF_ANGLE_RAD:
                photo_back = True

        return {"type": "sensor_data", "photo": {"front": photo_front, "back": photo_back}}

    def draw(self, screen: pygame.Surface, simulator: 'Simulator'):
        # ロボットを描画
        robot_color_rgb = config.COLOR_YELLOW_ROBOT if self.color_name == "yellow" else config.COLOR_BLUE_ROBOT
        screen_x, screen_y = simulator.world_to_screen_pos(self.x_m, self.y_m)
        robot_screen_radius = max(
            1, int(self.radius_m * simulator.current_pixels_per_meter))

        pygame.draw.circle(screen, robot_color_rgb,
                           (screen_x, screen_y), robot_screen_radius)  # 本体
        pygame.draw.circle(screen, (0, 0, 0), (screen_x, screen_y),
                           robot_screen_radius, config.ROBOT_OUTLINE_WIDTH_PX)  # 輪郭

        # self.angle_rad はワールド+X軸からCW (Y軸上向きワールド)
        # スクリーン座標系: Y軸は下向き。
        # CW角度でスクリーンY軸下向きの場合、Y成分は +sin(angle_cw)
        front_indicator_len_screen = self.radius_m * \
            0.8 * simulator.current_pixels_per_meter

        front_x_indicator_offset = front_indicator_len_screen * \
            math.cos(self.angle_rad)
        front_y_indicator_offset = front_indicator_len_screen * \
            math.sin(self.angle_rad)  # スクリーンY軸下向き、CW角度のため

        front_x_indicator = screen_x + front_x_indicator_offset
        front_y_indicator = screen_y + front_y_indicator_offset

        pygame.draw.line(screen, config.COLOR_ROBOT_FRONT, (screen_x, screen_y),
                         # 正面インジケータ
                         (int(front_x_indicator), int(front_y_indicator)), 3)

        if simulator.show_debug_vectors and self.current_command and not self.current_command.get("stop", False) and self.debug_move_speed_mps > 0:
            # debug_effective_move_angle_global_rad はワールド+X軸からCW
            vec_len_world = 0.3 * \
                (self.debug_move_speed_mps / params.ROBOT_MAX_SPEED_MPS)

            vec_end_x_world = self.x_m + vec_len_world * \
                math.cos(self.debug_effective_move_angle_global_rad)
            vec_end_y_world = self.y_m + vec_len_world * - \
                math.sin(self.debug_effective_move_angle_global_rad)  # ワールドY軸上向き

            vec_end_screen_x, vec_end_screen_y = simulator.world_to_screen_pos(
                vec_end_x_world, vec_end_y_world)
            pygame.draw.line(screen, config.COLOR_DEBUG_VECTOR, (screen_x,
                             # デバッグベクトル
                                                                 screen_y), (vec_end_screen_x, vec_end_screen_y), 2)
