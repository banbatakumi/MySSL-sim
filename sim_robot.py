import pygame
import math
import time

from sim_constants import (
    ROBOT_RADIUS_M, COURT_WIDTH_M, COURT_HEIGHT_M,
    ROBOT_MAX_SPEED_MPS, ROBOT_MAX_ACCE_MPSS, ROBOT_MAX_ANGULAR_SPEED_RADPS,
    SENSOR_FOV_HALF_ANGLE_RAD, COLOR_YELLOW_ROBOT, COLOR_BLUE_ROBOT,
    COLOR_ROBOT_FRONT, COLOR_DEBUG_VECTOR, ROBOT_OUTLINE_WIDTH_PX,
    BALL_RADIUS_M  # キック/ドリブル距離計算に必要
)
from sim_utils import normalize_angle_rad

# 循環インポートを避けるための前方型ヒント
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from sim_ball import SimulatedBall
    from simulator import Simulator  # メインシミュレータクラス


class SimulatedRobot:
    def __init__(self, robot_id_num, color_name, initial_x_m, initial_y_m, initial_angle_deg, radius_m=ROBOT_RADIUS_M):
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

    def set_command(self, command_dict):
        """ロボットへのコマンドを設定する"""
        self.current_command = command_dict.get("cmd")
        self.last_command_time = time.time()

    def update_physics(self, dt, ball: 'SimulatedBall'):
        """ロボットの物理演算を更新する"""
        # デバッグ値をリセット
        self.debug_effective_move_angle_global_rad = self.angle_rad  # 移動コマンドがない場合は現在の向き
        self.debug_move_speed_mps = 0.0

        if self.current_command is None or (time.time() - self.last_command_time > 0.5):
            # コマンドがない、または古い場合は減速
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
                # 停止コマンド
                self.vx_mps = 0.0
                self.vy_mps = 0.0
                self.omega_radps = 0.0
                if ball.is_dribbled_by == self:
                    ball.stop_dribble()  # ドリブル中なら解除
            else:
                # 移動コマンド処理
                move_speed_mps = cmd.get("move_speed", 0.0)  # 移動速度
                move_angle_relative_deg = cmd.get(
                    "move_angle", 0.0)  # 移動方向 (ロボット相対、度)
                move_angle_relative_rad = math.radians(move_angle_relative_deg)
                # 実効的な移動方向 (グローバル座標系)
                effective_move_angle_global_rad = normalize_angle_rad(
                    self.angle_rad + move_angle_relative_rad)

                # デバッグ描画用に保存
                self.debug_effective_move_angle_global_rad = effective_move_angle_global_rad
                self.debug_move_speed_mps = move_speed_mps

                # 目標速度 (グローバル座標系)
                target_vx_global = move_speed_mps * \
                    math.cos(effective_move_angle_global_rad)
                target_vy_global = move_speed_mps * - \
                    math.sin(effective_move_angle_global_rad)  # ワールド座標系ではY軸が上

                # 加速度制限
                max_accel_this_command = cmd.get(
                    "move_acce", ROBOT_MAX_ACCE_MPSS)
                if max_accel_this_command == 0:
                    max_accel_this_command = ROBOT_MAX_ACCE_MPSS
                elif max_accel_this_command == 1 and ROBOT_MAX_ACCE_MPSS > 1:
                    max_accel_this_command = ROBOT_MAX_ACCE_MPSS

                accel_x = (target_vx_global - self.vx_mps) / \
                    dt if dt > 0 else 0
                accel_y = (target_vy_global - self.vy_mps) / \
                    dt if dt > 0 else 0
                current_accel_magnitude = math.hypot(accel_x, accel_y)

                if current_accel_magnitude > max_accel_this_command:
                    scale = max_accel_this_command / current_accel_magnitude
                    accel_x *= scale
                    accel_y *= scale

                self.vx_mps += accel_x * dt
                self.vy_mps += accel_y * dt

                # 速度制限
                current_speed_magnitude = math.hypot(self.vx_mps, self.vy_mps)
                if current_speed_magnitude > ROBOT_MAX_SPEED_MPS:
                    scale = ROBOT_MAX_SPEED_MPS / current_speed_magnitude
                    self.vx_mps *= scale
                    self.vy_mps *= scale

                # 向き制御
                target_face_angle_deg = cmd.get(
                    "face_angle", math.degrees(self.angle_rad))  # 目標の向き (度)
                target_face_angle_rad = math.radians(target_face_angle_deg)
                face_speed_limit_radps = cmd.get(
                    "face_speed", ROBOT_MAX_ANGULAR_SPEED_RADPS)  # 目標角速度
                if cmd.get("face_axis") == 0:
                    face_speed_limit_radps = ROBOT_MAX_ANGULAR_SPEED_RADPS  # 軸指定なしなら最大角速度

                angle_diff_rad = normalize_angle_rad(
                    target_face_angle_rad - self.angle_rad)  # 現在の向きとの差

                if dt > 0:
                    if abs(angle_diff_rad) < abs(face_speed_limit_radps * dt):  # 1ステップで到達可能か
                        self.omega_radps = angle_diff_rad / dt
                    else:
                        self.omega_radps = math.copysign(
                            face_speed_limit_radps, angle_diff_rad)  # 符号を合わせて制限速度で回転
                else:
                    self.omega_radps = 0

                # 角速度制限
                if abs(self.omega_radps) > ROBOT_MAX_ANGULAR_SPEED_RADPS:
                    self.omega_radps = math.copysign(
                        ROBOT_MAX_ANGULAR_SPEED_RADPS, self.omega_radps)

            # キックとドリブル処理
            kick_power = cmd.get("kick", 0)
            dribble_power = cmd.get("dribble", 0)
            kicked_this_step = False  # このステップでキックしたか

            if isinstance(kick_power, (int, float)) and kick_power > 0:
                dist_to_ball_center = math.hypot(
                    ball.x_m - self.x_m, ball.y_m - self.y_m)
                max_kick_dist = self.radius_m + BALL_RADIUS_M * 0.5  # キック可能な最大距離
                if dist_to_ball_center < max_kick_dist:
                    # ボールへの角度 (グローバル、時計回り)
                    angle_to_ball_global_cw = - \
                        math.atan2(ball.y_m - self.y_m, ball.x_m - self.x_m)
                    # ボールへの相対角度
                    relative_angle_to_ball = normalize_angle_rad(
                        angle_to_ball_global_cw - self.angle_rad)
                    if abs(relative_angle_to_ball) < math.radians(30):  # 前方30度以内ならキック
                        if ball.is_dribbled_by == self:
                            ball.stop_dribble()
                        ball.kick(self, kick_power)
                        kicked_this_step = True

            if not kicked_this_step:  # キックしなかった場合のみドリブル判定
                if isinstance(dribble_power, (int, float)) and dribble_power > 0:
                    dist_center_to_center = math.hypot(
                        ball.x_m - self.x_m, ball.y_m - self.y_m)
                    touch_distance = self.radius_m + BALL_RADIUS_M - 0.005  # ドリブル継続に必要な接触距離
                    initiate_dribble_max_dist = self.radius_m + \
                        BALL_RADIUS_M + 0.02  # ドリブル開始可能な最大距離
                    if dist_center_to_center < initiate_dribble_max_dist:
                        angle_to_ball_global_cw = - \
                            math.atan2(ball.y_m - self.y_m,
                                       ball.x_m - self.x_m)
                        relative_angle_to_ball = normalize_angle_rad(
                            angle_to_ball_global_cw - self.angle_rad)
                        if abs(relative_angle_to_ball) < math.radians(70):  # 前方70度以内ならドリブル試行
                            if dist_center_to_center < touch_distance or ball.is_dribbled_by == self:
                                ball.start_dribble(self)
                else:  # ドリブルパワーが0または指定なし
                    if ball.is_dribbled_by == self:  # ドリブル中なら解除
                        ball.stop_dribble()

        # 位置と角度を更新
        self.x_m += self.vx_mps * dt
        self.y_m += self.vy_mps * dt
        self.angle_rad = normalize_angle_rad(
            self.angle_rad + self.omega_radps * dt)

        # コート境界内での位置制限
        half_court_width = COURT_WIDTH_M / 2.0
        half_court_height = COURT_HEIGHT_M / 2.0
        self.x_m = max(-half_court_width + self.radius_m,
                       min(self.x_m, half_court_width - self.radius_m))
        self.y_m = max(-half_court_height + self.radius_m,
                       min(self.y_m, half_court_height - self.radius_m))

    def get_vision_data(self):
        """Visionシステム用のロボットデータを取得する"""
        return {
            "type": self.color_name,
            "angle": round(math.degrees(self.angle_rad), 2),  # 度単位
            "pos": (round(self.x_m * 100.0, 2), round(self.y_m * 100.0, 2))  # cm単位
        }

    def get_sensor_data(self, ball: 'SimulatedBall'):
        """ロボットのセンサーデータを取得する (ボール検知)"""
        photo_front = False  # 前方センサー
        photo_back = False  # 後方センサー
        dx = ball.x_m - self.x_m
        dy = ball.y_m - self.y_m
        dist_center_to_center = math.hypot(dx, dy)
        overlap_depth_m = BALL_RADIUS_M * 0.2
        dribbler_offset_m = self.radius_m - overlap_depth_m  # ドリブラーの有効範囲

        if dist_center_to_center <= dribbler_offset_m + 0.005:  # ボールがドリブラー範囲内か
            angle_to_ball_global_cw = - \
                math.atan2(dy, dx)  # ボールへの角度 (グローバル、時計回り)
            relative_angle_to_ball_cw = normalize_angle_rad(
                angle_to_ball_global_cw - self.angle_rad)  # ボールへの相対角度

            if abs(relative_angle_to_ball_cw) < SENSOR_FOV_HALF_ANGLE_RAD:
                photo_front = True  # 前方センサー検知

            angle_to_back_sensor_cw = normalize_angle_rad(
                relative_angle_to_ball_cw - math.pi)  # 後方センサーの向き
            if abs(angle_to_back_sensor_cw) < SENSOR_FOV_HALF_ANGLE_RAD:
                photo_back = True  # 後方センサー検知

        return {"type": "sensor_data", "photo": {"front": photo_front, "back": photo_back}}

    def draw(self, screen: pygame.Surface, simulator: 'Simulator'):
        """ロボットを描画する"""
        robot_color_rgb = COLOR_YELLOW_ROBOT if self.color_name == "yellow" else COLOR_BLUE_ROBOT
        screen_x, screen_y = simulator.world_to_screen_pos(self.x_m, self.y_m)
        robot_screen_radius = max(
            1, int(self.radius_m * simulator.current_pixels_per_meter))  # 最小1ピクセル

        pygame.draw.circle(screen, robot_color_rgb,
                           (screen_x, screen_y), robot_screen_radius)
        pygame.draw.circle(screen, (0, 0, 0), (screen_x, screen_y),
                           robot_screen_radius, ROBOT_OUTLINE_WIDTH_PX)  # 輪郭

        # 前面インジケータ
        front_indicator_len_screen = self.radius_m * \
            0.8 * simulator.current_pixels_per_meter

        # 元の前面インジケータ描画ロジック (angle_rad はここでY成分に対してスクリーン角度として扱われる)
        # self.angle_rad はワールド座標系 (Y軸上向き、反時計回り)。スクリーンY軸は下向き。
        # 元のコード: screen_y + L * sin(angle_rad) -> angle_radがワールド角の場合、sinが正だと「間違った」方向を指す。
        # これは元のコードの不整合箇所であり、そのまま保持する。
        front_x_indicator = screen_x + \
            front_indicator_len_screen * math.cos(self.angle_rad)
        front_y_indicator = screen_y + \
            front_indicator_len_screen * math.sin(self.angle_rad)
        pygame.draw.line(screen, COLOR_ROBOT_FRONT, (screen_x, screen_y), (int(
            front_x_indicator), int(front_y_indicator)), 3)

        # デバッグ用移動ベクトル描画
        if simulator.show_debug_vectors and self.current_command and not self.current_command.get("stop", False) and self.debug_move_speed_mps > 0:
            vec_len_world = 0.3 * \
                (self.debug_move_speed_mps / ROBOT_MAX_SPEED_MPS)  # 速度に応じて長さを変える

            # debug_effective_move_angle_global_rad はワールド角度 (反時計回り、Y軸上向き)
            vec_end_x_world = self.x_m + vec_len_world * \
                math.cos(self.debug_effective_move_angle_global_rad)
            vec_end_y_world = self.y_m + vec_len_world * - \
                math.sin(
                    self.debug_effective_move_angle_global_rad)  # ワールド座標系ではY軸が上

            vec_end_screen_x, vec_end_screen_y = simulator.world_to_screen_pos(
                vec_end_x_world, vec_end_y_world)
            pygame.draw.line(screen, COLOR_DEBUG_VECTOR, (screen_x,
                             screen_y), (vec_end_screen_x, vec_end_screen_y), 2)
