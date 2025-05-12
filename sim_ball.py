import pygame
import math

import config
import sim_params as params  # シミュレーションのパラメータをインポート

# 循環インポートを避けるための前方型ヒント
from typing import TYPE_CHECKING, List
if TYPE_CHECKING:
    from sim_robot import SimulatedRobot
    from simulator import Simulator


class SimulatedBall:
    def __init__(self, x_m=0.0, y_m=0.0, radius_m=params.BALL_RADIUS_M):
        self.x_m = x_m  # X座標 (メートル)
        self.y_m = y_m  # Y座標 (メートル)
        self.vx_mps = 0.0  # X方向の速度 (メートル/秒)
        self.vy_mps = 0.0  # Y方向の速度 (メートル/秒)
        self.radius_m = radius_m  # 半径 (メートル)
        self.is_dribbled_by: 'SimulatedRobot | None' = None  # ドリブルしているロボット
        self.just_started_dribble_by: 'SimulatedRobot | None' = None  # ドリブル開始直後フラグ

    def _apply_dribble_physics(self, dt: float):
        """ドリブル中のボールの物理演算"""
        if not self.is_dribbled_by:
            return

        robot = self.is_dribbled_by
        dribbler_offset_m = robot.radius_m - self.radius_m * 0.2

        target_x = robot.x_m + dribbler_offset_m * \
            math.cos(robot.angle_rad)
        target_y = robot.y_m + dribbler_offset_m * - \
            math.sin(robot.angle_rad)

        self.vx_mps = (target_x - self.x_m) * params.DRIBBLE_PULL_FACTOR
        self.vy_mps = (target_y - self.y_m) * params.DRIBBLE_PULL_FACTOR

        current_dribble_speed = math.hypot(self.vx_mps, self.vy_mps)
        max_dribble_speed = params.ROBOT_MAX_SPEED_MPS * 1.5
        if current_dribble_speed > max_dribble_speed and current_dribble_speed > 0:
            scale = max_dribble_speed / current_dribble_speed
            self.vx_mps *= scale
            self.vy_mps *= scale

        self.x_m += self.vx_mps * dt
        self.y_m += self.vy_mps * dt

    def _apply_free_ball_physics(self, dt: float):
        """ドリブルされていない自由なボールの物理演算（摩擦と運動）"""
        current_speed_mps = math.hypot(self.vx_mps, self.vy_mps)
        if current_speed_mps > 0:
            friction_deceleration = params.BALL_FRICTION_COEFF * \
                params.GRAVITY_MPSS

            if current_speed_mps <= friction_deceleration * dt:
                self.vx_mps = 0.0
                self.vy_mps = 0.0
            else:
                self.vx_mps -= (self.vx_mps / current_speed_mps) * \
                    friction_deceleration * dt
                self.vy_mps -= (self.vy_mps / current_speed_mps) * \
                    friction_deceleration * dt

        self.x_m += self.vx_mps * dt
        self.y_m += self.vy_mps * dt

    def _handle_wall_collision(self, dt: float):
        """ボールと壁の衝突処理"""
        wall_inner_boundary_x_positive = config.COURT_WIDTH_M / \
            2.0 + params.WALL_OFFSET_M - self.radius_m
        wall_inner_boundary_x_negative = -config.COURT_WIDTH_M / \
            2.0 - params.WALL_OFFSET_M + self.radius_m
        wall_inner_boundary_y_positive = config.COURT_HEIGHT_M / \
            2.0 + params.WALL_OFFSET_M - self.radius_m
        wall_inner_boundary_y_negative = -config.COURT_HEIGHT_M / \
            2.0 - params.WALL_OFFSET_M + self.radius_m

        if self.x_m > wall_inner_boundary_x_positive:
            self.x_m = wall_inner_boundary_x_positive
            self.vx_mps *= -params.BALL_WALL_RESTITUTION_COEFF
        elif self.x_m < wall_inner_boundary_x_negative:
            self.x_m = wall_inner_boundary_x_negative
            self.vx_mps *= -params.BALL_WALL_RESTITUTION_COEFF

        if self.y_m > wall_inner_boundary_y_positive:
            self.y_m = wall_inner_boundary_y_positive
            self.vy_mps *= -params.BALL_WALL_RESTITUTION_COEFF
        elif self.y_m < wall_inner_boundary_y_negative:
            self.y_m = wall_inner_boundary_y_negative
            self.vy_mps *= -params.BALL_WALL_RESTITUTION_COEFF

    def _resolve_ball_robot_collision(self, robot: 'SimulatedRobot', restitution: float,
                                      nx: float, ny: float, vn_rel: float):
        dx = self.x_m - robot.x_m
        dy = self.y_m - robot.y_m
        dist = math.hypot(dx, dy)

        if dist < 1e-9:
            pass
        else:
            nx = dx / dist
            ny = dy / dist

        overlap = (self.radius_m + robot.radius_m) - dist
        if overlap > 0:
            total_mass = params.BALL_MASS_KG + robot.mass_kg
            if total_mass > 1e-9:
                move_ball = overlap * (robot.mass_kg / total_mass)
                move_robot = -overlap * (params.BALL_MASS_KG / total_mass)
                self.x_m += nx * move_ball
                self.y_m += ny * move_ball
                robot.x_m += nx * move_robot
                robot.y_m += ny * move_robot
            else:
                move_dist_each = overlap / 2.0
                self.x_m += nx * move_dist_each
                robot.x_m -= nx * move_dist_each

        m_ball = params.BALL_MASS_KG
        m_robot = robot.mass_kg

        if (m_ball <= 1e-9 or m_robot <= 1e-9 or (1/m_ball + 1/m_robot) <= 1e-9):
            impulse_j = 0
        else:
            impulse_j = -(1 + restitution) * vn_rel
            impulse_j /= (1/m_ball + 1/m_robot)

        if m_ball > 1e-9:
            self.vx_mps += (impulse_j / m_ball) * nx
            self.vy_mps += (impulse_j / m_ball) * ny
        if m_robot > 1e-9:
            robot.vx_mps -= (impulse_j / m_robot) * nx
            robot.vy_mps -= (impulse_j / m_robot) * ny

    def update_physics(self, dt: float, robots: List['SimulatedRobot']):
        collided_this_frame_with_robot = None
        dribble_was_just_started_this_frame_for_current_dribbler = False

        if self.is_dribbled_by:
            dribbling_robot = self.is_dribbled_by

            if self.just_started_dribble_by == dribbling_robot:
                dribble_was_just_started_this_frame_for_current_dribbler = True
                self.just_started_dribble_by = None
                # print(f"DEBUG: Dribble for {dribbling_robot.color_name}_{dribbling_robot.robot_id_num} was just started. Relaxing initial collision check.")

            dx_db = self.x_m - dribbling_robot.x_m
            dy_db = self.y_m - dribbling_robot.y_m
            dist_db = math.hypot(dx_db, dy_db)

            nx_db, ny_db = 0, 0
            if dist_db > 1e-9:
                nx_db = dx_db / dist_db
                ny_db = dy_db / dist_db
            else:
                nx_db = math.cos(dribbling_robot.angle_rad)
                ny_db = -math.sin(dribbling_robot.angle_rad)

            vrx_db = self.vx_mps - dribbling_robot.vx_mps
            vry_db = self.vy_mps - dribbling_robot.vy_mps
            vn_rel_db = vrx_db * nx_db + vry_db * ny_db

            if not dribble_was_just_started_this_frame_for_current_dribbler and \
               vn_rel_db < -params.ROBOT_BALL_FAST_PASS_THRESHOLD_MPS:
                # print(f"DEBUG: Dribble broken by fast impact FROM BALL to {dribbling_robot.color_name}_{dribbling_robot.robot_id_num}, vn_rel={vn_rel_db:.2f}")
                original_dribbler = self.is_dribbled_by
                self.stop_dribble()
                self._resolve_ball_robot_collision(original_dribbler,
                                                   params.BALL_ROBOT_FAST_PASS_RESTITUTION_COFF,
                                                   nx_db, ny_db, vn_rel_db)
                collided_this_frame_with_robot = original_dribbler
            else:
                self._apply_dribble_physics(dt)

        if not self.is_dribbled_by:
            self._apply_free_ball_physics(dt)

            for r in robots:
                if r == collided_this_frame_with_robot:
                    continue

                dx_br = self.x_m - r.x_m
                dy_br = self.y_m - r.y_m
                dist_br = math.hypot(dx_br, dy_br)
                combined_radii = self.radius_m + r.radius_m

                if dist_br < combined_radii and dist_br > 1e-9:
                    nx_br = dx_br / dist_br
                    ny_br = dy_br / dist_br

                    vrx_br = self.vx_mps - r.vx_mps
                    vry_br = self.vy_mps - r.vy_mps
                    vn_rel_br = vrx_br * nx_br + vry_br * ny_br

                    if vn_rel_br < 0:
                        if self.is_dribbled_by == r:  # このチェックは前回の修正の名残だが、上のif not self.is_dribbled_byブロック内なので、基本的にはここは通らないはず。ただし、start_dribbleがこのフレームで呼ばれた場合を想定
                            # print(f"DEBUG: Ball now dribbled by {r.color_name}_{r.robot_id_num}. Skipping collision. vn_rel={vn_rel_br:.2f}")
                            collided_this_frame_with_robot = r
                            continue

                        restitution_coeff: float
                        if vn_rel_br < -params.ROBOT_BALL_FAST_PASS_THRESHOLD_MPS:
                            restitution_coeff = params.BALL_ROBOT_FAST_PASS_RESTITUTION_COEFF
                            # print(f"DEBUG: Ball-Robot FAST collision with {r.color_name}_{r.robot_id_num}, vn_rel={vn_rel_br:.2f}")
                        else:
                            restitution_coeff = params.BALL_ROBOT_NORMAL_RESTITUTION_COEFF
                            # print(f"DEBUG: Ball-Robot NORMAL collision with {r.color_name}_{r.robot_id_num}, vn_rel={vn_rel_br:.2f}")

                        self._resolve_ball_robot_collision(
                            r, restitution_coeff, nx_br, ny_br, vn_rel_br)
                        collided_this_frame_with_robot = r
                        break

        self._handle_wall_collision(dt)

    def kick(self, robot: 'SimulatedRobot', power: float):
        if self.is_dribbled_by == robot:
            self.stop_dribble()

        kick_speed_boost_mps = power * params.KICK_POWER_TO_SPEED_MPS
        kick_angle_rad = robot.angle_rad

        self.vx_mps = robot.vx_mps + \
            kick_speed_boost_mps * math.cos(kick_angle_rad)
        self.vy_mps = robot.vy_mps + kick_speed_boost_mps * - \
            math.sin(kick_angle_rad)

    def start_dribble(self, robot: 'SimulatedRobot'):
        if self.is_dribbled_by is not None and self.is_dribbled_by != robot:
            self.is_dribbled_by = None

        if self.is_dribbled_by != robot:
            self.is_dribbled_by = robot
            self.just_started_dribble_by = robot  # ★ドリブル開始をマーク
            # print(f"DEBUG: Ball start dribble by {robot.color_name}_{robot.robot_id_num}, marked as just_started")

    def stop_dribble(self):
        if self.is_dribbled_by:
            # print(f"DEBUG: Ball stop dribble by {self.is_dribbled_by.color_name}_{self.is_dribbled_by.robot_id_num}")
            if self.just_started_dribble_by == self.is_dribbled_by:  # もし開始直後に停止されたらフラグもリセット
                self.just_started_dribble_by = None
            self.is_dribbled_by = None

    def get_vision_data(self):
        return {
            "pos": (round(self.x_m, 3), round(self.y_m, 3))
        }

    def draw(self, screen: pygame.Surface, simulator: 'Simulator'):
        screen_x, screen_y = simulator.world_to_screen_pos(self.x_m, self.y_m)
        ball_screen_radius = max(
            1, int(self.radius_m * simulator.current_pixels_per_meter))
        pygame.draw.circle(screen, config.COLOR_BALL,
                           (screen_x, screen_y), ball_screen_radius)
