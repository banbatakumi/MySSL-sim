import pygame
import math

import config
import sim_params as params  # シミュレーションのパラメータをインポート

# 循環インポートを避けるための前方型ヒント
from typing import TYPE_CHECKING
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
        # ドリブルしているロボット (None の場合はドリブルされていない)
        self.is_dribbled_by: 'SimulatedRobot | None' = None

    def update_physics(self, dt):
        """ボールの物理演算を更新する"""
        if self.is_dribbled_by:
            robot = self.is_dribbled_by
            # ロボットの角度はワールド+X軸からCWが正、ワールドY軸は上向き
            dribbler_offset_m = robot.radius_m - self.radius_m * 0.2  # ドリブラー位置オフセット

            target_x = robot.x_m + dribbler_offset_m * \
                math.cos(robot.angle_rad)
            # ワールドY軸上向き、ロボット角度CWのため、sinの符号を反転
            target_y = robot.y_m + dribbler_offset_m * - \
                math.sin(robot.angle_rad)

            # P制御的に目標位置に近づける
            self.vx_mps = (target_x - self.x_m) * params.DRIBBLE_PULL_FACTOR
            self.vy_mps = (target_y - self.y_m) * params.DRIBBLE_PULL_FACTOR

            # 速度上限 (ロボットの最大速度程度に抑える)
            current_dribble_speed = math.hypot(self.vx_mps, self.vy_mps)
            max_dribble_speed = params.ROBOT_MAX_SPEED_MPS * 1.5  # 少し余裕を持たせる
            if current_dribble_speed > max_dribble_speed and current_dribble_speed > 0:
                scale = max_dribble_speed / current_dribble_speed
                self.vx_mps *= scale
                self.vy_mps *= scale

            self.x_m += self.vx_mps * dt
            self.y_m += self.vy_mps * dt
        else:
            # ドリブルされていない場合、摩擦による減速
            current_speed_mps = math.hypot(self.vx_mps, self.vy_mps)
            if current_speed_mps > 0:
                friction_deceleration = params.BALL_FRICTION_COEFF * \
                    params.GRAVITY_MPSS  # 摩擦による減速度

                # 速度が非常に小さい場合は停止
                if current_speed_mps <= friction_deceleration * dt:
                    self.vx_mps = 0.0
                    self.vy_mps = 0.0
                else:
                    # 速度方向に摩擦を適用
                    self.vx_mps -= (self.vx_mps / current_speed_mps) * \
                        friction_deceleration * dt
                    self.vy_mps -= (self.vy_mps / current_speed_mps) * \
                        friction_deceleration * dt

            self.x_m += self.vx_mps * dt
            self.y_m += self.vy_mps * dt

        # --- 壁との衝突処理 ---
        # 壁の内側の境界 (ボールの中心がこれを超えると衝突)
        wall_inner_boundary_x_positive = config.COURT_WIDTH_M / \
            2.0 + params.WALL_OFFSET_M - self.radius_m
        wall_inner_boundary_x_negative = -config.COURT_WIDTH_M / \
            2.0 - params.WALL_OFFSET_M + self.radius_m
        wall_inner_boundary_y_positive = config.COURT_HEIGHT_M / \
            2.0 + params.WALL_OFFSET_M - self.radius_m
        wall_inner_boundary_y_negative = -config.COURT_HEIGHT_M / \
            2.0 - params.WALL_OFFSET_M + self.radius_m

        # X軸方向の壁との衝突
        if self.x_m > wall_inner_boundary_x_positive:
            self.x_m = wall_inner_boundary_x_positive  # 位置補正
            self.vx_mps *= -params.BALL_WALL_RESTITUTION_COEFF  # 反発
        elif self.x_m < wall_inner_boundary_x_negative:
            self.x_m = wall_inner_boundary_x_negative  # 位置補正
            self.vx_mps *= -params.BALL_WALL_RESTITUTION_COEFF  # 反発

        # Y軸方向の壁との衝突
        if self.y_m > wall_inner_boundary_y_positive:
            self.y_m = wall_inner_boundary_y_positive  # 位置補正
            self.vy_mps *= -params.BALL_WALL_RESTITUTION_COEFF  # 反発
        elif self.y_m < wall_inner_boundary_y_negative:
            self.y_m = wall_inner_boundary_y_negative  # 位置補正
            self.vy_mps *= -params.BALL_WALL_RESTITUTION_COEFF  # 反発

    def kick(self, robot: 'SimulatedRobot', power):
        """ロボットによってボールがキックされる"""
        if self.is_dribbled_by == robot:  # ドリブル中のロボットがキックした場合
            self.is_dribbled_by = None  # ドリブル状態を解除

        kick_speed_boost_mps = power * params.KICK_POWER_TO_SPEED_MPS  # キックによる速度増加
        kick_angle_rad = robot.angle_rad  # キック方向はロボットの向き (ワールド+X軸からCW)

        self.vx_mps = robot.vx_mps + \
            kick_speed_boost_mps * math.cos(kick_angle_rad)
        # ワールドY軸上向き、キック角度CWのため、sinの符号を反転
        self.vy_mps = robot.vy_mps + kick_speed_boost_mps * - \
            math.sin(kick_angle_rad)

    def start_dribble(self, robot: 'SimulatedRobot'):
        """ロボットによるドリブルを開始する"""
        if self.is_dribbled_by is None or self.is_dribbled_by != robot:
            self.is_dribbled_by = robot
            # オプション: ドリブル開始時にボールをロボットの前面に即座に引き寄せる
            # dribbler_offset_m = robot.radius_m - self.radius_m * 0.2
            # self.x_m = robot.x_m + dribbler_offset_m * math.cos(robot.angle_rad)
            # self.y_m = robot.y_m + dribbler_offset_m * -math.sin(robot.angle_rad)
            # self.vx_mps = robot.vx_mps # 初速をロボットに合わせる
            # self.vy_mps = robot.vy_mps

    def stop_dribble(self):
        """ドリブルを停止する"""
        if self.is_dribbled_by:
            # ドリブル停止時、ボールにロボットの現在の速度を与える (オプション)
            # self.vx_mps = self.is_dribbled_by.vx_mps
            # self.vy_mps = self.is_dribbled_by.vy_mps
            self.is_dribbled_by = None

    def get_vision_data(self):
        """Visionシステム用のボールデータを取得する"""
        return {
            "pos": (round(self.x_m, 3), round(self.y_m, 3))  # m単位
        }

    def draw(self, screen: pygame.Surface, simulator: 'Simulator'):
        """ボールを描画する"""
        screen_x, screen_y = simulator.world_to_screen_pos(self.x_m, self.y_m)
        ball_screen_radius = max(
            1, int(self.radius_m * simulator.current_pixels_per_meter))
        pygame.draw.circle(screen, config.COLOR_BALL,
                           (screen_x, screen_y), ball_screen_radius)
