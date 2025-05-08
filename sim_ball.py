import pygame
import math

from sim_constants import (
    COURT_WIDTH_M, COURT_HEIGHT_M, BALL_RADIUS_M,
    BALL_FRICTION_COEFF, GRAVITY_MPSS, KICK_POWER_TO_SPEED_MPS,
    DRIBBLE_PULL_FACTOR, COLOR_BALL
)

# 循環インポートを避けるための前方型ヒント
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from sim_robot import SimulatedRobot
    from simulator import Simulator  # 'simulator' パラメータの型ヒントのためのメインシミュレータクラス


class SimulatedBall:
    def __init__(self, x_m=0.0, y_m=0.0, radius_m=BALL_RADIUS_M):
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
            overlap_depth_m = self.radius_m * 0.2  # ボールがロボットにめり込む深さ
            dribbler_offset_m = robot.radius_m - overlap_depth_m  # ドリブラーからのオフセット

            # ドリブル時の目標位置 (ロボットの前面)
            target_x = robot.x_m + dribbler_offset_m * \
                math.cos(robot.angle_rad)
            target_y = robot.y_m + dribbler_offset_m * - \
                math.sin(robot.angle_rad)  # ワールド座標系ではY軸が上

            # 目標位置への追従速度
            self.vx_mps = (target_x - self.x_m) * DRIBBLE_PULL_FACTOR
            self.vy_mps = (target_y - self.y_m) * DRIBBLE_PULL_FACTOR

            self.x_m += self.vx_mps * dt
            self.y_m += self.vy_mps * dt
        else:
            # ドリブルされていない場合、摩擦による減速
            current_speed_mps = math.hypot(self.vx_mps, self.vy_mps)
            if current_speed_mps > 0:
                friction_decel = BALL_FRICTION_COEFF * GRAVITY_MPSS  # 摩擦による減速度
                if current_speed_mps < friction_decel * dt:
                    # 速度が非常に小さい場合は停止
                    self.vx_mps = 0.0
                    self.vy_mps = 0.0
                else:
                    # 速度方向に摩擦を適用
                    self.vx_mps -= (self.vx_mps /
                                    current_speed_mps) * friction_decel * dt
                    self.vy_mps -= (self.vy_mps /
                                    current_speed_mps) * friction_decel * dt

            self.x_m += self.vx_mps * dt
            self.y_m += self.vy_mps * dt

        # コートの境界との衝突処理
        half_court_width = COURT_WIDTH_M / 2.0
        half_court_height = COURT_HEIGHT_M / 2.0

        if self.x_m - self.radius_m < -half_court_width:
            self.x_m = -half_court_width + self.radius_m
            self.vx_mps *= -0.7  # 反発係数
        elif self.x_m + self.radius_m > half_court_width:
            self.x_m = half_court_width - self.radius_m
            self.vx_mps *= -0.7

        if self.y_m - self.radius_m < -half_court_height:
            self.y_m = -half_court_height + self.radius_m
            self.vy_mps *= -0.7
        elif self.y_m + self.radius_m > half_court_height:
            self.y_m = half_court_height - self.radius_m
            self.vy_mps *= -0.7

    def kick(self, robot: 'SimulatedRobot', power):
        """ロボットによってボールがキックされる"""
        self.is_dribbled_by = None  # ドリブル状態を解除
        kick_speed_boost_mps = power * KICK_POWER_TO_SPEED_MPS  # キックによる速度増加
        kick_angle_rad = robot.angle_rad  # キック方向はロボットの向き

        # キック後のボール速度 (ロボットの現在の速度も考慮)
        self.vx_mps = robot.vx_mps + \
            kick_speed_boost_mps * math.cos(kick_angle_rad)
        self.vy_mps = robot.vy_mps + \
            kick_speed_boost_mps * -math.sin(kick_angle_rad)  # ワールド座標系ではY軸が上

    def start_dribble(self, robot: 'SimulatedRobot'):
        """ロボットによるドリブルを開始する"""
        self.is_dribbled_by = robot
        overlap_depth_m = self.radius_m * 0.2
        dribbler_offset_m = robot.radius_m - overlap_depth_m
        # ボールをロボットの前面に吸い付ける
        self.x_m = robot.x_m + dribbler_offset_m * math.cos(robot.angle_rad)
        self.y_m = robot.y_m + dribbler_offset_m * - \
            math.sin(robot.angle_rad)  # ワールド座標系ではY軸が上
        self.vx_mps = robot.vx_mps  # ロボットの速度に合わせる
        self.vy_mps = robot.vy_mps

    def stop_dribble(self):
        """ドリブルを停止する"""
        if self.is_dribbled_by:
            self.is_dribbled_by = None

    def get_vision_data(self):
        """Visionシステム用のボールデータを取得する"""
        return {
            "pos": (round(self.x_m * 100.0, 2), round(self.y_m * 100.0, 2))  # cm単位
        }

    def draw(self, screen: pygame.Surface, simulator: 'Simulator'):
        """ボールを描画する"""
        screen_x, screen_y = simulator.world_to_screen_pos(self.x_m, self.y_m)
        pygame.draw.circle(screen, COLOR_BALL, (screen_x, screen_y), int(
            self.radius_m * simulator.current_pixels_per_meter))
