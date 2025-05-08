import math


def normalize_angle_rad(angle_rad):
    """角度を -π から π の範囲に正規化する"""
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad
