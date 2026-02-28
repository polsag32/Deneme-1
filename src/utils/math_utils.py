"""Mathematical utilities for IMU processing."""

import numpy as np


def accel_to_roll_pitch(accel_x: float, accel_y: float, accel_z: float) -> tuple[float, float]:
    """
    İvmeölçer verisinden roll ve pitch açılarını hesaplar (radyan).

    Args:
        accel_x: X ekseni ivme (m/s²)
        accel_y: Y ekseni ivme (m/s²)
        accel_z: Z ekseni ivme (m/s²)

    Returns:
        (roll, pitch) tuple'ı radyan cinsinden
    """
    roll = np.arctan2(accel_y, np.sqrt(accel_x**2 + accel_z**2))
    pitch = np.arctan2(-accel_x, np.sqrt(accel_y**2 + accel_z**2))
    return float(roll), float(pitch)
