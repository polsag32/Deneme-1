"""Kalman filter variants for IMU data."""

import numpy as np
from src.processing.base import BaseEstimator, Measurement
from src.utils.math_utils import accel_to_roll_pitch


class KalmanFilter(BaseEstimator):
    """
    IMU verisi için lineer Kalman filtresi.
    İki mod: ham accel (Mod 1) veya roll/pitch/yaw (Mod 2).
    """

    def __init__(
        self,
        Q: float = 0.01,
        R: float = 0.1,
        P0: float = 1.0,
        use_attitude: bool = False,
    ):
        """
        Args:
            Q: Süreç gürültü kovaryansı (skaler -> 6x6 köşegen)
            R: Ölçüm gürültü kovaryansı (skaler -> 6x6 köşegen)
            P0: Başlangıç durum kovaryansı (skaler -> 6x6 köşegen)
            use_attitude: True ise roll/pitch/yaw, False ise ham accel kullanılır
        """
        self.Q = Q
        self.R = R
        self.P0 = P0
        self.use_attitude = use_attitude
        self.x: np.ndarray | None = None
        self.P: np.ndarray | None = None
        self.initialized = False

    def _build_z(self, measurement: Measurement) -> np.ndarray:
        """Ölçüm vektörü z'yi oluşturur."""
        gyro = np.array(measurement.data["gyro"], dtype=float)
        if self.use_attitude:
            roll = measurement.data.get("roll")
            pitch = measurement.data.get("pitch")
            yaw = measurement.data.get("yaw", 0.0)
            if roll is None or pitch is None:
                accel = measurement.data["accel"]
                roll, pitch = accel_to_roll_pitch(accel[0], accel[1], accel[2])
            z = np.concatenate([gyro, np.array([roll, pitch, yaw])])
        else:
            accel = np.array(measurement.data["accel"], dtype=float)
            z = np.concatenate([gyro, accel])
        return z

    def update(self, measurement: Measurement, dt: float) -> dict:
        """
        Kalman filtresi güncelleme adımı.

        Args:
            measurement: IMU ölçümü (gyro, accel veya gyro, roll, pitch, yaw)
            dt: Zaman adımı (saniye)

        Returns:
            state, covariance, gyro_estimated, accel_estimated (Mod 1)
            veya roll, pitch, yaw, euler (Mod 2)
        """
        z = self._build_z(measurement)
        F = np.eye(6)
        H = np.eye(6)
        Q_mat = self.Q * np.eye(6)
        R_mat = self.R * np.eye(6)

        if not self.initialized:
            self.x = z.copy()
            self.P = self.P0 * np.eye(6)
            self.initialized = True

        # Tahmin (Prediction)
        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + Q_mat

        # Yenilik (Innovation)
        y = z - H @ x_pred

        # Yenilik kovaryansı ve Kalman kazancı
        S = H @ P_pred @ H.T + R_mat + 1e-6 * np.eye(6)
        K = P_pred @ H.T @ np.linalg.inv(S)

        # Güncelleme (Update)
        self.x = x_pred + K @ y
        self.P = (np.eye(6) - K @ H) @ P_pred

        # Çıktı oluştur
        result: dict = {
            "state": self.x.tolist(),
            "covariance": self.P.tolist(),
            "gyro_estimated": self.x[:3].tolist(),
        }
        if self.use_attitude:
            result["roll"] = float(self.x[3])
            result["pitch"] = float(self.x[4])
            result["yaw"] = float(self.x[5])
            result["euler"] = [result["roll"], result["pitch"], result["yaw"]]
        else:
            result["accel_estimated"] = self.x[3:6].tolist()

        return result
