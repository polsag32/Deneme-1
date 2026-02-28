"""Base classes for processing and estimation."""

from abc import ABC, abstractmethod
from typing import Any


class Measurement:
    """Ölçüm verisi taşıyıcı sınıfı."""

    def __init__(self, data: dict[str, Any], meta: dict[str, Any] | None = None):
        """
        Args:
            data: Ölçüm verisi (gyro, accel, roll, pitch, yaw vb.)
            meta: Meta bilgiler (timestamp, row_index, imu vb.)
        """
        self.data = data
        self.meta = meta or {}


class BaseEstimator(ABC):
    """Tüm filtrelerin temel soyut sınıfı."""

    @abstractmethod
    def update(self, measurement: Measurement, dt: float) -> dict[str, Any]:
        """
        Ölçümü işler ve filtre çıktısını döndürür.

        Args:
            measurement: Gelen ölçüm
            dt: Zaman adımı (saniye)

        Returns:
            Filtre çıktısı (state, covariance, vb.)
        """
        pass
