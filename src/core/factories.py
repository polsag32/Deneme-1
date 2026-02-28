"""Filter factory for creating filter instances by name."""

from src.processing.base import BaseEstimator
from src.processing.kalman_variants import KalmanFilter

_registry: dict[str, type] = {
    "Kalman": KalmanFilter,
}


class FilterFactory:
    """Filtre oluşturucu factory sınıfı."""

    @classmethod
    def create(cls, name: str, **kwargs) -> BaseEstimator:
        """
        İsimle filtre örneği oluşturur.

        Args:
            name: Filtre adı (örn. "Kalman")
            **kwargs: Filtre parametreleri (Q, R, P0, use_attitude vb.)

        Returns:
            Filtre örneği

        Raises:
            KeyError: Bilinmeyen filtre adı
        """
        if name not in _registry:
            raise KeyError(f"Bilinmeyen filtre: {name}. Mevcut: {list(_registry.keys())}")
        return _registry[name](**kwargs)

    @classmethod
    def register(cls, name: str, filter_class: type) -> None:
        """Yeni filtre sınıfı kaydeder."""
        _registry[name] = filter_class

    @classmethod
    def available(cls) -> list[str]:
        """Kayıtlı filtre adlarını döndürür."""
        return list(_registry.keys())
