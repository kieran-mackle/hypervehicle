from abc import abstractmethod
from hypervehicle import Vehicle


class AbstractGenerator:
    """Abstract Generator Interface."""

    @abstractmethod
    def __init__(self, **kwargs) -> None:
        pass

    @abstractmethod
    def create_instance(self) -> Vehicle:
        pass


class Generator(AbstractGenerator):
    """Hypervehicle Parametric Generator."""

    def __init__(self, **kwargs) -> None:
        """Initialises the generator."""
        # Unpack kwargs and overwrite parameter named attributes
        for item in kwargs:
            setattr(self, item, kwargs[item])
