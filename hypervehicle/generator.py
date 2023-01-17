from hypervehicle import Vehicle
from abc import ABC, abstractmethod


class AbstractGenerator(ABC):
    @abstractmethod
    def __init__(self, **kwargs) -> None:
        pass

    @abstractmethod
    def create_instance(self) -> Vehicle:
        pass


class Generator(AbstractGenerator):
    def __init__(self, **kwargs) -> None:
        # Unpack kwargs and overwrite parameter named attributes
        for item in kwargs:
            setattr(self, item, kwargs[item])
