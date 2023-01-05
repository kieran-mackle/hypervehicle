from hypervehicle import Vehicle
from abc import ABC, abstractmethod


class AbstractGenerator(ABC):
    @abstractmethod
    def __init__(self, **kwargs) -> None:
        pass

    @abstractmethod
    def create_instance(self) -> Vehicle:
        pass
