from abc import ABC, abstractmethod


class AbstractComponent(ABC):
    componenttype = None

    @abstractmethod
    def __init__(self) -> None:
        pass

    @abstractmethod
    def __repr__(self):
        pass

    @abstractmethod
    def __str__(self):
        pass

    @property
    @abstractmethod
    def componenttype(self):
        # This is a placeholder for a class variable defining the component type
        pass

    @abstractmethod
    def generate_patches(self):
        pass


class Component(AbstractComponent):
    def __repr__(self):
        return f"{self.componenttype} component"

    def __str__(self):
        return f"{self.componenttype} component"
