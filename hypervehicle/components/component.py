from abc import ABC, abstractmethod


class AbstractComponent(ABC):
    componenttype = None

    @abstractmethod
    def __init__(self, params: dict, verbosity: int = 1) -> None:
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

    # TODO - add all the STL methods from Vehicle here


class Component(AbstractComponent):
    def __init__(self, params: dict, verbosity: int = 1) -> None:
        # Set verbosity
        self.verbosity = verbosity

        # Save parameters
        self.params = params

        # Processed objects
        self.patches = {}  # Parametric patches
        self.grids = {}  # Structured grids
        self.surfaces = {}  # STL surfaces
        self.stl_data = {}  # STL data
        self.meshes = {}  # STL meshes

    def __repr__(self):
        return f"{self.componenttype} component"

    def __str__(self):
        return f"{self.componenttype} component"

    # TODO - implement all the STL methods from Vehicle here
