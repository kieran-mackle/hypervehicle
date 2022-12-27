from typing import List, Tuple
from hypervehicle.components.component import Component


# TODO - what if the vehicle inherited from Component? That
# way, I could 'rotate' and 'curve' the entire vehicle?

# TODO - add verbosity


class Vehicle:
    name = "vehicle"

    def __init__(self, **kwargs) -> None:
        # Vehicle components
        self.components: List[Component] = []

        # TODO - tidy below
        self.vehicle_angle_offset: float = 0
        self.transformations = []
        self._generated = False

        # TODO - call self.configure with kwargs

    def __repr__(self):
        return f"Parameterised {self.name}"

    def configure(self):
        """Configure the Vehicle instance."""
        pass

    def add_component(self, component: Component) -> None:
        self.components.append(component)

    def generate(self):
        """Generate all components of the vehicle."""
        for component in self.components:
            # Generate component patches
            component.generate_patches()

            # Add curvature
            component.curve()

            # Add offset angle to correct curve-induced AoA
            component.rotate(angle=self.vehicle_angle_offset)

            # Reflect
            component.reflect()

        # Set generated boolean to True
        self._generated = True

    def transform(self, transformations: List[Tuple[int, str]]):
        if not self._generated:
            raise Exception("Vehicle has not been generated yet.")

        # Rotate to frame
        for component in self.components:
            for transform in self.transformations:
                component.rotate(angle=transform[0], axis=transform[1])

    def to_stl(self, prefix: str = None):
        """Writes the vehicle components to STL file."""
        # TODO - check if generate() has been run yet.
        raise NotImplementedError("This method has not been implemented yet.")

    def show(self):
        """Plots the vehicle"""
        # TODO - check if generate() has been run yet.
        raise NotImplementedError("This method has not been implemented yet.")

    def analyse(self):
        """Evaluates the mesh properties."""
        # TODO - check if generate() has been run yet.
        raise NotImplementedError("This method has not been implemented yet.")
