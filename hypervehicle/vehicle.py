from typing import List, Tuple
from hypervehicle.components.component import Component
from hypervehicle.components.constants import (
    FIN_COMPONENT,
    WING_COMPONENT,
    FUSELAGE_COMPONENT,
)


# TODO - what if the vehicle inherited from Component? That
# way, I could 'rotate' and 'curve' the entire vehicle?

# TODO - add verbosity


class Vehicle:
    ALLOWABLE_COMPONENTS = [FIN_COMPONENT, WING_COMPONENT, FUSELAGE_COMPONENT]

    def __init__(self, **kwargs) -> None:
        # Vehicle components
        self.components: List[Component] = []

        # TODO - tidy below
        self.name = "vehicle"
        self.vehicle_angle_offset: float = 0
        self.transformations = []
        self._generated = False
        self.verbosity = 1
        self._component_counts = {}

        # TODO - call self.configure with kwargs

    def __repr__(self):
        basestr = self.__str__()
        compstr = ", ".join([f"{e[1]} {e[0]}" for e in self._component_counts.items()])
        return f"{basestr} ({compstr})"

    def __str__(self) -> str:
        vstr = f"Parameterised {self.name}"
        if len(self.components) > 0:
            vstr += f" with {len(self.components)} components"
        return vstr

    def configure(self, name: str = None, verbosity: int = 1):
        """Configure the Vehicle instance."""
        if name is not None:
            self.name = name

        self.verbosity = verbosity

    def add_component(self, component: Component) -> None:
        if component.componenttype in Vehicle.ALLOWABLE_COMPONENTS:
            # Add component
            self.components.append(component)

            # Add component count
            self._component_counts[component.componenttype] = (
                self._component_counts.get(component.componenttype, 0) + 1
            )

        else:
            raise Exception(f"Unrecognised component type: {component.componenttype}")

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

            # TODO - reset all generated meshes etc

    def to_stl(self, prefix: str = None):
        """Writes the vehicle components to STL file."""
        prefix = self.name if prefix is None else prefix
        types_generated = {}
        for component in self.components:
            # Get component count
            no = types_generated.get(component.componenttype, 0)

            # Write component to stl
            component.to_stl(f"{prefix}-{component.componenttype}-{no}.stl")

            # Update component count
            types_generated[component.componenttype] = no + 1

    def show(self):
        """Plots the vehicle"""
        # TODO - check if generate() has been run yet.
        raise NotImplementedError("This method has not been implemented yet.")

    def analyse(self):
        """Evaluates the mesh properties."""
        # TODO - check if generate() has been run yet.
        raise NotImplementedError("This method has not been implemented yet.")
