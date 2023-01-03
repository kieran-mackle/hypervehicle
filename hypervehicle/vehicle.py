from typing import List, Tuple
from hypervehicle.components.component import Component
from hypervehicle.components.constants import (
    FIN_COMPONENT,
    WING_COMPONENT,
    FUSELAGE_COMPONENT,
)


class Vehicle:
    ALLOWABLE_COMPONENTS = [FIN_COMPONENT, WING_COMPONENT, FUSELAGE_COMPONENT]

    def __init__(self, **kwargs) -> None:
        # Vehicle components
        self.components: List[Component] = []

        # TODO - tidy below
        self.name = "vehicle"
        self.vehicle_angle_offset: float = 0
        self.verbosity = 1

        # Internal attributes
        self._generated = False
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

    def add_component(
        self,
        component: Component,
        reflection_axis: str = None,
        append_reflection: bool = True,
    ) -> None:
        """Adds a new component to the vehicle.

        Parameters
        ----------
        reflection_axis : str, optional
            Include a reflection of the component about the axis specified
            (eg. 'x', 'y' or 'z'). The default is None.
        append_reflection : bool, optional
            When reflecting a new component, add the reflection to the existing
            component, rather than making it a new component. The default is True.
        """
        if component.componenttype in Vehicle.ALLOWABLE_COMPONENTS:
            # Overload component verbosity
            if self.verbosity == 0:
                # Zero verbosity
                component.verbosity = 0
            else:
                # Use max of vehicle and component verbosity
                component.verbosity = max(component.verbosity, self.verbosity)

            # Add component reflections
            if reflection_axis is not None:
                component._reflection_axis = reflection_axis
                component._append_reflection = append_reflection

            # Add component
            self.components.append(component)

            # Add component count
            self._component_counts[component.componenttype] = (
                self._component_counts.get(component.componenttype, 0) + 1
            )

            if self.verbosity > 1:
                print(f"Added new {component.componenttype} component.")

        else:
            raise Exception(f"Unrecognised component type: {component.componenttype}")

    def generate(self):
        """Generate all components of the vehicle."""
        if self.verbosity > 0:
            print("Generating component patches.")

        for component in self.components:
            if self.verbosity > 1:
                print(f"  Generating patches for {component.componenttype} component.")

            # Generate component patches
            component.generate_patches()

            # Add curvature
            component.curve()

            # Add offset angle to correct curve-induced AoA
            component.rotate(angle=self.vehicle_angle_offset)

            # Reflect
            component.reflect()

        if self.verbosity > 0:
            print("All component patches generated.")

        # Set generated boolean to True
        self._generated = True

    def transform(self, transformations: List[Tuple[int, str]]):
        # TODO - specify transform type (eg. rotate) in the Tuple
        if not self._generated:
            raise Exception("Vehicle has not been generated yet.")

        # Rotate to frame
        for component in self.components:
            for transform in transformations:
                component.rotate(angle=transform[0], axis=transform[1])

            # Reset any meshes generated from un-transformed patches
            component.surfaces = None
            component.mesh = None

    def to_stl(self, prefix: str = None):
        """Writes the vehicle components to STL file."""
        prefix = self.name if prefix is None else prefix

        if self.verbosity > 0:
            print(f"Writing vehicle components to STL, with prefix {prefix}.")

        types_generated = {}
        for component in self.components:
            # Get component count
            no = types_generated.get(component.componenttype, 0)

            # Write component to stl
            component.to_stl(f"{prefix}-{component.componenttype}-{no}.stl")

            # Update component count
            types_generated[component.componenttype] = no + 1

        if self.verbosity > 0:
            print("All components written to STL file format.")

    def show(self):
        """Plots the vehicle"""
        # TODO - check if generate() has been run yet.
        raise NotImplementedError("This method has not been implemented yet.")

    def analyse(self):
        """Evaluates the mesh properties."""
        # TODO - check if generate() has been run yet.
        raise NotImplementedError("This method has not been implemented yet.")
