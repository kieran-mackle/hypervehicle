from art import tprint, art
from typing import List, Tuple, Callable, Dict, Any
from hypervehicle.components.component import Component
from hypervehicle.components.constants import (
    FIN_COMPONENT,
    WING_COMPONENT,
    FUSELAGE_COMPONENT,
    COMPOSITE_COMPONENT,
)


class Vehicle:
    ALLOWABLE_COMPONENTS = [
        FIN_COMPONENT,
        WING_COMPONENT,
        FUSELAGE_COMPONENT,
        COMPOSITE_COMPONENT,
    ]

    def __init__(self, **kwargs) -> None:
        # Vehicle attributes
        self.components: List[Component] = []
        self.name = "vehicle"
        self.vehicle_angle_offset: float = 0
        self.verbosity = 1

        # Internal attributes
        self._generated = False
        self._component_counts = {}
        self._enumerated_components = {}
        self._named_components = {}
        self._vehicle_transformations = None

    def __repr__(self):
        basestr = self.__str__()
        if len(self.components) > 0:
            compstr = ", ".join(
                [f"{e[1]} {e[0]}" for e in self._component_counts.items()]
            )
        else:
            compstr = "no components"
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
        name: str = None,
        reflection_axis: str = None,
        append_reflection: bool = True,
        curvatures: List[Tuple[str, Callable, Callable]] = None,
        clustering: Dict[str, float] = None,
        transformations: List[Tuple[str, Any]] = None,
    ) -> None:
        """Adds a new component to the vehicle.

        Parameters
        ----------
        component : Component
            The component to add.
        name : str, optional
            The name to assign to this component. If provided, it will be used when
            writing to STL. The default is None.
        reflection_axis : str, optional
            Include a reflection of the component about the axis specified
            (eg. 'x', 'y' or 'z'). The default is None.
        append_reflection : bool, optional
            When reflecting a new component, add the reflection to the existing
            component, rather than making it a new component. The default is True.
        curvatures : List[Tuple[str, Callable, Callable]], optional
            A list of the curvatures to apply to the component being added.
            This list contains a tuple for each curvature. Each curvatue
            is defined by (axis, curve_func, curve_func_derivative).
            The default is None.
        clustering : Dict[str, float], optional
            Optionally provide clustering options for the stl meshes. The
            default is None.
        transformations : List[Tuple[str, Any]], optional
            A list of transformations to apply to the nominal component. The
            default is None
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

            # Add component curvature functions
            if curvatures is not None:
                component._curvatures = curvatures

            # Add component clustering
            if clustering is not None:
                component._clustering = clustering

            # Add transformations
            if transformations is not None:
                component._transformations = transformations

            # Add component
            self.components.append(component)

            # Add component count
            component_count = self._component_counts.get(component.componenttype, 0) + 1
            self._component_counts[component.componenttype] = component_count
            self._enumerated_components[
                f"{component.componenttype}_{component_count}"
            ] = component

            # Process component name
            if name is not None:
                # Assign component name
                component.name = name
            else:
                name = f"{component.componenttype}_{component_count}"
            self._named_components[name] = component

            if self.verbosity > 1:
                print(f"Added new {component.componenttype} component.")

        else:
            raise Exception(f"Unrecognised component type: {component.componenttype}")

    def generate(self):
        """Generate all components of the vehicle."""
        if self.verbosity > 0:
            tprint("Hypervehicle", "tarty4")
            p = art("airplane2")
            print(f" {p}               {p}               {p}               {p}")
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

            # Apply transformations
            component.transform()

        # Set generated boolean to True
        self._generated = True

        # Apply any vehicle transformations
        if self._vehicle_transformations:
            self.transform(self._vehicle_transformations)

        if self.verbosity > 0:
            print("All component patches generated.")

    def add_vehicle_transformations(self, transformations: List[Tuple[str, float]]):
        """Add transformations to apply to the vehicle after running generate()."""
        self._vehicle_transformations = transformations

    def transform(self, transformations: List[Tuple[str, float]]):
        """Transform vehicle by applying the tranformations. Currently
        only supports rotations.

        To rotate 180 degrees about the x axis, followed by 90 degrees
        about the y axis, transformations = [(180, "x"), (90, "y")]"""
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
        """Writes the vehicle components to STL file.

        Parameters
        ----------
        prefix : str, optional
            The prefix to use when saving components to STL. Note that if
            components have been individually assigned name tags, the prefix
            provided will take precedence. If no prefix is specified, and no
            component name tag is available, the Vehicle name will be used.
            The default is None.
        """
        if self.verbosity > 0:
            s = "Writing vehicle components to STL"
            if prefix:
                print(f"{s}, with prefix '{prefix}'.")
            else:
                print(f"{s}.")

        types_generated = {}
        for component in self.components:
            # Get component count
            no = types_generated.get(component.componenttype, 0)

            # Write component to stl
            if prefix:
                # Use prefix provided
                stl_name = f"{prefix}-{component.componenttype}-{no}.stl"
            elif component.name:
                stl_name = f"{component.name}.stl"
            else:
                # No prefix or component name, use vehicle name as fallback
                stl_name = f"{self.name}-{component.componenttype}-{no}.stl"

            if self.verbosity > 0:
                print(f"\r  Writing: {stl_name}", end="")

            component.to_stl(stl_name)

            # Update component count
            types_generated[component.componenttype] = no + 1

        if self.verbosity > 0:
            print("\rAll components written to STL file format.", end="\n")

    def analyse(self, densities: dict):
        """Evaluates the mesh properties."""
        from hypervehicle.utilities import assess_inertial_properties

        self.volume, self.mass, self.cog, self.inertia = assess_inertial_properties(
            vehicle=self, component_densities=densities
        )

        return self.volume, self.mass, self.cog, self.inertia
