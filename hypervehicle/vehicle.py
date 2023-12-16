import os
import pandas as pd
from art import tprint, art
from typing import List, Tuple, Callable, Dict, Any
from hypervehicle.components.component import Component
from hypervehicle.components.constants import (
    FIN_COMPONENT,
    WING_COMPONENT,
    COMPOSITE_COMPONENT,
    SWEPT_COMPONENT,
    REVOLVED_COMPONENT,
)


class Vehicle:
    ALLOWABLE_COMPONENTS = [
        FIN_COMPONENT,
        WING_COMPONENT,
        SWEPT_COMPONENT,
        REVOLVED_COMPONENT,
        COMPOSITE_COMPONENT,
    ]

    def __init__(self, **kwargs) -> None:
        # Vehicle attributes
        self.components: List[Component] = []
        self.name = "vehicle"
        self.vehicle_angle_offset: float = 0
        self.verbosity = 1
        self.analysis_results = None

        # Internal attributes
        self._generated = False
        self._component_counts = {}
        self._enumerated_components = {}
        self._named_components = {}
        self._vehicle_transformations = []
        self._analyse_on_generation = None

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
            default is None.
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
            component_count = self._component_counts.get(
                component.componenttype, 0) + 1
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
            raise Exception(
                f"Unrecognised component type: {component.componenttype}")

    def generate(self):
        """Generate all components of the vehicle."""
        if self.verbosity > 0:
            tprint("Hypervehicle", "tarty4")
            p = art("airplane2")
            print(f" {p}               {p}               {p}               {p}")
            print("Generating component patches.")

        for component in self.components:
            if self.verbosity > 1:
                print(
                    f"  Generating patches for {component.componenttype} component.")

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

        # Run analysis
        if self._analyse_on_generation:
            analysis_results = self.analyse(self._analyse_on_generation)
            self.analysis_results = dict(
                zip(("volume", "mass", "cog", "moi"), analysis_results)
            )

        if self.verbosity > 0:
            print("All component patches generated.")

    def add_vehicle_transformations(
        self, transformations: List[Tuple[str, Any]]
    ) -> None:
        """Add transformations to apply to the vehicle after running generate().
        Each transformation in the list should be of the form (type, *args), where
        type can be "rotate" or "translate". The *args for rotate are angle: float
        and axis: str. The *args for translate are offset: Union[Callable, Vector3].
        """
        # Check input
        if isinstance(transformations, tuple):
            # Coerce into list
            transformations = [transformations]
        self._vehicle_transformations += transformations

    def analyse_after_generating(self, densities: Dict[str, Any]) -> None:
        """Run the vehicle analysis method immediately after generating
        patches. Results will be saved to the analysis_results attribute of
        the vehicle.

        Parameters
        ----------
        densities : Dict[str, Any]
            A dictionary containing the effective densities for each component.
            Note that the keys of the dict must match the keys of
            vehicle._named_components. These keys will be consistent with any
            name tags assigned to components.
        """
        self._analyse_on_generation = densities

    def transform(self, transformations: List[Tuple[str, Any]]) -> None:
        """Transform vehicle by applying the tranformations. Currently
        only supports rotations.

        To rotate 180 degrees about the x axis, followed by 90 degrees
        about the y axis, transformations = [("rotate", 180, "x"),
        ("rotate", 90, "y")].
        """
        if not self._generated:
            raise Exception("Vehicle has not been generated yet.")

        # Check input
        if isinstance(transformations, tuple):
            # Coerce into list
            transformations = [transformations]

        # Apply transformations
        for component in self.components:
            for transform in transformations:
                func = getattr(component, transform[0])
                func(*transform[1:])

            # Reset any meshes generated from un-transformed patches
            component.surfaces = None
            component.mesh = None

    def to_file(self, prefix: str = None, file_type: str = 'stl') -> None:
        """Writes the vehicle components to output file. If analysis results are
        present, they will also be written to file, either as CSV, or using
        the Numpy tofile method.
        The output file format is defined in 'file_type' and can be stl or vtk

        Parameters
        ----------
        prefix : str, optional
            The prefix to use when saving components to output file. Note that
            if components have been individually assigned name tags, the prefix
            provided will take precedence. If no prefix is specified, and no
            component name tag is available, the Vehicle name will be used.
            The default is None.

        file_type: str
            Defines the output file format to be written to. Can be stl or 
            vtk. Default file_type is 'stl'
        """
        file_type = file_type.lower()
        if file_type not in ['stl', 'vtk']:
            raise ('Invalid output file type. STL or VTK supported')

        if self.verbosity > 0:
            s = "Writing vehicle components to"
            if prefix:
                print(f"{s} {file_type.upper()}, with prefix '{prefix}'.")
            else:
                print(f"{s} {file_type.upper()}.")

        types_generated = {}
        for component in self.components:
            # Get component count
            no = types_generated.get(component.componenttype, 0)

            # Write component to output file
            if prefix:
                # Use prefix provided
                file_name = f"{prefix}-{component.componenttype}-{no}.{file_type}"
            elif component.name:
                file_name = f"{component.name}.{file_type}"
            else:
                # No prefix or component name, use vehicle name as fallback
                file_name = f"{self.name}-{component.componenttype}-{no}.{file_type}"

            if self.verbosity > 0:
                print(f"  Writing: {file_name}                 ", end="\r")

            if file_type == 'stl':
                component.to_stl(file_name)
            else:
                component.to_vtk(file_name)

            # Update component count
            types_generated[component.componenttype] = no + 1

        # Write geometric analysis results to csv too
        if self.analysis_results:
            if not prefix:
                prefix = self.name

            # Make analysis results directory
            properties_dir = f"{prefix}_properties"
            if not os.path.exists(properties_dir):
                os.mkdir(properties_dir)

            # Write volume and mass to file
            pd.Series({k: self.analysis_results[k] for k in ["volume", "mass"]}).to_csv(
                os.path.join(properties_dir, f"{prefix}_volmass.csv")
            )

            # Write c.o.g. to file
            self.analysis_results["cog"].tofile(
                os.path.join(properties_dir, f"{prefix}_cog.txt"), sep=", "
            )

            # Write M.O.I. to file
            self.analysis_results["moi"].tofile(
                os.path.join(properties_dir, f"{prefix}_moi.txt"), sep=", "
            )

        if self.verbosity > 0:
            print(
                f"\rAll components written to {file_type.upper()} file format.", end="\n")

    def to_stl(self, prefix: str = None) -> None:
        """Writes the vehicle components to STL. If analysis results are
        present, they will also be written to file, either as CSV, or using
        the Numpy tofile method.

        Parameters
        ----------
        prefix : str, optional
            The prefix to use when saving components to STL. Note that if
            components have been individually assigned name tags, the prefix
            provided will take precedence. If no prefix is specified, and no
            component name tag is available, the Vehicle name will be used.
            The default is None.
        """

        self.to_file(prefix, file_type='stl')

    def to_vtk(self, prefix: str = None) -> None:
        """Writes the vehicle components to VTK. If analysis results are
        present, they will also be written to file, either as CSV, or using
        the Numpy tofile method.

        Parameters
        ----------
        prefix : str, optional
            The prefix to use when saving components to VTK. Note that if
            components have been individually assigned name tags, the prefix
            provided will take precedence. If no prefix is specified, and no
            component name tag is available, the Vehicle name will be used.
            The default is None.
        """

        self.to_file(prefix, file_type='vtk')

    def analyse(self, densities: dict) -> Tuple:
        """Evaluates the mesh properties of the vehicle instance.

        Parameters
        ----------
        densities : Dict[str, float]
            A dictionary containing the effective densities for each component.
            Note that the keys of the dict must match the keys of
            vehicle._named_components. These keys will be consistent with any
            name tags assigned to components.

        Returns
        -------
        total_volume : float
            The total volume.
        total_mass : float
            The toal mass.
        composite_cog : np.array
            The composite center of gravity.
        composite_inertia : np.array
            The composite mass moment of inertia.
        """
        from hypervehicle.utilities import assess_inertial_properties

        self.volume, self.mass, self.cog, self.inertia = assess_inertial_properties(
            vehicle=self, component_densities=densities
        )

        return self.volume, self.mass, self.cog, self.inertia
