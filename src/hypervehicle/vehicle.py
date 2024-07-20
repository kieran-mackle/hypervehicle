import os
import pandas as pd
from hypervehicle import utilities
from hypervehicle.components.component import Component
from typing import List, Tuple, Callable, Dict, Any, Optional, Union
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
        self.properties = {}  # user-defined vehicle properties from generator

        # Analysis attributes
        self.analysis_results = None
        self.component_properties = None
        self._volmass = None
        self.volume = None
        self.mass = None
        self.area = None
        self.cog = None
        self.moi = None

        # Internal attributes
        self._generated = False
        self._component_counts = {}
        self._enumerated_components = {}
        self._named_components: dict[str, Component] = {}
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
        clustering: Dict[str, Callable] = None,
        transformations: List[Tuple[str, Any]] = None,
        modifier_function: Optional[Callable] = None,
        ghost: Optional[bool] = False,
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
            component, rather than making it a new component. This is recommended
            when the combined components will form a closed mesh, but if the
            components will remain as two isolated bodies, a new component should
            be created (ie. append_reflection=False). In this case, you can use
            copy.deepcopy to make a copy of the reflected component when adding it
            to the vehicle. See the finner example in the hypervehicle hangar.
            The default is True.

        curvatures : List[Tuple[str, Callable, Callable]], optional
            A list of the curvatures to apply to the component being added.
            This list contains a tuple for each curvature. Each curvatue
            is defined by (axis, curve_func, curve_func_derivative).
            The default is None.

        clustering : Dict[str, Callable], optional
            Optionally provide clustering options for the stl meshes. See
            parametricSurfce2stl for more information. The default is None.

        transformations : List[Tuple[str, Any]], optional
            A list of transformations to apply to the nominal component. The
            default is None.

        modifier_function : Callable, optional
            A function which accepts x,y,z coordinates and returns a Vector3
            object with a positional offset. This function is used with an
            OffsetPatchFunction. The default is None.

        ghost : bool, optional
            Add a ghost component. When True, this component will be excluded
            from the files written to STL.

        See Also
        --------
        Vehicle.add_vehicle_transformations
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
                component.add_clustering_options(**clustering)

            # Add transformations
            if transformations is not None:
                component._transformations = transformations

            # Add modifier function
            if modifier_function is not None:
                component._modifier_function = modifier_function

            # Ghost component
            component._ghost = ghost

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
                # No name provided, check component name
                if component.name:
                    # Use component name
                    name = component.name
                else:
                    # Generate default name
                    name = f"{component.componenttype}_{component_count}"
            self._named_components[name] = component

            if self.verbosity > 1:
                print(f"Added new {component.componenttype} component ({name}).")

        else:
            raise Exception(f"Unrecognised component type: {component.componenttype}")

    def generate(self):
        """Generate all components of the vehicle."""
        if self.verbosity > 0:
            utilities.print_banner()
            print("Generating component patches.")

        for component in self.components:
            if self.verbosity > 1:
                print(f"  Generating patches for {component.componenttype} component.")

            # Generate component patches
            component.generate_patches()

            # Apply the modifier function
            component.apply_modifier()

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
        Each transformation in the list should be a tuple of the form
        (transform_type, *args), where transform_type can be "rotate", or
        "translate". Note that transformations can be chained.

        Extended Summary
        ----------------
        - "rotate" : rotate the entire vehicle. The *args for rotate are
        angle (float) and axis (str). For example: `[("rotate", 180, "x"),
        ("rotate", 90, "y")]`.

        - "translate" : translate the entire vehicle. The *args for translate
        includes the translational offset, specified either as a function (Callable),
        or as Vector3 object.
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
        """Transform vehicle by applying the tranformations."""
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

    def to_stl(self, prefix: str = None, merge: Union[bool, List[str]] = False) -> None:
        """Writes the vehicle components to STL file. If analysis results are
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

        merge : [bool, list[str]], optional
            Merge components of the vehicle into a single STL file. The merge
            argument can either be a boolean (with True indicating to merge all
            components of the vehicle), or a list of the component names to
            merge. This functionality depends on PyMesh. The default is False.

        See Also
        --------
        utilities.merge_stls
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
                if component.name:
                    stl_name = f"{prefix}-{component.name}.stl"
                else:
                    stl_name = f"{prefix}-{component.componenttype}-{no}.stl"
            elif component.name:
                stl_name = f"{component.name}.stl"
            else:
                # No prefix or component name, use vehicle name as fallback
                stl_name = f"{self.name}-{component.componenttype}-{no}.stl"

            if self.verbosity > 0:
                print(f"  Writing: {stl_name}                 ", end="\r")

            component.to_stl(stl_name)

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
            self._volmass.to_csv(os.path.join(properties_dir, f"{prefix}_volmass.csv"))

            # Write c.o.g. to file
            self.analysis_results["cog"].tofile(
                os.path.join(properties_dir, f"{prefix}_cog.txt"), sep=", "
            )

            # Write M.O.I. to file
            self.analysis_results["moi"].tofile(
                os.path.join(properties_dir, f"{prefix}_moi.txt"), sep=", "
            )

        # Write user-defined vehicle properties
        if self.properties:
            if not prefix:
                prefix = self.name

            # Make analysis results directory
            properties_dir = f"{prefix}_properties"
            if not os.path.exists(properties_dir):
                os.mkdir(properties_dir)

            # Write properties to file
            pd.Series(self.properties).to_csv(
                os.path.join(properties_dir, f"{prefix}_properties.csv")
            )

        # Merge STL components
        if merge:
            if isinstance(merge, list):
                # Merge specified components
                raise NotImplementedError(
                    "Merging components by name not yet implemented."
                )

            else:
                # Merge all components
                if not prefix:
                    prefix = self.name

                # Get component names (excluding ghost components)
                filenames = []
                for name, comp in self._named_components.items():
                    if not comp._ghost:
                        # Append
                        filenames.append(f"{name}.stl")
                utilities.merge_stls(stl_files=filenames, name=prefix)

        if self.verbosity > 0:
            print("\rAll components written to STL file format.", end="\n")

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

        vehicle_properties, component_properties = assess_inertial_properties(
            vehicle=self, component_densities=densities
        )

        # Unpack vehicle properties
        for k, v in vehicle_properties.items():
            setattr(self, k, v)

        # Save component properties
        self.component_properties = component_properties

        # Save summary of volume and mass results
        component_vm = pd.DataFrame(
            {k: component_properties[k] for k in ["mass", "volume", "area"]}
        )
        self._volmass = pd.concat(
            [
                component_vm,
                pd.DataFrame(
                    data={
                        "mass": self.mass,
                        "volume": self.volume,
                        "area": vehicle_properties["area"],
                    },
                    index=["vehicle"],
                ),
            ]
        )

        return self.volume, self.mass, self.cog, self.moi

    def add_property(self, name: str, value: float):
        """Add a named property to the vehicle. Currently only supports
        float property types.
        """
        self.properties[name] = value

    def get_non_ghost_components(self) -> dict[str, Component]:
        """Returns all non-ghost components."""
        non_ghost = {}
        for name, comp in self._named_components.items():
            if not comp._ghost:
                # Append
                non_ghost[name] = comp
        return non_ghost
