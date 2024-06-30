from typing import List, Callable, Tuple, Dict, Any
from hypervehicle.components.component import Component
from hypervehicle.components.constants import (
    FIN_COMPONENT,
    WING_COMPONENT,
    SWEPT_COMPONENT,
    REVOLVED_COMPONENT,
    COMPOSITE_COMPONENT,
    SPHERE,
    CUBE,
)


class CompositeComponent(Component):
    """A composite component.

    This Component allows adding multiple components to it, which
    will all form the patches of this component. Note that this
    means all components added will share the same stl resolution,
    regardless of what their previously assigned resolution was. This
    is because all individual component patches become merged to form
    the CompositeComponent.patches.
    """

    componenttype = COMPOSITE_COMPONENT
    ALLOWABLE_COMPONENTS = [
        FIN_COMPONENT,
        WING_COMPONENT,
        SWEPT_COMPONENT,
        REVOLVED_COMPONENT,
        SPHERE,
        CUBE,
    ]

    def __init__(
        self,
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
    ) -> None:
        # Initialise base class
        super().__init__(
            params=None,
            stl_resolution=stl_resolution,
            verbosity=verbosity,
            name=name,
        )

        self.components: List[Component] = []

    def add_component(
        self,
        component: Component,
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
        if component.componenttype in CompositeComponent.ALLOWABLE_COMPONENTS:
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

        else:
            raise Exception(f"Unrecognised component type: {component.componenttype}")

    def generate_patches(self):
        # Generate child patches
        for component in self.components:
            component.generate_patches()

        # Merge patches
        merged = {}
        for ix, component in enumerate(self.components):
            for key, patch in component.patches.items():
                merged[f"{key}_{ix}"] = patch

        self.patches = merged
