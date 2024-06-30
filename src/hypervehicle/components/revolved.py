import numpy as np
from hypervehicle.geometry.geometry import RevolvedPatch
from hypervehicle.components.component import Component
from hypervehicle.components.constants import REVOLVED_COMPONENT


class RevolvedComponent(Component):
    componenttype = REVOLVED_COMPONENT

    def __init__(
        self,
        revolve_line,
        stl_resolution: int = 4,
        verbosity: int = 1,
        name: str = None,
    ) -> None:
        """Create a revolved component.

        Parameters
        ----------
        revolve_line : Line|PolyLine|Bezier
            A line to be revolved about the primary axis.
        """
        self.revolve_line = revolve_line
        super().__init__(stl_resolution=stl_resolution, verbosity=verbosity, name=name)

    def generate_patches(self):
        for i in range(4):
            self.patches[f"revolved_fuse_{i}"] = RevolvedPatch(
                self.revolve_line, i * np.pi / 2, (i + 1) * np.pi / 2
            )
