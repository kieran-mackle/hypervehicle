from typing import List
from hypervehicle.components.component import Component
from hypervehicle.geometry import SweptPatch, CoonsPatch
from hypervehicle.components.constants import SWEPT_COMPONENT


class SweptComponent(Component):
    componenttype = SWEPT_COMPONENT

    def __init__(
        self,
        cross_sections: List[CoonsPatch],
        sweep_axis: str = "z",
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
        tags: dict = None
    ) -> None:
        """Create a swept component.

        Parameters
        ----------
        cross_sections : list, optional
            A list of cross-sectional patches to sweep through.
        sweep_axis : str, optional
            The axis to sweep the cross sections through. The default
            is z.
        """
        self.cross_sections = cross_sections
        self.sweep_axis = sweep_axis
        self.patches_tags = tags
        super().__init__(stl_resolution=stl_resolution, verbosity=verbosity, name=name)


    def generate_patches(self):
        p = SweptPatch(
            cross_sections=self.cross_sections,
            sweep_axis=self.sweep_axis,
        )

        self.patches["swept_patch"] = p
        self.patches["swept_patch_end_0"] = self.cross_sections[0]
        self.patches["swept_patch_end_1"] = self.cross_sections[-1]

        # Assign tags
        self.patches["swept_patch"].tag = self.patches_tags['swept_tag']
        self.patches["swept_patch_end_0"].tag = self.patches_tags['end_0_tag']
        self.patches["swept_patch_end_1"].tag = self.patches_tags['end_1_tag']

