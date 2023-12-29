from typing import List
from hypervehicle.components.component import Component
from hypervehicle.geometry import SweptPatch, CoonsPatch, SweptPatchMultiFace
from hypervehicle.components.constants import (
    SWEPT_COMPONENT,
    SWEPT_COMPONENT_MULTI_FACE,
)


class SweptComponent(Component):
    componenttype = SWEPT_COMPONENT

    def __init__(
        self,
        cross_sections: List[CoonsPatch],
        sweep_axis: str = "z",
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
        tags: dict = None,
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
        super().__init__(
            stl_resolution=stl_resolution,
            verbosity=verbosity,
            name=name,
            patch_name_to_tags=tags,
        )

    def generate_patches(self):
        p = SweptPatch(
            cross_sections=self.cross_sections,
            sweep_axis=self.sweep_axis,
        )

        self.patches["swept_patch"] = p
        self.patches["swept_patch_end_0"] = self.cross_sections[0]
        self.patches["swept_patch_end_1"] = self.cross_sections[-1]
        self.add_tag_to_patches()


class SweptComponentMultiFace(Component):
    componenttype = SWEPT_COMPONENT_MULTI_FACE

    def __init__(
        self,
        cross_sections: List[CoonsPatch],
        sweep_axis: str = "z",
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
        tags: dict = None,
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
        super().__init__(
            stl_resolution=stl_resolution,
            verbosity=verbosity,
            name=name,
            patch_name_to_tags=tags,
        )

    def generate_patches(self):

        self.patches["swept_patch_north"] = SweptPatchMultiFace(
            cross_sections=self.cross_sections,
            sweep_axis=self.sweep_axis,
            face_direction="north",
        )
        self.patches["swept_patch_east"] = SweptPatchMultiFace(
            cross_sections=self.cross_sections,
            sweep_axis=self.sweep_axis,
            face_direction="east",
        )
        self.patches["swept_patch_south"] = SweptPatchMultiFace(
            cross_sections=self.cross_sections,
            sweep_axis=self.sweep_axis,
            face_direction="south",
        )
        self.patches["swept_patch_west"] = SweptPatchMultiFace(
            cross_sections=self.cross_sections,
            sweep_axis=self.sweep_axis,
            face_direction="west",
        )

        self.patches["swept_patch_end_0"] = self.cross_sections[0]
        self.patches["swept_patch_end_1"] = self.cross_sections[-1]

        self.add_tag_to_patches()
