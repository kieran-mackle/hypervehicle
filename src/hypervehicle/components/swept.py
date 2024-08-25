from typing import List, Dict, Optional, Union
from hypervehicle.geometry.surface import CoonsPatch
from hypervehicle.components.component import Component
from hypervehicle.components.constants import SWEPT_COMPONENT
from hypervehicle.geometry.geometry import SweptPatchfromEdges, ReversedPath, Path


class SweptComponent(Component):
    componenttype = SWEPT_COMPONENT

    def __init__(
        self,
        cross_sections: List[List[Path]],
        close_ends: Optional[bool] = True,
        stl_resolution: Optional[Union[int, Dict[str, int]]] = 1,
        verbosity: Optional[int] = 1,
        name: Optional[str] = None,
    ) -> None:
        """Create a swept component.

        Parameters
        ----------
        cross_sections : list
            A list containing cross-sections. Each cross-section is a list of
            paths that define one cross-section. They should connect end-to-end.

        close_ends : bool, optional
            If true the first and last cross-section will be used to close
            the sweap component. Only supported is cross-section is defined by
            4 paths. The default is True.

        stl_resolution : int | dict[str, int], optional
            Defines different stl resolution for different edges.  Keys are
            are: 'e0', 'e1', ... 'eN', 'sweep' for edges in first cross-section
            and for swept edges. The default is 1.
        """
        super().__init__(stl_resolution=stl_resolution, verbosity=verbosity, name=name)
        self.cross_sections = cross_sections
        self.close_ends = close_ends
        self.n_slices = len(self.cross_sections)
        self.n_edges = len(self.cross_sections[0])
        self._check_options()

    def _check_options(self, small_number: Optional[int] = 1e-12):
        for ns, cs in enumerate(self.cross_sections):
            if len(cs) != self.n_edges:
                # check that each c/s has correct number of edges
                raise Exception(
                    f"Swept Component {self.name}."
                    + f"Slice {ns} has incorrect number of edges.\n"
                    + f"N_e={len(cs)} - {self.n_edges} expected."
                )
            for i in range(len(cs)):
                # check that edges in each c/s form a closed loop
                if i < len(cs) - 1:
                    ip = i + 1
                else:
                    ip = 0
                p1 = cs[i](1)
                p0 = cs[ip](0)
                if (
                    abs(p0.x - p1.x) > small_number
                    or abs(p0.y - p1.y) > small_number
                    or abs(p0.z - p1.z) > small_number
                ):
                    raise Exception(
                        f"Swept Component {self.name}, Slice {ns}, edges not closed.\n"
                        + f"edges[{i}](1) != edges[{ip}](0)\n"
                        + f"{p1} != {p0}"
                    )
        if self.close_ends is True and self.n_edges != 4:
            raise Exception(
                f"Swept Component {self.name}. Combination of "
                + f"close_ends={self.close_ends} and N_edge={self.n_edges} is "
                + f"not supported."
            )
        if self.close_ends and isinstance(self.stl_resolution, Dict):
            flag = 0
            if self.stl_resolution["e0"] != self.stl_resolution["e2"]:
                print("edge 'e0' and 'e2' don't have same stl_resolution.")
                flag = 1
            if self.stl_resolution["e1"] != self.stl_resolution["e3"]:
                print("edge 'e1' and 'e3' don't have same stl_resolution.")
                flag = 1
            if flag > 0:
                raise Exception(f"stl_resolution not compatible for close_end.")

    def generate_patches(self):
        for ne in range(self.n_edges):
            k = f"swept_patch_{ne}"
            edges = []
            for cs in self.cross_sections:
                edges.append(cs[ne])
            p = SweptPatchfromEdges(edges=edges)
            self.patches[k] = p
            if isinstance(self.stl_resolution, int):
                self.patch_res_r[k] = self.stl_resolution
                self.patch_res_s[k] = self.stl_resolution
            else:
                self.patch_res_r[k] = self.stl_resolution["sweep"]
                self.patch_res_s[k] = self.stl_resolution[f"e{ne}"]

        if self.close_ends == True:
            edges = self.cross_sections[0]  # front
            south = edges[0]
            east = edges[1]
            north = ReversedPath(edges[2])
            west = ReversedPath(edges[3])
            self.patches["swept_patch_end_0"] = CoonsPatch(
                south=south, north=north, west=west, east=east
            )
            if isinstance(self.stl_resolution, int):
                self.patch_res_r["swept_patch_end_0"] = self.stl_resolution
                self.patch_res_s["swept_patch_end_0"] = self.stl_resolution
            else:
                self.patch_res_r["swept_patch_end_0"] = self.stl_resolution["e0"]
                self.patch_res_s["swept_patch_end_0"] = self.stl_resolution["e1"]
            edges = self.cross_sections[-1]  # rear
            south = ReversedPath(edges[0])
            east = ReversedPath(edges[3])
            north = edges[2]
            west = edges[1]
            self.patches["swept_patch_end_1"] = CoonsPatch(
                south=south, north=north, west=west, east=east
            )
            if isinstance(self.stl_resolution, int):
                self.patch_res_r["swept_patch_end_1"] = self.stl_resolution
                self.patch_res_s["swept_patch_end_1"] = self.stl_resolution
            else:
                self.patch_res_r["swept_patch_end_1"] = self.stl_resolution["e0"]
                self.patch_res_s["swept_patch_end_1"] = self.stl_resolution["e1"]
