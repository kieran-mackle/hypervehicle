import numpy as np
from copy import deepcopy
from typing import Optional
from abc import ABC, abstractmethod
from hypervehicle.geometry.path import ClusterFunction, LinearFunction, Line, Path
from hypervehicle.geometry.vector import Vector3, approximately_equal_vectors


class ParametricSurface(ABC):
    """Astract parametric surface."""

    @abstractmethod
    def __repr__(self):
        pass

    @abstractmethod
    def __call__(self, r, s):
        pass


class CoonsPatch(ParametricSurface):
    """A surface constructed by transfinite interpolation of the edges."""

    _slots_ = [
        "north",
        "east",
        "south",
        "west",
        "p00",
        "p10",
        "p11",
        "p01",
        "defined_by_corners",
        "offset",
    ]

    def __init__(
        self,
        north: Path = None,
        east: Path = None,
        south: Path = None,
        west: Path = None,
        p00: Vector3 = None,
        p10: Vector3 = None,
        p11: Vector3 = None,
        p01: Vector3 = None,
        offset=Vector3(0, 0, 0),
    ):
        """Initialise from edges or corner points."""
        if all([north, east, south, west]):
            self.north = deepcopy(north)
            self.east = deepcopy(east)
            self.south = deepcopy(south)
            self.west = deepcopy(west)
            self.p00 = self.south(0.0)
            self.p10 = self.south(1.0)
            self.p01 = self.north(0.0)
            self.p11 = self.north(1.0)
            p00_alt = self.west(0.0)
            p10_alt = self.east(0.0)
            p01_alt = self.west(1.0)
            p11_alt = self.east(1.0)
            if not approximately_equal_vectors(self.p00, p00_alt):
                raise Exception(f"CoonsPatch open corner p00={self.p00}, {p00_alt}")
            if not approximately_equal_vectors(self.p10, p10_alt):
                raise Exception(f"CoonsPatch open corner p10={self.p10}, {p10_alt}")
            if not approximately_equal_vectors(self.p01, p01_alt):
                raise Exception(f"CoonsPatch open corner p01={self.p01}, {p01_alt}")
            if not approximately_equal_vectors(self.p11, p11_alt):
                raise Exception(f"CoonsPatch open corner p11={self.p11}, {p11_alt}")
            self.defined_by_corners = False

        elif all([p00, p10, p11, p01]):
            self.north = Line(p01, p11)
            self.east = Line(p10, p11)
            self.south = Line(p00, p10)
            self.west = Line(p00, p01)
            self.p00 = deepcopy(p00)
            self.p10 = deepcopy(p10)
            self.p11 = deepcopy(p11)
            self.p01 = deepcopy(p01)
            self.defined_by_corners = True

        else:
            raise Exception(
                "CoonsPatch should be defined by four edge paths or four corners."
            )
        self.offset = offset

    def __repr__(self):
        str = "CoonsPatch("
        if self.defined_by_corners:
            str += f"p00={self.p00}, p10={self.p10}, p11={self.p10}, p01={self.p10}"
        else:
            str += f"north={self.north}, east={self.east}, south={self.south}, west={self.west}"
        str += f", offset={self.offset})"
        return str

    def __call__(self, r, s):
        """
        Transfinite interpolation to an interior point, p.
        """
        south_r = self.south(r)
        north_r = self.north(r)
        west_s = self.west(s)
        east_s = self.east(s)
        p = (
            south_r * (1.0 - s)
            + north_r * s
            + west_s * (1.0 - r)
            + east_s * r
            - (
                self.p00 * (1.0 - r) * (1.0 - s)
                + self.p01 * (1.0 - r) * s
                + self.p10 * r * (1.0 - s)
                + self.p11 * r * s
            )
            + self.offset
        )
        return p

    def __add__(self, offset):
        """
        Returns a copy of the original, displaced by a Vector3 object.
        """
        if not isinstance(offset, Vector3):
            raise Exception(f"Cannot add a {type(offset)} to a CoonsPatch.")
        new_patch = deepcopy(self)
        new_patch.offset += offset
        return new_patch

    def __sub__(self, offset):
        """
        Returns a copy of the original, displaced by a Vector3 object.
        """
        if not isinstance(offset, Vector3):
            raise Exception(f"Cannot subtract a {type(offset)} from a CoonsPatch.")
        new_patch = deepcopy(self)
        new_patch.offset -= offset
        return new_patch


class StructuredGrid:
    """Structured grid."""

    _slots_ = ["dimensions", "niv", "njv", "nkv", "vertices", "label", "tags"]

    def __init__(
        self,
        surface: ParametricSurface,
        niv: Optional[int] = 1,
        njv: Optional[int] = 1,
        cf_list: Optional[list[ClusterFunction]] = None,
        tags: Optional[list[str]] = None,
        label: Optional[str] = "no label",
    ):
        cf_list = cf_list if cf_list is not None else [None, None, None, None]
        cf_list = [
            cf if isinstance(cf, ClusterFunction) else LinearFunction()
            for cf in cf_list
        ]
        self.generate(surface, niv, njv, cf_list)
        self.tags = (tags if tags is not None else ["", "", "", ""],)
        self.label = label

    def __repr__(self):
        str = "StructuredGrid("
        str += f"dimensions={self.dimensions}, niv={self.niv}, njv={self.njv}, nkv={self.nkv}"
        str += f", vertices={self.vertices}"
        str += f", tags={self.tags}"
        str += ")"
        return str

    def generate(self, surface, niv, njv, cf_list):
        if not isinstance(surface, ParametricSurface):
            raise Exception("Need to supply a ParametricSurface to construct the grid.")
        if niv < 2:
            raise Exception(f"niv is too small: {niv}")
        if njv < 2:
            raise Exception(f"njv is too small: {njv}")
        self.niv = niv
        self.njv = njv
        self.nkv = 1
        self.dimensions = 2

        # Start with uniformly-distributed sample points.
        r = np.fromfunction(lambda i, j: i, (niv, njv), dtype=float) / (niv - 1)
        s = np.fromfunction(lambda i, j: j, (niv, njv), dtype=float) / (njv - 1)

        # Compute independent cluster function along each edge.
        rNorth = cf_list[0](r)
        sEast = cf_list[1](s)
        rSouth = cf_list[2](r)
        sWest = cf_list[3](s)

        # Blend the clustered sample points from each edge of the rs unit square.
        sdash = (1.0 - r) * sWest + r * sEast
        rdash = (1.0 - s) * rSouth + s * rNorth

        # Compute the xyz spatial coordinates of the surface.
        self.vertices = surface(rdash, sdash)
        return

    def subgrid(self, i0=0, j0=0, k0=0, niv=1, njv=1, nkv=1):
        """
        Returns a copy of a subgrid of vertices.
        Start at vertex i0,j0,k0 and extend for niv,njv,nkv vertices.
        """
        new_grid = StructuredGrid(empty=True)
        if self.dimensions == 3:
            new_xs = self.vertices.x[i0 : i0 + niv, j0 : j0 + njv, k0 : k0 + nkv].copy()
            new_ys = self.vertices.y[i0 : i0 + niv, j0 : j0 + njv, k0 : k0 + nkv].copy()
            new_zs = self.vertices.z[i0 : i0 + niv, j0 : j0 + njv, k0 : k0 + nkv].copy()
            new_grid.vertices = Vector3(new_xs, new_ys, new_zs)
        elif self.dimensions == 2:
            new_xs = self.vertices.x[i0 : i0 + niv, j0 : j0 + njv].copy()
            new_ys = self.vertices.y[i0 : i0 + niv, j0 : j0 + njv].copy()
            new_zs = self.vertices.z[i0 : i0 + niv, j0 : j0 + njv].copy()
            new_grid.vertices = Vector3(new_xs, new_ys, new_zs)
        elif self.dimensions == 1:
            new_xs = self.vertices.x[i0 : i0 + niv].copy()
            new_ys = self.vertices.y[i0 : i0 + niv].copy()
            new_zs = self.vertices.z[i0 : i0 + niv].copy()
            new_grid.vertices = Vector3(new_xs, new_ys, new_zs)
        new_grid.niv = niv
        new_grid.njv = njv
        new_grid.nkv = nkv
        if niv == 1 and njv == 1 and nkv == 1:
            new_grid.dimensions = 0
        elif njv == 1 and nkv == 1:
            new_grid.dimensions = 1
        elif nkv == 1:
            new_grid.dimensions = 2
        else:
            new_grid.dimensions = 3
        return new_grid
