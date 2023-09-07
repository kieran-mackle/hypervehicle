from copy import deepcopy
from hypervehicle.components import Component
from hypervehicle.components.constants import CUBE, SPHERE, CUBOID, HEXAHEDRON
from hypervehicle.geometry import (
    Vector3,
    CubePatch,
    CuboidPatch,
    SpherePatch,
    RoughnessPatch,
    RoughnessElement,
)


class Cube(Component):
    componenttype = CUBE

    def __init__(
        self,
        a: float,
        centre: Vector3 = Vector3(0, 0, 0),
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
    ) -> None:
        """
        Parameters
        -----------
        a : float
            The cube side half-length.
        centre : Vector3, optional
            The centre point of the cube. The default is Vector3(0,0,0).
        """
        self.a = a
        self.centre = centre
        super().__init__(stl_resolution=stl_resolution, verbosity=verbosity, name=name)

    def generate_patches(self):
        faces = ["east", "west", "south", "north", "bottom", "top"]
        for face in faces:
            self.patches[face] = CubePatch(self.a, self.centre, face)


class Cuboid(Component):
    componenttype = CUBOID

    def __init__(
        self,
        a: float,
        b: float,
        c: float,
        centre: Vector3 = Vector3(0, 0, 0),
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
    ) -> None:
        """
        Parameters
        -----------
        a : float
            The cuboid x-side half-length.
        b : float
            The cuboid y-side half-length.
        c : float
            The cuboid z-side half-length.
        centre : Vector3, optional
            The centre point of the cube. The default is Vector3(0,0,0).
        """
        self.a = a
        self.b = b
        self.c = c
        self.centre = centre
        super().__init__(stl_resolution=stl_resolution, verbosity=verbosity, name=name)

    def generate_patches(self):
        faces = ["east", "west", "south", "north", "bottom", "top"]
        for face in faces:
            self.patches[face] = CuboidPatch(self.a, self.b, self.c, self.centre, face)


class Sphere(Component):
    componenttype = SPHERE

    def __init__(
        self,
        r: float,
        centre: Vector3 = Vector3(0, 0, 0),
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
    ) -> None:
        """
        Parameters
        -----------
        r : float
            The radius of the sphere.
        centre : Vector3
            The centre point of the sphere. The default is Vector3(0,0,0).
        """
        self.r = r
        self.centre = centre
        super().__init__(stl_resolution=stl_resolution, verbosity=verbosity, name=name)

    def generate_patches(self):
        faces = ["east", "west", "south", "north", "bottom", "top"]

        for face in faces:
            self.patches[face] = SpherePatch(self.r, self.centre, face)


class RoughHexahedron(Component):
    componenttype = HEXAHEDRON

    def __init__(
        self,
        params: dict = None,
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
    ) -> None:
        """Create a hexehedron with rough surface."""

        super().__init__(stl_resolution=stl_resolution, verbosity=verbosity, name=name)

    def generate_patches(self):
        # Generate nominal cube
        cube = Cube()
        cube.generate_patches()

        patches = deepcopy(cube.patches)

        # Generate nominal roughness patch
        roughness_element = RoughnessElement(
            80e-3, 80e-3, 0.1, 40e-3, Nx=0, Ny=0, data=None, ramp_x=0.1, ramp_y=0.1
        )
        rp = RoughnessPatch(roughness_element, x_range, y_range)

        # Replace top surface with roughness patch
        patches.pop("north")
        patches["north"] = rp

        self.patches = patches
