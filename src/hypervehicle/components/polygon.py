from hypervehicle.components import Component
from hypervehicle.components.constants import CUBE, SPHERE
from hypervehicle.geometry.vector import Vector3
from hypervehicle.geometry.geometry import CubePatch, SpherePatch


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
