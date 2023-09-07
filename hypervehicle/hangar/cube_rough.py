"""
Example code to create a cuboid (hexahedron with square angles) with dimensions
2*a, 2*b, 2*c and that has a designed roughness imprinted into the top surface.


Created: 22/08/2023
Last Modified: 07/09/2023
Author(s): Ingo Jahn
"""

import numpy as np
from hypervehicle.components import Cuboid, Cube
from hypervehicle.components import Component
from hypervehicle.geometry import Vector3, CuboidPatch, FacePatchRough


class CuboidRough(Component):
    # componenttype = CUBOID_ROUGH

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

    def define_rough_face(self, roughness_data, lambda_0, roughness_height, roll_off):
        self.roughness_data = roughness_data
        self.lambda_0 = lambda_0
        self.roughness_height = roughness_height
        self.roll_off = roll_off

    def generate_patches(self):
        faces = ["east", "west", "south", "north", "bottom"]
        # create smooth faces
        for face in faces:
            self.patches[face] = CuboidPatch(self.a, self.b, self.c, self.centre, face)

        # create rough face
        face = "top"
        self.patches[face] = FacePatchRough(
            self.a,
            self.b,
            self.c,
            self.centre,
            face,
            self.roughness_data,
            self.lambda_0,
            self.roughness_height,
            self.roll_off,
        )


if __name__ == "__main__":
    # config_case = "cube"
    # config_case = "cuboid"
    config_case = "cuboid_rough"

    print(f"Working on case '{config_case}'.")
    match config_case:
        case "cube":
            print(f"    Example - Cube with side length = 1")
            body = Cube(a=1)
            body.generate_patches()
            stl_filename = "cube.stl"

        case "cuboid":
            print(f"    Example - Cuboid")
            a = 1
            b = 1
            c = 0.1
            print(f"    Dimensions a={a}, b={b}, c={c}")
            body = Cuboid(a=a, b=b, c=c)
            body.generate_patches()
            stl_filename = "cuboid.stl"

        case "cuboid_rough":
            print(f"    Example - Cuboid with 'rough' top surface")
            a = 0.08 / 2
            b = 0.08 / 2
            c = 0.005
            print(f"    Cuboid Dimensions a={a}, b={b}, c={c}")
            body = CuboidRough(a=a, b=b, c=c, stl_resolution=30)

            roughness_height = 0.005
            lambda_0 = 0.04  # [m] - length of longest frequency to be included
            roll_off = 0.01  # [m] - distance over which stencil rolls off

            roughness_data = np.array(
                [
                    [3, 3],
                    [0.23858489823e+00, 0.76882559458e+00],
                    [0.13405469344e+00, 0.32250873843e+01],
                    [0.37001241020e+00, 0.39367026114e+01],
                    [0.77008101266e-01, 0.13989718680e+01],
                    [0.36533620243e+00, 0.37127829101e+01],
                    [0.00000000000e+00, 0.13001876919e+01],
                    [0.28378862436e+00, 0.21209129392e+00],
                    [0.00000000000e+00, 0.44289617133e+01],
                    [0.00000000000e+00, 0.55468162808e+01],
                ]
            )

            body.define_rough_face(roughness_data, lambda_0, roughness_height, roll_off)
            body.generate_patches()
            stl_filename = "cuboid_rough_2.stl"
        case _:
            print(f"case - '{config_case}' not yet implemented. Bailing Out!")

    print(f"    START: Writing data to STL: {stl_filename}")
    body.to_stl(stl_filename)
    print(f"      DONE: Writing data to STL file.")
