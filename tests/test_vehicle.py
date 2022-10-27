import os
import numpy as np
from stl import Mesh
from hypervehicle import Vehicle
from eilmer.geom.vector3 import Vector3
from eilmer.geom.path import Bezier, Line, Polyline


def test_wing():

    wing = Vehicle()

    L = 3.7
    beta = np.deg2rad(2)
    LE_height = 0.02
    TE_height = 0.2

    A0 = Vector3(x=0, y=0)
    A1 = Vector3(x=L * 1.8 / 3.7, y=0)
    TT = Vector3(x=L, y=0)
    B0 = Vector3(x=0, y=0.5)
    B1 = Vector3(x=A1.x, y=B0.y)
    B2 = Vector3(x=TT.x, y=0.4 * B0.y)

    B0B1 = Line(p0=B0, p1=B1)
    B1B2 = Line(p0=B1, p1=B2)
    B2TT = Line(p0=B2, p1=TT)

    Line_B0TT = Polyline([B0B1, B1B2, B2TT])

    flap_length = 0
    flap_angle = 0

    # Top rounding Bezier points
    rounding_thickness = 0.025
    p1 = Vector3(x=0, y=0, z=rounding_thickness)
    p2 = Vector3(x=0, y=0.9 * B0.y, z=p1.z)
    p3 = Vector3(x=0, y=B0.y, z=0)
    rounding_bez = Bezier([p1, p2, p3])

    def wing1_tf_top(x, y, z=0):
        # Nominal thickness
        z_m = -L * np.tan(beta)
        z_u = z_m - 0.5 * TE_height
        beta_u = -np.arctan((z_u + 0.5 * LE_height) / L)
        z_1 = (x - L) * np.tan(beta_u) - LE_height / 2

        # Rounding thickness
        if x < B1.x:
            local_y = B0.y
        else:
            local_y = (x - B2.x) * (B2.y - B1.y) / (B2.x - B1.x) + B2.y
        z_2 = -(L - x) * rounding_bez(y / local_y).z

        # Sum
        z_val = z_1 + z_2

        return Vector3(x=0, y=0, z=z_val)

    def wing1_tf_bot(x, y, z=0):
        z_m = -L * np.tan(beta)
        z_l = z_m + 0.5 * TE_height
        beta_l = -np.arctan((z_l - 0.5 * LE_height) / L)
        z_val = (x - L) * np.tan(beta_l) + LE_height / 2
        return Vector3(x=0, y=0, z=z_val)

    def leading_edge_width_function(r):
        temp = Bezier(
            [Vector3(x=0.0, y=0.02), Vector3(x=0.75, y=0.05), Vector3(x=1.0, y=0.05)]
        )
        le_width = temp(r).y
        return le_width

    def wing1_curv_x(x, y):
        "Curvature in x-direction (about y-axis)"
        return 0  # .05 * y**2 * x

    def wing1_curv_xd(x, y):
        return 0  # .05 * y**2 * 1

    def wing1_curv_y(x, y):
        "Curvature in y-direction (about x-axis)"
        return 0  # .1 * y**2

    def wing1_curv_yd(x, y):
        return 0  # .1 * 2*y

    wing.add_wing(
        A0=A0,
        A1=A1,
        TT=TT,
        B0=B0,
        Line_B0TT=Line_B0TT,
        top_tf=wing1_tf_top,
        bot_tf=wing1_tf_bot,
        flap_length=flap_length,
        flap_angle=flap_angle,
        LE_wf=leading_edge_width_function,
    )

    # Generate stl
    wing.configure(
        verbosity=1, write_stl=True, stl_resolution=2, stl_filename="wingtest"
    )
    wing.mirror = True
    wing.generate()

    # Compare output to reference stl
    tests_dir = os.path.dirname(os.path.realpath(__file__))

    reference = Mesh.from_file(os.path.join(tests_dir, "data", "test-wing1.stl"))
    actual = Mesh.from_file(os.path.join(tests_dir, "..", "wingtest-wing1.stl"))

    # Delete file created
    os.remove(os.path.join(tests_dir, "..", "wingtest-wing1.stl"))

    assert np.all(reference.vectors == actual.vectors), "Wing stl failed"
