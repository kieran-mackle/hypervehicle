import numpy as np
from hypervehicle import Vehicle
from gdtk.geom.vector3 import Vector3
from gdtk.geom.path import Bezier, Line, Polyline


rocket = Vehicle()
rocket.configure(
    name="rocket",
    verbosity=1,
    write_stl=True,
    stl_resolution=5,
    stl_filename="rocket",
    mirror_fins=False,
)

# Construct fuselage
# ====================
#
#  R3--------------------------------R2____
#  |                                       R1__
#  |                                           --_
#  X3--------------------------------X2----X1----Xn

Xn = 2.0
X1 = 1.8
X2 = 1.5
X3 = 0.0

R1 = 0.1
R2 = 0.15
R3 = 0.15

rocket.add_fuselage(Xn=Xn, X1=X1, X2=X2, X3=X3, R1=R1, R2=R2, R3=R3)


# Construct fins
# ====================
#   p1-----p2
#    \        \
#     \         \
#      \          \
#       p0_________p3

fin_height = 0.1
fin_thickness = 0.01
no_fins = 3

p0 = Vector3(x=0.1, y=0.15)
p1 = Vector3(x=0, y=0.15 + fin_height)
p2 = Vector3(x=0.2, y=0.15 + fin_height)
p3 = Vector3(x=0.4, y=0.15)

# Thickness functions
def fin_thickness_function_top(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=-fin_thickness / 2)


def fin_thickness_function_bot(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=fin_thickness / 2)


def leading_edge_width_function(r):
    temp = Bezier(
        [Vector3(x=0.0, y=0.02), Vector3(x=0.75, y=0.05), Vector3(x=1.0, y=0.05)]
    )
    le_width = temp(r).y
    return le_width


# Add fins around body
for i in range(no_fins):
    angle = np.deg2rad((i / no_fins) * 360)
    rocket.add_fin(
        p0=p0,
        p1=p1,
        p2=p2,
        p3=p3,
        fin_thickness=fin_thickness,
        fin_angle=angle,
        top_thickness_function=fin_thickness_function_top,
        bot_thickness_function=fin_thickness_function_bot,
        leading_edge_width_function=leading_edge_width_function,
    )


# Generate geometry
# ====================
rocket.generate()
