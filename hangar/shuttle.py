import numpy as np
from hypervehicle import Vehicle
from gdtk.geom.vector3 import Vector3
from gdtk.geom.path import Bezier, Line, Polyline


# Instantiate Vehicle
shuttle = Vehicle()
shuttle.configure(
    name="Space Shuttle",
    verbosity=1,
    write_stl=True,
    stl_resolution=15,
    stl_filename="shuttle",
    mirror_fins=False,
)


# Construct wing
# ====================
wing_L = 2
wing_thickness = 0.1
wing_width = 0.9
wing_origin = 0.2
flap_length = 0.15

# Required points
A0 = Vector3(x=wing_origin, y=0)
A1 = Vector3(x=A0.x + 0.35 * wing_L, y=0)
TT = Vector3(x=A0.x + wing_L, y=0)
B0 = Vector3(x=A0.x, y=wing_width)

# Construction points
B1 = Vector3(x=A0.x + 0.02 * wing_L, y=1.05 * wing_width)
B2 = Vector3(x=A1.x, y=0.4 * wing_width)
B3 = Vector3(x=TT.x, y=0.2 * wing_width)

# Leading edge line
B0B1 = Line(p0=B0, p1=B1)
B1B2 = Line(p0=B1, p1=B2)
B2B3 = Line(p0=B2, p1=B3)
B3TT = Line(p0=B3, p1=TT)
Line_B0TT = Polyline([B0B1, B1B2, B2B3, B3TT])


def wing_tf_top(x, y, z=0):
    return Vector3(x=0, y=0, z=-wing_thickness / 2)


def wing_tf_bot(x, y, z=0):
    return Vector3(x=0, y=0, z=wing_thickness / 2)


def leading_edge_width_function(r):
    temp = Bezier(
        [Vector3(x=0.0, y=0.02), Vector3(x=0.75, y=0.05), Vector3(x=1.0, y=0.05)]
    )
    le_width = temp(r).y
    return le_width


# shuttle.add_wing(A0=A0, A1=A1, TT=TT, B0=B0, Line_B0TT=Line_B0TT,
#                  top_tf=wing_tf_top, bot_tf=wing_tf_bot,
#                  LE_type='flat',
#                 #  LE_wf=leading_edge_width_function,
#                  flap_length=flap_length)


# Construct Body
# ====================
# B0----------------------------------------------B1
# |                                               |
# |                                               |
# |                                               |
# A0-----------------------------A1---------------TT

body_L = 2.2
body_width = 0.2
body_height = 0.25
body_x_origin = 0  # 0.3

# Required points
A0 = Vector3(x=body_x_origin, y=0)
A1 = Vector3(x=A0.x + 0.5 * body_L, y=0)
TT = Vector3(x=A0.x + body_L, y=0)
B0 = Vector3(x=A0.x, y=body_width)

# Construction points
B1 = Vector3(x=TT.x, y=body_width)

# Leading edge line
B0B1 = Line(p0=B0, p1=B1)
B1TT = Line(p0=B1, p1=TT)
Line_B0TT = Polyline([B0B1, B1TT])


def wing_tf_top(x, y, z=0):
    return Vector3(x=0, y=0, z=0 / 2)


def wing_tf_bot(x, y, z=0):
    z_nom = 0.8 * body_height
    z_round_top = 1.3 * np.sqrt(0.2 * body_height - y**2)
    return Vector3(x=0, y=0, z=z_nom + z_round_top)


def leading_edge_width_function(r):
    temp = Bezier(
        [Vector3(x=0.0, y=0.02), Vector3(x=0.75, y=0.05), Vector3(x=1.0, y=0.05)]
    )
    le_width = temp(r).y
    return le_width


# shuttle.add_wing(A0=A0, A1=A1, TT=TT, B0=B0, Line_B0TT=Line_B0TT,
#                  top_tf=wing_tf_top, bot_tf=wing_tf_bot,
#                  LE_type='flat',
#                 #  LE_wf=leading_edge_width_function,
#                  )


# Construct fuselage
# ====================
R_base = 0.175
nose_length = 0.3
nose_offset_z = 0.15


def offset(x, y, z):
    return Vector3(0, 0, nose_offset_z)


nose_start = body_L + body_x_origin
Xn = nose_start + nose_length
X1 = nose_start + 0.95 * nose_length
X2 = nose_start + 0.85 * nose_length
X3 = nose_start

R1 = 0.3 * R_base
R2 = 0.5 * R_base
R3 = R_base

shuttle.add_fuselage(Xn=Xn, X1=X1, X2=X2, X3=X3, R1=R1, R2=R2, R3=R3, offset=offset)


# Construct fin
# ====================
#   p1-----p2
#    \        \
#     \         \
#      \          \
#       p0_________p3

fin_x_origin = 0.2  # x coord of p0
fin_y_origin = 0.4  # y coord of p0
fin_height = 0.5
fin_length = 0.3
fin_thickness = 0.01
no_fins = 3

p0 = Vector3(x=fin_x_origin, y=fin_y_origin)
p1 = Vector3(x=fin_x_origin - 0.9 * fin_length, y=fin_y_origin + fin_height)
p2 = Vector3(x=fin_x_origin - 0.3 * fin_length, y=fin_y_origin + fin_height)
p3 = Vector3(x=fin_x_origin + fin_length, y=fin_y_origin)

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


# shuttle.add_fin(p0=p0, p1=p1, p2=p2, p3=p3,
#                 fin_thickness=fin_thickness,
#                 fin_angle=np.deg2rad(90),
#                 top_thickness_function=fin_thickness_function_top,
#                 bot_thickness_function=fin_thickness_function_bot,
#                 leading_edge_width_function=leading_edge_width_function,
#                 rudder_length=0.3*fin_length, rudder_type='sharp')


# Generate
# ================
shuttle.generate()
