import numpy as np
from hypervehicle import Vehicle
from eilmer.geom.vector3 import Vector3
from eilmer.geom.path import Bezier, Line, Polyline


# Instantiate hypervehicle and add all components
hifire8 = Vehicle()
hifire8.configure(
    name="HIFiRE-8",
    verbosity=1,
    write_stl=True,
    stl_resolution=5,
    stl_filename="hifire8",
    mirror_fins=False,
)


L = 1.2

# Construct fuselage
# ====================
R_base = 0.175
R_nose = 0.1
nozzle_length = 0.1

fuse_slope = np.arctan((R_base - R_nose) / L)

Xn = L
X1 = 1 * L
X2 = 0.0
X3 = -nozzle_length  # Nozzle length

Rn = R_nose
R1 = R_base - X1 * np.tan(fuse_slope)
R2 = R_base - X2 * np.tan(fuse_slope)
R3 = R2  # R_base - X3*np.tan(fuse_slope)

hifire8.add_fuselage(Xn=Xn, X1=X1, X2=X2, X3=X3, Rn=Rn, R1=R1, R2=R2, R3=R3)


# Construct fins
# ====================
#   |--p1-----p2
#   |  |         \
#   |  |           \
#   |  |             \
#   |--p0______________p3

fin_length = 0.4  # length from p0 to p3
fin_height = 0.4 * fin_length
fin_thickness = 0.02 * fin_length
fin_angle = np.deg2rad(90 - 45)


def eval_r_at_x(x):
    """Returns the R value at a given x."""
    return R_base - x * np.tan(fuse_slope)


p0 = Vector3(x=0, y=eval_r_at_x(0))
p1 = Vector3(x=0.2 * fin_length, y=eval_r_at_x(0.2 * fin_length) + fin_height)
p2 = Vector3(x=0.7 * fin_length, y=eval_r_at_x(0.7 * fin_length) + fin_height)
p3 = Vector3(x=fin_length, y=eval_r_at_x(fin_length))

# Construct p1p3 path ?
p1p2 = Line(p1, p2)
p2p3 = Line(p2, p3)
p1p3 = Polyline(segments=[p1p2, p2p3])

# Thickness functions - constants
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


fin1_geom_dict = {
    "p0": p0,
    "p1": p1,
    "p2": p2,
    "p3": p3,
    "FIN_THICKNESS": fin_thickness,
    "FIN_ANGLE": fin_angle,
    "FIN_TOP_THICKNESS_FUNC": fin_thickness_function_top,
    "FIN_BOTTOM_THICKNESS_FUNC": fin_thickness_function_bot,
    "FIN_LEADING_EDGE_FUNC": leading_edge_width_function,
    "MIRROR_NEW_COMPONENT": False,
    "rudder_type": "flat",
    "rudder_length": 0,
    "rudder_angle": 0,
    "pivot_angle": 0,
    "pivot_point": Vector3(x=0, y=0),
}

# Copy for second fin
fin2_geom_dict = fin1_geom_dict.copy()
fin2_geom_dict["FIN_ANGLE"] = fin1_geom_dict["FIN_ANGLE"] + np.deg2rad(90)

# Copy for third fin - this should be improved. Maybe
# make a fin geom func and modify fin height
fin3_geom_dict = fin1_geom_dict.copy()
fin3_geom_dict["FIN_ANGLE"] = np.deg2rad(-90)
fin3_geom_dict["p1"] = 0.5 * fin1_geom_dict["p1"]
fin3_geom_dict["p2"] = 0.5 * fin1_geom_dict["p2"]

# Add fin dicts
hifire8.add_component("fin", fin1_geom_dict)
hifire8.add_component("fin", fin2_geom_dict)
hifire8.add_component("fin", fin3_geom_dict)


# Construct wing
# ====================

# |---B0-------B1__
# |   |            ----____
# |   |                    ----_____
# |   |                              B2-----------____B3
# |---A0-----------------------------A1---------------TT
#
#     |------------------- wing_L ---------------------|

wing_L = 1.3
wing_thickness = 0.04
wing_width = 0.4
wing_origin = 0.2
flap_length = 0.2

# Required points
A0 = Vector3(x=wing_origin, y=0)
A1 = Vector3(x=A0.x + 0.5 * wing_L, y=0)
TT = Vector3(x=A0.x + wing_L, y=0)
B0 = Vector3(x=A0.x, y=wing_width)

# Construction points
B1 = Vector3(x=A0.x + 0.1 * wing_L, y=B0.y)
B2 = Vector3(x=A1.x, y=0.5 * wing_width)
B3 = Vector3(x=TT.x, y=0.13 * wing_width)

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


hifire8.add_wing(
    A0=A0,
    A1=A1,
    TT=TT,
    B0=B0,
    Line_B0TT=Line_B0TT,
    top_tf=wing_tf_top,
    bot_tf=wing_tf_bot,
    LE_wf=leading_edge_width_function,
    flap_length=flap_length,
)


# Construct nose cap
# ====================
L_n = 0.5
r_b = eval_r_at_x(Xn)
t_n = 0.005
w_n = 0.15

# Side width angle
side_theta = np.arctan((r_b - 0.5 * w_n) / L_n)

# Points
A0n = Vector3(x=Xn, y=0)
A1n = Vector3(x=Xn + 0.5 * L_n, y=0)
TTn = Vector3(x=Xn + L_n, y=0)

B0n = Vector3(x=A0n.x, y=r_b)
B1n = Vector3(x=A1n.x, y=r_b - (A1n.x - Xn) * np.tan(side_theta))
B2n = Vector3(x=TTn.x, y=0.5 * w_n)

B0B1n = Line(p0=B0n, p1=B1n)
B1B2n = Line(p0=B1n, p1=B2n)
B2TTn = Line(p0=B2n, p1=TTn)

Line_B0TTn = Polyline([B0B1n, B1B2n, B2TTn])


def nose_tf_top(x, y, z=0):
    # Calculate function weightings for linear blending
    circular_weighting = 1 - (1 / L_n) * (x - Xn)
    flat_weighting = 1 - circular_weighting

    # Calculate functions at x
    z_circle = np.sqrt(r_b**2 - y**2)
    z_flat = t_n
    # z_flat = t_n*np.sqrt(w_n - y**2)

    # Weighted sum of function contributions
    z = circular_weighting * z_circle + flat_weighting * z_flat
    # z = t_n
    return Vector3(x=0, y=0, z=-z)


def nose_tf_bot(x, y, z=0):
    # Calculate function weightings for linear blending
    circular_weighting = 1 - (1 / L_n) * (x - Xn)
    flat_weighting = 1 - circular_weighting

    # Calculate functions at x
    z_circle = np.sqrt(r_b**2 - y**2)
    z_flat = t_n
    # z_flat = np.sqrt(w_n - y**2)

    # Weighted sum of function contributions
    z = circular_weighting * z_circle + flat_weighting * z_flat
    # z = t_n
    return Vector3(x=0, y=0, z=z)


hifire8.add_wing(
    A0=A0n,
    A1=A1n,
    TT=TTn,
    B0=B0n,
    Line_B0TT=Line_B0TTn,
    top_tf=nose_tf_top,
    bot_tf=nose_tf_bot,
    LE_wf=leading_edge_width_function,
    LE_type="FLAT",
)

# Generate STL files
hifire8.generate()
