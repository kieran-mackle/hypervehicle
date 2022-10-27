import numpy as np
from hypervehicle import Vehicle
from scipy.optimize import bisect
from gdtk.geom.vector3 import Vector3
from gdtk.geom.path import Bezier, Line, Polyline, Arc


params = {
    "R_b": 1,  # Body radius
    "L_b": 20,  # Body length
    "L_w": 16,  # Wing length
    "W_w": 5,  # Wing width,
    "flap_angle": 0,
}

wing_stl_res = 3
te_fin_stl_res = 3

# Initialise
d21 = Vehicle()
d21.configure(
    name="Lockhead D-21",
    verbosity=1,
    write_stl=True,
    stl_filename="D21",
    mirror_fins=False,
)

# Construct fuselage
# ------------------------
n = Vector3(x=0, y=0)
b0 = Vector3(x=0, y=params["R_b"])
b1 = b0 - Vector3(x=params["L_b"], y=0)
b2 = b1 - Vector3(x=0, y=params["R_b"])
bL0 = Line(n, b0)
bL1 = Line(b0, b1)
bL2 = Line(b1, b2)

fuseline = Polyline([bL0, bL1, bL2])
d21.add_fuselage(revolve_line=fuseline, stl_resolution=20)

# Construct nose
# --------------------------
nose_type = 1
if nose_type == 1:
    # Aerospike nose
    shift = Vector3(x=-0.1, y=0.0)
    tip = Vector3(x=0.15 * params["L_b"], y=0) + shift
    top = Vector3(x=0.0, y=1 * params["R_b"]) + shift
    mid_cp = Vector3(x=0.0, y=0.3 * params["R_b"]) + shift

    front_bz = Bezier([tip, mid_cp, top])
    back_cap = Line(p0=top, p1=Vector3(x=-0.5, y=0.0) + shift)

    noseline = Polyline([front_bz, back_cap])
    d21.add_fuselage(revolve_line=noseline, stl_resolution=20)

else:
    # Other
    pass


# Construct wings
# ------------------------
#    ---   |---B0------________________B1
#     |    |   |                       ----____
#     |    |   |                                ----_____
#     |    |   |                                         --____B2
#    ---   |---A0----------------------A1----------------------TT
#
#             |------------------- L_w ---------------------|

wing_thickness = 0.15 * params["R_b"]
wing_span = params["W_w"]
flap_length = 0.1 * params["L_w"]
flap_angle = 0
wing_LE_shift = Vector3(x=0.3, y=0)

# Required points
wing_tip = Vector3(x=0.0, y=0.0)  # Tip of wing reference point
A0 = wing_tip - Vector3(x=params["L_w"], y=0)
A1 = Vector3(x=A0.x + 0.6 * params["L_w"], y=0)
TT = Vector3(x=A0.x + params["L_w"], y=0) - wing_LE_shift
B0 = Vector3(x=A0.x, y=wing_span)

# Construction points
B1 = Vector3(x=A1.x, y=0.5 * B0.y)
B2 = Vector3(x=TT.x, y=0.15 * wing_span)

# Leading edge line
B0B1 = Line(p0=B0, p1=B1)
# B1B2 = Line(p0=B1, p1=B2)

# Create Bezier for front edge
a = B1
b = Vector3(x=0.5 * (B1.x + B2.x), y=B1.y)
c = B2
B1B2 = Bezier([a, b, c])

B2TT = Line(p0=B2, p1=TT)
Line_B0TT = Polyline([B0B1, B1B2, B2TT])


def local_ws(x):
    func = lambda t: Line_B0TT(t).x - x
    t = bisect(func, 0.0, 1.0)
    return Line_B0TT(t).y


def wing_tf_top(x, y, z=0):
    thickener = 0.5 * (1 - y / local_ws(x))
    return Vector3(x=0, y=0, z=-wing_thickness / 2 - thickener)


def wing_tf_bot(x, y, z=0):
    return Vector3(x=0, y=0, z=wing_thickness / 2)


d21.add_wing(
    A0=A0,
    A1=A1,
    TT=TT,
    B0=B0,
    Line_B0TT=Line_B0TT,
    top_tf=wing_tf_top,
    bot_tf=wing_tf_bot,
    flap_length=flap_length,
    flap_angle=np.deg2rad(params["flap_angle"]),
    stl_resolution=wing_stl_res,
)


# Construct tail
# ------------------------
#   |--p1-----p2
#   |  |         \
#   |  |           \
#   |  |             \
#   |--p0______________p3

fin_height = 0.15 * params["W_w"]
fin_thickness = wing_thickness
fin_length = params["L_b"] - params["L_w"]

p0 = b2
p1 = p0 + Vector3(x=0, y=fin_height)
p2 = p1 + Vector3(x=0.4 * fin_length, y=0)
p3 = p0 + Vector3(x=fin_length, y=0)

# Thickness functions
def fin_thickness_function_top(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=-fin_thickness / 2)


def fin_thickness_function_bot(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=fin_thickness / 2)


offset_func = lambda x, y, z: Vector3(x=0, y=0, z=-0.95 * params["R_b"])
# d21.add_fin(
#     p0=p0,
#     p1=p1,
#     p2=p2,
#     p3=p3,
#     offset_func=offset_func,
#     fin_thickness=fin_thickness,
#     fin_angle=np.deg2rad(-90),
#     top_thickness_function=fin_thickness_function_top,
#     bot_thickness_function=fin_thickness_function_bot,
#     stl_resolution=te_fin_stl_res,
# )

# Construct trailing wings
# ------------------------
# Straight wings at base of vehicle
# Construct as rectangle fins
fin_height = 0.15 * params["W_w"]
fin_thickness = wing_thickness
fin_length = params["L_b"] - params["L_w"]

te_wing_o = b2
p0 = te_wing_o
p1 = p0 + Vector3(x=0, y=fin_height)
p2 = p1 + Vector3(x=fin_length, y=0)
p3 = p0 + Vector3(x=fin_length, y=0)

# d21.add_fin(
#     p0=p0,
#     p1=p1,
#     p2=p2,
#     p3=p3,
#     fin_thickness=fin_thickness,
#     fin_angle=np.deg2rad(0),
#     top_thickness_function=fin_thickness_function_top,
#     bot_thickness_function=fin_thickness_function_bot,
#     stl_resolution=te_fin_stl_res,
# )

# d21.add_fin(
#     p0=p0,
#     p1=p1,
#     p2=p2,
#     p3=p3,
#     fin_thickness=fin_thickness,
#     fin_angle=np.deg2rad(180),
#     top_thickness_function=fin_thickness_function_top,
#     bot_thickness_function=fin_thickness_function_bot,
#     stl_resolution=te_fin_stl_res,
# )


# Generate STL's
# ------------------------
d21.generate()
