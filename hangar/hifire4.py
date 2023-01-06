import numpy as np
from hypervehicle import Vehicle
from hypervehicle.components import Wing, Fin, Fuselage, common
from hypervehicle.geometry import Vector3, Bezier, Line, Polyline, Arc


params = {"x_curve_mult": 0.3, "y_curve_mult": 0.5}

# Initialise
hifire4 = Vehicle()
hifire4.configure(
    name="HIFiRE4",
    verbosity=1,
)

R = 0.2
L_n = 2
L_b = 4
L_t = L_n + L_b


# Construct tangent ogive nose
# --------------------------------------------------
rho = (R**2 + L_n**2) / (2 * R)
y_o = lambda x: round(np.sqrt(rho**2 - (L_n - x) ** 2) + R - rho, 6)
a_o = Vector3(x=0, y=0)
b_o = Vector3(x=-L_n, y=y_o(L_n))
c_o = Vector3(x=b_o.x, y=b_o.y - rho)
ogive_arc = Arc(a_o, b_o, c_o)

# Construct body
b0 = b_o
b1 = b0 - Vector3(x=L_b, y=0)
b2 = b1 - Vector3(x=0, y=R)
bL1 = Line(b0, b1)
bL2 = Line(b1, b2)

# Construct fuselage Polyline
fuseline = Polyline([ogive_arc, bL1, bL2])

# Construct curvature functions
def curv_x(x, y):
    "Curvature in x-direction (about y-axis)"
    return params["x_curve_mult"] * 0.05 * x**2


def curv_xd(x, y):
    return params["x_curve_mult"] * 0.05 * 2 * x


def curv_y(x, y):
    "Curvature in y-direction (about x-axis)"
    return params["y_curve_mult"] * 0.1 * y**2


def curv_yd(x, y):
    return params["y_curve_mult"] * 0.1 * 2 * y


# Create curvature operations chain
curvatures = [("x", curv_x, curv_xd), ("y", curv_y, curv_yd)]

fuselage = Fuselage(
    revolve_line=fuseline,
    x_curve_func=curv_x,
    x_dash_func=curv_xd,
    y_curve_func=curv_y,
    y_dash_func=curv_yd,
    stl_resolution=20,
)


# Create wing body
# --------------------------------------------------
#    ---   |---B0------________________B1
#     |    |   |                       ----____
#     |    |   |                                ----_____
#     |    |   |                                         --____B2
#    ---   |---A0----------------------A1----------------------TT
#
#             |------------------- wing_L ---------------------|

wing_L = 0.8 * L_t
wing_thickness = 0.25 * R
wing_span = 3.5 * R
flap_length = 0.1 * L_t
flap_angle = 0
wing_LE_shift = Vector3(x=0.3, y=0)

# Required points
A0 = a_o - Vector3(x=wing_L, y=0)
A1 = Vector3(x=A0.x + 0.5 * wing_L, y=0)
TT = Vector3(x=A0.x + wing_L, y=0) - wing_LE_shift
B0 = Vector3(x=A0.x, y=wing_span)

# Construction points
B1 = Vector3(x=A1.x, y=0.95 * B0.y)
B2 = Vector3(x=TT.x, y=0.15 * wing_span)

# Leading edge line
B0B1 = Line(p0=B0, p1=B1)
B1B2 = Line(p0=B1, p1=B2)
B2TT = Line(p0=B2, p1=TT)
Line_B0TT = Polyline([B0B1, B1B2, B2TT])

wing = Wing(
    A0=A0,
    A1=A1,
    TT=TT,
    B0=B0,
    Line_B0TT=Line_B0TT,
    top_tf=common.uniform_thickness_function(wing_thickness, "top"),
    bot_tf=common.uniform_thickness_function(wing_thickness, "bot"),
    curve_x=curv_x,
    curve_dx=curv_xd,
    curve_y=curv_y,
    curve_dy=curv_yd,
    flap_length=flap_length,
    flap_angle=np.deg2rad(flap_angle),
    stl_resolution=3,
)


# Create fins
# --------------------------------------------------
#   |--p1-----p2
#   |  |         \
#   |  |           \
#   |  |             \
#   |--p0______________p3

fin_height = R
fin_thickness = wing_thickness
fin_length = 0.2 * wing_L

p0 = A0 - Vector3(x=flap_length, y=0)
p1 = p0 + Vector3(x=0, y=fin_height)
p2 = p1 + Vector3(x=0.5 * fin_length, y=-0.5 * fin_height)
p3 = p0 + Vector3(x=fin_length, y=0)

# Modifty each point for curvature
p0 += Vector3(x=0, y=-curv_x(p0.x, p0.y))
p1 += Vector3(x=0, y=-curv_x(p1.x, p1.y))
p2 += Vector3(x=0, y=-curv_x(p2.x, p2.y))
p3 += Vector3(x=0, y=-curv_x(p3.x, p3.y))

# Add fins
offset1 = lambda x, y, z: Vector3(x=0, y=wing_span)
offset2 = lambda x, y, z: Vector3(x=0, y=-wing_span)
offset3 = lambda x, y, z: Vector3(x=0, y=0, z=-0.95 * R)
stl_res = 3
fin1 = Fin(
    p0=p0,
    p1=p1,
    p2=p2,
    p3=p3,
    offset_func=offset1,
    fin_thickness=fin_thickness,
    fin_angle=np.deg2rad(-90),
    top_thickness_function=common.uniform_thickness_function(fin_thickness, "top"),
    bot_thickness_function=common.uniform_thickness_function(fin_thickness, "bot"),
    rudder_type="sharp",
    rudder_length=fin_thickness,
    stl_resolution=stl_res,
)
fin2 = Fin(
    p0=p0,
    p1=p1,
    p2=p2,
    p3=p3,
    offset_func=offset2,
    fin_thickness=fin_thickness,
    fin_angle=np.deg2rad(-90),
    top_thickness_function=common.uniform_thickness_function(fin_thickness, "top"),
    bot_thickness_function=common.uniform_thickness_function(fin_thickness, "bot"),
    rudder_type="sharp",
    rudder_length=fin_thickness,
    stl_resolution=stl_res,
)
fin3 = Fin(
    p0=p0,
    p1=p1,
    p2=p2,
    p3=p3,
    offset_func=offset3,
    fin_thickness=fin_thickness,
    fin_angle=np.deg2rad(-90),
    top_thickness_function=common.uniform_thickness_function(fin_thickness, "top"),
    bot_thickness_function=common.uniform_thickness_function(fin_thickness, "bot"),
    rudder_type="sharp",
    rudder_length=fin_thickness,
    stl_resolution=stl_res,
)

# Add components
hifire4.add_component(
    fuselage,
    curvatures=curvatures,
)
hifire4.add_component(
    wing,
    reflection_axis="y",
    curvatures=curvatures,
)
hifire4.add_component(fin1)
hifire4.add_component(fin2)
hifire4.add_component(fin3)

# Generate
hifire4.generate()
hifire4.to_stl("hifire4")
