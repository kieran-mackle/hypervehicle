import numpy as np
from hypervehicle import Vehicle
from eilmer.geom.vector3 import Vector3
from eilmer.geom.path import Bezier, Line, Polyline
from scipy.optimize import bisect


# Create Vehicle instance
htv = Vehicle()
htv.configure(
    name="Hypersonic Technology Vehicle",
    verbosity=1,
    write_stl=True,
    cart3d=True,
    stl_filename="htv",
    mirror_fins=False,
)
htv.vehicle_angle = 5

# Parameters
# ====================
# Read parameter file to load geometry parameters
params = {}
with open("task_parameters.txt") as f:
    lines = f.readlines()

for line in lines:
    split_line = line.split(" = ")
    if len(split_line) != len([line]):
        # Line included a variable assignment, parse it
        variable_name = split_line[0]
        variable_value = float(split_line[1].split("#")[0])
        params[variable_name] = variable_value

L = params["body_length"]
body_width = params["wing_span"]
h = params["h"]
t_LE = 0.025
flap_angle = params["flap_angle"]

# Nominal parameters for proportional scaling
L_nom = 1
body_width_nom = 0.8
h_nom = 0.2
t_LE_nom = 0.025


# Create main body
# ================
A0 = Vector3(x=0, y=0)
A1 = Vector3(x=0.15 * L, y=0)
TT = Vector3(x=L, y=0)
B0 = Vector3(x=0, y=body_width / 2)

B1 = Vector3(x=A1.x, y=B0.y)
B0B1 = Line(p0=B0, p1=B1)
B1TT = Line(p0=B1, p1=TT)
Line_B0TT = Polyline([B0B1, B1TT])


def get_local_width(x):
    """Returns the vehicle width at a given x."""
    func = lambda t: Line_B0TT(t).x - x
    t = bisect(func, 0.0, 1.0)
    return 2 * Line_B0TT(t).y


def wing1_tf_top(x, y, z=0):
    local_width = get_local_width(x)

    if y < 0.5 * (local_width - 2 * t_LE):
        z_val = 0.5 * (h - t_LE) * np.cos(
            2 * np.pi * y / (local_width - 2 * t_LE)
        ) + 0.5 * (h + t_LE)
    elif y == 0.5 * (local_width - 2 * t_LE):
        z_val = t_LE
    else:
        z_val = t_LE

    # Apply axial tapering
    z_val *= (L - x) + 0.05

    return Vector3(x=0, y=0, z=-z_val)


def wing1_tf_bot(x, y, z=0):
    z_val = 0.2 * t_LE

    return Vector3(x=0, y=0, z=z_val)


def lewf(r):
    le_width = 0.01
    return le_width


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


# Add wing
htv.add_wing(
    A0=A0,
    A1=A1,
    TT=TT,
    B0=B0,
    Line_B0TT=Line_B0TT,
    top_tf=wing1_tf_top,
    bot_tf=wing1_tf_bot,
    LE_wf=lewf,
    stl_resolution=7,
    flap_length=0.15,
    flap_angle=np.deg2rad(flap_angle),
    curve_x=curv_x,
    curve_dx=curv_xd,
    curve_y=curv_y,
    curve_dy=curv_yd,
)


# Create flaps
# ================
length_scaler = L / L_nom
width_scaler = body_width / body_width_nom

flap_length = 0.2 * length_scaler
flap_gap = 0.01 * width_scaler

offset = 0.05 * t_LE

A0f = Vector3(x=flap_length, y=0 + 0.5 * flap_gap)
A1f = Vector3(x=flap_length + 0.05 * L, y=0 + 0.5 * flap_gap)
TTf = Vector3(x=flap_length + 0.1 * L, y=0 + 0.5 * flap_gap)
B0f = Vector3(x=flap_length, y=0.8 * body_width / 2 + 0.5 * flap_gap)

B1f = Vector3(x=A1f.x, y=0.5 * B0f.y)
B0B1f = Line(p0=B0f, p1=B1f)
B1TTf = Line(p0=B1f, p1=TTf)
Line_B0TTf = Polyline([B0B1f, B1TTf])


def wing2_tf_top(x, y, z=0):
    return Vector3(x=0, y=0, z=-0.7 * t_LE - offset)


def wing2_tf_bot(x, y, z=0):
    return Vector3(x=0, y=0, z=0.2 * t_LE - offset)


# htv.add_wing(A0=A0f, A1=A1f, TT=TTf, B0=B0f, Line_B0TT=Line_B0TTf,
#              top_tf=wing2_tf_top, bot_tf=wing2_tf_bot,
#              flap_length=flap_length,
#              flap_angle=np.deg2rad(flap_angle), mirror=False, close_wing=True,
#              mirror_new_component=False, stl_resolution=4,
#              curve_x=curv_x, curve_dx=curv_xd, curve_y=curv_y,
#              curve_dy=curv_yd)

htv.generate()
