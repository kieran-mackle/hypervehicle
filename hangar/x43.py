import numpy as np
from hypervehicle import Vehicle
from gdtk.geom.vector3 import Vector3
from gdtk.geom.path import Bezier, Line, Polyline


# Geometric parameters
body_length = 3.7  # From body rear to nose
body_width = 1  # Body width
body_angle = 0  # Body slant angle (+ve nose down) (degrees)

wing_span = 0.4  # Span of each wing
wing_length = 1.00  # Wing length
wing_TE_back = 0.25
flap_length = 0.3  # Flap length

engine_h = 0.1  # Height of engine section

fin_height = 0.3  # Height of tail fin
fin_length = 1.0  # Length of fin
rudder_length = 0.2  # Length of tail fin rudder

# Configuration parameters
flap_angle = 0  # Elevon angle (+ve up) (degrees)
rudder_angle = 0  # Rudder angle (+ve to +y) (degrees)


# Create Vehicle instance
x43 = Vehicle()
x43.configure(
    name="X-43A", verbosity=1, write_stl=True, stl_resolution=5, stl_filename="x43"
)


#####################################################
##                  WING GEOMETRY                  ##
#####################################################

# WING 1
#     B0------------------------ B1
#     |                             ---___
#     |                                    ---__ B2
#     |                                           |
#     A0-------------------------A1---------------TT

# Define nominal parameters
L_nom = 3.7

L = body_length
beta = np.deg2rad(body_angle)
LE_height = 0.05 * L / L_nom
TE_height = 0.2 * L / L_nom

A0 = Vector3(x=0, y=0)
A1 = Vector3(x=1.8 * L / L_nom, y=0)
TT = Vector3(x=L, y=0)
B0 = Vector3(x=0, y=body_width / 2)
B1 = Vector3(x=A1.x, y=B0.y)
B2 = Vector3(x=TT.x, y=0.4 * B0.y)

B0B1 = Line(p0=B0, p1=B1)
B1B2 = Line(p0=B1, p1=B2)
B2TT = Line(p0=B2, p1=TT)

Line_B0TT = Polyline([B0B1, B1B2, B2TT])

# Top rounding Bezier points
rounding_thickness = 0.025
p1 = Vector3(x=0, y=0, z=rounding_thickness)
p2 = Vector3(x=0, y=0.9 * B0.y, z=p1.z)
p3 = Vector3(x=0, y=B0.y, z=0)
rounding_bez = Bezier([p1, p2, p3])


def get_local_y(x):
    if x < B1.x:
        local_y = B0.y
    else:
        local_y = (x - B2.x) * (B2.y - B1.y) / (B2.x - B1.x) + B2.y
    return local_y


def wing1_tf_top(x, y, z=0):
    # Nominal thickness
    z_m = -L * np.tan(beta)
    z_u = z_m - 0.5 * TE_height
    beta_u = -np.arctan((z_u + 0.5 * LE_height) / L)
    z_1 = (x - L) * np.tan(beta_u) - LE_height / 2

    # Rounding thickness
    local_y = get_local_y(x)
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


# Add wing to x43 vehicle
x43.add_wing(
    A0=A0,
    A1=A1,
    TT=TT,
    B0=B0,
    Line_B0TT=Line_B0TT,
    top_tf=wing1_tf_top,
    bot_tf=wing1_tf_bot,
    LE_wf=leading_edge_width_function,
)


# WING 2 (flaps)
# --------------

#  fB0--__fB1
#   \      \_
#    \       \
#     B0-----fTT----------------- B1
#     |                             ---___
#     |                                    ---__ B2
#     |                                           |
#     A0-------------------------A1---------------TT

flap_thickness = 0.03 * L / L_nom
fflap_angle = np.deg2rad(-flap_angle)

fA0 = Vector3(x=B0.x + flap_length, y=B0.y)
fA1 = Vector3(x=A0.x + 0.5 * wing_length, y=get_local_y(A0.x + 0.5 * wing_length))
fTT = Vector3(x=A0.x + wing_length, y=get_local_y(A0.x + wing_length))

fB0 = Vector3(x=A0.x - wing_TE_back + flap_length, y=B0.y + wing_span)
fB1 = Vector3(x=A0.x + 0.4 * wing_length, y=B0.y + 0.75 * wing_span)

fB0B1 = Line(p0=fB0, p1=fB1)
fB1TT = Line(p0=fB1, p1=fTT)

Line_fB0TT = Polyline([fB0B1, fB1TT])


def wing2_tf_top(x, y, z=0):
    z_ba = -(x - L) * np.tan(beta)
    return Vector3(x=0, y=0, z=-flap_thickness / 2 - z_ba)


def wing2_tf_bot(x, y, z=0):
    z_ba = -(x - L) * np.tan(beta)
    return Vector3(x=0, y=0, z=flap_thickness / 2 - z_ba)


x43.add_wing(
    A0=fA0,
    A1=fA1,
    TT=fTT,
    B0=fB0,
    Line_B0TT=Line_fB0TT,
    top_tf=wing2_tf_top,
    bot_tf=wing2_tf_bot,
    flap_length=flap_length,
    flap_angle=flap_angle,
    LE_wf=leading_edge_width_function,
)


# WING 3 (Inlet/exit)
#     B0------------------------ B1
#     |                            --__
#     |                                 --_ B2
#     |                                       |
#     A0-------------------------A1-----------TT
iA0 = A0
iA1 = A1
iTT = Vector3(x=0.5 * (TT.x + B1.x), y=0)
iB0 = Vector3(x=B0.x, y=B0.y)
iB1 = Vector3(x=B1.x, y=B1.y)
iB2 = Vector3(x=iTT.x, y=((iTT.x - B2.x) * (B2.y - B1.y) / (B2.x - B1.x) + B2.y))

iB0B1 = Line(p0=iB0, p1=iB1)
iB1B2 = Line(p0=iB1, p1=iB2)
iB2TT = Line(p0=iB2, p1=iTT)

Line_iB0TT = Polyline([iB0B1, iB1B2, iB2TT])

inlet_start = 0.8 * B1.x
exit_start = 0.3 * (B1.x - B0.x)


def inlet_tf_top(x, y, z=0):
    vehicle_z = wing1_tf_bot(x, y, z).z
    return Vector3(x=0, y=0, z=vehicle_z - 0.05)


def inlet_tf_bot(x, y, z=0):

    # Get z-location of vehicle body
    vehicle_body = wing1_tf_bot(x, y, z).z

    if x < exit_start:
        z_val = (engine_h / (exit_start - A0.x)) * x
    elif x > inlet_start:
        z_val = (-engine_h / (iTT.x - inlet_start)) * (x - iTT.x)
    else:
        z_val = engine_h

    # Calculate taper z
    taper_start_pc = 0.8  # Percentage of y_local where taper begins
    y_local = get_local_y(x)
    if y > taper_start_pc * y_local:
        z_taper = (5 * y / y_local - 4) * z_val
    else:
        z_taper = 0

    return Vector3(x=0, y=0, z=z_val + vehicle_body - z_taper + 0.00001)


def leading_edge_width_function2(r):
    temp = Bezier(
        [Vector3(x=0.0, y=0.001), Vector3(x=0.5, y=0.001), Vector3(x=1.0, y=0.001)]
    )
    le_width = temp(r).y
    return le_width


x43.add_wing(
    A0=iA0,
    A1=iA1,
    TT=iTT,
    B0=iB0,
    Line_B0TT=Line_iB0TT,
    top_tf=inlet_tf_top,
    bot_tf=inlet_tf_bot,
    LE_wf=leading_edge_width_function2,
)


#####################################################
##                     FINS                        ##
#####################################################
#   |--p1-----p2
#   |  |         \
#   |  |           \
#   |  |             \
#   |--p0______________p3

fin_p3x = fin_length
fin_thickness = 0.02 * L / L_nom
fin_offset = -(fin_p3x - L) * np.tan(beta)  # offset upwards
y_shift = B0.y

p0 = Vector3(x=rudder_length, y=fin_offset)
p1 = Vector3(x=rudder_length, y=fin_offset + fin_height)
p2 = Vector3(x=0.5 * fin_p3x, y=fin_offset + fin_height)
p3 = Vector3(x=fin_p3x, y=fin_offset)

# Construct p1p3 path
p1p2 = Line(p1, p2)
p2p3 = Line(p2, p3)
p1p3 = Polyline(segments=[p1p2, p2p3])

# Thickness functions - constants
def fin_thickness_function_top(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=-fin_thickness / 2)


def fin_thickness_function_bot(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=fin_thickness / 2)


def fin_offset_function(x, y, z):
    return Vector3(x=0, y=y_shift, z=0)


# Define the fin angles: this is the angle to rotate the fin, constructed in
# the x-y plane, to its final position, by rotating about the x axis
fin_angle = np.deg2rad(-90)

# Rudder angle (fin flap)
rudder_angle = np.deg2rad(rudder_angle)

fin_geom_dict = {
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
    "rudder_type": "sharp",  # options ['flat', 'sharp']
    "rudder_length": rudder_length,
    "rudder_angle": rudder_angle,
    "pivot_angle": 0,
    "pivot_point": Vector3(x=0, y=0),
    "offset_function": fin_offset_function,
}

# Make second fin to account for rudder angle
def fin_offset_function2(x, y, z):
    return Vector3(x=0, y=-y_shift, z=0)


fin2_geom_dict = fin_geom_dict.copy()
fin2_geom_dict["offset_function"] = fin_offset_function2
fin2_geom_dict["FIN_ANGLE"] = fin_geom_dict["FIN_ANGLE"]

x43.add_component("fin", fin_geom_dict)
x43.add_component("fin", fin2_geom_dict)

# Generate stl
x43.generate()
