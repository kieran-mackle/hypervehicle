import numpy as np
from hypervehicle import Vehicle
from hypervehicle.geometry import Vector3, Bezier, Line, Polyline, Arc

from hypervehicle.components import Wing, Fuselage, Fin


refex = Vehicle()
refex.configure(
    name="DLR ReFEX",
    verbosity=1,
    # write_stl=True,
    # stl_filename="refex",
    # mirror_fins=False,
    # cart3d=True,
)

# Inputs
h = 2.5  # Ogive height ??
r_n = 0.75  # Nose radius
r_o = 10  # Ogive radius
L_o = 0.5
L_b = 22  # Body length
canard_angle = 0  # degrees

# Ogive Dependencies
x_o = -np.sqrt((r_o - r_n) ** 2 - (r_o - h) ** 2)
y_t = r_n * (r_o - h) / (r_o - r_n)
x_t = x_o - np.sqrt(r_n**2 - y_t**2)
x_a = x_o - r_n

# Ogive arc
a_o = Vector3(-x_t, y_t)
b_o = Vector3(0, h)
c_o = Vector3(0, -r_o + h)
ogive_arc = Arc(a_o, b_o, c_o)

# Nose arc
a_n = Vector3(-x_a, 0)
b_n = a_o
c_n = Vector3(-x_o, 0)
nose_arc = Arc(a_n, b_n, c_n)

# Nose body
f0 = b_o
f1 = f0 - Vector3(L_o, 0)
fairing_line = Line(f0, f1)

# Nose body base
fb0 = f1
fb1 = Vector3(f1.x, 0)
fb_line = Line(fb0, fb1)

# Nose component
fairing = Polyline([nose_arc, ogive_arc, fairing_line, fb_line])
fairing_fuse = Fuselage.legacy(revolve_line=fairing)
fairing_fuse.stl_resolution = 50
refex.add_component(fairing_fuse)

# Vehicle body
b00 = Vector3(0, 0)
b0 = Vector3(0, f1.y)
b1 = b0 - Vector3(L_b, 0)
body_cap_line = Line(b00, b0)
body_top_line = Line(b0, b1)
bb0 = b1  # Bottom outside of body
bb1 = Vector3(bb0.x, 0)  # Body base axis point
base_line = Line(bb0, bb1)

body_line = Polyline([body_cap_line, body_top_line, base_line])
body_fuse = Fuselage.legacy(revolve_line=body_line)
body_fuse.stl_resolution = 50
refex.add_component(body_fuse)

# Cannards
#   p1-----p2
#    \        \
#     \         \
#      \          \
#       p0_________p3

fin_height = 1.5 * h
fin_thickness = 0.2
fin_length = 1 * fin_height
shift_in = Vector3(x=0, y=-0.02 * fin_height)

p0 = f1 - Vector3(x=fin_length, y=0) + shift_in
p1 = p0 + Vector3(x=0.05 * fin_length, y=fin_height) + shift_in
p2 = p1 + Vector3(x=0.3 * fin_length, y=0) + shift_in
p3 = f1 + shift_in
pivot_point = Vector3(x=0.5 * (p0.x + p3.x), y=p0.y)

# Thickness functions
def fin_thickness_function_top(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=-fin_thickness / 2)


def fin_thickness_function_bot(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=fin_thickness / 2)


def leading_edge_width_function(r):
    temp = Bezier(
        [Vector3(x=0.0, y=0.02), Vector3(x=0.75, y=0.1), Vector3(x=1.0, y=0.3)]
    )
    le_width = temp(r).y
    return le_width


# Add canards
for i in range(2):
    angle = np.deg2rad((i / 2) * 360)
    fin = Fin.legacy(
        p0=p0,
        p1=p1,
        p2=p2,
        p3=p3,
        fin_thickness=fin_thickness,
        fin_angle=angle,
        top_thickness_function=fin_thickness_function_top,
        bot_thickness_function=fin_thickness_function_bot,
        LE_func=leading_edge_width_function,
        pivot_angle=np.deg2rad(canard_angle),
        pivot_point=pivot_point,
        rudder_type="sharp",
        rudder_length=fin_thickness,
    )
    fin.stl_resolution = 3
    refex.add_component(fin)

# Wings
wing_thickness = 2 * fin_thickness
wing_length = 10
wing_span = 6
flap_length = 0.1 * wing_length

A0 = Vector3(x=bb0.x + flap_length, y=0)
TT = A0 + Vector3(x=wing_length, y=0)
A1 = Vector3(x=0.5 * (A0.x + TT.x), y=0)

B0 = A0 + Vector3(x=0, y=wing_span + h)
B1 = B0 + Vector3(x=0.4 * wing_length - flap_length, y=0)

B0B1 = Line(p0=B0, p1=B1)
B1TT = Line(p0=B1, p1=TT)

Line_B0TT = Polyline([B0B1, B1TT])


def wing2_tf_top(x, y, z=0):
    return Vector3(x=0, y=0, z=h - wing_thickness)


def wing2_tf_bot(x, y, z=0):
    return Vector3(x=0, y=0, z=h)


wing = Wing.legacy(
    A0=A0,
    A1=A1,
    TT=TT,
    B0=B0,
    Line_B0TT=Line_B0TT,
    top_tf=wing2_tf_top,
    bot_tf=wing2_tf_bot,
    LE_wf=leading_edge_width_function,
    flap_length=flap_length,
)
wing.stl_resolution = 3
refex.add_component(wing)

# Tail rudder/fin
tail_height = 4
tail_thickness = 0.2
tail_length = 1.2 * tail_height
rudder_length = tail_thickness
shift_in = Vector3(x=0, y=-0.02 * tail_height)

t0 = bb0 + shift_in + Vector3(x=2 * rudder_length, y=0)
t1 = t0 + Vector3(x=0, y=tail_height) + shift_in
t2 = t1 + Vector3(x=0.4 * tail_length, y=0) + shift_in
t3 = t0 + Vector3(x=tail_length, y=0) + shift_in

# Thickness functions
def tail_thickness_function_top(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=-tail_thickness / 2)


def tail_thickness_function_bot(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=tail_thickness / 2)


# Add tail fin
tail = Fin.legacy(
    p0=t0,
    p1=t1,
    p2=t2,
    p3=t3,
    fin_thickness=tail_thickness,
    fin_angle=np.deg2rad(-90),
    top_thickness_function=tail_thickness_function_top,
    bot_thickness_function=tail_thickness_function_bot,
    LE_func=leading_edge_width_function,
    rudder_type="sharp",
    rudder_length=rudder_length,
)
tail.stl_resolution = 3
refex.add_component(tail)


# Generate Vehicle
refex.generate()

for i, component in enumerate(refex.components):
    # Need to specify stl res for each component
    component.to_stl(outfile=f"{i}.stl")
