import numpy as np
from hypervehicle import Vehicle
from hypervehicle.transformations import CART3D
from hypervehicle.components import Wing, Fuselage, Fin
from hypervehicle.geometry import Vector3, Bezier, Line, Polyline, Arc
from hypervehicle.components.common import (
    uniform_thickness_function,
    leading_edge_width_function,
    OgiveNose,
)


refex = Vehicle()
refex.configure(name="DLR ReFEX", verbosity=1)

# Inputs
h = 2.5  # Ogive height ??
r_n = 0.75  # Nose radius
r_o = 10  # Ogive radius
L_o = 0.5
L_b = 22  # Body length
canard_angle = 0  # degrees

# Ogive nose
nose = OgiveNose(h=h, r_n=r_n, r_o=r_o, L_o=L_o, stl_resolution=50)
refex.add_component(nose)

# Vehicle body
# TODO - need to improve locating nose to avoid below
f1 = Vector3(0, h) - Vector3(L_o, 0)
b00 = Vector3(0, 0)
b0 = Vector3(0, f1.y)
b1 = b0 - Vector3(L_b, 0)
body_cap_line = Line(b00, b0)
body_top_line = Line(b0, b1)
bb0 = b1  # Bottom outside of body
bb1 = Vector3(bb0.x, 0)  # Body base axis point
base_line = Line(bb0, bb1)

body_line = Polyline([body_cap_line, body_top_line, base_line])
body_fuse = Fuselage(revolve_line=body_line, stl_resolution=50)
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

# Add canards
for i in range(2):
    angle = np.deg2rad((i / 2) * 360)
    fin = Fin(
        p0=p0,
        p1=p1,
        p2=p2,
        p3=p3,
        fin_thickness=fin_thickness,
        fin_angle=angle,
        top_thickness_function=uniform_thickness_function(fin_thickness, "top"),
        bot_thickness_function=uniform_thickness_function(fin_thickness, "bot"),
        LE_func=leading_edge_width_function,
        pivot_angle=np.deg2rad(canard_angle),
        pivot_point=pivot_point,
        rudder_type="sharp",
        rudder_length=fin_thickness,
        stl_resolution=3,
    )
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


wing = Wing(
    A0=A0,
    A1=A1,
    TT=TT,
    B0=B0,
    Line_B0TT=Line_B0TT,
    top_tf=wing2_tf_top,
    bot_tf=wing2_tf_bot,
    LE_wf=leading_edge_width_function,
    flap_length=flap_length,
    stl_resolution=3,
)
wing._reflect = True
# refex.add_component(wing)

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

# Add tail fin
tail = Fin(
    p0=t0,
    p1=t1,
    p2=t2,
    p3=t3,
    fin_thickness=tail_thickness,
    fin_angle=np.deg2rad(-90),
    top_thickness_function=uniform_thickness_function(tail_thickness, "top"),
    bot_thickness_function=uniform_thickness_function(tail_thickness, "bot"),
    LE_func=leading_edge_width_function,
    rudder_type="sharp",
    rudder_length=rudder_length,
    stl_resolution=3,
)
# refex.add_component(tail)

# Generate Vehicle
refex.generate()
refex.transform(transformations=CART3D)
refex.to_stl(prefix="refex")
