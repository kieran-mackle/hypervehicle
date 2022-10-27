import numpy as np
from hypervehicle import Vehicle, utils
from gdtk.geom.vector3 import Vector3
from gdtk.geom.path import Bezier, Line, Polyline, Arc


falcon9 = Vehicle()
falcon9.configure(
    name="SpaceX Falcon 9",
    verbosity=1,
    write_stl=True,
    stl_filename="falcon",
    cart3d=True,
)

D = 3.75
L_b = 60

# Inputs
h = 5 / 2
r_n = 0.75
r_o = 7

# Dependencies
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

# Fairing
f_L = 6
f0 = b_o
f1 = f0 - Vector3(f_L, 0)
fairing_line = Line(f0, f1)

# Body radius
r_b = D / 2

# Fairing to body
fb0 = f1
fb1 = Vector3(f1.x - 0.1 * f_L, r_b)
fb2 = Vector3(f1.x - 0.3 * f_L, 0)
fb_line = Polyline([Line(fb0, fb1), Line(fb1, fb2)])

# Fairing component
fairing = Polyline([nose_arc, ogive_arc, fairing_line, fb_line])
falcon9.add_fuselage(revolve_line=fairing, stl_resolution=100)

# Body
b00 = Vector3(f1.x, 0)
b0 = Vector3(f1.x, r_b)
b1 = b0 - Vector3(L_b, 0)
body_cap_line = Line(b00, b0)
body_line = Line(b0, b1)

# Base
bb0 = b1
bb1 = Vector3(bb0.x, 0)
base_line = Line(bb0, bb1)

# Join
line = Polyline([body_cap_line, body_line, base_line])

falcon9.add_fuselage(revolve_line=line, stl_resolution=50)
falcon9.generate()
