import numpy as np
from hypervehicle import Vehicle
from hypervehicle.geometry import Vector3, Bezier
from hypervehicle.components import Fuselage, Fin, common


rocket = Vehicle()
rocket.configure(name="rocket", verbosity=1)

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

body = Fuselage(Xn=Xn, X1=X1, X2=X2, X3=X3, R1=R1, R2=R2, R3=R3)
rocket.add_component(body)


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

# Add fins around body
for i in range(no_fins):
    angle = np.deg2rad((i / no_fins) * 360)
    fin = Fin(
        p0=p0,
        p1=p1,
        p2=p2,
        p3=p3,
        fin_thickness=fin_thickness,
        fin_angle=angle,
        top_thickness_function=common.uniform_thickness_function(fin_thickness, "top"),
        bot_thickness_function=common.uniform_thickness_function(fin_thickness, "bot"),
    )
    rocket.add_component(fin)


# Generate geometry
# ====================
rocket.generate()
rocket.to_stl("rocket")
