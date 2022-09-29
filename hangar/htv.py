import numpy as np
from hypervehicle import Vehicle
from eilmer.geom.vector3 import Vector3
from eilmer.geom.path import Bezier, Line, Polyline
from scipy.optimize import bisect


# Create Vehicle instance
htv = Vehicle()
htv.configure(
    name='Hypersonic Technology Vehicle', 
    verbosity=1, 
    write_stl=True,
    stl_resolution=15, 
    stl_filename='htv',
    mirror_fins=False,)

# Parameters
# ====================
L = 1
body_width = 0.5
h = 0.1
t_LE = 0.01
flap_angle = 15

# Nominal parameters for proportional scaling
L_nom = 1
body_width_nom = 0.8
h_nom = 0.2
t_LE_nom = 0.025


# Create main body
# ================
A0 = Vector3(x=0, y=0)
A1 = Vector3(x=0.15*L, y=0)
TT = Vector3(x=L, y=0)
B0 = Vector3(x=0, y=body_width/2)

B1 = Vector3(x=A1.x, y=B0.y)
B0B1 = Line(p0=B0, p1=B1)
B1TT = Line(p0=B1, p1=TT)
Line_B0TT = Polyline([B0B1, B1TT])

def get_local_width(x):
    """Returns the vehicle width at a given x."""
    func = lambda t: Line_B0TT(t).x - x
    t = bisect(func, 0., 1.)
    return 2*Line_B0TT(t).y

def wing1_tf_top(x,y,z=0):
    local_width = get_local_width(x)

    if y < 0.5*(local_width - 2*t_LE):
        z_val = 0.5*(h-t_LE)*np.cos(2*np.pi*y/(local_width-2*t_LE)) + 0.5*(h+t_LE)
    elif y == 0.5*(local_width - 2*t_LE):
        z_val = t_LE
    else:
        z_val = np.sqrt(round(t_LE**2 - (y-0.5*local_width + t_LE)**2,6))

    # Apply axial tapering
    z_val *= (L-x)+0.05

    return Vector3(x=0, y=0, z=-z_val)

def wing1_tf_bot(x,y,z=0):
    local_width = get_local_width(x)

    if y < 0.5*(local_width - 2*t_LE):
        z_val = t_LE
    else:
        z_val = np.sqrt(round(t_LE**2 - (y-0.5*local_width + t_LE)**2,6))

    return Vector3(x=0, y=0, z=z_val)

# Add wing
htv.add_wing(A0=A0, A1=A1, TT=TT, B0=B0, Line_B0TT=Line_B0TT,
             top_tf=wing1_tf_top, bot_tf=wing1_tf_bot,
             LE_type='flat')


# Create flaps
# ================
length_scaler = L/L_nom
width_scaler = body_width/body_width_nom

flap_length = 0.2 * length_scaler
flap_gap = 0.01 * width_scaler

A0f = Vector3(x=flap_length, y=0+0.5*flap_gap)
A1f = Vector3(x=flap_length+0.05*L, y=0+0.5*flap_gap)
TTf = Vector3(x=flap_length+0.1*L, y=0+0.5*flap_gap)
B0f = Vector3(x=flap_length, y=0.8*body_width/2+0.5*flap_gap)

B1f = Vector3(x=A1f.x, y=0.5*B0f.y)
B0B1f = Line(p0=B0f, p1=B1f)
B1TTf = Line(p0=B1f, p1=TTf)
Line_B0TTf = Polyline([B0B1f, B1TTf])

def wing2_tf_top(x,y,z=0):
    return Vector3(x=0, y=0, z=-t_LE)

def wing2_tf_bot(x,y,z=0):
    return Vector3(x=0, y=0, z=t_LE)

def leading_edge_width_function(r):
    temp = Bezier([Vector3(x=0., y=0.02),
                    Vector3(x=0.75, y=0.05),
                    Vector3(x=1., y=0.05)])
    le_width = temp(r).y
    return le_width

# htv.add_wing(A0=A0f, A1=A1f, TT=TTf, B0=B0f, Line_B0TT=Line_B0TTf,
#              top_tf=wing2_tf_top, bot_tf=wing2_tf_bot,
#              LE_wf=leading_edge_width_function, flap_length=flap_length,
#              flap_angle=np.deg2rad(flap_angle), mirror=False, close_wing=True, 
#              mirror_new_component=False)

htv.generate()