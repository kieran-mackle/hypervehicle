"""
Example job file for hyper_vehicle.py.

NOTE: This file may be outdated, check the examples in the test directory for 
a working example.
"""

from gdtk.geom.vector3 import Vector3
from gdtk.geom.path import Bezier, Line, Polyline
import numpy as np

# Coordinate System Definition
# The Body Coordinate system is defined as described in Peter H. Zipfel -
#     Modeling and Simualtion of Aerospace Vehicle Dynamics.
#
# 1^B (X-axis) - points through the nose of the vehicle
# 2^B (Y-axis) - points out of the right hand wing
# 3^B (Z-axis) - points out of the bottom of the vehicle
# The 1^B and 3^B axis define the plane of symmetry

#####################################################
##                      WING                       ##
#####################################################
# For step 1:
#############
#  /\  Y-axis
#  |
#  |
#  +---> X-axis
# Looking at vehicle form below
#
# a) For leading edge define by bezier (or polyline) curve going B0 to TT
# Type: bezier
# Inputs: Line_B0TT
#     B0----------------\__
#     |                    B1--__
#     |                    |     B2
#     |                    |       \
#     A0------------------A1-------TT

# Define Planform of the Wing Half (This excludes the leading edge)
length = 0.75
width = 0.25
A0 = Vector3(x=0.0, y=0.0)
A1 = Vector3(x=length * 2 / 3, y=0.0)
TT = Vector3(x=length, y=0.0)
B0 = Vector3(x=0, y=width)
p3 = Vector3(x=length, y=width * 1 / 2)  # control points for Bezier curve
Line_B0TT = Bezier([B0, p3, TT])

# for a wing planform defined by two lines
# B2 = Vector3(x=0.60, y=0.15)
# Line_B0B2 = Line(p0=B0, p1=B2)
# Line_B2TT = Line(p0=B2, p1=TT)
# Line_B0TT = Polyline(segments=[Line_B0B2, Line_B2TT])


# For step 2:
#############
# Define function that set wing thickness as a function of x and y coordinate.
# Note - z direction is defined positive as 'downwards'
wing_thickness = 0.03


def wing_thickness_function_top(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=-wing_thickness / 2)
    # temp = wing_thickness + (0.05 - x/10.)
    # return Vector3(x=0., y=0., z=temp)


def wing_thickness_function_bot(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=wing_thickness / 2)


# For step 3:
#############
# Define function that set the width of the leading edge in wing planform.
# The shape of the elipse is then evaluatwed based on local thickness for top and bottom half of leading edge respectively.
# r is the parametrice position along the leading edge, starting from the wing trailing edge.
def leading_edge_width_function(r):
    temp = Bezier(
        [Vector3(x=0.0, y=0.02), Vector3(x=0.5, y=0.05), Vector3(x=1.0, y=0.05)]
    )
    le_width = temp(r).y
    return le_width


# For step 4:
#############
# Define Trailing Edge
flap_angle = np.deg2rad(0)  # +ve is tail of flap upwards
flap_length = 0.10


# For step 5:
#############
# Define functions that apply curvature to vehicle this is done in two steps:
# (a) curvature is applied in x-direction - longitudally.
# (b) curvature is applied in y-direction - spanwise.
# For each sub-step a fucntion to set local displacement and then gradient in
# respective direction needs to be defined.
#
# Add Curvature in longitudal (x) direction
def fun_x(x, y):
    return 0.5 * y**2 * x


def fun_x_dash(x, y):
    return 0.5 * y**2 * 1


# Add Curvature in spanwise (y) direction
# Due to symmetry, curvature needs to be zero on the symmetry plane (y=0)
def fun_y(x, y):
    return 0.1 * y**2


def fun_y_dash(x, y):
    return 0.1 * 2 * y


wing_geom_dict = {
    "A0": A0,  # Vector3 object form Eilmer Geom Package
    "A1": A1,  # Vector3 object form Eilmer Geom Package
    "TT": TT,  # Vector3 object form Eilmer Geom Package
    "B0": B0,  # Vector3 object form Eilmer Geom Package
    "Line_B0TT": Line_B0TT,  # Path objcet form Eilmer Geom Package
    "Line_B0TT_TYPE": "Bezier",  # How Line_B0TT has been defined
    "t_B1": None,  # Optional parameter to set location of B1
    "t_B2": None,  # Optional parameter to set location of B2
    # Define thickness functions
    "FUNC_TOP_THICKNESS": wing_thickness_function_top,
    "FUNC_BOT_THICKNESS": wing_thickness_function_bot,
    # Define Leading Edge Function
    "FUNC_LEADING_EDGE_WIDTH": leading_edge_width_function,
    # Define Trailing Edge
    "TAIL_OPTION": "FLAP",
    "FLAP_LENGTH": flap_length,
    "FLAP_ANGLE": flap_angle,
    # Define Curvature
    "WING_FUNC_CURV_X": fun_x,
    "WING_FUNC_CURV_X_DASH": fun_x_dash,
    "WING_FUNC_CURV_Y": fun_y,
    "WING_FUNC_CURV_Y_DASH": fun_y_dash,
}

#####################################################
##                  FUSELAGE                       ##
#####################################################
#   /\  R-axis
#   |
#   |
#   +---> X-axis
#
# Cross-section examples:
#
# FUSELAGE_NOSE_OPTION = 'sharp-cone'
# GConf.FUSELAGE_TAIL_OPTION = 'flat'
#    R3-----------------R2-____
#     |                 |      -----R1-_
#     |                 |           |   -_
#     |                 |           |     -_
#    X3----------------X2----------X1-------Xn
#
#
# FUSELAGE_NOSE_OPTION = 'blunt-cone'  # NOT IMPLEMENTED YET
# GConf.FUSELAGE_TAIL_OPTION = 'flat'
#    R3-----------------R2-____
#     |                 |      -----R1-- __
#     |                 |           |      -Rn-_
#     |                 |           |        |  \
#    X3----------------X2----------X1-------Xn--x
# Note: Nose tip will be at Xn+Rn
#
#
# FUSELAGE_NOSE_OPTION = 'sharp-cone'
# GConf.FUSELAGE_TAIL_OPTION = 'sharp-cone'
#         _-R3---------------R2-____
#       _- |                 |      -----R1-_
#     _-   |                 |           |   -_
#   _-     |                 |           |     -_
#  X4-----X3----------------X2----------X1-------Xn

# Define coordinates of construction points
Xn = 0.7
X1 = 0.6
X2 = 0.4
X3 = 0.0
X4 = -0.1
Rn = 0.1
R1 = 0.05
R2 = 0.1
R3 = 0.1


fuse_geom_dict = {  # Define Nose and Tail
    "FUSELAGE_NOSE_OPTION": "sharp-cone",  # Set Nose shape.
    # Options: ['sharp-cone']
    "FUSELAGE_TAIL_OPTION": "sharp-cone",  # Set Tail shape.
    # Options: ['sharp-cone', 'flat']
    # Define construction parameters
    "Xn": Xn,
    "X1": X1,
    "X2": X2,
    "X3": X3,
    "X4": X4,
    "Rn": Rn,
    "R1": R1,
    "R2": R2,
    "R3": R3,
    # Define Curvature
    "FUSELAGE_FUNC_CURV_X": fun_x,
    "FUSELAGE_FUNC_CURV_X_DASH": fun_x_dash,
    "FUSELAGE_FUNC_CURV_Y": None,
    "FUSELAGE_FUNC_CURV_Y_DASH": None,
}

#####################################################
##                     FINS                        ##
#####################################################
#  /\  Y-axis
#  |
#  |
#  +---> X-axis
# Looking at vehicle from below.
# Note that the coordinates defining the fin are in the X-Y
# plan, and that the fins are positioned by rotation about the
# x-axis. The rotation angle is specified by fin_angles.
# Defining multiple angles will produce multiple fins.
#
# Require the path from p1 to p3, plus coordinates of p0.
#
#
#   |--p1---p2
#   |  |    \
#   |  |     \
#   |  |      \
#   |  |       \
#   |  |        \
#   |--p0________p3


# IN WING (X-Y) COORDINATES
p0 = Vector3(x=0.0, y=0.0)
p1 = Vector3(x=0.0, y=0.2)
p2 = Vector3(x=0.15, y=0.15)
p3 = Vector3(x=0.2, y=0.0)

# Construct p1p3 path
p1p2 = Line(p1, p2)
p2p3 = Line(p2, p3)
p1p3 = Polyline(segments=[p1p2, p2p3])

fin_thickness = 0.02

# Thickness functions - constants
def fin_thickness_function_top(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=-fin_thickness / 2)


def fin_thickness_function_bot(x, y, z=0):
    return Vector3(x=0.0, y=0.0, z=fin_thickness / 2)


fin_angles = [np.deg2rad(-90)]

# Rudder angle (fin flap)
rudder_length = 0.1
rudder_angle = np.deg2rad(-45)

# Set fin pivot angle (rotation about z-axis)
pivot_angle = np.deg2rad(0)  # radians
pivot_point = Vector3(x=0.5 * (p0.x + p3.x), y=0, z=0)

fin_geom_dict = {
    "p0": p0,
    "p1": p1,
    "p2": p2,
    "p3": p3,
    "FIN_THICKNESS": fin_thickness,
    "FIN_ANGLES": fin_angles,
    "FIN_TOP_THICKNESS_FUNC": fin_thickness_function_top,
    "FIN_BOTTOM_THICKNESS_FUNC": fin_thickness_function_bot,
    "FIN_LEADING_EDGE_FUNC": leading_edge_width_function,
    "rudder_type": "flat",  # options ['flat', 'sharp']
    "rudder_length": rudder_length,
    "rudder_angle": rudder_angle,
    "pivot_angle": pivot_angle,
    "pivot_point": pivot_point,
}

#####################################################
##              MODEL CONFIGURATION                ##
#####################################################

# Global configuration dictionary:
global_config = {
    "VERBOSITY": 1,  # 0, 1, 2 - set reporting level.
    # OPTIONS
    "CREATE_WING": True,  # create wing geometry
    "CREATE_FUSELAGE": True,  # create fuselage geometry
    # vtk output
    "CREATE_VTK_MESH": False,
    "VTK_FILENAME": "test",
    # for "test" the final files will be
    # "test_wing_patch.vtk" and "test_fuselage_patch.vtk"
    "VTK_RESOLUTION": 10,  # number of cell vertices per edge
    # stl output
    "CREATE_STL_OBJECT": True,
    "STL_FILENAME": "test",
    # for "test" the final files will be
    # "test_wing.stl" and "test_fuselage.stl"
    "STL_RESOLUTION": 5,  # number of triangle vertices per edge
    "STL_INCLUDE_MIRROR": True,  # include mirror image in STL
    "STL_SHOW_IN_MATPLOT": False,  # Create Matplotlib image
    #
    # WING GEOMETRY DEFINITION
    "WING_GEOMETRY_DICT": wing_geom_dict,
    # FUSELAGE GEOMETRY DEFINITION
    "FUSELAGE_GEOMETRY_DICT": fuse_geom_dict,
    # FIN GEOMETRY DEFINITION
    "FIN_GEOMETRY_DICT": fin_geom_dict,
}
# end global_config
