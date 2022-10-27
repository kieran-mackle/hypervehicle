"""
Example job file for polygon_formation.py.

NOTE: The Polygon_formation method is still under-development so it may need some edits.
"""

import os
import numpy as np
from stl import Mesh
from hypervehicle import Polygon_formation
from eilmer.geom.vector3 import Vector3
from eilmer.geom.path import Bezier, Line, Polyline

# Supported Polygons:
#
#   Cube - side length 2a (m) placed at centre.
#   Sphere - radius r (m) placed at centre.

poly_geom_list = [
    {"cube": {"a": 0.0127, "centre": Vector3(0, 0, 0)}},
    {"sphere": {"r": 0.01, "centre": Vector3(0.03, 0, 0)}},
]

formation = Polygon_formation()
formation.add_polys(poly_geom_list)

formation.configure(
    verbosity=1,
    write_stl=True,
    stl_resolution=7,
    stl_filename="hyper-cube",
    show_in_figure=True,
    write_vtk=False,
)

formation.generate()

# Possible configure parameters

# (self, verbosity: int = None, write_stl: bool = None,
# stl_resolution: int = None, stl_filename: str = None,
# show_in_figure: bool = None, write_vtk: bool = None,
# vtk_resolution: int = None, vtk_filename: str = None,
# filename_prefix: str = None, evaluate_properties: bool = None,
# name: str = None) -> None:
#        """Configures run options for Vehicle geometry generation.

#        Parameters
#        ----------
#        verbosity : int, optional
#            The verbosity of the code. The default is 1.
#        write_stl : bool, optional
#            Write vehicle geometry to .stl files. The default is True.
#        stl_resolution : int, optional
#            The number of cell vertices per edge for the STL files. The default is 5.
#        stl_filename : str, optional
#            The filename prefix for STL files. The default is 'test'.
#        show_in_figure : bool, optional
#            Show the STL files in a matplotlib figure. The default is False.
#        write_vtk : bool, optional
#            Write vehicle geometry to .vtk files. The default is False.
#        vtk_resolution : int, optional
#            The number of cell vertices per edge for the VTK files. The default is 5.
#        vtk_filename : str, optional
#            The filename prefix for VTK files. The default is 'test'.
#        filename_prefix : str, optional
#            The filename prefix for STL and VTK files. The default is 'test'.
#        evaluate_properties : bool, optional
#            Flag to evaluate STL mesh properties. The default is False.
#        name : str, optional
#            The vehicle name. The default is "generic hypersonic vehicle".
