import numpy as np
from stl import mesh
from typing import Callable
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from gdtk.geom.vector3 import Vector3
from gdtk.geom.sgrid import StructuredGrid
from gdtk.geom.path import Polyline, Bezier

# from hypervehicle.components import hyper_fuselage_main, hyper_wing_main, hyper_fin_main
# from hypervehicle.geometry import (
#     parametricSurfce2stl,
#     CurvedPatch,
#     RotatedPatch,
#     MirroredPatch,
# )


class Vehicle:
    """hypervehicle object.

    Attributes
    ----------
    verbosity : int, optional
        The verbosity of the code. The default is 1.
    write_stl : bool, optional
        Write vehicle geometry to .stl files. The default is True.
    stl_resolution : int, optional
        The number of cell vertices per edge for the STL files. The default is 5.
    stl_filename : str, optional
        The filename prefix for STL files. The default is 'test'.
    show_in_figure : bool, optional
        Show the STL files in a matplotlib figure. The default is False.
    write_vtk : bool, optional
        Write vehicle geometry to .vtk files. The default is False.
    vtk_resolution : int, optional
        The number of cell vertices per edge for the VTK files. The default is 5.
    vtk_filename : str, optional
        The filename prefix for VTK files. The default is 'test'.
    filename_prefix : str, optional
        The filename prefix for STL and VTK files. The default is 'test'.
    name : str, optional
        The vehicle name. The default is "generic hypersonic vehicle".

    """

    def __init__(self) -> None:
        """Vehicle constructor method."""
        self.verbosity = None
        self.vehicle_name = "hypersonic vehicle"

        # Components
        self.wings = []
        self.fuselage = []
        self.fins = []
        self.wing_resolutions = []
        self.fuselage_resolutions = []
        self.fin_resolutions = []
        self.mirror_fins = True
        self.vehicle_angle = 0

        # STL options
        self.build_stl = True  # Build stl mesh objects
        self.write_stl = True  # Write stl meshes to file
        self.stl_filename = None
        self.stl_resolution = None
        self.mirror = True
        self.show_mpl = None
        self.evaluate_properties = False

        # VTK options
        self.write_vtk = None
        self.vtk_resolution = None
        self.vtk_filename = None

        # Processed objects
        self.patches = {}  # Parametric patches
        self.grids = {}  # Structured grids
        self.surfaces = {}  # STL surfaces
        self.stl_data = {}  # STL data
        self.meshes = {}  # STL meshes

        self.cart3d = False

        # New attributes
        self.components = []

    def __repr__(self):
        return f"Parameterised {self.vehicle_name}"

    def configure(
        self,
        verbosity: int = None,
        write_stl: bool = None,
        stl_resolution: int = None,
        stl_filename: str = None,
        show_in_figure: bool = None,
        write_vtk: bool = None,
        vtk_resolution: int = None,
        vtk_filename: str = None,
        filename_prefix: str = None,
        build_stl: bool = None,
        evaluate_properties: bool = None,
        name: str = None,
        mirror_fins: bool = None,
        cart3d: bool = None,
    ) -> None:
        """Configures run options for Vehicle geometry generation.

        Parameters
        ----------
        verbosity : int, optional
            The verbosity of the code. The default is 1.
        build_stl : bool, optional
            Build STL mesh objects. The default is True.
        write_stl : bool, optional
            Write vehicle geometry to .stl files. The default is True.
        stl_resolution : int, optional
            The number of cell vertices per edge for the STL files. The default is 5.
        stl_filename : str, optional
            The filename prefix for STL files. The default is 'test'.
        show_in_figure : bool, optional
            Show the STL files in a matplotlib figure. The default is False.
        write_vtk : bool, optional
            Write vehicle geometry to .vtk files. The default is False.
        vtk_resolution : int, optional
            The number of cell vertices per edge for the VTK files. The default is 5.
        vtk_filename : str, optional
            The filename prefix for VTK files. The default is 'test'.
        filename_prefix : str, optional
            The filename prefix for STL and VTK files. The default is 'test'.
        evaluate_properties : bool, optional
            Flag to evaluate STL mesh properties. The default is False.
        name : str, optional
            The vehicle name. The default is "generic hypersonic vehicle".
        mirror_fins : bool, optional
            Set to True to mirror individual fin components. Useful to control
            individual fin rudder angles. The default is True.

        Warnings
        --------
        Any attributes assigned from loading a global configuration dictionary
        will be overwritten when using this method. For example, setting the
        verbosity using this method will overwrite the verbosity provided in
        the global configuration dictionary.

        Returns
        -------
        None
            This method assigns the settings to the vehicle instance.
        """

        self.verbosity = verbosity if verbosity is not None else self.verbosity
        self.vehicle_name = name if name is not None else self.vehicle_name

        # STL options
        self.build_stl = build_stl if build_stl is not None else self.build_stl
        self.write_stl = write_stl if write_stl is not None else self.write_stl
        self.stl_filename = (
            stl_filename if stl_filename is not None else self.stl_filename
        )
        self.stl_resolution = (
            stl_resolution if stl_resolution is not None else self.stl_resolution
        )
        self.show_mpl = show_in_figure if show_in_figure is not None else self.show_mpl
        self.evaluate_properties = (
            evaluate_properties
            if evaluate_properties is not None
            else self.evaluate_properties
        )

        # VTK options
        self.write_vtk = write_vtk if write_vtk is not None else self.write_vtk
        self.vtk_resolution = (
            vtk_resolution if vtk_resolution is not None else self.vtk_resolution
        )
        self.vtk_filename = (
            vtk_filename if vtk_filename is not None else self.vtk_filename
        )

        self.mirror_fins = mirror_fins if mirror_fins is not None else self.mirror_fins

        # Cart3d coordinate frame
        self.cart3d = cart3d if cart3d is not None else self.cart3d

    def add_component(
        self, component_type: str, component_dict: dict, stl_resolution: int = None
    ) -> None:
        """Adds a vehicle component.

        Parameters
        ----------
        component_type : str
            The type of component being added (wing, fin or fuselage).
        component_dict : dict
            The geometry definition dictionary for the component.
        """
        # TODO - can compare component types to COMPONENT constants

        if component_type.lower() == "wing":
            self.wings.append(component_dict)
            self.wing_resolutions.append(stl_resolution)
        elif component_type.lower() == "fin":
            self.fins.append(component_dict)
            self.fin_resolutions.append(stl_resolution)
        elif component_type.lower() == "fuselage":
            self.fuselage.append(component_dict)
            self.fuselage_resolutions.append(stl_resolution)

    def add_wing(
        self,
        A0: Vector3 = Vector3(0, 0, 0),
        A1: Vector3 = Vector3(0, 0, 0),
        TT: Vector3 = Vector3(0, 0, 0),
        B0: Vector3 = Vector3(0, 0, 0),
        Line_B0TT: Polyline = None,
        Line_B0TT_TYPE: str = "Bezier",
        t_B1: float = None,
        t_B2: float = None,
        top_tf: Callable[[float, float, float], Vector3] = None,
        bot_tf: Callable[[float, float, float], Vector3] = None,
        LE_wf: Callable[[float], Vector3] = None,
        LE_type: str = "custom",
        tail_option: str = "FLAP",
        flap_length: float = 0,
        flap_angle: float = 0,
        curve_x: Callable[[float, float, float], Vector3] = None,
        curve_dx: Callable[[float, float, float], Vector3] = None,
        curve_y: Callable[[float, float, float], Vector3] = None,
        curve_dy: Callable[[float, float, float], Vector3] = None,
        mirror: bool = True,
        mirror_new_component: bool = False,
        close_wing: bool = False,
        stl_resolution: int = None,
    ) -> None:
        """Creates and appends a new wing to the vehicle.

        Parameters
        ----------
        A0 : Vector3, optional
            Point A0 of the wing geometry. The default is Vector3(0,0,0).
        A1 : Vector3, optional
            Point A1 of the wing geometry. The default is Vector3(0,0,0).
        TT : Vector3, optional
            Point TT of the wing geometry. The default is Vector3(0,0,0).
        B0 : Vector3, optional
            Point B0 of the wing geometry. The default is Vector3(0,0,0).
        Line_B0TT : Polyline, optional
            A line object joining B0 to TT. The default is None.
        Line_B0TT_TYPE : str, optional
            The type of the line object Line_B0TT. The default is "Bezier".
        t_B1 : float, optional
            First parameter to split B0TT. The default is None.
        t_B2 : float, optional
            Second parameter to split B0TT. The default is None.
        top_tf : Callable[[float, float, float], Vector3], optional
            The thickness function of the top surface. The default is None.
        bot_tf : Callable[[float, float, float], Vector3], optional
            The thickness function of the bottom surface. The default is None.
        LE_wf : Callable[[float], Vector3], optional
            The leading edge width function. The default is None.
        LE_type : str
            The leading edge type. Specify 'flat' to create a flat edge,
            otherwise provide LE_wf. The default is 'custom'.
        tail_option : str, optional
            The type of trailing edge to use. The default is 'FLAP'.
        flap_length : float, optional
            The length of the trailing edge flap. The default is 0.
        flap_angle : float, optional
            The angle of the trailing edge flap, specified in radians. The
            default is 0.
        curve_x : Callable[[float, float, float], Vector3], optional
            The curvature function in the x-direction. The default is None.
        curve_dx : Callable[[float, float, float], Vector3], optional
            The derivative of the curvature function in the x-direction. The
            default is None.
        curve_y : Callable[[float, float, float], Vector3], optional
            The curvature function in the y-direction. The default is None.
        curve_dy : Callable[[float, float, float], Vector3], optional
            The derivative of the curvature function in the x-direction. The
            default is None.
        mirror : bool, optional
            Mirror the wing about the x-z plane. The default is True.
        mirror_new_component : bool, optional
            If mirror is True, use this boolean flag to specify whether the
            mirrored component will be treated as a new component, or as
            part of the original component. The default is False.
        close_wing : bool, optional
            Close the interior edges of the wing component. The default is
            False.
        stl_resolution : int, optional
            The stl resolution to use when creating the mesh for this
            component. The default is None.

        Returns
        -------
        None
            Calling this method will append a wing to the vehicle via the
            add_component method.

        """
        # Check if a LE function was provided
        if LE_wf is None and LE_type == "custom":
            # Use default function
            def leading_edge_width_function(r):
                temp = Bezier(
                    [
                        Vector3(x=0.0, y=0.02),
                        Vector3(x=0.75, y=0.1),
                        Vector3(x=1.0, y=0.3),
                    ]
                )
                le_width = temp(r).y
                return le_width

            # Assign
            LE_wf = leading_edge_width_function

        new_wing = {
            "A0": A0,
            "A1": A1,
            "TT": TT,
            "B0": B0,
            "Line_B0TT": Line_B0TT,
            "Line_B0TT_TYPE": "Bezier",
            "t_B1": t_B1,
            "t_B2": t_B2,
            "FUNC_TOP_THICKNESS": top_tf,
            "FUNC_BOT_THICKNESS": bot_tf,
            "FUNC_LEADING_EDGE_WIDTH": LE_wf,
            "LE_TYPE": LE_type.upper(),
            "TAIL_OPTION": tail_option,
            "FLAP_LENGTH": flap_length,
            "FLAP_ANGLE": flap_angle,
            "WING_FUNC_CURV_X": curve_x,
            "WING_FUNC_CURV_X_DASH": curve_dx,
            "WING_FUNC_CURV_Y": curve_y,
            "WING_FUNC_CURV_Y_DASH": curve_dy,
            "MIRROR": mirror,
            "MIRROR_NEW_COMPONENT": mirror_new_component,
            "CLOSE_WING": close_wing,
        }
        self.add_component("wing", new_wing, stl_resolution)

    def add_fin(
        self,
        p0: Vector3,
        p1: Vector3,
        p2: Vector3,
        p3: Vector3,
        fin_thickness: float,
        fin_angle: float,
        top_thickness_function,
        bot_thickness_function,
        leading_edge_width_function=None,
        mirror: bool = False,
        rudder_type: str = "flat",
        rudder_length: float = 0,
        rudder_angle: float = 0,
        pivot_angle: float = 0,
        pivot_point: Vector3 = Vector3(x=0, y=0),
        offset_func=None,
        stl_resolution: int = None,
    ) -> None:
        """Creates and appends a new fin to the vehicle.

        Parameters
        ----------
        p0 : Vector3
            Point p0 of the fin geometry.
        p1 : Vector3
            Point p1 of the fin geometry.
        p2 : Vector3
            Point p2 of the fin geometry.
        p3 : Vector3
            Point p3 of the fin geometry.
        stl_resolution : int, optional
            The stl resolution to use when creating the mesh for this
            component. The default is None.
        """
        if leading_edge_width_function is None:
            # Use default LE function
            def leading_edge_width_function(r):
                temp = Bezier(
                    [
                        Vector3(x=0.0, y=0.02),
                        Vector3(x=0.75, y=0.1),
                        Vector3(x=1.0, y=0.3),
                    ]
                )
                le_width = temp(r).y
                return le_width

        new_fin = {
            "p0": p0,
            "p1": p1,
            "p2": p2,
            "p3": p3,
            "FIN_THICKNESS": fin_thickness,
            "FIN_ANGLE": fin_angle,
            "FIN_TOP_THICKNESS_FUNC": top_thickness_function,
            "FIN_BOTTOM_THICKNESS_FUNC": bot_thickness_function,
            "FIN_LEADING_EDGE_FUNC": leading_edge_width_function,
            "MIRROR_NEW_COMPONENT": mirror,
            "rudder_type": rudder_type,
            "rudder_length": rudder_length,
            "rudder_angle": rudder_angle,
            "pivot_angle": pivot_angle,
            "pivot_point": pivot_point,
            "offset_function": offset_func,
        }
        self.add_component("fin", new_fin, stl_resolution)

    def add_fuselage(
        self,
        Xn: float = None,
        X1: float = None,
        X2: float = None,
        X3: float = None,
        R1: float = None,
        R2: float = None,
        R3: float = None,
        X4: float = None,
        revolve_line=None,
        cross_sections: list = None,
        sweep_axis: str = "z",
        nose_type: str = "sharp-cone",
        tail_type: str = "flat",
        x_curve_func=None,
        x_dash_func=None,
        y_curve_func=None,
        y_dash_func=None,
        offset=None,
        stl_resolution: int = None,
    ) -> None:
        """Adds the fuselage to the vehicle.

        Parameters
        ----------
        Xn: float
            The axial location of the fuselage nose.
        X1: float
            The axial location of the X1 point.
        X2: float
            The axial location of the X2 point.
        X3: float
            The axial location of the X3 point.
        X4: float, optional
            The axial location of the X4 point. The default is None.
        R1: float
            The radius of the fuselage at X1.
        R2: float
            The radius of the fuselage at X2.
        R3: float
            The radius of the fuselage at X3.
        revolve_line : Line|PolyLine|Bezier
            A line to be revolved about the primary axis.
        cross_sections : list, optional
            A list of cross-sectional patches to sweep through.
        sweep_axis : str, optional
            The axis to sweep the cross sections through. The default
            is z.
        nose_type: str, optional
            The fuselage nose type. The default is 'sharp-cone'.
        tail_type: str, optional
            The fuselage tail type. Options include 'sharp-cone' and 'flat'.
            When using 'sharp-cone', X4 must be provided. The default is flat.
        x_curve_func: function, optional
            The x-curvature function. The default is None.
        x_dash_func: function, optional
            The x-curvature derivative function. The default is None.
        y_curve_func: function, optional
            The y-curvature function. The default is None.
        y_dash_func: function, optional
            The y-curvature derivative function. The default is None.
        stl_resolution : int, optional
            The stl resolution to use when creating the mesh for this
            component. The default is None.
        """
        fuselage = {
            "FUSELAGE_NOSE_OPTION": nose_type,
            "FUSELAGE_TAIL_OPTION": tail_type,
            "Xn": Xn,
            "X1": X1,
            "X2": X2,
            "X3": X3,
            "X4": X4,
            "R1": R1,
            "R2": R2,
            "R3": R3,
            "revolve_line": revolve_line,
            "cross_sections": cross_sections,
            "sweep_axis": sweep_axis,
            "FUSELAGE_FUNC_CURV_X": x_curve_func,
            "FUSELAGE_FUNC_CURV_X_DASH": x_dash_func,
            "FUSELAGE_FUNC_CURV_Y": y_curve_func,
            "FUSELAGE_FUNC_CURV_Y_DASH": y_dash_func,
            "OFFSET": offset,
        }

        self.add_component("fuselage", fuselage, stl_resolution)

    def generate(self) -> None:
        """Run hypervehicle geometry generation code to build geometry."""

        if self.verbosity > 0:
            # banner = pyfiglet.figlet_format("HYPERVEHICLE", font='contessa')
            # print("")
            # print(banner)
            print("\nHYPERVEHICLE\n")

        # Create component patches
        self.patches["wing"] = hyper_wing_main(self.wings, self.verbosity)
        self.patches["fuselage"] = hyper_fuselage_main(self.fuselage, self.verbosity)
        self.patches["fin"] = hyper_fin_main(self.fins, self.verbosity)

        # Add curvature
        if self.verbosity > 0:
            print("\nAdding curvature to vehicle components.")
        self._add_curvature()

        # Add vehicle offset angle to correct any curve induced AOA change
        if self.vehicle_angle != 0:
            if self.verbosity > 0:
                print("\nAdding vehicle angle.")
            self._rotate_vehicle(self.vehicle_angle)

        # Add mirror image
        if self.mirror:
            if self.verbosity > 0:
                print("\nAdding Mirror Image of wings.")
            self.patches["wing"] = self._mirror_patches(
                self.patches["wing"], components=self.wings
            )

        if self.mirror_fins:
            if self.verbosity > 0:
                print("Adding Mirror Image of fins.")
            self.patches["fin"] = self._mirror_patches(
                self.patches["fin"], components=self.fins
            )

        # Rotate vehicle to align with Cart3D
        if self.cart3d:
            if self.verbosity > 0:
                print("\nRotating vehicle to align with Cart3D flow coordinate system.")
            self._rotate_vehicle(angle=180, axis="z")
            self._rotate_vehicle(angle=90, axis="x")

        # Eilmer Grid surface grids
        if self.write_vtk:
            if self.verbosity > 0:
                print("\nCreating Eilmer Grid and vtk files.")
            self._create_grids()
            self._write_to_vtk()

        # STL object
        self.build_stl = True if self.write_stl else self.build_stl  # Enforce
        if self.build_stl:
            if self.verbosity > 0:
                print("\nBuilding STL object(s).")
            self._create_surfaces()
            self._create_stl_data()
            self._create_stl()
            if self.write_stl:
                self._write_stl()
            if self.evaluate_properties:
                self._evaluate_mesh_properties()

            if self.show_mpl:
                self._mpl_plot()

    def _add_curvature(
        self,
    ) -> None:
        """Adds curvature by the provided curvature functions."""
        # Wing curvature
        if len(self.wings) > 0:
            for ix, wing_geometry_dict in enumerate(self.wings):
                # Curvature about the x-axis
                if (
                    wing_geometry_dict["WING_FUNC_CURV_X"] is None
                    and wing_geometry_dict["WING_FUNC_CURV_X_DASH"] is None
                ):
                    if self.verbosity > 0:
                        print("    Skipping wing X-curvature.")
                else:
                    # (a) Longitudal Curvature
                    for key in self.patches["wing"][ix]:
                        self.patches["wing"][ix][key] = CurvedPatch(
                            underlying_surf=self.patches["wing"][ix][key],
                            direction="x",
                            fun=wing_geometry_dict["WING_FUNC_CURV_X"],
                            fun_dash=wing_geometry_dict["WING_FUNC_CURV_X_DASH"],
                        )

                # Curvature about the y-axis
                if (
                    wing_geometry_dict["WING_FUNC_CURV_Y"] is None
                    and wing_geometry_dict["WING_FUNC_CURV_Y_DASH"] is None
                ):
                    if self.verbosity > 0:
                        print("    Skipping wing Y-curvature.")
                else:
                    # (b) Spanwise Curvature
                    for key in self.patches["wing"][ix]:
                        self.patches["wing"][ix][key] = CurvedPatch(
                            underlying_surf=self.patches["wing"][ix][key],
                            direction="y",
                            fun=wing_geometry_dict["WING_FUNC_CURV_Y"],
                            fun_dash=wing_geometry_dict["WING_FUNC_CURV_Y_DASH"],
                        )

        # Fuselage curvature
        if len(self.fuselage) > 0:
            for ix, fuse_geometry_dict in enumerate(self.fuselage):
                # Longitudal Curvature
                if (
                    fuse_geometry_dict["FUSELAGE_FUNC_CURV_X"] is None
                    and fuse_geometry_dict["FUSELAGE_FUNC_CURV_X_DASH"] is None
                ):
                    if self.verbosity > 0:
                        print("    Skipping fuselage X-curvature.")

                else:
                    for key in self.patches["fuselage"][ix]:
                        self.patches["fuselage"][ix][key] = CurvedPatch(
                            underlying_surf=self.patches["fuselage"][ix][key],
                            direction="x",
                            fun=fuse_geometry_dict["FUSELAGE_FUNC_CURV_X"],
                            fun_dash=fuse_geometry_dict["FUSELAGE_FUNC_CURV_X_DASH"],
                        )
                # Spanwise Curvature
                if (
                    fuse_geometry_dict["FUSELAGE_FUNC_CURV_Y"] is None
                    and fuse_geometry_dict["FUSELAGE_FUNC_CURV_Y_DASH"] is None
                ):
                    if self.verbosity > 0:
                        print("    Skipping fuselage Y-curvature.")
                else:
                    for key in self.patches["fuselage"][ix]:
                        self.patches["fuselage"][ix][key] = CurvedPatch(
                            underlying_surf=self.patches["fuselage"][ix][key],
                            direction="y",
                            fun=fuse_geometry_dict["FUSELAGE_FUNC_CURV_Y"],
                            fun_dash=fuse_geometry_dict["FUSELAGE_FUNC_CURV_Y_DASH"],
                        )

    def _rotate_vehicle(self, angle: float = 0, axis: str = "y") -> None:
        """Rotates entire vehicle by specified angle about a given axis.

        Parameters
        ----------
        angle : float, optional
            Rotation angle. The default is 0.
        axis : str, optional
            Rotation axis. The default is 'y'.

        Returns
        -------
        None
            This method overwrites the vehicle patches of the Vehicle instance.
        """

        # Rotate wings
        for patch_dict in self.patches["wing"]:
            for key in patch_dict:
                patch_dict[key] = RotatedPatch(
                    patch_dict[key], np.deg2rad(angle), axis=axis
                )

        # Rotate fins
        for patch_dict in self.patches["fin"]:
            for key in patch_dict:
                patch_dict[key] = RotatedPatch(
                    patch_dict[key], np.deg2rad(angle), axis=axis
                )

        # Rotate fuselage
        for patch_dict in self.patches["fuselage"]:
            for key in patch_dict:
                patch_dict[key] = RotatedPatch(
                    patch_dict[key], np.deg2rad(angle), axis=axis
                )

    def _create_grids(self) -> None:
        """Creates structured grid objects."""
        # Wings
        self.grids["wing"] = []
        for wing_patch_dict in self.patches["wing"]:
            wing_grid_dict = {}
            for key in wing_patch_dict:
                wing_grid_dict[key] = StructuredGrid(
                    psurf=wing_patch_dict[key],
                    niv=self.vtk_resolution,
                    njv=self.vtk_resolution,
                )
            self.grids["wing"].append(wing_grid_dict)

        # Fuselage
        self.grids["fuselage"] = {}
        for key in self.patches["fuselage"]:
            self.grids["fuselage"][key] = StructuredGrid(
                psurf=self.patches["fuselage"][key],
                niv=self.vtk_resolution,
                njv=self.vtk_resolution,
            )

        # Fins
        self.grids["fin"] = []
        for fin_patch_dict in self.patches["fin"]:
            fin_grid_dict = {}
            for key in fin_patch_dict:
                fin_grid_dict[key] = StructuredGrid(
                    psurf=fin_patch_dict[key],
                    niv=self.vtk_resolution,
                    njv=self.vtk_resolution,
                )
            self.grids["fin"].append(fin_grid_dict)

    def _write_to_vtk(
        self,
    ) -> None:
        """Writes grids to VTK."""
        if self.verbosity > 0:
            print("    Structure Grid Creates")
            print("    Writing grid files to {}-label.vtk".format(self.vtk_filename))

        # Wings
        for wing_grid_dict in self.grids["wing"]:
            for key in wing_grid_dict:
                wing_grid_dict[key].write_to_vtk_file(
                    "{0}-wing_{1}.vtk".format(self.vtk_filename, key)
                )

        # Fuselage
        for key in self.grids["fuselage"]:
            self.grids["fuselage"][key].write_to_vtk_file(
                "{0}-fuse_{1}.vtk".format(self.vtk_filename, key)
            )

        # Fins
        for fin_grid_dict in self.grids["fin"]:
            for key in fin_grid_dict:
                fin_grid_dict[key].write_to_vtk_file(
                    f"{self.vtk_filename}-fin_{key}.vtk"
                )

    def _create_surfaces(
        self,
    ) -> None:
        """Creates parameteric surfaces."""

        # Wings
        self.surfaces["wing"] = []
        for ix, wing_patch_dict in enumerate(self.patches["wing"]):
            wing_stl_mesh_list = []
            resolution = (
                self.wing_resolutions[ix]
                if self.wing_resolutions[ix] is not None
                else self.stl_resolution
            )
            for key in wing_patch_dict:
                flip = True if key.split("_")[-1] == "mirrored" else False
                wing_stl_mesh_list.append(
                    parametricSurfce2stl(
                        wing_patch_dict[key], resolution, flip_faces=flip
                    )
                )
            self.surfaces["wing"].append(wing_stl_mesh_list)

        # Fuselage
        self.surfaces["fuselage"] = []
        for ix, fuse_patch_dict in enumerate(self.patches["fuselage"]):
            fuse_stl_mesh_list = []
            resolution = (
                self.fuselage_resolutions[ix]
                if self.fuselage_resolutions[ix] is not None
                else self.stl_resolution
            )
            for key, item in fuse_patch_dict.items():
                flip = False

                # Correct STL resolutions and flip
                if "swept" in key:
                    res = (
                        int(resolution / 4) if "end" in key else int(resolution / 4) * 4
                    )
                    flip = True if "1" in key else False
                else:
                    res = resolution
                fuse_stl_mesh_list.append(
                    parametricSurfce2stl(item, res, flip_faces=flip)
                )

            self.surfaces["fuselage"].append(fuse_stl_mesh_list)

        # Fins
        self.surfaces["fin"] = []
        for ix, fin_patch_dict in enumerate(self.patches["fin"]):
            fin_stl_mesh_list = []
            resolution = (
                self.fin_resolutions[ix]
                if self.fin_resolutions[ix] is not None
                else self.stl_resolution
            )
            for key in fin_patch_dict:
                flip = True if key.split("_")[-1] == "mirrored" else False
                fin_stl_mesh_list.append(
                    parametricSurfce2stl(
                        fin_patch_dict[key], resolution, flip_faces=flip
                    )
                )
            self.surfaces["fin"].append(fin_stl_mesh_list)

    def _create_stl_data(self) -> None:
        """Creates STL data elements."""
        # Wings
        self.stl_data["wing"] = []
        for wing_stl_mesh_list in self.surfaces["wing"]:
            wing_stl_data = []
            for val in wing_stl_mesh_list:
                wing_stl_data.append(val.data.copy())
            self.stl_data["wing"].append(wing_stl_data)

        # Fuselage
        self.stl_data["fuselage"] = []
        for fuse_stl_mesh_list in self.surfaces["fuselage"]:
            fuse_stl_data = []
            for val in fuse_stl_mesh_list:
                fuse_stl_data.append(val.data.copy())
            self.stl_data["fuselage"].append(fuse_stl_data)

        # Fins
        self.stl_data["fin"] = []
        for fin_stl_mesh_list in self.surfaces["fin"]:
            fin_stl_data = []
            for val in fin_stl_mesh_list:
                fin_stl_data.append(val.data.copy())
            self.stl_data["fin"].append(fin_stl_data)

    def _create_stl(self):
        """Creates stl objects."""
        # Wings
        if len(self.wings) > 0:
            # Create stl object
            self.meshes["wing"] = []
            for wing_no, wing_stl_data in enumerate(self.stl_data["wing"]):
                self.meshes["wing"].append(mesh.Mesh(np.concatenate(wing_stl_data)))
                if self.verbosity > 0:
                    print(f"    wing {wing_no} stl object created")

        # Fuselage
        if len(self.fuselage) > 0:
            self.meshes["fuselage"] = []
            for fuse_no, fuse_stl_data in enumerate(self.stl_data["fuselage"]):
                self.meshes["fuselage"].append(mesh.Mesh(np.concatenate(fuse_stl_data)))
                if self.verbosity > 0:
                    print(f"    fuselage {fuse_no} stl object created")

        # FinsS
        if len(self.fins) > 0:
            self.meshes["fin"] = []
            for fin_no, fin_stl_data in enumerate(self.stl_data["fin"]):
                self.meshes["fin"].append(mesh.Mesh(np.concatenate(fin_stl_data)))
                if self.verbosity > 0:
                    print(f"    fin {fin_no} stl object created")

    def _write_stl(self):
        """Writes STL files."""
        # Wings
        if len(self.wings) > 0:
            # Create stl object
            for wing_no, wing_mesh in enumerate(self.meshes["wing"]):
                # Save to .stl file
                wing_filename = f"{self.stl_filename}-wing{wing_no+1}.stl"
                wing_mesh.save(wing_filename)
                if self.verbosity > 0:
                    print(f"    Writing wing {wing_no+1} stl to {wing_filename}.")

        # Fuselage
        if len(self.fuselage) > 0:
            for fuse_no, fuse_mesh in enumerate(self.meshes["fuselage"]):
                fuse_filename = f"{self.stl_filename}-fuse{fuse_no+1}.stl"
                fuse_mesh.save(fuse_filename)
                if self.verbosity > 0:
                    print(f"    Writing fuselage {fuse_no} stl to {fuse_filename}.")

        # Fins
        if len(self.fins) > 0:
            for fin_no, fin_mesh in enumerate(self.meshes["fin"]):
                fin_filename = f"{self.stl_filename}-fin{fin_no+1}.stl"
                fin_mesh.save(fin_filename)
                if self.verbosity > 0:
                    print(f"    Writing fin stl to {fin_filename}.")

    def _evaluate_mesh_properties(
        self,
    ):
        """Evaluates properties of stl."""
        if len(self.wings) > 0:
            print("")
            print("START: Evaluating Wing Mesh Properties:")
            for wing_no, wing_mesh in enumerate(self.meshes["wing"]):
                volume, cog, inertia = wing_mesh.get_mass_properties()
                print(f"    WING {wing_no+1}")
                print("    --------")
                print(
                    "    Volume                                  = {0}".format(volume)
                )
                print("    Position of the center of gravity (COG) = {0}".format(cog))
                print(
                    "    Inertia matrix at expressed at the COG  = {0}".format(
                        inertia[0, :]
                    )
                )
                print(
                    "                                              {0}".format(
                        inertia[1, :]
                    )
                )
                print(
                    "                                              {0}".format(
                        inertia[2, :]
                    )
                )
            print("  DONE: Evaluating Wing Mesh Properties")
            print("")

        if len(self.fuselage) > 0:
            print("")
            print("START: Evaluating Fuselage Mesh Properties:")
            for fuse_no, fuse_mesh in enumerate(self.meshes["fuselage"]):
                volume, cog, inertia = fuse_mesh.get_mass_properties()
                print(f"    FUSELAGE {fuse_no+1}")
                print("    --------")
                print(
                    "    Volume                                  = {0}".format(volume)
                )
                print("    Position of the center of gravity (COG) = {0}".format(cog))
                print(
                    "    Inertia matrix at expressed at the COG  = {0}".format(
                        inertia[0, :]
                    )
                )
                print(
                    "                                              {0}".format(
                        inertia[1, :]
                    )
                )
                print(
                    "                                              {0}".format(
                        inertia[2, :]
                    )
                )
            print("  DONE: Evaluating Fuselage Mesh Properties")
            print("")

        if len(self.fins) > 0:
            print("")
            print("START: Evaluating Fin Mesh Properties:")
            for fin_no, fin_mesh in enumerate(self.meshes["fin"]):
                volume, cog, inertia = fin_mesh.get_mass_properties()
                print(f"    Fin {fin_no+1}")
                print("    --------")
                print(
                    "    Volume                                  = {0}".format(volume)
                )
                print("    Position of the center of gravity (COG) = {0}".format(cog))
                print(
                    "    Inertia matrix at expressed at the COG  = {0}".format(
                        inertia[0, :]
                    )
                )
                print(
                    "                                              {0}".format(
                        inertia[1, :]
                    )
                )
                print(
                    "                                              {0}".format(
                        inertia[2, :]
                    )
                )
            print("DONE Evaluating Fin Mesh Properties")
            print("")

    @staticmethod
    def _mirror_patches(
        patch_list: list, axis: str = "y", components: list = None
    ) -> list:
        """Mirrors all patches in a patch list.

        Parameters
        ----------
        patch_list : list
            The list containing dictionaries of patches. Each component is
            represented by its own dictionary or patches.
        axis : str, optional
            The axis to mirror about. The default is 'y'.

        components : list, optional
            The list of component dictionaries, to allow passing
            meta-data in. The default is None.

        create_new : list[bool], optional
            Create a new component for the mirrored component. If True,
            a new component will be created rather than appended to the
            patches of the parent component. The default is False.

        Returns
        -------
        list
            Original list of patches with mirrored patches appended.
        """

        new_components = []

        for component, patch_dict in enumerate(patch_list):
            # Create mirrored patches
            mirrored_patches = {}
            for patch in patch_dict:
                mirrored_patches[patch + "_mirrored"] = MirroredPatch(
                    patch_dict[patch], axis=axis
                )

            if not components[component]["MIRROR_NEW_COMPONENT"]:
                # Append mirrored patches to original patch_dict
                for patch in mirrored_patches:
                    patch_dict[patch] = mirrored_patches[patch]
            else:
                # Mirrored component is to be new component
                new_components.append(mirrored_patches)

        # Add new components to patch_list
        for patch_dict in new_components:
            patch_list.append(patch_dict)

        return patch_list

    def _mpl_plot(
        self,
    ):
        """Plots stl components with matplotlib."""
        if len(self.wings) > 0:
            # Create a new plot
            figure = plt.figure()
            ax = mplot3d.Axes3D(figure)

            # Render the wing(s)
            for wing_mesh in self.meshes["wing"]:
                ax.add_collection3d(mplot3d.art3d.Poly3DCollection(wing_mesh.vectors))
                scale = wing_mesh.points.flatten()

            # Auto scale to the mesh size
            ax.auto_scale_xyz(scale, scale, scale)
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.set_zlabel("Z-axis")

        if len(self.fuselage) > 0:
            # Create a new plot
            figure = plt.figure()
            ax = mplot3d.Axes3D(figure)

            # Render the fuselage
            for fuse_mesh in self.meshes["fuselage"]:
                ax.add_collection3d(mplot3d.art3d.Poly3DCollection(fuse_mesh.vectors))
                scale = fuse_mesh.points.flatten()

            # Auto scale to the mesh size
            ax.auto_scale_xyz(scale, scale, scale)
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.set_zlabel("Z-axis")

        if self.wings is not None and self.fuselage is not None:
            # Create a new plot
            figure = plt.figure()
            ax = mplot3d.Axes3D(figure)

            # Render the wing and fuselage
            for wing_mesh in self.meshes["wing"]:
                wing_coll = mplot3d.art3d.Poly3DCollection(wing_mesh.vectors)
                ax.add_collection3d(wing_coll)
                scale = wing_mesh.points.flatten()

            fuse_coll = mplot3d.art3d.Poly3DCollection(self.meshes["fuselage"].vectors)
            fuse_coll.set_facecolor("r")
            ax.add_collection3d(fuse_coll)

            if len(self.fins) > 0:
                for fin_mesh in self.meshes["fin"]:
                    fin_coll = mplot3d.art3d.Poly3DCollection(fin_mesh.vectors)
                    fin_coll.set_facecolor("c")
                    ax.add_collection3d(fin_coll)

            # Auto scale to the mesh size
            ax.auto_scale_xyz(scale, scale, scale)
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.set_zlabel("Z-axis")

        plt.show()