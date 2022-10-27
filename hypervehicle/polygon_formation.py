#!/usr/bin/python3.8
# from __future__ import annotations # Only supported in python3.7+
import os
import sys
import shutil

# import pyfiglet
import numpy as np
from stl import mesh
from getopt import getopt
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from eilmer.geom.sgrid import StructuredGrid

from typing import Callable

from eilmer.geom.vector3 import Vector3
from eilmer.geom.path import Polyline

from hypervehicle.components import poly_gen_main

from hypervehicle.utils import (
    parametricSurfce2stl,
    CurvedPatch,
    RotatedPatch,
    MirroredPatch,
)

"""
To do:
    - Need to edit the vehicle angle, we can have polygons
      at different AOA's
    - Some more checks and balances
"""


class Polygon_formation:
    """Formation of Polygons

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
        """Poly formation constructor method."""

        print("Polygon Mesh property calculation currently NOT SUPPORTED!!!")
        print("Abandoning calculation.")
        print()

        self.verbosity = None
        self.vehicle_name = "polygons"

        # Components
        self.polygons = []

        # STL options
        self.write_stl = None
        self.stl_filename = None
        self.stl_resolution = None
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

        self.vehicle_angle = 0
        self.cart3d = False

    def __repr__(self):
        return f"Parameterised {self.vehicle_name}."

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
        evaluate_properties: bool = None,
        name: str = None,
    ) -> None:
        """Configures run options for Vehicle geometry generation.

        Parameters
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
        evaluate_properties : bool, optional
            Flag to evaluate STL mesh properties. The default is False.
        name : str, optional
            The vehicle name. The default is "generic hypersonic vehicle".

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

    def add_polys(self, polygon_geometries: list) -> None:
        """Run poly_gen_main to add polygons to the formation."""
        self.polygons = polygon_geometries
        self.patches["polygons"] = poly_gen_main(polygon_geometries, self.verbosity)

    def generate(self) -> None:
        """Run hypervehicle geometry generation code to build geometry."""

        if self.verbosity > 0:
            # banner = pyfiglet.figlet_format("HYPERVEHICLE", font='contessa')
            # print("")
            # print(banner)
            print("\nHYPERVEHICLE\n")

        # Create polygon patches
        assert bool(self.patches["polygons"]), (
            "Error! No polygon patches detected."
            + "Have you run the add_polys() method?"
        )

        #        self.patches['polygons'] = poly_gen_main(self.polygons, self.verbosity)

        # Add vehicle offset angle to correct any curve induced AOA change
        if self.vehicle_angle != 0:
            if self.verbosity > 0:
                print("\nAdding vehicle angle.")
            self._rotate_vehicle(self.vehicle_angle)

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
        if self.write_stl:
            if self.verbosity > 0:
                print(
                    f"\nCreating STL object(s) with a resolution of {self.stl_resolution}."
                )
            self._create_surfaces()
            self._create_stl_data()
            self._create_stl()
            self._write_stl()
            if self.evaluate_properties:
                self._evaluate_mesh_properties()

            if self.show_mpl:
                self._mpl_plot()

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

        for patch_dict in self.patches["polygons"]:
            for key in patch_dict:
                patch_dict[key] = RotatedPatch(
                    patch_dict[key], np.deg2rad(angle), axis=axis
                )

    def _create_grids(self) -> None:
        """Creates structured grid objects."""

        self.grids["polygons"] = []
        for poly_patch_dict in self.patches["polygons"]:
            poly_grid_dict = {}
            for key in poly_patch_dict:
                poly_grid_dict[key] = StructuredGrid(
                    psurf=poly_patch_dict[key],
                    niv=self.vtk_resolution,
                    njv=self.vtk_resolution,
                )
            self.grids["polygons"].append(poly_grid_dict)

    def _write_to_vtk(
        self,
    ) -> None:
        """Writes grids to VTK."""
        if self.verbosity > 0:
            print("    Structure Grid Creates")
            print("    Writing grid files to {}-label.vtk".format(self.vtk_filename))

        # Polygons
        COUNTER = 0
        for poly_grid_dict in self.grids["polygons"]:
            COUNTER += 1
            for key in poly_grid_dict:
                poly_grid_dict[key].write_to_vtk_file(
                    "{}-poly{}_{}.vtk".format(self.vtk_filename, COUNTER, key)
                )

    def _create_surfaces(
        self,
    ) -> None:
        """Creates parameteric surfaces."""

        # Polygons
        self.surfaces["polygons"] = []
        for poly_patch_dict in self.patches["polygons"]:
            poly_stl_mesh_list = []
            for key in poly_patch_dict:
                poly_stl_mesh_list.append(
                    parametricSurfce2stl(poly_patch_dict[key], self.stl_resolution)
                )
            self.surfaces["polygons"].append(poly_stl_mesh_list)

    def _create_stl_data(self) -> None:
        """Creates STL data elements."""

        # Polygons
        self.stl_data["polygons"] = []
        for poly_stl_mesh_list in self.surfaces["polygons"]:
            poly_stl_data = []
            for val in poly_stl_mesh_list:
                poly_stl_data.append(val.data.copy())
            self.stl_data["polygons"].append(poly_stl_data)

    def _create_stl(self):
        """Creases stl objects"""

        # Polygons
        if len(self.polygons) > 0:
            self.meshes["polygons"] = []

            for poly_no, poly_stl_data in enumerate(self.stl_data["polygons"]):
                self.meshes["polygons"].append(mesh.Mesh(np.concatenate(poly_stl_data)))
                if self.verbosity > 0:
                    print("    poly stl object created")

    def _write_stl(self):
        """Writes STL files."""

        # Polygons
        if len(self.polygons) > 0:
            for poly_no, poly_mesh in enumerate(self.meshes["polygons"]):
                poly_filename = f"{self.stl_filename}-poly{poly_no+1}.stl"
                poly_mesh.save(poly_filename)
                if self.verbosity > 0:
                    print(f"    Writing polygon stl to - {poly_filename}")

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
        # Polygons
        if len(self.polygons) > 0:
            # Create a new plot
            figure = plt.figure()
            ax = mplot3d.Axes3D(figure)

            # Render the polygon(s)

            for poly_mesh in self.meshes["polygons"]:
                poly_coll = mplot3d.art3d.Poly3DCollection(poly_mesh.vectors)
                ax.add_collection3d(poly_coll)
                scale = poly_mesh.points.flatten()

            # Autoscale the mesh size
            ax.auto_scale_xyz(scale, scale, scale)
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.set_zlabel("Z-axis")

        plt.show()

    def _evaluate_mesh_properties(self):  ### NOT CURRENTLY SUPPORTED!!
        """Evaluates properties of stl."""


#        if len(self.wings) > 0:
#            print("")
#            print("START: Evaluating Wing Mesh Properties:")
#            for wing_no, wing_mesh in enumerate(self.meshes['wing']):
#                volume, cog, inertia = wing_mesh.get_mass_properties()
#                print(f"    WING {wing_no+1}")
#                print("    --------")
#                print("    Volume                                  = {0}".format(volume))
#                print("    Position of the center of gravity (COG) = {0}".format(cog))
#                print("    Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
#                print("                                              {0}".format(inertia[1,:]))
#                print("                                              {0}".format(inertia[2,:]))
#            print("  DONE: Evaluating Wing Mesh Properties")
#            print("")
#
#        if self.fuselage is not None:
#            print("")
#            print("START: Evaluating Fuselage Mesh Properties:")
#            volume, cog, inertia = self.meshes['fuselage'].get_mass_properties()
#            print("    Volume                                  = {0}".format(volume))
#            print("    Position of the center of gravity (COG) = {0}".format(cog))
#            print("    Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
#            print("                                              {0}".format(inertia[1,:]))
#            print("                                              {0}".format(inertia[2,:]))
#            print("  DONE: Evaluating Mesh Properties")
#            print("")
#
#        if len(self.fins) > 0:
#            print("")
#            print("START: Evaluating Fin Mesh Properties:")
#            for fin_no, fin_mesh in enumerate(self.meshes['fin']):
#                volume, cog, inertia = fin_mesh.get_mass_properties()
#                print(f"    Fin {fin_no+1}")
#                print("    --------")
#                print("    Volume                                  = {0}".format(volume))
#                print("    Position of the center of gravity (COG) = {0}".format(cog))
#                print("    Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
#                print("                                              {0}".format(inertia[1,:]))
#                print("                                              {0}".format(inertia[2,:]))
#            print("DONE Evaluating Fin Mesh Properties")
#            print("")
