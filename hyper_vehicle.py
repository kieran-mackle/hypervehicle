#!/usr/bin/python3.8
from __future__ import annotations
from getopt import getopt
import numpy as np
import os
import shutil
import sys
from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from eilmer.geom.sgrid import StructuredGrid

from idmoc.hypervehicle.utils import parametricSurfce2stl, CurvedPatch, RotatedPatch
from idmoc.hypervehicle.fusgen import hyper_fuselage_main
from idmoc.hypervehicle.wingen import hyper_wing_main
from idmoc.hypervehicle.fingen import hyper_fin_main


class Vehicle:
    """hypervehicle object.
    
    """
    
    def __init__(self, verbosity: int = 1) -> None:
        """Vehicle constructor method. 
        
        Parameters
        ----------
        verbosity : int, optional
            The verbosity of the code (default value is 1; moderate verbosity).
        """
        self.verbosity = verbosity
        self.global_config = None
        
        # TODO - include defaults here
        
        self.patches = {}
        self.grids = {}
        self.surfaces = {}
        self.stl_data = {}
        self.meshes = {}
        
    
    def __repr__(self):
        return "Parameterised hypersonic vehicle geometry."
    
    
    def add_component(component_type: str, component_dict: dict):
        """Adds a vehicle component.
        """
        # TODO - implement
        # Will append dict to a components dict, containing the component 
        # type and definition. Will allow for greater code efficiency
        
        
    def add_global_config(self, global_config: dict) -> None:
        """Unpacks a global configuration dictionary.
        
        Parameters
        ----------
        global_config : dict
            A dictionary containing a dictionary for each component to be generated.
        """
        
        # Read and check inputs
        global_config = self._read_inputs(global_config)
        self._check_inputs(global_config)
        
        # Unpack global config
        self._unpack_inputs(global_config)
        
        # Assign global config
        self.global_config = global_config
    
    
    @staticmethod
    def _read_inputs(global_config: dict) -> dict:
        """Reads all keys provided in the global configuration dictionary
        and replaces default values if present.
        """
        
        default_config = {"VERBOSITY": 1,               # 0, 1, 2 - set reporting level.
                          "VEHICLE_ANGLE": 0,           # Vehicle angle adjustment
                          "CREATE_WING": True,          # create wing geometry
                          "CREATE_FUSELAGE": False,     # create fuselage geometry
                          "CREATE_VTK_MESH": False,
                          "VTK_FILENAME": "test.vtk",
                          "VTK_RESOLUTION": 10,         # number of cell vertices per edge
                          "CREATE_STL_OBJECT": True,
                          "STL_FILENAME": "test.stl",
                          "STL_RESOLUTION": 10,         # number of triangle vertices per edge
                          "STL_INCLUDE_MIRROR": True,   # include mirror image in STL
                          "STL_SHOW_IN_MATPLOT": False, # Create Matplotlib image
                          "WING_GEOMETRY_DICT": None,
                          "FUSELAGE_GEOMETRY_DICT": None,
                          "FIN_GEOMETRY_DICT": [None],
                          "MIRROR_FINS": True}
        
        for option, value in global_config.items():
            if option not in default_config:
                raise Exception(f'Invalid configuration variable "{option}"'+\
                                ' provided. Please check and try again.')
            
            # Overwrite
            default_config[option] = value
        
        return default_config
        
    
    def _unpack_inputs(self, global_config: dict) -> None:
        """Unpacks global configuration dictionary and assigns attributes
        to Vehicle.
        """
        self.verbosity = global_config["VERBOSITY"]
        self.wings = global_config["WING_GEOMETRY_DICT"]
        self.fuselage = global_config["FUSELAGE_GEOMETRY_DICT"]
        self.fins = global_config["FIN_GEOMETRY_DICT"]
        self.mirror_fins = global_config["MIRROR_FINS"]
        self.vehicle_angle = global_config["VEHICLE_ANGLE"]
        
        # STL
        self.write_stl = global_config["CREATE_STL_OBJECT"]
        self.stl_filename = global_config["STL_FILENAME"]
        self.stl_resolution = global_config["STL_RESOLUTION"]
        self.mirror = global_config["STL_INCLUDE_MIRROR"]
        self.show_mpl = global_config["STL_SHOW_IN_MATPLOT"]
        
        # VTK 
        self.write_vtk = global_config["CREATE_VTK_MESH"]
        self.vtk_resolution = global_config["VTK_RESOLUTION"]
        self.vtk_filename = global_config["VTK_FILENAME"]
        
    
    @staticmethod
    def _check_inputs(global_config: dict) -> None:
        """Checks that global config inputs are valid.
        
        The function checks that the values provided in the global 
        configuration dictionary are in the set of valid inputs for that
        option.
        """
        
        # Define valid inputs
        valid_input_dict = {"VERBOSITY": [0, 1, 2, 3],
                            "CREATE_WING": [True, False],
                            "CREATE_FUSELAGE": [True, False],
                            "CREATE_VTK_MESH": [True, False],
                            "CREATE_STL_OBJECT": [True, False],
                            "STL_INCLUDE_MIRROR": [True, False],
                            "STL_SHOW_IN_MATPLOT": [True, False],
                            "WING_GEOMETRY_DICT": {"Line_B0TT_TYPE": ["Bezier"],
                                                   "TAIL_OPTION": ["NONE", "FLAP"],},
                            "FUSELAGE_GEOMETRY_DICT": {"FUSELAGE_NOSE_OPTION": ["sharp-cone"],
                                                       "FUSELAGE_TAIL_OPTION": ["sharp-cone", "flat"],},
                            "MIRROR_FINS": [True, False],
                            }
        
        # Ensure wing and fin geometry dict(s) in list form
        for component in ['WING', 'FIN']:
            if (f'{component}_GEOMETRY_DICT' in global_config) and (type(global_config[f'{component}_GEOMETRY_DICT']) != list):
                global_config[f'{component}_GEOMETRY_DICT'] = [global_config[f'{component}_GEOMETRY_DICT']]
        
        # Check provided options
        for option, valid_inputs in valid_input_dict.items():
            if option == "WING_GEOMETRY_DICT":
                # Check inputs for wing geometry dictionary
                for wing_geom_dict in global_config["WING_GEOMETRY_DICT"]:
                    for wing_option, valid_wing_inputs in valid_inputs.items():
                        if wing_geom_dict[wing_option] not in valid_wing_inputs:
                            raise Vehicle.InvalidInputError(wing_option, valid_wing_inputs)
            
            elif option == "FUSELAGE_GEOMETRY_DICT":
                # Check inputs for fuselage geometry dictionary
                if global_config["FUSELAGE_GEOMETRY_DICT"] is not None:
                    for fuse_option, valid_fuse_inputs in valid_inputs.items():
                        if global_config["FUSELAGE_GEOMETRY_DICT"][fuse_option] not in valid_fuse_inputs:
                            raise Vehicle.InvalidInputError(fuse_option, valid_fuse_inputs)
            
            elif global_config[option] not in valid_inputs:
                raise Vehicle.InvalidInputError(option, valid_inputs)
    
    
    class InvalidInputError(Exception):
        """Invalid input exception.
        """
        
        def __init__(self, option, valid_inputs):
            self.option = option
            self.message = f'Value provided for option "{option}" is ' +\
                f'not valid. Valid options are: {valid_inputs}.'
            super().__init__(self.message)
        
        def __str__(self):
            return f'{self.message}'
        
    
    def main(self) -> None:
        """Run hypervehicle geometry generation code.
        """
        
        # Create the vehicle component patches
        self.patches['wing'] = hyper_wing_main(self.wings)
        self.patches['fuselage'] = hyper_fuselage_main(self.fuselage)
        self.patches['fin'] = hyper_fin_main(self.fins)
    

        # Add curvature
        if self.verbosity > 0:
            print("")
            print("START: Adding curvature.")
        
        self._add_curvature()
        
        if self.verbosity > 0:
            print("  DONE: Adding curvature.")
            print("")
        
        
        # Add vehicle offset angle to correct any curve induced AOA change
        if self.vehicle_angle != 0:
            self._rotate_vehicle()
            
            
        # Eilmer Grid surface grids
        if self.write_vtk:
            if self.verbosity > 0:
                print("START: Creating Eilmer Grid and vtk files.")
            
            self._create_grids()
            self._write_to_vtk()
            
            if self.verbosity > 0:
                print("  DONE: Creating Eilmer Grid and vtk files.")
                print("")
    
        
        # STL object
        if self.write_stl:
            if self.verbosity > 0:
                print("START: Creating STL object(s).")
                print(f"    Creating STL with resolution of {self.stl_resolution}")
            
            self._create_surfaces()
            self._create_stl_data()
            self._create_stl()
            self._write_stl()
            print("  DONE: Creating STL object(s).")
    
            self._evaluate_mesh_properties()
    
            if self.show_mpl:
                if self.verbosity > 0:
                    print("START: Show in matplotlib")
        
                self._mpl_plot()
                
                if self.verbosity > 0:
                    print("  DONE: Show in matplotlib")
                    print("")
    
    
    def _add_curvature(self,) -> None:
        """Adds curvature by the provided curvature functions.
        """
        # Wing curvature
        if self.wings is not None:
            for ix, wing_geometry_dict in enumerate(self.wings):
                # Curvature about the x-axis
                if wing_geometry_dict['WING_FUNC_CURV_X'] is None and \
                    wing_geometry_dict['WING_FUNC_CURV_X_DASH'] is None:
                    print("    Skipping wing X-curvature.")
                else:
                    # (a) Longitudal Curvature
                    for key in self.patches['wing'][ix]:
                        self.patches['wing'][ix][key] = \
                            CurvedPatch(underlying_surf=self.patches['wing'][ix][key],
                                        direction='x', fun=wing_geometry_dict['WING_FUNC_CURV_X'],
                                        fun_dash=wing_geometry_dict['WING_FUNC_CURV_X_DASH'])
                
                # Curvature about the y-axis
                if wing_geometry_dict['WING_FUNC_CURV_Y'] is None and \
                    wing_geometry_dict['WING_FUNC_CURV_Y_DASH'] is None:
                    print("    Skipping wing Y-curvature.")
                else:
                    # (b) Spanwise Curvature
                    for key in self.patches['wing'][ix]:
                        self.patches['wing'][ix][key] = \
                            CurvedPatch(underlying_surf=self.patches['wing'][ix][key],
                                        direction='y', fun=wing_geometry_dict['WING_FUNC_CURV_Y'],
                                        fun_dash=wing_geometry_dict['WING_FUNC_CURV_Y_DASH'])
                    
        # Fuselage curvature       
        if self.fuselage is not None:
            
            # Longitudal Curvature
            if self.fuselage['FUSELAGE_FUNC_CURV_X'] is None and \
                self.fuselage['FUSELAGE_FUNC_CURV_X_DASH'] is None:
                print("    Skipping fuselage X-curvature.")
                
            else:
                for key in self.patches['fuselage']:
                    self.patches['fuselage'][key] = \
                        CurvedPatch(underlying_surf=self.patches['fuselage'][key],
                                    direction='x', fun=self.fuselage['FUSELAGE_FUNC_CURV_X'],
                                    fun_dash=self.fuselage['FUSELAGE_FUNC_CURV_X_DASH'])
            # Spanwise Curvature
            if self.fuselage['FUSELAGE_FUNC_CURV_Y'] is None and \
                self.fuselage['FUSELAGE_FUNC_CURV_Y_DASH'] is None:
                print("    Skipping fuselage Y-curvature.")
            else:
                for key in self.patches['fuselage']:
                    self.patches['fuselage'][key] = \
                        CurvedPatch(underlying_surf=self.patches['fuselage'][key],
                                    direction='y', fun=self.fuselage['FUSELAGE_FUNC_CURV_Y'],
                                    fun_dash=self.fuselage['FUSELAGE_FUNC_CURV_Y_DASH'])
        
    
    def _rotate_vehicle(self):
        """Rotates entire vehicle to add offset angle.
        """
        # Rotate wings
        for ix, wing_geometry_dict in enumerate(self.wings):
            if wing_geometry_dict is not None:
                for key in self.patches['wing'][ix]:
                    self.patches['wing'][ix][key] = RotatedPatch(self.patches['wing'][ix][key], 
                                                            np.deg2rad(self.vehicle_angle), 
                                                            axis='y')
        
        # Rotate fins
        for ix, fin_geometry_dict in enumerate(self.fins):
            if fin_geometry_dict is not None:
                for key in self.patches['fin'][ix]:
                    self.patches['fin'][ix][key] = RotatedPatch(self.patches['fin'][ix][key], 
                                                           np.deg2rad(self.vehicle_angle), 
                                                           axis='y')
        
        # Rotate fuselage
        for key in self.patches['fuselage']:
            self.patches['fuselage'][key] = RotatedPatch(self.patches['fuselage'][key], 
                                                np.deg2rad(self.vehicle_angle), 
                                                axis='y')
    
    def _create_grids(self) -> None:
        """Writes VTK files.
        """
        # Wings
        self.grids['wing'] = []
        for wing_patch_dict in self.patches['wing']:
            wing_grid_dict = {}
            for key in wing_patch_dict:
                wing_grid_dict[key] = StructuredGrid(psurf=wing_patch_dict[key],
                              niv=self.vtk_resolution, njv=self.vtk_resolution)
            self.grids['wing'].append(wing_grid_dict)
            
        # Fuselage
        self.grids['fuselage'] = {}
        for key in self.patches['fuselage']:
            self.grids['fuselage'][key] = StructuredGrid(psurf=self.patches['fuselage'][key],
                                                         niv=self.vtk_resolution, 
                                                         njv=self.vtk_resolution)
        
        # Fins
        self.grids['fin'] = []
        for fin_patch_dict in self.patches['fin']:
            fin_grid_dict = {}
            for key in fin_patch_dict:
                fin_grid_dict[key] = StructuredGrid(psurf=fin_patch_dict[key],
                              niv=self.vtk_resolution, njv=self.vtk_resolution)
            self.grids['fin'].append(fin_grid_dict)
        
    
    def _write_to_vtk(self,) -> None:
        """Writes grids to VTK.
        """
        if self.verbosity > 0:
            print("    Structure Grid Creates")
            print("    Writing grid files to {}-label.vtk".format(self.vtk_filename))
            
        # Wings
        for wing_grid_dict in  self.grids['wing']:
            for key in wing_grid_dict:
                wing_grid_dict[key].write_to_vtk_file("{0}-wing_{1}.vtk".format(self.vtk_filename, key))

        # Fuselage
        for key in self.grids['fuselage']:
            self.grids['fuselage'][key].write_to_vtk_file("{0}-fuse_{1}.vtk".format(self.vtk_filename, key))
        
        # Fins
        for fin_grid_dict in self.grids['fin']:
            for key in fin_grid_dict:
                fin_grid_dict[key].write_to_vtk_file(f"{self.vtk_filename}-fin_{key}.vtk")
                
    
    def _create_surfaces(self,) -> None:
        """Creates parameteric surfaces.
        """
        
        # Wings
        self.surfaces['wing'] = []
        for wing_patch_dict in self.patches['wing']:
            wing_stl_mesh_list = []
            for key in wing_patch_dict:
                wing_stl_mesh_list.append(parametricSurfce2stl(wing_patch_dict[key],
                                                               self.stl_resolution))
            self.surfaces['wing'].append(wing_stl_mesh_list)
        
        # Fuselage
        self.surfaces['fuselage'] = []
        for key in self.patches['fuselage']:
            self.surfaces['fuselage'].append(parametricSurfce2stl(self.patches['fuselage'][key],
                                                                self.stl_resolution))
        
        # Fins
        self.surfaces['fin'] = []
        for fin_patch_dict in self.patches['fin']:
            fin_stl_mesh_list = []
            for key in fin_patch_dict:
                fin_stl_mesh_list.append(parametricSurfce2stl(fin_patch_dict[key],
                                                              self.stl_resolution))
            self.surfaces['fin'].append(fin_stl_mesh_list)
        
    
    def _create_stl_data(self) -> None:
        """Creates STL data elements.
        """
        # Wings
        self.stl_data['wing'] = []
        for wing_stl_mesh_list in self.surfaces['wing']:
            wing_stl_data = []
            for val in wing_stl_mesh_list:
                wing_stl_data.append(val.data.copy())
            self.stl_data['wing'].append(wing_stl_data)
        
        # Fuselage
        self.stl_data['fuselage'] = []
        for val in self.surfaces['fuselage']:
            self.stl_data['fuselage'].append(val.data.copy())
        
        # Fins
        self.stl_data['fin'] = []
        for fin_stl_mesh_list in self.surfaces['fin']:
            fin_stl_data = []
            for val in fin_stl_mesh_list:
                fin_stl_data.append(val.data.copy())
            self.stl_data['fin'].append(fin_stl_data)
        
        # add stl elements for mirrored section
        if self.mirror:
            if self.verbosity > 0:
                print("    Adding Mirror Image of wing.")
            
            all_wing_stl_mesh_list_m = []
            for wing_patch_dict in self.patches['wing']:
                wing_stl_mesh_list_m = []
                for key in wing_patch_dict:
                    wing_stl_mesh_list_m.append(parametricSurfce2stl(
                        wing_patch_dict[key], self.stl_resolution, mirror_y=True))
                    
                all_wing_stl_mesh_list_m.append(wing_stl_mesh_list_m)
                
            # add elements to stl_data lists
            for ix, wing_stl_mesh_list_m in enumerate(all_wing_stl_mesh_list_m):
                for val in wing_stl_mesh_list_m:
                    self.stl_data['wing'][ix].append(val.data.copy())
        
        if self.mirror_fins: 
            # Repeat for fins
            if self.verbosity > 0:
                print("    Adding Mirror Image of fin.")
            
            all_fin_stl_mesh_list_m = []
            for fin_patch_dict in self.patches['fin']:
                fin_stl_mesh_list_m = []
                for key in fin_patch_dict:
                    fin_stl_mesh_list_m.append(parametricSurfce2stl(
                        fin_patch_dict[key], self.stl_resolution, mirror_y=True))
                    
                all_fin_stl_mesh_list_m.append(fin_stl_mesh_list_m)
                
            # add elements to stl_data lists
            for ix, fin_stl_mesh_list_m in enumerate(all_fin_stl_mesh_list_m):
                for val in fin_stl_mesh_list_m:
                    self.stl_data['fin'][ix].append(val.data.copy())
    
    def _create_stl(self):
        """Creases stl objects
        """
        # Wings
        if self.wings is not None:
            # Create stl object
            self.meshes['wing'] = []
            for wing_no, wing_stl_data in enumerate(self.stl_data['wing']):
                self.meshes['wing'].append(mesh.Mesh(np.concatenate(wing_stl_data)))
                
        # Fuselage
        if self.fuselage is not None:
            self.meshes['fuselage'] = mesh.Mesh(np.concatenate(self.stl_data['fuselage']))
            if self.verbosity > 0:
                    print("    fuselage stl object created")

        # Fins
        if self.fins is not None:
            self.meshes['fin'] = []
            
            for fin_no, fin_stl_data in enumerate(self.stl_data['fin']):
                self.meshes['fin'].append(mesh.Mesh(np.concatenate(fin_stl_data)))
                if self.verbosity > 0:
                        print("    fin stl object created")

    
    def _write_stl(self):
        """Writes STL files.
        """
        # Wings
        if self.wings is not None:
            # Create stl object
            for wing_no, wing_mesh in enumerate(self.meshes['wing']):
                # Save to .stl file
                wing_filename = f"{self.stl_filename}-wing{wing_no+1}.stl"
                wing_mesh.save(wing_filename)
                if self.verbosity > 0:
                    print(f"    Wing {wing_no+1} stl object created - written to {wing_filename}.")
            
        # Fuselage
        if self.fuselage is not None:
            fuse_filename = "{}-fuse.stl".format(self.stl_filename)
            self.meshes['fuselage'].save(fuse_filename)
            if self.verbosity > 0:
                print(f"    Writing fuselage stl to - {fuse_filename}")
        
        # Fins
        if self.fins is not None:
            for fin_no, fin_mesh in enumerate(self.meshes['fin']):
                fin_filename = f"{self.stl_filename}-fin{fin_no+1}.stl"
                fin_mesh.save(fin_filename)
                if self.verbosity > 0:
                    print(f"    Writing fin stl to - {fin_filename}")
        
    
    def _evaluate_mesh_properties(self,):
        """Evaluates properties of stl.
        """
        if self.wings[0] is not None:
            print("")
            print("START: Evaluating Wing Mesh Properties:")
            for wing_no, wing_mesh in enumerate(self.meshes['wing']):
                volume, cog, inertia = wing_mesh.get_mass_properties()
                print(f"    WING {wing_no+1}")
                print("    --------")
                print("    Volume                                  = {0}".format(volume))
                print("    Position of the center of gravity (COG) = {0}".format(cog))
                print("    Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
                print("                                              {0}".format(inertia[1,:]))
                print("                                              {0}".format(inertia[2,:]))
            print("  DONE: Evaluating Wing Mesh Properties")
            print("")
            
        if self.fuselage is not None:
            print("")
            print("START: Evaluating Fuselage Mesh Properties:")
            volume, cog, inertia = self.meshes['fuselage'].get_mass_properties()
            print("    Volume                                  = {0}".format(volume))
            print("    Position of the center of gravity (COG) = {0}".format(cog))
            print("    Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
            print("                                              {0}".format(inertia[1,:]))
            print("                                              {0}".format(inertia[2,:]))
            print("  DONE: Evaluating Mesh Properties")
            print("")
            
        if self.fins[0] is not None:
            print("")
            print("START: Evaluating Fin Mesh Properties:")
            for fin_no, fin_mesh in enumerate(self.meshes['fin']):
                volume, cog, inertia = fin_mesh.get_mass_properties()
                print(f"    Fin {fin_no+1}")
                print("    --------")
                print("    Volume                                  = {0}".format(volume))
                print("    Position of the center of gravity (COG) = {0}".format(cog))
                print("    Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
                print("                                              {0}".format(inertia[1,:]))
                print("                                              {0}".format(inertia[2,:]))
            print("DONE Evaluating Fin Mesh Properties")
            print("")
    
    
    def _mpl_plot(self,):
        """Plots stl components with matplotlib.
        """
        if self.wings is not None:
            # Create a new plot
            figure = plt.figure()
            ax = mplot3d.Axes3D(figure)

            # Render the wing(s)
            for wing_mesh in self.meshes['wing']:
               ax.add_collection3d(mplot3d.art3d.Poly3DCollection(wing_mesh.vectors))
               scale = wing_mesh.points.flatten()
            
            # Auto scale to the mesh size
            ax.auto_scale_xyz(scale, scale, scale)
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.set_zlabel("Z-axis")
            
        if self.fuselage is not None:
            # Create a new plot
            figure = plt.figure()
            ax = mplot3d.Axes3D(figure)

            # Render the fuselage
            ax.add_collection3d(mplot3d.art3d.Poly3DCollection(self.meshes['fuselage'].vectors))
            # Auto scale to the mesh size
            scale = self.meshes['fuselage'].points.flatten()
            ax.auto_scale_xyz(scale, scale, scale)
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.set_zlabel("Z-axis")
            
        if self.wings is not None and self.fuselage is not None:
            # Create a new plot
            figure = plt.figure()
            ax = mplot3d.Axes3D(figure)

            # Render the wing and fuselage
            for wing_mesh in self.meshes['wing']:
                wing_coll = mplot3d.art3d.Poly3DCollection(wing_mesh.vectors)
                ax.add_collection3d(wing_coll)
                scale = wing_mesh.points.flatten()
                
            fuse_coll = mplot3d.art3d.Poly3DCollection(self.meshes['fuselage'].vectors)
            fuse_coll.set_facecolor('r')
            ax.add_collection3d(fuse_coll)
            
            if self.fins[0] is not None:
                for fin_mesh in self.meshes['fin']:
                    fin_coll = mplot3d.art3d.Poly3DCollection(fin_mesh.vectors)
                    fin_coll.set_facecolor('c')
                    ax.add_collection3d(fin_coll)

            # Auto scale to the mesh size
            ax.auto_scale_xyz(scale, scale, scale)
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.set_zlabel("Z-axis")

        # Show the plot to the screen
        plt.show()
        
        
    def generate(self):
        """Public API to generate vehicle geometry.
        """
    
    @classmethod
    def from_config(cls, global_config: dict, verbosity: int = 1) -> Vehicle:
        """Creates hypervehicle Vehicle object directly from global configuration
        dictionary.

        Parameters
        ----------
        global_config : dict
            A dictionary containing a dictionary for each component to be generated..
        verbosity : int, optional
            The verbosity of the code. The default is 1.

        Returns
        -------
        Vehicle
            Parameterised hypersonic vehicle geometry.
        """
        
        # TODO - implement
    
    
    @staticmethod
    def usage():
        print("Hypersonic Vehicle Geometry Generation Tool")
        print("============================================")
        print("Usage Instructions\n")
        print("  vehicle_gen.py --job=jobfile    ")
        print("")
        print("Argument:                    Comment:")
        print("------------------------------------------------------------------------")
        print(" --job_template       Create a template job-file.")
        print(" --help               Displays this message.")
        print(" --instructions       Displays and overview of run instructions.")
        print(" --job=               String containing file name for job-file.")
        print("                          dictionary.")
        print("\n Please refer to the hypervehicle docs for more information.")
    
    @staticmethod
    def create_template(directory: str = None) -> None:
        """Copies a hypervehicle job template to the specified directoty.
        
        Parameters
        ----------
        directory : str
            The absolute path of the working directory (where the template is
            to be copied to).
        """
        # TODO - copy from idmoc bin (use MANIFEST.in ?)
        if directory is not None:
            install_dir = os.environ["IDMOC"]
            template_job_location = os.path.join(install_dir, 'bin', 'job_template.py')
            shutil.copy(template_job_location, directory)
        else:
            raise Exception("Please provide the absolute path to your " + \
                            "working directory.")

    @staticmethod
    def load_global_config(uodict: dict) -> dict:
        """Loads the global config dict from a hypervehicle job file.
        
        Parameters
        ----------
        uodict : dict
            The user-options dictionary.
        """
        # Read the jobfile
        exec(open(uo_dict["--job"]).read(), globals())
        
        return global_config

    @staticmethod
    def check_uo_dict(uo_dict: dict) -> None:
        """Checks all mandatory options have been provided.
        
        Parameters
        ----------
        uodict : dict
            The user-options dictionary.
        """
        reqd_options = ["--job"]
        for op in reqd_options:
            if op not in uo_dict:
                raise Exception("".join(("bulk_run_CFD.py requires argument '",
                                  op, "', but this argument was not provided.\n")))
    
        # Check that jobfile exists
        if not os.path.isfile(uo_dict["--job"]):
            raise Exception("".join(("Jobfile '", uo_dict["--job"], "' not found,",
                              " check that file path is correct.\n")))
        

short_options = ""
long_options = ["help", "job=", "job_template"]

if __name__ == '__main__':
    user_options = getopt(sys.argv[1:], short_options, long_options)
    uo_dict = dict(user_options[0])
    
    # Create vehicle instance
    hv = Vehicle()
        
    if len(user_options[0]) == 0 or "--help" in uo_dict:
        hv.usage()

    elif "--job_template" in uo_dict:
        hv.create_template(os.getcwd())

    else:
        hv.check_uo_dict(uo_dict)
        global_config = hv.load_global_config(uo_dict)
        hv.add_global_config(global_config)
        hv.main()