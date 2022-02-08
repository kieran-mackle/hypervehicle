#!/usr/bin/python3.8

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
from idmoc.hypervehicle.config import GConf
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
        
    
    def __repr__(self):
        return "Parameterised hypersonic vehicle geometry."
    
    
    def add_component(component_type: str, component_dict: dict):
        """Adds a vehicle component.
        """
        
        
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
                          "MIRROR_FIN": True}
        
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
        self.vehicle_angle = global_config["VEHICLE_ANGLE"]
        
        # STL
        self.write_stl = global_config["CREATE_STL_OBJECT"]
        self.stl_filename = global_config["STL_FILENAME"]
        self.stl_resolution = global_config["STL_RESOLUTION"]
        self.mirror = global_config["STL_INCLUDE_MIRROR"]
        
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
                            "MIRROR_FIN": [True, False],
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
        
        GConf.read_inputs(self.global_config)
        GConf.check_inputs()
        
        # create the vehicle components
        if self.wings[0] is not None:
            wing_patch_list = hyper_wing_main(self.wings)
        else:
            wing_patch_list = []
            
        if self.fuselage is not None:
            fuse_patch_dict = hyper_fuselage_main(self.fuselage)
        else:
            fuse_patch_dict = {}
        
        if self.fins[0] is not None:
            fin_patch_list = hyper_fin_main(self.fins)
        else:
            fin_patch_list = []
    
        # Step 5:
        #########
        # Add curvature
        if self.verbosity > 0:
            print("")
            print("START: Adding curvature.")
        
        # Wing curvature
        if self.wings is not None:
            for ix, wing_geometry_dict in enumerate(self.wings):
                # Curvature about the x-axis
                if wing_geometry_dict['WING_FUNC_CURV_X'] is None and \
                    wing_geometry_dict['WING_FUNC_CURV_X_DASH'] is None:
                    print("    Skipping wing X-curvature.")
                else:
                    # (a) Longitudal Curvature
                    for key in wing_patch_list[ix]:
                        wing_patch_list[ix][key] = \
                            CurvedPatch(underlying_surf=wing_patch_list[ix][key],
                                        direction='x', fun=wing_geometry_dict['WING_FUNC_CURV_X'],
                                        fun_dash=wing_geometry_dict['WING_FUNC_CURV_X_DASH'])
                
                # Curvature about the y-axis
                if wing_geometry_dict['WING_FUNC_CURV_Y'] is None and \
                    wing_geometry_dict['WING_FUNC_CURV_Y_DASH'] is None:
                    print("    Skipping wing Y-curvature.")
                else:
                    # (b) Spanwise Curvature
                    for key in wing_patch_list[ix]:
                        wing_patch_list[ix][key] = \
                            CurvedPatch(underlying_surf=wing_patch_list[ix][key],
                                        direction='y', fun=wing_geometry_dict['WING_FUNC_CURV_Y'],
                                        fun_dash=wing_geometry_dict['WING_FUNC_CURV_Y_DASH'])
                    
        # Fuselage curvature       
        if self.fuselage is not None:
            
            # Longitudal Curvature
            if self.fuselage['FUSELAGE_FUNC_CURV_X'] is None and \
                self.fuselage['FUSELAGE_FUNC_CURV_X_DASH'] is None:
                print("    Skipping fuselage X-curvature.")
                
            else:
                for key in fuse_patch_dict:
                    fuse_patch_dict[key] = \
                        CurvedPatch(underlying_surf=fuse_patch_dict[key],
                                    direction='x', fun=self.fuselage['FUSELAGE_FUNC_CURV_X'],
                                    fun_dash=self.fuselage['FUSELAGE_FUNC_CURV_X_DASH'])
            # Spanwise Curvature
            if self.fuselage['FUSELAGE_FUNC_CURV_Y'] is None and \
                self.fuselage['FUSELAGE_FUNC_CURV_Y_DASH'] is None:
                print("    Skipping fuselage Y-curvature.")
            else:
                for key in fuse_patch_dict:
                    fuse_patch_dict[key] = \
                        CurvedPatch(underlying_surf=fuse_patch_dict[key],
                                    direction='y', fun=self.fuselage['FUSELAGE_FUNC_CURV_Y'],
                                    fun_dash=self.fuselage['FUSELAGE_FUNC_CURV_Y_DASH'])
        
        if self.verbosity > 0:
            print("  DONE: Adding curvature.")
            print("")
        
        
        # Add vehicle offset angle to correct any curve induced AOA change
        if self.vehicle_angle != 0:
            # Rotate wings
            for ix, wing_geometry_dict in enumerate(self.wings):
                for key in wing_patch_list[ix]:
                    wing_patch_list[ix][key] = RotatedPatch(wing_patch_list[ix][key], 
                                                            np.deg2rad(self.vehicle_angle), 
                                                            axis='y')
            
            # Rotate fins
            for ix, fin_geometry_dict in enumerate(self.fins):
                for key in fin_patch_list[ix]:
                    fin_patch_list[ix][key] = RotatedPatch(fin_patch_list[ix][key], 
                                                           np.deg2rad(self.vehicle_angle), 
                                                           axis='y')
            
            # Rotate fuselage
            for key in fuse_patch_dict:
                fuse_patch_dict[key] = RotatedPatch(fuse_patch_dict[key], 
                                                    np.deg2rad(self.vehicle_angle), 
                                                    axis='y')
            
        # Step 6:
        #########
        # Eilmer Grid surface grids
        if self.write_vtk:
            if self.verbosity > 0:
                print("START: Creating Eilmer Grid and vtk files.")
            
            # Wings
            wing_grid_list = []
            for wing_patch_dict in wing_patch_list:
                wing_grid_dict = {}
                for key in wing_patch_dict:
                    wing_grid_dict[key] = StructuredGrid(psurf=wing_patch_dict[key],
                                  niv=self.vtk_resolution, njv=self.vtk_resolution)
                wing_grid_list.append(wing_grid_dict)
                
            # Fuselage
            fuse_grid_dict = {}
            for key in fuse_patch_dict:
                fuse_grid_dict[key] = StructuredGrid(psurf=fuse_patch_dict[key],
                              niv=self.vtk_resolution, njv=self.vtk_resolution)
            
            # Fins
            fin_grid_list = []
            for fin_patch_dict in fin_patch_list:
                fin_grid_dict = {}
                for key in fin_patch_dict:
                    fin_grid_dict[key] = StructuredGrid(psurf=fin_patch_dict[key],
                                  niv=self.vtk_resolution, njv=self.vtk_resolution)
                fin_grid_list.append(fin_grid_dict)
            
            if self.verbosity > 0:
                print("    Structure Grid Creates")
                print("    Writing grid files to {}-label.vtk".format(self.vtk_filename))
    
    
            # Step 7:
            #########
            # write the grids to VTK
            
            # Wings
            for wing_grid_dict in  wing_grid_list:
                for key in wing_grid_dict:
                    wing_grid_dict[key].write_to_vtk_file("{0}-wing_{1}.vtk".format(self.vtk_filename, key))
    
            # Fuselage
            for key in fuse_grid_dict:
                fuse_grid_dict[key].write_to_vtk_file("{0}-fuse_{1}.vtk".format(self.vtk_filename, key))
            
            # Fins
            for fin_grid_dict in fin_grid_list:
                for key in fin_grid_dict:
                    fin_grid_dict[key].write_to_vtk_file(f"{self.vtk_filename}-fin_{key}.vtk")
            
            if self.verbosity > 0:
                print("  DONE: Creating Eilmer Grid and vtk files.")
                print("")
    
        # Step 8:
        #########
        # STL object
        if self.write_stl:
            if self.verbosity > 0:
                print("START: Creating STL object(s).")
                print(f"    Creating STL with resolution of {self.stl_resolution}")
            
            # Wings
            all_wing_stl_mesh_list = []
            for wing_patch_dict in wing_patch_list:
                wing_stl_mesh_list = []
                for key in wing_patch_dict:
                    wing_stl_mesh_list.append(parametricSurfce2stl(wing_patch_dict[key],
                                              self.stl_resolution))
                all_wing_stl_mesh_list.append(wing_stl_mesh_list)
            
            # Fuselage
            fuse_stl_mesh_list = []
            for key in fuse_patch_dict:
                fuse_stl_mesh_list.append(parametricSurfce2stl(fuse_patch_dict[key],
                                          self.stl_resolution))
            
            # Fins
            all_fin_stl_mesh_list = []
            for fin_patch_dict in fin_patch_list:
                fin_stl_mesh_list = []
                for key in fin_patch_dict:
                    fin_stl_mesh_list.append(parametricSurfce2stl(fin_patch_dict[key],
                                              self.stl_resolution))
                all_fin_stl_mesh_list.append(fin_stl_mesh_list)
    
            # create data structure containting all stl elements
            # Wings
            all_wing_stl_data = []
            for wing_stl_mesh_list in all_wing_stl_mesh_list:
                wing_stl_data = []
                for val in wing_stl_mesh_list:
                    wing_stl_data.append(val.data.copy())
                all_wing_stl_data.append(wing_stl_data)
            
            # Fuselage
            fuse_stl_data = []
            for val in fuse_stl_mesh_list:
                fuse_stl_data.append(val.data.copy())
            
            # Fins
            all_fin_stl_data = []
            for fin_stl_mesh_list in all_fin_stl_mesh_list:
                fin_stl_data = []
                for val in fin_stl_mesh_list:
                    fin_stl_data.append(val.data.copy())
                all_fin_stl_data.append(fin_stl_data)
            
            # add stl elements for mirrored section
            if self.mirror:
                if self.verbosity > 0:
                    print("    Adding Mirror Image of wing.")
                
                all_wing_stl_mesh_list_m = []
                for wing_patch_dict in wing_patch_list:
                    wing_stl_mesh_list_m = []
                    for key in wing_patch_dict:
                        wing_stl_mesh_list_m.append(parametricSurfce2stl(
                            wing_patch_dict[key], self.stl_resolution, mirror_y=True))
                        
                    all_wing_stl_mesh_list_m.append(wing_stl_mesh_list_m)
                    
                # add elements to stl_data lists
                for ix, wing_stl_mesh_list_m in enumerate(all_wing_stl_mesh_list_m):
                    for val in wing_stl_mesh_list_m:
                        all_wing_stl_data[ix].append(val.data.copy())
            
            if GConf.MIRROR_FIN: 
                # Repeat for fins
                if self.verbosity > 0:
                    print("    Adding Mirror Image of fin.")
                
                all_fin_stl_mesh_list_m = []
                for fin_patch_dict in fin_patch_list:
                    fin_stl_mesh_list_m = []
                    for key in fin_patch_dict:
                        fin_stl_mesh_list_m.append(parametricSurfce2stl(
                            fin_patch_dict[key], self.stl_resolution, mirror_y=True))
                        
                    all_fin_stl_mesh_list_m.append(fin_stl_mesh_list_m)
                    
                # add elements to stl_data lists
                for ix, fin_stl_mesh_list_m in enumerate(all_fin_stl_mesh_list_m):
                    for val in fin_stl_mesh_list_m:
                        all_fin_stl_data[ix].append(val.data.copy())
            
            
            # create stl mesh and save to file
            # Wings
            if self.wings is not None:
                # Create stl object
                for wing_no, wing_stl_data in enumerate(all_wing_stl_data):
                    wing_stl_mesh = mesh.Mesh(np.concatenate(wing_stl_data))
                    
                    # Save to .stl file
                    wing_filename = f"{GConf.STL_FILENAME}-wing{wing_no+1}.stl"
                    wing_stl_mesh.save(wing_filename)
                    if self.verbosity > 0:
                        print(f"    Wing {wing_no+1} stl object created - written to {wing_filename}.")
                
            # Fuselage
            if GConf.CREATE_FUSELAGE == True:
                fuse_stl_mesh = mesh.Mesh(np.concatenate(fuse_stl_data))
                if self.verbosity > 0:
                        print("    fuselage stl object created")
    
                fuse_filename = "{}-fuse.stl".format(GConf.STL_FILENAME)
                fuse_stl_mesh.save(fuse_filename)
                if self.verbosity > 0:
                    print("    Writing stl to - {}".format(fuse_filename))
            
            # Fins
            if self.fins is not None:
                for fin_no, fin_stl_data in enumerate(all_fin_stl_data):
                    fin_stl_mesh = mesh.Mesh(np.concatenate(fin_stl_data))
                    if self.verbosity > 0:
                            print("    fin stl object created")
            
                    fin_filename = f"{self.stl_filename}-fin{fin_no+1}.stl"
                    fin_stl_mesh.save(fin_filename)
                    if self.verbosity > 0:
                        print("    Writing stl to - {}".format(fin_filename))
            
                    if self.verbosity > 0:
                        print("    Done writing stl file")
            print("  DONE: Creating STL object(s).")
    
    
            if self.wings[0] is not None:
                print("")
                print("START: Evaluating Wing Mesh Properties:")
                for wing_no, wing_stl_data in enumerate(all_wing_stl_data):
                    volume, cog, inertia = wing_stl_mesh.get_mass_properties()
                    print(f"    WING {wing_no+1}")
                    print("    --------")
                    print("    Volume                                  = {0}".format(volume))
                    print("    Position of the center of gravity (COG) = {0}".format(cog))
                    print("    Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
                    print("                                              {0}".format(inertia[1,:]))
                    print("                                              {0}".format(inertia[2,:]))
                print("  DONE: Evaluating Wing Mesh Properties")
                print("")
                
            if GConf.CREATE_FUSELAGE == True:
                print("")
                print("START: Evaluating Fuselage Mesh Properties:")
                volume, cog, inertia = fuse_stl_mesh.get_mass_properties()
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
                for fin_no, fin_stl_data in enumerate(all_fin_stl_data):
                    volume, cog, inertia = fin_stl_mesh.get_mass_properties()
                    print(f"    Fin {fin_no+1}")
                    print("    --------")
                    print("    Volume                                  = {0}".format(volume))
                    print("    Position of the center of gravity (COG) = {0}".format(cog))
                    print("    Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
                    print("                                              {0}".format(inertia[1,:]))
                    print("                                              {0}".format(inertia[2,:]))
                print("DONE Evaluating Fin Mesh Properties")
                print("")
    
            if GConf.STL_SHOW_IN_MATPLOT == True:
                # self._mpl_plot()
                if self.verbosity > 0:
                    print("START: Show in matplotlib")
        
                if GConf.CREATE_WING == True:
                    # Create a new plot
                    figure = plt.figure()
                    ax = mplot3d.Axes3D(figure)
        
                    # Render the wing
                    ax.add_collection3d(mplot3d.art3d.Poly3DCollection(wing_stl_mesh.vectors))
                    # Auto scale to the mesh size
                    scale = wing_stl_mesh.points.flatten()
                    ax.auto_scale_xyz(scale, scale, scale)
                    ax.set_xlabel("X-axis")
                    ax.set_ylabel("Y-axis")
                    ax.set_zlabel("Z-axis")
                    
                if GConf.CREATE_FUSELAGE == True:
                    # Create a new plot
                    figure = plt.figure()
                    ax = mplot3d.Axes3D(figure)
        
                    # Render the fuselage
                    ax.add_collection3d(mplot3d.art3d.Poly3DCollection(fuse_stl_mesh.vectors))
                    # Auto scale to the mesh size
                    scale = fuse_stl_mesh.points.flatten()
                    ax.auto_scale_xyz(scale, scale, scale)
                    ax.set_xlabel("X-axis")
                    ax.set_ylabel("Y-axis")
                    ax.set_zlabel("Z-axis")
                    
                if GConf.CREATE_WING == True and GConf.CREATE_FUSELAGE == True:
                    # Create a new plot
                    figure = plt.figure()
                    ax = mplot3d.Axes3D(figure)
        
                    # Render the wing and fuselage
                    wing_coll = mplot3d.art3d.Poly3DCollection(wing_stl_mesh.vectors)
                    ax.add_collection3d(wing_coll)
                    fuse_coll = mplot3d.art3d.Poly3DCollection(fuse_stl_mesh.vectors)
                    fuse_coll.set_facecolor('r')
                    ax.add_collection3d(fuse_coll)
                    
                    if GConf.FIN_GEOMETRY_DICT[0] is not None:
                        # TODO - this needs to be updated (and probably the wing plotting)
                        fin_coll = mplot3d.art3d.Poly3DCollection(fin_stl_mesh.vectors)
                        fin_coll.set_facecolor('c')
                        ax.add_collection3d(fin_coll)
        
        
                    # Auto scale to the mesh size
                    scale = wing_stl_mesh.points.flatten()
                    ax.auto_scale_xyz(scale, scale, scale)
                    ax.set_xlabel("X-axis")
                    ax.set_ylabel("Y-axis")
                    ax.set_zlabel("Z-axis")
        
                # Show the plot to the screen
                plt.show()
                if self.verbosity > 0:
                    print("  DONE: Show in matplotlib")
                    print("")
    
    def _add_curvature(self,):
        """Adds curvature by the provided curvature functions.
        """
    
    def _rotate_vehicle(self):
        """Rotates entire vehicle to add offset angle.
        """
    
    def _write_vtk(self):
        """Writes VTK files.
        """
    
    def _write_stl(self):
        """Writes STL files.
        """
    
    def _evaluate_mesh_properties(self,):
        """Evaluates properties of stl.
        """
    
    def _mpl_plot(self,):
        """Plots stl components.
        """
        
    def generate(self):
        """Public API to generate vehicle geometry.
        """
    
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