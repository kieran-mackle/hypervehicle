#! /usr/bin/env python

class GConf:
    """Parameter configuration class for hypervehicle module.
    """
    
    # Define all global-level config vars (with default value if applicable)
    _config = {
    "VERBOSITY"     : 1,     # 0, 1, 2 - set reporting level.
    
    "VEHICLE_ANGLE": 0, # Vehicle angle adjustment
        
    # OPTIONS
    "CREATE_WING"       : True,     # create wing geometry
    "CREATE_FUSELAGE"   : False,    # create fuselage geometry
    # vtk output
    "CREATE_VTK_MESH"   : False,
    "VTK_FILENAME"      : "test.vtk",
                          # for "test.vtk" the final files will be
                          # "test_wing_patch.vtk" and "test_fuselage_patch.vtk"
    "VTK_RESOLUTION"    : 10,       # number of cell vertices per edge
    # stl output
    "CREATE_STL_OBJECT" : True,
    "STL_FILENAME"      : "test.stl",
                          # for "test.stl" the final files will be
                          # "test_wing.stl" and "test_fuselage.stl"
    "STL_RESOLUTION"        : 10,       # number of triangle vertices per edge
    "STL_INCLUDE_MIRROR"    : True,     # include mirror image in STL
    "STL_SHOW_IN_MATPLOT"   : False,     # Create Matplotlib image
    #
    # WING GEOMETRY DEFINITION
    "WING_GEOMETRY_DICT": None,
    # "MULTI_WING": False, 
    
    # FUSELAGE GEOMETRY DEFINITION
    "FUSELAGE_GEOMETRY_DICT": None,
    
    # FIN GEOMETRY DEFINITION
    "FIN_GEOMETRY_DICT": [None],
    "MIRROR_FIN": True
    }

    class classproperty(property):
        def __get__(self, cls, owner):
            return classmethod(self.fget).__get__(None, owner)()

    for var in _config.keys():
        exec("@classproperty\ndef {0}(cls): return cls._config['{0}']".format(var))

    @staticmethod
    def read_inputs(settings):
        """Check if the input variables are valid then overwrite default _config values."""
        for option, value in settings.items():
            if option not in GConf._config:
                raise Exception(
                        "Invalid configuration variable '" + option +
                        "' supplied. Valid configuration variables are [" +
                        ", ".join([GConf._config]) + "]"
                        )
            
            # Ensure wing geometry dict (s) is in list
            if (option == 'WING_GEOMETRY_DICT') & (type(value) != list):
                value = [value]
            
            # Ensure fin geometry dict (s) is in list
            if (option == 'FIN_GEOMETRY_DICT') & (type(value) != list):
                value = [value]
            
            # Overwrite
            GConf._config[option] = value

    @staticmethod
    def check_inputs():
        """Check valid value ranges for each config option."""
        # Config variables not in this dict will NOT be checked
        valid_input_dict = {
            "VERBOSITY"         : [0, 1, 2, 3],
            
            # OPTIONS
            "CREATE_WING"       : [True, False],
            "CREATE_FUSELAGE"   : [True, False],
            # vtk output
            "CREATE_VTK_MESH"   : [True, False],
            # stl output
            "CREATE_STL_OBJECT" : [True, False],
            "STL_INCLUDE_MIRROR"    : [True, False],
            "STL_SHOW_IN_MATPLOT"   : [True, False],
            #
            # WING GEOMETRY DEFINITION
            "WING_GEOMETRY_DICT": {
                                   # Define Trailing Edge
                                   "Line_B0TT_TYPE"    : ["Bezier"],
                                   "TAIL_OPTION"       : ["NONE", "FLAP"],
                                   },
            # "MULTI_WING": [True, False],
            
            # FUSELAGE GEOMETRY DEFINITION
            "FUSELAGE_GEOMETRY_DICT": {
                "FUSELAGE_NOSE_OPTION"  : ["sharp-cone"],
                "FUSELAGE_TAIL_OPTION"  : ["sharp-cone", "flat"],
                },
            
            "MIRROR_FIN": [True, False],
            }
        for option, valid_inputs in valid_input_dict.items():
            if option == "WING_GEOMETRY_DICT":
                # Check inputs for wing geometry dictionary
                
                for wing_geom_dict in GConf._config["WING_GEOMETRY_DICT"]:
                    for wing_option, valid_wing_inputs in valid_inputs.items():
                        if wing_geom_dict[wing_option] not in valid_wing_inputs:
                            raise Exception(
                                    "Configuration variable " + wing_option + " = " +
                                    str(wing_geom_dict[wing_option]) + " is not valid." +
                                    "Valid values are " + str([valid_wing_inputs]))
            
            elif option == "FUSELAGE_GEOMETRY_DICT":
                # Check inputs for fuselage geometry dictionary
                if GConf._config["FUSELAGE_GEOMETRY_DICT"] is not None:
                    for fuse_option, valid_fuse_inputs in valid_inputs.items():
                        if GConf._config["FUSELAGE_GEOMETRY_DICT"][fuse_option] not in valid_fuse_inputs:
                            raise Exception(
                                    "Configuration variable " + fuse_option + " = " +
                                    str(GConf._config["FUSELAGE_GEOMETRY_DICT"][fuse_option]) + " is not valid." +
                                    "Valid values are " + str([valid_fuse_inputs]))
            
            elif GConf._config[option] not in valid_inputs:
                raise Exception(
                        "Configuration variable " + option + " = " +
                        str(GConf._config[option]) + " is not valid." +
                        "Valid values are " + str([valid_inputs]))
