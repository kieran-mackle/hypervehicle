#!/usr/bin/python3.8
import numpy as np
from scipy.optimize import bisect

from eilmer.geom.vector3 import Vector3
from eilmer.geom.path import Line
from eilmer.geom.surface import CoonsPatch 

from idmoc.hypervehicle.utils import (OffsetPatchFunction, 
                                      SubRangedPath, 
                                      ElipsePath, 
                                      ArcLengthParameterizedPath, 
                                      TrailingEdgePath,
                                      OffsetPathFunction,
                                      GeometricMeanPathFunction,
                                      MeanLeadingEdgePatchFunction,
                                      MeanTrailingEdgePatch,
                                      RotatedPatch,
                                      FlatLeadingEdgePatchFunction
                                      )

def hyper_wing_main(wing_geometries: list, verbosity: int = 1) -> list:
    """Wing Geometry Generator for gliding hypersonic vehicle.

    Parameters
    ----------
    wing_geometries : list
        A list containing wing geometry definition dictionaries.
    verbosity : int, optional
        The verbosity of the code output. The default is 1.

    Raises
    ------
    Exception
        When an invalid trailing edge option is provided.

    Returns
    -------
    patches : list
        A list of the wing surface patches.
    
    References
    ----------
    This code was authored by Ingo Jahn and Kieran Mackle.
    """
    
    patches = []
    
    if len(wing_geometries) > 0:
        if verbosity > 0:
            print("\nCreating wing patches...")
        
        for wing_no, wing_geometry in enumerate(wing_geometries):
            if verbosity > 1:
                print("  ------------------------")
                print(f"           Wing {wing_no+1}")
                print("  ------------------------")
                
            # Initialise 
            patch_dict = {}
            
            # Extract construction points for planform
            A0 = wing_geometry['A0']
            A1 = wing_geometry['A1']
            TT = wing_geometry['TT']
            B0 = wing_geometry['B0']
            Line_B0TT = wing_geometry['Line_B0TT']
        
            if verbosity > 1:
                print(f"  Generating wing {wing_no+1} planform.")
                
            if wing_geometry['Line_B0TT_TYPE'].lower() == "bezier":
                if verbosity > 1:
                    print("    Constructing planform using Bezier Curve as Leading Edge shape.")
                
                # Find Locations
                if wing_geometry['t_B1'] == None:
                    fun_B1 = lambda t: Line_B0TT(t).x - A1.x
                    t_B1 = bisect(fun_B1, 0., 1.)
                else:
                    t_B1 = wing_geometry['t_B1']
                    
                if wing_geometry['t_B2'] == None:
                    t_B2 = 0.5*(1+t_B1)
                else:
                    t_B2 = wing_geometry['t_B1']
                    
                B1 = Line_B0TT(t_B1)
                Line_B0B1 = SubRangedPath(underlying_path=Line_B0TT, t0=0.0, t1=t_B1)
                Line_B1B2 = SubRangedPath(underlying_path=Line_B0TT, t0=t_B1, t1=t_B2)
                Line_TTB2 = SubRangedPath(underlying_path=Line_B0TT, t0=1.0, t1=t_B2)
            else:
                raise Exception("Option for 'Line_B0TT'={} not supported.".format(wing_geometry['Line_B0TT_TYPE']))
        
            # create wing planform shape
            wing_patch = [np.nan, np.nan]
            wing_patch_flipped = [np.nan, np.nan]
            wing_patch[0] = CoonsPatch(south=Line(p0=A0, p1=A1),
                                       north=Line_B0B1,
                                       west=Line(p0=A0, p1=B0),
                                       east=Line(p0=A1, p1=B1))
            wing_patch[1] = CoonsPatch(south=Line(p0=A1, p1=TT),
                                       north=Line_B1B2,
                                       west=Line(p0=A1, p1=B1),
                                       east=Line_TTB2)
            
            # Need flipped planform for lower side to ensure vecors point in correct direction
            Line_B0B1_flipped = SubRangedPath(underlying_path=Line_B0B1, t0=1., t1=0.)
            Line_B1B2_flipped = SubRangedPath(underlying_path=Line_B1B2, t0=1., t1=0.)
            wing_patch_flipped[0] = CoonsPatch(south=Line(p0=A1, p1=A0),
                                           north=Line_B0B1_flipped,
                                           east=Line(p0=A0, p1=B0),
                                           west=Line(p0=A1, p1=B1))
            wing_patch_flipped[1] = CoonsPatch(south=Line(p0=TT, p1=A1),
                                           north=Line_B1B2_flipped,
                                           east=Line(p0=A1, p1=B1),
                                           west=Line_TTB2)
        
            # Create wing top & bottom surface
            if verbosity > 1:
                print("    Adding thickness to wing.")
            top_patch = [np.nan, np.nan]
            bot_patch = [np.nan, np.nan]
        
            for i in range(2):
                top_patch[i] = OffsetPatchFunction(wing_patch_flipped[i],
                                           function=wing_geometry['FUNC_TOP_THICKNESS'])
                bot_patch[i] = OffsetPatchFunction(wing_patch[i],
                                           function=wing_geometry['FUNC_BOT_THICKNESS'])
                # flipped moves to top as z-positive points downwards
                
            patch_dict[f"w{wing_no}_top_patch_0"] = top_patch[0]    # top B0B1A1A0
            patch_dict[f"w{wing_no}_top_patch_1"] = top_patch[1]    # top B1B2TTA1
            patch_dict[f"w{wing_no}_bot_patch_0"] = bot_patch[0]    # bot B0B1A1A0
            patch_dict[f"w{wing_no}_bot_patch_1"] = bot_patch[1]    # bot B1B2TTA1
        
        
            # Add leading edge.
            if verbosity > 1:
                print("    Adding Leading Edge to wing.")
            # Get mean line between upper and lower wing patches
            top_edge_path = OffsetPathFunction(Line_B0TT, wing_geometry['FUNC_TOP_THICKNESS'])
            bot_edge_path = OffsetPathFunction(Line_B0TT, wing_geometry['FUNC_BOT_THICKNESS'])
            mean_path = GeometricMeanPathFunction(top_edge_path, bot_edge_path)
            
            if 'LE_TYPE' in wing_geometry and wing_geometry['LE_TYPE'] == 'FLAT':
                patch_dict[f"w{wing_no}_LE_patch0"] = FlatLeadingEdgePatchFunction(top_edge_path,
                                                                            bot_edge_path,
                                                                            0, t_B1)
                patch_dict[f"w{wing_no}_LE_patch1"] = FlatLeadingEdgePatchFunction(top_edge_path,
                                                                            bot_edge_path,
                                                                            t_B1, t_B2)
                patch_dict[f"w{wing_no}_LE_patch2"] = FlatLeadingEdgePatchFunction(top_edge_path,
                                                                            bot_edge_path,
                                                                            t_B1, 1)
                
            else:
                LE_top_patch = [np.nan, np.nan, np.nan]
                LE_bot_patch = [np.nan, np.nan, np.nan]
                
                # Eliptical LE
                LE_top_patch[0] = MeanLeadingEdgePatchFunction(mean_path, top_edge_path,
                                    LE_width_function=wing_geometry['FUNC_LEADING_EDGE_WIDTH'],
                                    t0=0., t1=t_B1, side='top')
                LE_top_patch[1] = MeanLeadingEdgePatchFunction(mean_path, top_edge_path,
                                    LE_width_function=wing_geometry['FUNC_LEADING_EDGE_WIDTH'],
                                    t0=t_B1, t1=t_B2, side='top')
                LE_top_patch[2] = MeanLeadingEdgePatchFunction(mean_path, top_edge_path,
                                    LE_width_function=wing_geometry['FUNC_LEADING_EDGE_WIDTH'],
                                    t0=t_B2, t1=1., side='top')
            
                LE_bot_patch[0] = MeanLeadingEdgePatchFunction(mean_path, bot_edge_path,
                                    LE_width_function=wing_geometry['FUNC_LEADING_EDGE_WIDTH'],
                                    t0=0., t1=t_B1, side='bot')
                LE_bot_patch[1] = MeanLeadingEdgePatchFunction(mean_path, bot_edge_path,
                                    LE_width_function=wing_geometry['FUNC_LEADING_EDGE_WIDTH'],
                                    t0=t_B1, t1=t_B2, side='bot')
                LE_bot_patch[2] = MeanLeadingEdgePatchFunction(mean_path, bot_edge_path,
                                    LE_width_function=wing_geometry['FUNC_LEADING_EDGE_WIDTH'],
                                    t0=t_B2, t1=1.0, side='bot')
                
                # Append to patch_dict
                patch_dict[f"w{wing_no}_LE_top_patch_0"] = LE_top_patch[0]
                patch_dict[f"w{wing_no}_LE_top_patch_1"] = LE_top_patch[1]
                patch_dict[f"w{wing_no}_LE_top_patch_2"] = LE_top_patch[2]
                patch_dict[f"w{wing_no}_LE_bot_patch_0"] = LE_bot_patch[0]
                patch_dict[f"w{wing_no}_LE_bot_patch_1"] = LE_bot_patch[1]
                patch_dict[f"w{wing_no}_LE_bot_patch_2"] = LE_bot_patch[2]
        
            # Add trailing Edge
            if verbosity > 1:
                print("    Adding Trailing Edge.")
                print("      Tail options - {}".format(wing_geometry['TAIL_OPTION']))
                
            if wing_geometry['TAIL_OPTION'] == 'FLAP':
                if verbosity > 1:
                    print("        Flap length = {}".format(wing_geometry['FLAP_LENGTH']))
                    print("        Flap angle  = {}".format(wing_geometry['FLAP_ANGLE']))
        
                # Define top and bottom TE paths
                TE_top = TrailingEdgePath(A0, B0, thickness_function=wing_geometry['FUNC_TOP_THICKNESS'])
                TE_bot = TrailingEdgePath(A0, B0, thickness_function=wing_geometry['FUNC_BOT_THICKNESS'])
                TE_mean_line = GeometricMeanPathFunction(TE_top, TE_bot)
                
                # Make top and bottom of flap
                TE_top_patch = MeanTrailingEdgePatch(TE_mean_line, TE_path=TE_top,
                                flap_length=wing_geometry['FLAP_LENGTH'],
                                flap_angle=wing_geometry['FLAP_ANGLE'],
                                side='top')
                TE_bot_patch = MeanTrailingEdgePatch(TE_mean_line, TE_path=TE_bot,
                                flap_length=wing_geometry['FLAP_LENGTH'],
                                flap_angle=wing_geometry['FLAP_ANGLE'],
                                side='bot')
                
                # Append to patch_dict
                patch_dict[f"w{wing_no}_TE_top_patch"] = TE_top_patch
                patch_dict[f"w{wing_no}_TE_bot_patch"] = TE_bot_patch
                
                # Create edge (connecting side to TE)
                if 'LE_TYPE' in wing_geometry and wing_geometry['LE_TYPE'] == 'FLAT':
                    # Flat LE
                    west = Line(p0=TE_top(1), p1=TE_mean_line(1))
                    north = Line(p0=TE_mean_line(1), p1=TE_bot(1))
                
                else:
                    # Eiliptical LE
                    thickness_top = TE_mean_line(1).z - TE_top(1).z 
                    thickness_bot = TE_mean_line(1).z - TE_bot(1).z
                    
                    if thickness_top == 0 or thickness_bot == 0:
                        raise Exception("Elliptical LE cannot be created when thickness converges to zero.")
                    
                    elipse_top = ElipsePath(centre=TE_mean_line(1), 
                                            thickness=thickness_top,
                                            LE_width=wing_geometry['FUNC_LEADING_EDGE_WIDTH'](0.), 
                                            side='top')
                    elipse_bot = ElipsePath(centre=TE_mean_line(1), 
                                            thickness=thickness_bot,
                                            LE_width=wing_geometry['FUNC_LEADING_EDGE_WIDTH'](0.), 
                                            side='bot')
                    
                    elipse_top = ArcLengthParameterizedPath(underlying_path=elipse_top)
                    elipse_bot = ArcLengthParameterizedPath(underlying_path=elipse_bot)
                    
                    north=elipse_bot
                    west=elipse_top
                
                south=Line(p0=Vector3(x=TE_mean_line(1).x,
                                      y=TE_mean_line(1).y, 
                                      z=TE_top(1).z),
                           p1=Vector3(x=TE_mean_line(1).x - wing_geometry['FLAP_LENGTH']*np.cos(wing_geometry['FLAP_ANGLE']), 
                                      y=TE_mean_line(1).y,
                                      z=TE_mean_line(1).z + wing_geometry['FLAP_LENGTH']*np.sin(wing_geometry['FLAP_ANGLE'])))
                
                east=Line(p0=Vector3(x=TE_mean_line(1).x - wing_geometry['FLAP_LENGTH']*np.cos(wing_geometry['FLAP_ANGLE']), 
                                     y=TE_mean_line(1).y,
                                     z=TE_mean_line(1).z + wing_geometry['FLAP_LENGTH']*np.sin(wing_geometry['FLAP_ANGLE'])),
                          p1=Vector3(x=TE_mean_line(1).x, 
                                     y=TE_mean_line(1).y, 
                                     z=TE_bot(1).z))
                
                TELE_patch = CoonsPatch(north=north,
                                        west=west,
                                        south=south,
                                        east=east)
                
                # Append to patch_dict
                patch_dict[f"w{wing_no}_TELE_patch"] = TELE_patch
            
            else:
                raise Exception("Tail option = {} not supported.".format(wing_geometry['TAIL_OPTION']))
            
            
            # Close wing volume
            if 'CLOSE_WING' in wing_geometry and wing_geometry['CLOSE_WING']:
                # Add patch to close wing
                
                if verbosity > 1:
                    print("    Closing interior side of wing.")
                
                TT_top = TT + wing_geometry['FUNC_TOP_THICKNESS'](x=TT.x, y=TT.y, z=TT.z)
                TT_bot = TT + wing_geometry['FUNC_BOT_THICKNESS'](x=TT.x, y=TT.y, z=TT.z)
                TT_mid = 0.5*(TT_top+TT_bot)
                
                interior_top = Line(p0=TE_top(0), p1=TT_top)
                interior_bot = Line(p0=TE_bot(0), p1=TT_bot)
                interior_mid = Line(p0=TE_mean_line(0), p1=TT_mid)
                
                back_top = Line(p0=TE_top(0), p1=TE_mean_line(0))
                back_bot = Line(p0=TE_mean_line(0), p1=TE_bot(0))
                
                interior_patch1 = CoonsPatch(north=interior_mid,
                                             east=Line(p0=TT_top, p1=TT_mid),
                                             south=interior_top,
                                             west=back_top)
                
                interior_patch2 = CoonsPatch(north=interior_bot,
                                             east=Line(p0=TT_mid, p1=TT_bot),
                                             south=interior_mid,
                                             west=back_bot)
                
                elipse_top = ElipsePath(centre = TT_mid,
                                        thickness = TT_mid.z - TT_top.z,
                                        LE_width = wing_geometry['FUNC_LEADING_EDGE_WIDTH'](1),
                                        side = 'top')
                elipse_bot = ElipsePath(centre = TT_mid,
                                        thickness = TT_mid.z - TT_bot.z,
                                        LE_width = wing_geometry['FUNC_LEADING_EDGE_WIDTH'](1), 
                                        side = 'bot')
                elipse_top = ArcLengthParameterizedPath(underlying_path=elipse_top)
                elipse_bot = ArcLengthParameterizedPath(underlying_path=elipse_bot)
                
                # Now reverse the paths for correct orientation 
                elipse_bot = SubRangedPath(underlying_path = elipse_bot, t0=1., t1=0.)
                
                interior_ellip = CoonsPatch(north=elipse_bot,
                                            east=elipse_top,
                                            south=Line(p0=TT_mid, p1=TT_top),
                                            west=Line(p0=TT_mid, p1=TT_bot))
                
                # Rotate patch
                interior_ellip = RotatedPatch(interior_ellip, 
                                                    np.deg2rad(-90),
                                                    axis='z',
                                                    point = TT_mid)
                
                # Append to patch_dict
                patch_dict[f"w{wing_no}_interior_patch1"] = interior_patch1
                patch_dict[f"w{wing_no}_interior_patch2"] = interior_patch2
                patch_dict[f"w{wing_no}_interior_ellip"] = interior_ellip
                
                # Interior TE
                TE_point = Vector3(x=TE_mean_line(0).x - wing_geometry['FLAP_LENGTH']*np.cos(wing_geometry['FLAP_ANGLE']), 
                                      y=TE_mean_line(0).y,
                                      z=TE_mean_line(0).z + wing_geometry['FLAP_LENGTH']*np.sin(wing_geometry['FLAP_ANGLE']))
                
                interior_flap_patch = CoonsPatch(north=Line(p0=TE_top(0), p1=TE_point),
                                                  east=Line(p0=TE_bot(0), p1=TE_point),
                                                  south=Line(p0=TE_mean_line(0), p1=TE_bot(0)),
                                                  west=Line(p0=TE_mean_line(0), p1=TE_top(0)))
                
                patch_dict[f"w{wing_no}_interior_flap_patch"] = interior_flap_patch
            
            if verbosity > 0 and len(wing_geometries) > 1:
                print(f"  Wing {wing_no+1} complete.")
                
            # Append wing patches to list
            patches.append(patch_dict)

    return patches
