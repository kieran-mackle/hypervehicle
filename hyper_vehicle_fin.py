#!/usr/bin/python3.8
"""
Fin Geometry Generator for gliding hypersonic vehicle.

Contains functions to be called from hyper_vehicle.py

Authors: Ingo Jahn, Kieran Mackle
Created on: 26/10/2021
Last Modified: 29/10/2021
"""

import numpy as np

from eilmer.geom.vector3 import Vector3
from eilmer.geom.path import Line, Polyline
from eilmer.geom.surface import CoonsPatch 

from hyper_vehicle_functions import (
    LeadingEdgePatchFunction, 
    OffsetPatchFunction, 
    SubRangedPath, 
    ElipsePath, 
    ArcLengthParameterizedPath, 
    TrailingEdgePatch,
    TrailingEdgePath,
    RotatedPatch
    )

def hyper_fin_main(GConf):
    '''
    Hypersonic vehicle fin generation function.
    '''
    
    #####################################################
    ##                      FINS                       ##
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
    #  p1---p2
    #   |    \
    #   |     \
    #   |      \
    #   |       \
    #  p0_______p3
    
    
    patches = []
    
    print("FIN GEOMETRY GENERATION")    
    for fin_number, fin_geometry in enumerate(GConf.FIN_GEOMETRY_DICT):
        print(f"  Fin {fin_number+1}")
        
        # Initialise 
        fin_patch_dict = {}
        temp_fin_patch_dict = {}
        
        # Extract geometric properties
        p0 = fin_geometry['p0']
        p1 = fin_geometry['p1']
        p2 = fin_geometry['p2']
        p3 = fin_geometry['p3']
        fin_thickness = fin_geometry['FIN_THICKNESS']
        fin_angle = fin_geometry['FIN_ANGLE']
        fin_thickness_function_top = fin_geometry['FIN_TOP_THICKNESS_FUNC']
        fin_thickness_function_bot = fin_geometry['FIN_BOTTOM_THICKNESS_FUNC']
        leading_edge_width_function = fin_geometry['FIN_LEADING_EDGE_FUNC']
        
        p1p2 = Line(p1, p2)
        p2p3 = Line(p2, p3)
        p1p3 = Polyline(segments=[p1p2, p2p3])
        
        
        print("START: Creating fin planform.")
        p0p1 = Line(p0, p1)
        fin_patch = CoonsPatch(north    = Line(p1, p2),
                               east     = Line(p3, p2),
                               south    = Line(p0, p3),
                               west     = Line(p0, p1))
        flipped_fin_patch = CoonsPatch(north = Line(p3, p2),
                                       east = Line(p1, p2),
                                       south = Line(p0, p1),
                                       west = Line(p0, p3))
        print("  DONE: Creating fin planform.")
        
        
        print("\nSTART: Adding thickness to fin.")
        top_patch = OffsetPatchFunction(flipped_fin_patch, fin_thickness_function_top)
        bot_patch = OffsetPatchFunction(fin_patch, fin_thickness_function_bot)
        temp_fin_patch_dict[f'fin_{fin_number}_top_patch'] = top_patch
        temp_fin_patch_dict[f'fin_{fin_number}_bot_patch'] = bot_patch
        print("  DONE: Adding thickness to fin.")
        
        
        print("\nSTART: Adding Leading Edge to fin.")
        LE_top_patch = LeadingEdgePatchFunction(p1p3, 
                                                 fin_thickness_function_top, 
                                                 leading_edge_width_function, 
                                                 0, 
                                                 1,
                                                 side='top')
        LE_bot_patch = LeadingEdgePatchFunction(p1p3, 
                                                  fin_thickness_function_bot, 
                                                  leading_edge_width_function, 
                                                  0, 
                                                  1,
                                                  side='bot')
        temp_fin_patch_dict[f'fin_{fin_number}_top_LE'] = LE_top_patch
        temp_fin_patch_dict[f'fin_{fin_number}_bot_LE'] = LE_bot_patch
        print("  DONE: Adding Leading Edge to fin.")
        
        
        print("\nSTART: Adding bottom face.")
        thickness_top = fin_thickness_function_top(x=p3.x, y=p3.y).z
        thickness_bot = fin_thickness_function_bot(x=p3.x, y=p3.y).z
        elipse_top = ElipsePath(centre = p3, 
                                thickness = thickness_top,
                                LE_width = leading_edge_width_function(1),
                                side = 'top')
        elipse_bot = ElipsePath(centre = p3, 
                                thickness = thickness_bot,
                                LE_width = leading_edge_width_function(1), 
                                side = 'bot')
        elipse_top = ArcLengthParameterizedPath(underlying_path=elipse_top)
        elipse_bot = ArcLengthParameterizedPath(underlying_path=elipse_bot)
        
        p3p3_top = Line(p0 = p3 - Vector3(0,0,fin_thickness/2),
                        p1 = p3)
        p3p3_bot = Line(p0 = p3,
                        p1 = p3 + Vector3(0,0,fin_thickness/2))
        
        temp_bottom_ellipse_patch = CoonsPatch(north = p3p3_bot,
                                               south = elipse_top,
                                               east = elipse_bot,
                                               west = p3p3_top)
        
        # Rotate patch into x-z plane about p3
        bottom_ellipse_patch = RotatedPatch(temp_bottom_ellipse_patch, 
                                            np.deg2rad(-90),
                                            axis='z',
                                            point = p3)
        
        # Create rectangular patches for rest of fin bottom
        p3p0 = Line(p3,p0)
        p3p0_bot = Line(p0 = p3 + Vector3(0,0,fin_thickness/2), 
                        p1 = p0 + Vector3(0,0,fin_thickness/2))
        p3p0_top = Line(p0 = p3 - Vector3(0,0,fin_thickness/2), 
                        p1 = p0 - Vector3(0,0,fin_thickness/2))
        
        p0p0_top = Line(p0 = p0,
                        p1 = p0 - Vector3(0,0,fin_thickness/2))
        p0_botp0 = Line(p0 = p0 + Vector3(0,0,fin_thickness/2),
                        p1 = p0)
        p3p3_top = Line(p0 = p3,
                        p1 = p3 - Vector3(0,0,fin_thickness/2))
        p3_botp3 = Line(p0 = p3 + Vector3(0,0,fin_thickness/2),
                        p1 = p3)
        
        bot_1 = CoonsPatch(north = p3p0_top,
                           south = p3p0,
                           east = p0p0_top,
                           west = p3p3_top)
        bot_2 = CoonsPatch(north = p3p0,
                           south = p3p0_bot,
                           east = p0_botp0,
                           west = p3_botp3)
        
        temp_fin_patch_dict[f'fin_{fin_number}_bot_ellip'] = bottom_ellipse_patch
        temp_fin_patch_dict[f'fin_{fin_number}_bot_1'] = bot_1
        temp_fin_patch_dict[f'fin_{fin_number}_bot_2'] = bot_2
        
        print("  DONE: Adding bottom face.")
        
        
        print("\nSTART: Adding Trailing Edge.")
        if fin_geometry['rudder_type'] == 'sharp':
            # Create sharp trailing edge
            
            # First create top and bottom paths connecting fin to TE
            TE_top = TrailingEdgePath(p0, p1, thickness_function=fin_thickness_function_top)
            TE_bot = TrailingEdgePath(p0, p1, thickness_function=fin_thickness_function_bot)
    
            # Make top and bottom of rudder
            TE_top_patch = TrailingEdgePatch(A0=p0, B0=p1, TE_path=TE_top,
                            flap_length = fin_geometry['rudder_length'],
                            flap_angle = fin_geometry['rudder_angle'],
                            side = 'top')
            TE_bot_patch = TrailingEdgePatch(A0=p0, B0=p1, TE_path=TE_bot,
                            flap_length = fin_geometry['rudder_length'],
                            flap_angle = fin_geometry['rudder_angle'],
                            side='bot')
    
            # Create corner patches
            thickness_top = fin_thickness_function_top(x=p1.x, y=p1.y).z
            thickness_bot = fin_thickness_function_bot(x=p1.x, y=p1.y).z
            
            elipse_top = ElipsePath(centre = p1, 
                                    thickness = thickness_top,
                                    LE_width = leading_edge_width_function(0.),
                                    side = 'top')
            elipse_bot = ElipsePath(centre = p1, 
                                    thickness = thickness_bot,
                                    LE_width = leading_edge_width_function(0.), 
                                    side = 'bot')
            elipse_top = ArcLengthParameterizedPath(underlying_path=elipse_top)
            elipse_bot = ArcLengthParameterizedPath(underlying_path=elipse_bot)
    
            south = Line(p0=Vector3(x = p1.x, y = p1.y, z = thickness_top),
                         p1=Vector3(x = p1.x-fin_geometry['rudder_length'], 
                                    y = p1.y,
                                    z = fin_geometry['rudder_length']*np.sin(fin_geometry['rudder_angle'])))
            east = Line(p0=Vector3(x = p1.x-fin_geometry['rudder_length'], 
                                   y = p1.y,
                                   z = fin_geometry['rudder_length']*np.sin(fin_geometry['rudder_angle'])),
                       p1=Vector3(x = p1.x, y = p1.y, z = thickness_bot))
    
            TE_ellip_patch = CoonsPatch(north = elipse_bot,
                                        west = elipse_top,
                                        south = south,
                                        east = east)
            
            # Create bottom triangle patch
            p0_top = p0 - Vector3(0,0,fin_thickness/2)
            p0_bot = p0 + Vector3(0,0,fin_thickness/2)
            TE = Vector3(x = p0.x-fin_geometry['rudder_length'], 
                         y = p0.y,
                         z = fin_geometry['rudder_length']*np.sin(fin_geometry['rudder_angle']))
            
            p0p0_top = Line(p0 = p0 - Vector3(0,0,fin_thickness/2),
                            p1 = p0)
            p0p0_bot = Line(p0 = p0,
                            p1 = p0 + Vector3(0,0,fin_thickness/2))
            p0_top_TE = Line(p0 = p0_top,
                             p1 = TE)
            p0_bot_TE = Line(p0 = TE,
                             p1 = p0_bot)
            
            TE_triangle_patch = CoonsPatch(north = p0_bot_TE,
                                           south = p0p0_top,
                                           east = p0p0_bot,
                                           west = p0_top_TE)
            
            # Add to patch dict
            temp_fin_patch_dict[f'fin_{fin_number}_TE_top'] = TE_top_patch
            temp_fin_patch_dict[f'fin_{fin_number}_TE_bot'] = TE_bot_patch
            temp_fin_patch_dict[f'fin_{fin_number}_TE_ellipse'] = TE_ellip_patch
            temp_fin_patch_dict[f'fin_{fin_number}_TE_triangle'] = TE_triangle_patch
            
        else:
            # Create patch for top of fin TE (elliptical section)
            thickness_top = fin_thickness_function_top(x=p1.x, y=p1.y).z
            thickness_bot = fin_thickness_function_bot(x=p1.x, y=p1.y).z
            elipse_top = ElipsePath(centre = p1, 
                                    thickness = thickness_top,
                                    LE_width = leading_edge_width_function(0.),
                                    side = 'top')
            elipse_bot = ElipsePath(centre = p1, 
                                    thickness = thickness_bot,
                                    LE_width = leading_edge_width_function(0.), 
                                    side = 'bot')
            elipse_top = ArcLengthParameterizedPath(underlying_path=elipse_top)
            elipse_bot = ArcLengthParameterizedPath(underlying_path=elipse_bot)
            
            # Now reverse the paths for correct orientation 
            elipse_top = SubRangedPath(underlying_path = elipse_top, t0=1., t1=0.)
            elipse_bot = SubRangedPath(underlying_path = elipse_bot, t0=1., t1=0.)
            
            p1p1_top = Line(p0 = p1,
                            p1 = p1 - Vector3(0,0,fin_thickness/2))
            p1_botp1 = Line(p0 = p1 + Vector3(0,0,fin_thickness/2),
                            p1 = p1)
            
            back_ellipse_patch = CoonsPatch(north = p1p1_top,
                                            south = elipse_bot,
                                            east = elipse_top,
                                            west = p1_botp1)
            
            # Create rectangular patches for rest of fin TE
            p0_botp1_bot = Line(p0 = p0 + Vector3(0,0,fin_thickness/2), 
                                p1 = p1 + Vector3(0,0,fin_thickness/2))
            p0_top1_top = Line(p0 = p0 - Vector3(0,0,fin_thickness/2), 
                               p1 = p1 - Vector3(0,0,fin_thickness/2))
            
            p0p0_top = Line(p0 = p0,
                            p1 = p0 - Vector3(0,0,fin_thickness/2))
            p0_botp0 = Line(p0 = p0 + Vector3(0,0,fin_thickness/2),
                            p1 = p0)
            
            TE_back_1 = CoonsPatch(north = p0_top1_top,
                                   south = p0p1,
                                   east = p1p1_top,
                                   west = p0p0_top)
            TE_back_2 = CoonsPatch(north = p0p1,
                                   south = p0_botp1_bot,
                                   east = p1_botp1,
                                   west = p0_botp0)
            
            # Add to patch dict
            temp_fin_patch_dict[f'fin_{fin_number}_TE_ellip'] = back_ellipse_patch
            temp_fin_patch_dict[f'fin_{fin_number}_TE_1'] = TE_back_1
            temp_fin_patch_dict[f'fin_{fin_number}_TE_2'] = TE_back_2
        
        print("  DONE: Adding Trailing Edge.")
    
        
        # Rotate patches and add to fin_patch_dict
        for patch in temp_fin_patch_dict:
            fin_patch_dict[patch] = RotatedPatch(temp_fin_patch_dict[patch], 
                                                 fin_angle)
        
        if 'offset_function' in fin_geometry and fin_geometry['offset_function'] is not None:
            print("\nSTART: Applying offset function.")
            for patch in fin_patch_dict:
                fin_patch_dict[patch] = OffsetPatchFunction(fin_patch_dict[patch],
                                                            fin_geometry['offset_function'])
            print("  DONE: Applying offset function.")
        
        # Rotate patches again for rudder angle
        if 'pivot_angle' in fin_geometry:
            for patch in fin_patch_dict:
                fin_patch_dict[patch] = RotatedPatch(fin_patch_dict[patch], 
                                                     fin_geometry['pivot_angle'],
                                                     axis = 'z',
                                                     point=fin_geometry['pivot_point'])
        
        # Append fin patches to list
        patches.append(fin_patch_dict)
        
    return patches