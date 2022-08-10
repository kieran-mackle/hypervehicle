#!/usr/bin/python3.8
import numpy as np
from eilmer.geom.vector3 import Vector3
from eilmer.geom.path import Arc, Line
from eilmer.geom.surface import CoonsPatch
from hypervehicle.utils import ConePatch, RotatedPatch, OffsetPatchFunction, RevolvedPatch


def hyper_fuselage_main(fuselage_geometries: list, verbosity: int = 1) -> dict:
    """Fuselage Geometry Generator for hypersonic vehicle.

    Parameters
    ----------
    fuselage_geometry : list
        A list with dictionaries defining the fuselage geometries.
    verbosity : int, optional
        The verbosity of the code output. The default is 1.

    Raises
    ------
    Exception
        When an invalid nose option is provided.
        
    Returns
    -------
    patches : list
        A list of the fin surface patches.

    References
    ----------
    This code was authored by Ingo Jahn and Kieran Mackle.
    """

    patches = []

    if len(fuselage_geometries) > 0:
        if verbosity > 0:
            print("\nCreating fuselage patches...")
        
        for fuse_no, fuselage_geometry in enumerate(fuselage_geometries):
            if verbosity > 1:
                print("  ------------------------")
                print(f"           Fuselage {fuse_no+1}")
                print("  ------------------------")
            
            # Initialise
            patch_dict = {}
            
            if fuselage_geometry is not None:
                offset = fuselage_geometry['OFFSET'] if 'OFFSET' in fuselage_geometry else None
                revolve_line = fuselage_geometry['revolve_line']

                if revolve_line is not None:
                    # Create fuselage patches by revolving line
                    for i in range(4):
                        patch_dict[f'revolved_fuse_{i}'] = RevolvedPatch(revolve_line, i*np.pi/2, (i+1)*np.pi/2)

                else:
                    # Legacy fuselage construction
                    Xn = fuselage_geometry['Xn']
                    X1 = fuselage_geometry['X1']
                    X2 = fuselage_geometry['X2']
                    X3 = fuselage_geometry['X3']
                    X4 = fuselage_geometry['X4'] if 'X4' in fuselage_geometry else None
                    R1 = fuselage_geometry['R1']
                    R2 = fuselage_geometry['R2']
                    R3 = fuselage_geometry['R3']
                
                    if verbosity > 0:
                        print("Creating fuselage patches...")
                
                    if verbosity > 1:
                        print("  Fuselage nose - {}".format(fuselage_geometry['FUSELAGE_NOSE_OPTION']))
                    if fuselage_geometry['FUSELAGE_NOSE_OPTION'] == 'sharp-cone':
                        # create cylinder-0
                        patch_dict['cone_0_n'] = ConePatch(x0=Xn, x1=X1, r0=0., r1=R1, angle0=np.deg2rad(45.), angle1=np.deg2rad(135.) )
                        patch_dict['cone_0_e'] = RotatedPatch(underlying_surf=patch_dict['cone_0_n'], angle=np.deg2rad(90.))
                        patch_dict['cone_0_s'] = RotatedPatch(underlying_surf=patch_dict['cone_0_n'], angle=np.deg2rad(180.))
                        patch_dict['cone_0_w'] = RotatedPatch(underlying_surf=patch_dict['cone_0_n'], angle=np.deg2rad(270.))
                    else:
                        raise Exception("Value set for FUSELAGE_NOSE_OPTION = '{}' is not supported".format(fuselage_geometry['FUSELAGE_NOSE_OPTION']))
                
                    # create cylinder-1
                    patch_dict['cone_1_n'] = ConePatch(x0=X1, x1=X2, r0=R1, r1=R2, angle0=np.deg2rad(45.), angle1=np.deg2rad(135.) )
                    patch_dict['cone_1_e'] = RotatedPatch(underlying_surf=patch_dict['cone_1_n'], angle=np.deg2rad(90.))
                    patch_dict['cone_1_s'] = RotatedPatch(underlying_surf=patch_dict['cone_1_n'], angle=np.deg2rad(180.))
                    patch_dict['cone_1_w'] = RotatedPatch(underlying_surf=patch_dict['cone_1_n'], angle=np.deg2rad(270.))
                
                    # create cylinder-2
                    patch_dict['cone_2_n'] = ConePatch(x0=X2, x1=X3, r0=R2, r1=R3, angle0=np.deg2rad(45.), angle1=np.deg2rad(135.) )
                    patch_dict['cone_2_e'] = RotatedPatch(underlying_surf=patch_dict['cone_2_n'], angle=np.deg2rad(90.))
                    patch_dict['cone_2_s'] = RotatedPatch(underlying_surf=patch_dict['cone_2_n'], angle=np.deg2rad(180.))
                    patch_dict['cone_2_w'] = RotatedPatch(underlying_surf=patch_dict['cone_2_n'], angle=np.deg2rad(270.))
                
                    if verbosity > 1:
                        print("  Fuselage tail - {}".format(fuselage_geometry['FUSELAGE_TAIL_OPTION']))
                    
                    # create tail
                    if fuselage_geometry['FUSELAGE_TAIL_OPTION'].lower() == 'flat':
                        # create the tail (blunt)
                        t_ratio = 0.3
                        cos_angle = np.cos(np.deg2rad(45.))
                        p00=Vector3(x=X3, y=-t_ratio*R3*cos_angle, z= t_ratio*R3*cos_angle)
                        p10=Vector3(x=X3, y= t_ratio*R3*cos_angle, z= t_ratio*R3*cos_angle)
                        p01=Vector3(x=X3, y=-t_ratio*R3*cos_angle, z=-t_ratio*R3*cos_angle)
                        p11=Vector3(x=X3, y= t_ratio*R3*cos_angle, z=-t_ratio*R3*cos_angle)
                        tail_centre_patch = CoonsPatch(p00=p00, p10=p10, p01=p01, p11=p11)
                        patch_dict["tail_centre_patch"] = tail_centre_patch
                        
                        a=Vector3(x=X3, y=-R3*cos_angle, z=-R3*cos_angle)
                        b=Vector3(x=X3, y= R3*cos_angle, z=-R3*cos_angle)
                        c=Vector3(x=X3, y=0., z=0.)
                        tail_edge_patch_n = CoonsPatch(north=Arc(a=a, b=b, c=c),
                                                    south=Line(p0=p01, p1=p11),
                                                    west=Line(p0=p01, p1=a),
                                                    east=Line(p0=p11, p1=b))
                        patch_dict["tail_edge_patch_n"] = tail_edge_patch_n
                        patch_dict["tail_edge_patch_e"] = RotatedPatch(underlying_surf=tail_edge_patch_n, angle=np.deg2rad(90.))
                        patch_dict["tail_edge_patch_s"] = RotatedPatch(underlying_surf=tail_edge_patch_n, angle=np.deg2rad(180.))
                        patch_dict["tail_edge_patch_w"] = RotatedPatch(underlying_surf=tail_edge_patch_n, angle=np.deg2rad(270.))
                    
                    elif fuselage_geometry['FUSELAGE_TAIL_OPTION'].lower() == 'sharp-cone':
                        # create cylinder-0
                        patch_dict['cone_3_n'] = ConePatch(x0=X3, x1=X4, r0=R2, r1=0., angle0=np.deg2rad(45.), angle1=np.deg2rad(135.) )
                        patch_dict['cone_3_e'] = RotatedPatch(underlying_surf=patch_dict['cone_3_n'], angle=np.deg2rad(90.))
                        patch_dict['cone_3_s'] = RotatedPatch(underlying_surf=patch_dict['cone_3_n'], angle=np.deg2rad(180.))
                        patch_dict['cone_3_w'] = RotatedPatch(underlying_surf=patch_dict['cone_3_n'], angle=np.deg2rad(270.))
                    
                    else:
                        raise Exception("Value set for FUSELAGE_TAIL_OPTION = '{}' is not supported".format(fuselage_geometry['FUSELAGE_TAIL_OPTION']))

                if offset is not None:
                    for patch_name, patch in patch_dict.items():
                        # Overwrite patches with offset patches
                        patch_dict[patch_name] = OffsetPatchFunction(patch, offset)

                if verbosity > 0 and len(fuselage_geometries) > 1:
                    print(f"  Fuselage {fuse_no+1} complete.")
                
                # Append fuselage to list
                patches.append(patch_dict)
    
    return patches
