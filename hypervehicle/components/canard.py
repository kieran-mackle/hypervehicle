from __future__ import annotations
import numpy as np
from scipy.optimize import bisect
from gdtk.geom.vector3 import Vector3
from typing import Callable, Optional
from gdtk.geom.surface import CoonsPatch
from gdtk.geom.path import Line, Polyline, Arc
from hypervehicle.components.component import Component
from hypervehicle.components.constants import CANARD_COMPONENT
from hypervehicle.geometry import (
    OffsetPatchFunction,
    SubRangedPath,
    ElipsePath,
    ArcLengthParameterizedPath,
    TrailingEdgePatch,
    TrailingEdgePath,
    RotatedPatch,
    MeanLeadingEdgePatchFunction,
    OffsetPathFunction,
    GeometricMeanPathFunction,
)


class Canard(Component):
    componenttype = CANARD_COMPONENT

    def __init__(
        self,
        p0: Vector3,
        p1: Vector3,
        p2: Vector3,
        p3: Vector3,
        theta1: float,
        theta2: float,
        canard_le:float,
        canard_thickness:float,
        canard_in_length:float,
        canard_out_length:float,
        fin_angle: float,
        body_angle: float,
        mirror: Optional[bool] = False,
        symmetric_canard_def: Optional[bool] = False,
        pivot_angle: Optional[float] = 0,
        pivot_point: Optional[Vector3] = Vector3(x=0, y=0),
        offset_func: Optional[Callable] = None,
        stl_resolution: Optional[int] = 2,
        verbosity: Optional[int] = 1,
        name: Optional[str] = None,
    ) -> None:
        """Creates a new canard component.

        Parameters
        ----------
        p0 : Vector3
            Point p0 of the canard geometry.
        p1 : Vector3
            Point p1 of the canard geometry.
        p2 : Vector3
            Point p2 of the canard geometry.
        p3 : Vector3
            Point p3 of the canard geometry.
        fin_thickness : float
            The thickness of the canard.
        fin_angle : float
            The axial position angle of the placement of the canard.
        top_thickness_function : Callable
            The thickness function for the top surface of the canard.
        bot_thickness_function : Callable
            The thickness function for the top surface of the canard.
        LE_wf : Callable, optional
            The thickness function for the leading edge of the canard.
        mirror : bool, optional
            Mirror the canard. The default is False.
        rudder_type : str, optional
            The type of rudder to use, either "flat" or "sharp". The
            default is "flat".
        rudder_length : float, optional
            The length of the rudder. The default is 0.
        pivot_angle : float, optional
            The pivot angle of the entire canard, about its central axis.
            The default is 0.
        pivot_point : Vector3, optional
            The point about which to apply the pivot_angle. The default
            is Vector3(0,0,0).
        offset_func : Callable, optional
            The function to apply when offsetting the canard position.
            The default is None.
        stl_resolution : int, optional
            The stl resolution to use when creating the mesh for this
            component. The default is None.
        verbosity : int, optional
            The verbosity of the component. The default is 1.
        name : str, optional
            The name tag for the component. The default is None.
        """

        # if LE_wf is None:
        #     # Use default LE function
        #     from hypervehicle.components.common import leading_edge_width_function

        #     LE_wf = leading_edge_width_function

        params = {
            "p0": p0,
            "p1": p1,
            "p2": p2,
            "p3": p3,
            "theta1":theta1,
            "theta2":theta2,
            "canard_thickness":canard_thickness,
            "canard_in_length":canard_in_length,
            "canard_out_length":canard_out_length,
            "canard_le":canard_le,
            "MIRROR_NEW_COMPONENT": mirror,
            "pivot_angle": pivot_angle,
            "pivot_point": pivot_point,
            "fin_angle": fin_angle,
            "body_angle": body_angle,
            "offset_function": offset_func,
            "symmetric_canard_def": symmetric_canard_def,
        }

        super().__init__(params, stl_resolution, verbosity, name)

    def generate_patches(self):
        temp_canard_patch_dict = {}

        # Extract geometric properties
        p0 = self.params["p0"]
        p1 = self.params["p1"]
        p2 = self.params["p2"]
        p3 = self.params["p3"]
        theta1 = self.params["theta1"]
        theta2 = self.params["theta2"]
        canard_thickness = self.params["canard_thickness"]
        canard_in_length = self.params["canard_in_length"]
        canard_out_length = self.params["canard_out_length"]
        canard_le = self.params["canard_le"]
        fin_angle = self.params["fin_angle"]
        symmetric_canard_def = self.params["symmetric_canard_def"]

        # top and bottom vertices
        p4t = p0 + Vector3(x=canard_thickness/np.tan(theta1), y=0.0, z=canard_thickness) 
        p4b = p4t + Vector3(x=0.0, y=0.0, z=-2*canard_thickness)
        p5t = p0 + Vector3(x=canard_in_length-canard_thickness/np.tan(theta2), y=0.0, z=canard_thickness) 
        p5b = p5t + Vector3(x=0.0, y=0.0, z=-2*canard_thickness)
        p6t = p1 + Vector3(x=canard_out_length-canard_thickness/np.tan(theta2), y=0.0,z=canard_thickness) 
        p6b = p6t + + Vector3(x=0.0, y=0.0, z=-2*canard_thickness)
        p7t = p1 + Vector3(x=canard_thickness/np.tan(theta1),y=0.0, z=canard_thickness) 
        p7b = p7t + + Vector3(x=0.0, y=0.0, z=-2*canard_thickness)

        # leading and trailing edge radius intersection points for arcs
        # rear inside
        p0c = p0 + Vector3(x=canard_le/np.tan(theta1), y=0.0, z=0.0)
        p0t = p0 + Vector3(x=canard_le/np.tan(theta1), y=0.0, z=canard_le) 
        p0r = p0c + Vector3(x=-canard_le, y=0.0, z=0.0) 
        p0b = p0t + Vector3(x=0.0, y=0.0, z=-2*canard_le)
        # rear outside
        p1c = p1 + Vector3(x=canard_le/np.tan(theta1), y=0.0, z=0.0)
        p1t = p1 + Vector3(x=canard_le/np.tan(theta1), y=0.0, z=canard_le) 
        p1r = p1c + Vector3(x=-canard_le, y=0.0, z=0.0) 
        p1b = p1t + Vector3(x=0.0, y=0.0, z=-2*canard_le)
        # front inside
        p3c = p3 + Vector3(x=-canard_le/np.tan(theta1), y=0.0, z=0.0) 
        p3t = p3 + Vector3(x=-canard_le/np.tan(theta1), y=0.0, z=canard_le) 
        p3r = p3c + Vector3(x=+canard_le, y=0, z=0) 
        p3b = p3t + Vector3(x=0.0, y=0.0, z=-2*canard_le)
        # front outside
        p2c = p2 + Vector3(x=-canard_le/np.tan(theta1), y=0.0, z=0.0) 
        p2t = p2 + Vector3(x=-canard_le/np.tan(theta1), y=0.0, z=canard_le) 
        p2r = p2c + Vector3(x=+canard_le, y=0, z=0) 
        p2b = p2t + Vector3(x=0.0, y=0.0, z=-2*canard_le)

        # lines
        # inside direction
        p4t_p0t = Line(p4t,p0t)
        p4b_p4t = Line(p4b,p4t)
        p4b_p0b = Line(p4b,p0b)


        p3t_p5t = Line(p3t,p5t)
        p5b_p5t = Line(p5b,p5t)
        p3b_p5b = Line(p3b,p5b)

        p5t_p4t = Line(p5t,p4t)
        p5b_p4b = Line(p5b,p4b)

        p1t_p7t = Line(p1t,p7t)
        p7b_p7t = Line(p7b,p7t)
        p1b_p7b = Line(p1b,p7b)

        p7t_p6t = Line(p7t,p6t)
        p7b_p6b = Line(p7b,p6b)

        p6t_p2t = Line(p6t,p2t)
        p6b_p2b = Line(p6b,p2b)
        p6b_p6t = Line(p6b,p6t)

        # lines from inside to outside
        p0t_p1t = Line(p0t,p1t)
        p4t_p7t = Line(p4t,p7t)
        p5t_p6t = Line(p5t,p6t)
        p3t_p2t = Line(p3t,p2t)
        p3b_p2b = Line(p3b,p2b)
        p5b_p6b = Line(p5b,p6b)
        p4b_p7b = Line(p4b,p7b)
        p0b_p1b = Line(p0b,p1b)
        #reverse direction of inside top lines for patches
        p0t_p4t = Line(p0t,p4t)
        p4t_p5t = Line(p4t,p5t)
        p5t_p3t = Line(p5t,p3t)
        #reverse direction of outside bottom lines for patches
        p2b_p6b = Line(p2b,p6b)
        p6b_p7b = Line(p6b,p7b)
        p7b_p1b = Line(p7b,p1b)
        
        #leading and trailing edges
        # inside trailing edge
        ins_te_top = Arc(p0r,p0t,p0c)
        ins_te_bot = Arc(p0b,p0r,p0c)
        ins_te = Polyline(segments=(ins_te_bot,ins_te_top))
        # outside trailing edge
        out_te_top = Arc(p1r,p1t,p1c)
        out_te_bot = Arc(p1b,p1r,p1c)
        out_te = Polyline(segments=(out_te_bot,out_te_top))
        # inside leading edge
        ins_le_top = Arc(p3r,p3t,p3c)
        ins_le_bot = Arc(p3b,p3r,p3c)
        ins_le = Polyline(segments=(ins_le_bot,ins_le_top))
        # outside leading edge
        out_le_top = Arc(p2r,p2t,p2c)
        out_le_bot = Arc(p2b,p2r,p2c)
        out_le = Polyline(segments=(out_le_bot,out_le_top))

        # if you want flat leading and trailing edges replace arcs with the below. 
        p0b_p0t = Line(p0b,p0t)
        p1b_p1t = Line(p1b,p1t)
        p3b_p3t = Line(p3b,p3t)
        p2b_p2t = Line(p2b,p2t)

        # ------------- inside faces ----------------
        # trailing edge inside face
        print('inside face')
        n1 = p4t_p0t
        w1 = p4b_p4t
        s1 = p4b_p0b
        e1 = p0b_p0t #ins_te
        TE_IF = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["TE_IF"] = TE_IF

        # leading edge inside face
        n1 = p3t_p5t
        e1 = p5b_p5t
        s1 = p3b_p5b
        w1 = p3b_p3t # ins_le
        LE_IF = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["LE_IF"] = LE_IF

        #inside middle face
        n1 = p5t_p4t
        e1 = p4b_p4t
        s1 = p5b_p4b
        w1 = p5b_p5t
        IMF = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["IMF"] = IMF

        # ------------- outside faces ----------------
        # trailing edge outside face
        n1 = p1t_p7t
        e1 = p7b_p7t
        s1 = p1b_p7b
        w1 = p1b_p1t #out_te
        TE_OF = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["TE_OF"] = TE_OF

        # leading edge outside face
        n1 = p6t_p2t
        e1 = p2b_p2t #out_le
        s1 = p6b_p2b
        w1 = p6b_p6t 
        LE_OF = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["LE_OF"] = LE_OF

        # middle outside face
        n1 = p7t_p6t
        e1 = p6b_p6t
        s1= p7b_p6b
        w1 = p7b_p7t
        MOF = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["MOF"] = MOF

        # ---------- top surfaces -----------------
        # trailing edge from rear on
        n1 = p0t_p1t
        e1 = Line(p1b, p1t) 
        s1 = p0b_p1b
        w1 = Line(p0b,p0t) 
        TE = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["TE"] = TE

        # trailing edge top ramp
        n1 = p4t_p7t
        e1 = p1t_p7t
        s1 = p0t_p1t
        w1 = p0t_p4t
        TE_TR = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["TE_TR"] = TE_TR

        # top middle flat
        n1 = p5t_p6t
        e1 = p7t_p6t
        s1 = p4t_p7t
        w1 = p4t_p5t
        TMF = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["TMF"] = TMF
    
        # leading edge top ramp
        n1 = p3t_p2t
        e1 = p6t_p2t
        s1 = p5t_p6t
        w1 = p5t_p3t
        LE_TR = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["LE_TR"] = LE_TR

        # leading edge from front on
        # have to reverse the arc - from p2t -> p2b
        # inside leading edge reverse
        p3t_p3r = Arc(p3t,p3r,p3c)
        p3r_p3b = Arc(p3r,p3b,p3c)
        # p3t_p3b = Polyline(segments=(p3t_p3r,p3r_p3b))
        p3t_p3b = Line(p3t,p3b)           # replace the above if you want flat leading edge
        # outside leading edge reverse
        p2t_p2r = Arc(p2t,p2r,p2c)
        p2r_p2b = Arc(p2r,p2b,p2c)
        # p2t_p2b = Polyline(segments=(p2t_p2r,p2r_p2b))
        p2t_p2b = Line(p2t,p2b)           # replace the above if you want flat trailing edge

        n1 = p3b_p2b
        # e1 = Line(p2t,p2b) 
        e1 = p2t_p2b
        s1 = p3t_p2t
        # w1 = Line(p3t,p3b) 
        w1 = p3t_p3b
        O_LE_R = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["O_LE_R"] = O_LE_R
        # print('n1:',n1)
        # print('e1:',e1)
        # print('s1:',s1)
        # print('w1:',w1)


        # leading edge bottom ramp
        n1 = p5b_p6b
        e1 = p2b_p6b
        s1 = p3b_p2b
        w1 = p3b_p5b
        LE_BR = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["LE_BR"] = LE_BR

        # bottom middle flat
        n1 = p4b_p7b
        e1 = p6b_p7b
        s1 = p5b_p6b
        w1 = p5b_p4b        
        BMF = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["BMF"] = BMF

        # trailing edge bottom ramp
        w1 = p4b_p0b
        e1 = p7b_p1b
        n1 = p0b_p1b
        s1 = p4b_p7b
        TE_BR = CoonsPatch(north=n1, east=e1, south=s1, west=w1)
        temp_canard_patch_dict["TE_BR"] = TE_BR

        # Create fin patch dict
        canard_patch_dict = temp_canard_patch_dict.copy()

        # angle the two canards in towards the body 
        if "body_angle" in self.params:
            for key, patch in canard_patch_dict.items():
                canard_patch_dict[key] = RotatedPatch(
                    patch, self.params["body_angle"],
                    axis="z",
                    point=self.params["pivot_point"],
                )


        # if asymmetric canard deflection then place the canard on the other side BEFORE it has been rotated by deflection angle
        if symmetric_canard_def == False:
            # Rotate patches and add to fin_patch_dict
            for key, patch in canard_patch_dict.items():
                canard_patch_dict[key] = RotatedPatch(patch, fin_angle)


        # Rotate patches by canard deflection angle
        if "pivot_angle" in self.params:
            for key, patch in canard_patch_dict.items():
                canard_patch_dict[key] = RotatedPatch(
                    patch,
                    self.params["pivot_angle"],
                    axis="y",
                    point=self.params["pivot_point"],
                )


        # if symmetric canard deflection then place the canard on the other side AFTER it has been rotated by deflection angle
        if symmetric_canard_def == True:
            # Rotate patches and add to fin_patch_dict
            for key, patch in canard_patch_dict.items():
                canard_patch_dict[key] = RotatedPatch(patch, fin_angle)            



        # Save patches
        self.patches = canard_patch_dict
