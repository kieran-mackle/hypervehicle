from __future__ import annotations
import numpy as np
from typing import Callable
from scipy.optimize import bisect
from hypervehicle.geometry.vector import Vector3
from hypervehicle.geometry.surface import CoonsPatch
from hypervehicle.geometry.path import Line, Bezier, Polyline

from hypervehicle.components.component import Component
from hypervehicle.components.constants import WING_COMPONENT
from hypervehicle.geometry.geometry import (
    OffsetPatchFunction,
    SubRangedPath,
    ElipsePath,
    ArcLengthParameterizedPath,
    TrailingEdgePath,
    OffsetPathFunction,
    GeometricMeanPathFunction,
    MeanLeadingEdgePatchFunction,
    MeanTrailingEdgePatch,
    RotatedPatch,
    FlatLeadingEdgePatchFunction,
)


class Wing(Component):
    componenttype = WING_COMPONENT

    def __init__(
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
        close_wing: bool = False,
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
    ) -> None:
        """Creates a new fin component.

        Parameters
        ----------
        A0 : Vector3
            Point p0 of the fin geometry.

        A1 : Vector3
            Point p1 of the fin geometry.

        TT : Vector3
            Point p2 of the fin geometry.

        B0 : Vector3
            Point p3 of the fin geometry.

        Line_B0TT : Polyline
            The thickness of the fin.

        Line_B0TT_TYPE : str, optional
            The axial position angle of the placement of the fin.

        t_B1 : float, optional
            The t value of the first discretisation point. The default
            is None.

        t_B2 : float, optional
            The t value of the second discretisation point. The default
            is None.

        top_tf : Callable
            The thickness function for the top surface of the wing.

        bot_tf : Callable
            The thickness function for the top surface of the wing.

        LE_wf : Callable, optional
            The thickness function for the leading edge of the wing.

        LE_type : str, optional
            The type of LE to create, either "FLAT" or "custom". The
            default is "custom".

        tail_option : str, optional
            The type of trailing edge to use, currently only "FLAP". The
            default is "FLAP".

        flap_length : float, optional
            The length of the trailing edge flap. The default is 0.

        flap_angle : float, optional
            The angle of the flap, specified in radians. The default is 0.

        close_wing : bool, optional
            If the wing is not being mirrored, it is useful to set this
            to True, to close the STL object. The default is False.

        stl_resolution : int, optional
            The stl resolution to use when creating the mesh for this
            component. The default is None.

        verbosity : int, optional
            The verbosity of the component. The default is 1.

        name : str, optional
            The name tag for the component. The default is None.
        """
        # Check if a LE function was provided
        if LE_wf is None and LE_type == "custom":
            # Assign default LE function
            from hypervehicle.components.common import leading_edge_width_function

            LE_wf = leading_edge_width_function

        params = {
            "A0": A0,
            "A1": A1,
            "TT": TT,
            "B0": B0,
            "Line_B0TT": Line_B0TT,
            "Line_B0TT_TYPE": Line_B0TT_TYPE,
            "t_B1": t_B1,
            "t_B2": t_B2,
            "FUNC_TOP_THICKNESS": top_tf,
            "FUNC_BOT_THICKNESS": bot_tf,
            "FUNC_LEADING_EDGE_WIDTH": LE_wf,
            "LE_TYPE": LE_type.upper(),
            "TAIL_OPTION": tail_option,
            "FLAP_LENGTH": flap_length,
            "FLAP_ANGLE": flap_angle,
            "CLOSE_WING": close_wing,
        }

        super().__init__(
            params=params, stl_resolution=stl_resolution, verbosity=verbosity, name=name
        )

        # Extract construction points for planform
        # TODO - avoid pre-defined params dict structure for flexibility
        self.A0 = params["A0"]
        self.A1 = params["A1"]
        self.TT = params["TT"]
        self.B0 = params["B0"]
        self.Line_B0TT = params["Line_B0TT"]

        # Find Locations
        if self.params["t_B1"] == None:
            fun_B1 = lambda t: self.Line_B0TT(t).x - self.A1.x
            self.t_B1 = round(bisect(fun_B1, 0.0, 1.0), 6)
        else:
            self.t_B1 = self.params["t_B1"]

        if self.params["t_B2"] == None:
            self.t_B2 = 0.5 * (1 + self.t_B1)
        else:
            self.t_B2 = self.params["t_B1"]

        # Save other params
        self.TE_top = None
        self.TE_bot = None
        self.TE_mean_line = None

    def generate_patches(self):
        # Create wing planform shape
        self._create_planform_patches()

        # Create leading edge patches
        self._create_leading_edge()

        # Create trailing edge patches
        self._create_trailing_edge()

        if "CLOSE_WING" in self.params and self.params["CLOSE_WING"]:
            self._close_wing()

    def _create_planform_patches(self):
        if self.params["Line_B0TT_TYPE"].lower() == "bezier":
            if self.verbosity > 1:
                print(
                    "    Constructing planform using Bezier Curve as Leading Edge shape."
                )

            B1 = self.Line_B0TT(self.t_B1)
            Line_B0B1 = SubRangedPath(
                underlying_path=self.Line_B0TT, t0=0.0, t1=self.t_B1
            )
            Line_B1B2 = SubRangedPath(
                underlying_path=self.Line_B0TT, t0=self.t_B1, t1=self.t_B2
            )
            Line_TTB2 = SubRangedPath(
                underlying_path=self.Line_B0TT, t0=1.0, t1=self.t_B2
            )

        else:
            raise Exception(
                f"Option for 'Line_B0TT'={self.params['Line_B0TT_TYPE']} "
                + "not supported."
            )

        wing_patch = [np.nan, np.nan]
        wing_patch_flipped = [np.nan, np.nan]
        wing_patch[0] = CoonsPatch(
            south=Line(p0=self.A0, p1=self.A1),
            north=Line_B0B1,
            west=Line(p0=self.A0, p1=self.B0),
            east=Line(p0=self.A1, p1=B1),
        )
        wing_patch[1] = CoonsPatch(
            south=Line(p0=self.A1, p1=self.TT),
            north=Line_B1B2,
            west=Line(p0=self.A1, p1=B1),
            east=Line_TTB2,
        )

        # Need flipped planform for lower side to ensure vecors point in correct direction
        Line_B0B1_flipped = SubRangedPath(underlying_path=Line_B0B1, t0=1.0, t1=0.0)
        Line_B1B2_flipped = SubRangedPath(underlying_path=Line_B1B2, t0=1.0, t1=0.0)
        wing_patch_flipped[0] = CoonsPatch(
            south=Line(p0=self.A1, p1=self.A0),
            north=Line_B0B1_flipped,
            east=Line(p0=self.A0, p1=self.B0),
            west=Line(p0=self.A1, p1=B1),
        )
        wing_patch_flipped[1] = CoonsPatch(
            south=Line(p0=self.TT, p1=self.A1),
            north=Line_B1B2_flipped,
            east=Line(p0=self.A1, p1=B1),
            west=Line_TTB2,
        )

        # Create wing top & bottom surface
        if self.verbosity > 1:
            print("    Adding thickness to wing.")
        top_patch = [np.nan, np.nan]
        bot_patch = [np.nan, np.nan]

        for i in range(2):
            top_patch[i] = OffsetPatchFunction(
                wing_patch_flipped[i], function=self.params["FUNC_TOP_THICKNESS"]
            )

            # flipped moves to top as z-positive points downwards
            bot_patch[i] = OffsetPatchFunction(
                wing_patch[i], function=self.params["FUNC_BOT_THICKNESS"]
            )

        self.patches["top_patch_0"] = top_patch[0]  # top B0B1A1A0
        self.patches["top_patch_1"] = top_patch[1]  # top B1B2TTA1
        self.patches["bot_patch_0"] = bot_patch[0]  # bot B0B1A1A0
        self.patches["bot_patch_1"] = bot_patch[1]  # bot B1B2TTA1

    def _create_leading_edge(self):
        # Add leading edge
        if self.verbosity > 1:
            print("    Adding Leading Edge to wing.")

        top_edge_path = OffsetPathFunction(
            self.Line_B0TT, self.params["FUNC_TOP_THICKNESS"]
        )
        bot_edge_path = OffsetPathFunction(
            self.Line_B0TT, self.params["FUNC_BOT_THICKNESS"]
        )

        if "LE_TYPE" in self.params and self.params["LE_TYPE"] == "FLAT":
            self.patches["LE_patch0"] = FlatLeadingEdgePatchFunction(
                top_edge_path, bot_edge_path, 0, self.t_B1
            )
            self.patches["LE_patch1"] = FlatLeadingEdgePatchFunction(
                top_edge_path, bot_edge_path, self.t_B1, self.t_B2
            )
            self.patches["LE_patch2"] = FlatLeadingEdgePatchFunction(
                top_edge_path, bot_edge_path, self.t_B1, 1
            )

        else:
            # Get mean line between upper and lower wing patches
            mean_path = GeometricMeanPathFunction(top_edge_path, bot_edge_path)

            LE_top_patch = [np.nan, np.nan, np.nan]
            LE_bot_patch = [np.nan, np.nan, np.nan]

            # Eliptical LE
            LE_top_patch[0] = MeanLeadingEdgePatchFunction(
                mean_path,
                top_edge_path,
                LE_width_function=self.params["FUNC_LEADING_EDGE_WIDTH"],
                t0=0.0,
                t1=self.t_B1,
                side="top",
            )
            LE_top_patch[1] = MeanLeadingEdgePatchFunction(
                mean_path,
                top_edge_path,
                LE_width_function=self.params["FUNC_LEADING_EDGE_WIDTH"],
                t0=self.t_B1,
                t1=self.t_B2,
                side="top",
            )
            LE_top_patch[2] = MeanLeadingEdgePatchFunction(
                mean_path,
                top_edge_path,
                LE_width_function=self.params["FUNC_LEADING_EDGE_WIDTH"],
                t0=self.t_B2,
                t1=1.0,
                side="top",
            )

            LE_bot_patch[0] = MeanLeadingEdgePatchFunction(
                mean_path,
                bot_edge_path,
                LE_width_function=self.params["FUNC_LEADING_EDGE_WIDTH"],
                t0=0.0,
                t1=self.t_B1,
                side="bot",
            )
            LE_bot_patch[1] = MeanLeadingEdgePatchFunction(
                mean_path,
                bot_edge_path,
                LE_width_function=self.params["FUNC_LEADING_EDGE_WIDTH"],
                t0=self.t_B1,
                t1=self.t_B2,
                side="bot",
            )
            LE_bot_patch[2] = MeanLeadingEdgePatchFunction(
                mean_path,
                bot_edge_path,
                LE_width_function=self.params["FUNC_LEADING_EDGE_WIDTH"],
                t0=self.t_B2,
                t1=1.0,
                side="bot",
            )

            # Append to patch_dict
            self.patches["LE_top_patch_0"] = LE_top_patch[0]
            self.patches["LE_top_patch_1"] = LE_top_patch[1]
            self.patches["LE_top_patch_2"] = LE_top_patch[2]
            self.patches["LE_bot_patch_0"] = LE_bot_patch[0]
            self.patches["LE_bot_patch_1"] = LE_bot_patch[1]
            self.patches["LE_bot_patch_2"] = LE_bot_patch[2]

    def _create_trailing_edge(self):
        # Add trailing Edge
        if self.verbosity > 1:
            print("    Adding Trailing Edge.")
            print("      Tail options - {}".format(self.params["TAIL_OPTION"]))

        if self.params["TAIL_OPTION"] == "FLAP":
            if self.verbosity > 1:
                print("        Flap length = {}".format(self.params["FLAP_LENGTH"]))
                print("        Flap angle  = {}".format(self.params["FLAP_ANGLE"]))

            # Define top and bottom TE paths
            self.TE_top = TrailingEdgePath(
                self.A0, self.B0, thickness_function=self.params["FUNC_TOP_THICKNESS"]
            )
            self.TE_bot = TrailingEdgePath(
                self.A0, self.B0, thickness_function=self.params["FUNC_BOT_THICKNESS"]
            )
            self.TE_mean_line = GeometricMeanPathFunction(self.TE_top, self.TE_bot)

            # Make top and bottom of flap
            TE_top_patch = MeanTrailingEdgePatch(
                self.TE_mean_line,
                TE_path=self.TE_top,
                flap_length=self.params["FLAP_LENGTH"],
                flap_angle=self.params["FLAP_ANGLE"],
                side="top",
            )
            TE_bot_patch = MeanTrailingEdgePatch(
                self.TE_mean_line,
                TE_path=self.TE_bot,
                flap_length=self.params["FLAP_LENGTH"],
                flap_angle=self.params["FLAP_ANGLE"],
                side="bot",
            )

            # Append to patch_dict
            self.patches["TE_top_patch"] = TE_top_patch
            self.patches["TE_bot_patch"] = TE_bot_patch

            # Create edge (connecting side to TE)
            if "LE_TYPE" in self.params and self.params["LE_TYPE"] == "FLAT":
                # Flat LE
                west = Line(p0=self.TE_top(1), p1=self.TE_mean_line(1))
                north = Line(p0=self.TE_mean_line(1), p1=self.TE_bot(1))

            else:
                # Eiliptical LE
                thickness_top = self.TE_mean_line(1).z - self.TE_top(1).z
                thickness_bot = self.TE_mean_line(1).z - self.TE_bot(1).z

                if thickness_top == 0 or thickness_bot == 0:
                    raise Exception(
                        "Elliptical LE cannot be created when thickness converges to zero."
                    )

                elipse_top = ElipsePath(
                    centre=self.TE_mean_line(1),
                    thickness=thickness_top,
                    LE_width=self.params["FUNC_LEADING_EDGE_WIDTH"](0.0),
                    side="top",
                )
                elipse_bot = ElipsePath(
                    centre=self.TE_mean_line(1),
                    thickness=thickness_bot,
                    LE_width=self.params["FUNC_LEADING_EDGE_WIDTH"](0.0),
                    side="bot",
                )

                elipse_top = ArcLengthParameterizedPath(underlying_path=elipse_top)
                elipse_bot = ArcLengthParameterizedPath(underlying_path=elipse_bot)

                north = elipse_bot
                west = elipse_top

            south = Line(
                p0=Vector3(
                    x=self.TE_mean_line(1).x,
                    y=self.TE_mean_line(1).y,
                    z=self.TE_top(1).z,
                ),
                p1=Vector3(
                    x=self.TE_mean_line(1).x
                    - self.params["FLAP_LENGTH"] * np.cos(self.params["FLAP_ANGLE"]),
                    y=self.TE_mean_line(1).y,
                    z=self.TE_mean_line(1).z
                    + self.params["FLAP_LENGTH"] * np.sin(self.params["FLAP_ANGLE"]),
                ),
            )

            east = Line(
                p0=Vector3(
                    x=self.TE_mean_line(1).x
                    - self.params["FLAP_LENGTH"] * np.cos(self.params["FLAP_ANGLE"]),
                    y=self.TE_mean_line(1).y,
                    z=self.TE_mean_line(1).z
                    + self.params["FLAP_LENGTH"] * np.sin(self.params["FLAP_ANGLE"]),
                ),
                p1=Vector3(
                    x=self.TE_mean_line(1).x,
                    y=self.TE_mean_line(1).y,
                    z=self.TE_bot(1).z,
                ),
            )

            TELE_patch = CoonsPatch(north=north, west=west, south=south, east=east)

            # Append to patch_dict
            self.patches["TELE_patch"] = TELE_patch

        else:
            raise Exception(
                "Tail option = {} not supported.".format(self.params["TAIL_OPTION"])
            )

    def _close_wing(self):
        # Add patch to close wing volume
        if self.verbosity > 1:
            print("    Closing interior side of wing.")

        TT_top = self.TT + self.params["FUNC_TOP_THICKNESS"](
            x=self.TT.x, y=self.TT.y, z=self.TT.z
        )
        TT_bot = self.TT + self.params["FUNC_BOT_THICKNESS"](
            x=self.TT.x, y=self.TT.y, z=self.TT.z
        )
        TT_mid = 0.5 * (TT_top + TT_bot)

        interior_top_0 = Line(p0=self.TE_top(0), p1=self.patches["top_patch_0"](0, 0))
        interior_top_1 = Line(p0=self.patches["top_patch_0"](0, 0), p1=TT_top)

        interior_bot_0 = Line(p0=self.TE_bot(0), p1=self.patches["bot_patch_0"](1, 0))
        interior_bot_1 = Line(p0=self.patches["bot_patch_0"](1, 0), p1=TT_bot)

        mid_mid_point = Vector3(
            x=self.patches["bot_patch_0"](1, 0).x,
            y=self.patches["bot_patch_0"](1, 0).y,
            z=(
                self.patches["bot_patch_0"](1, 0).z
                + self.patches["top_patch_0"](0, 0).z
            )
            / 2,
        )
        interior_mid_0 = Line(p0=self.TE_mean_line(0), p1=mid_mid_point)
        interior_mid_1 = Line(p0=mid_mid_point, p1=TT_mid)

        # Create vertical edges
        mid_vert_top = Line(p1=mid_mid_point, p0=self.patches["top_patch_0"](0, 0))
        mid_vert_bot = Line(p0=mid_mid_point, p1=self.patches["bot_patch_0"](1, 0))

        # Vertical edges at TE ('back')
        back_top = Line(p0=self.TE_top(0), p1=self.TE_mean_line(0))
        back_bot = Line(p0=self.TE_mean_line(0), p1=self.TE_bot(0))

        # Create patches
        interior_top_patch0 = CoonsPatch(
            north=interior_mid_0,
            east=mid_vert_top,
            south=interior_top_0,
            west=back_top,
        )
        interior_top_patch1 = CoonsPatch(
            north=interior_mid_1,
            east=Line(p0=TT_top, p1=TT_mid),
            south=interior_top_1,
            west=mid_vert_top,
        )

        interior_bot_patch0 = CoonsPatch(
            north=interior_bot_0,
            east=mid_vert_bot,
            south=interior_mid_0,
            west=back_bot,
        )
        interior_bot_patch1 = CoonsPatch(
            north=interior_bot_1,
            east=Line(p0=TT_mid, p1=TT_bot),
            south=interior_mid_1,
            west=mid_vert_bot,
        )

        elipse_top = ElipsePath(
            centre=TT_mid,
            thickness=TT_mid.z - TT_top.z,
            LE_width=self.params["FUNC_LEADING_EDGE_WIDTH"](1),
            side="top",
        )
        elipse_bot = ElipsePath(
            centre=TT_mid,
            thickness=TT_mid.z - TT_bot.z,
            LE_width=self.params["FUNC_LEADING_EDGE_WIDTH"](1),
            side="bot",
        )
        elipse_top = ArcLengthParameterizedPath(underlying_path=elipse_top)
        elipse_bot = ArcLengthParameterizedPath(underlying_path=elipse_bot)

        # Now reverse the paths for correct orientation
        elipse_bot = SubRangedPath(underlying_path=elipse_bot, t0=1.0, t1=0.0)

        interior_ellip = CoonsPatch(
            north=elipse_bot,
            east=elipse_top,
            south=Line(p0=TT_mid, p1=TT_top),
            west=Line(p0=TT_mid, p1=TT_bot),
        )

        # Rotate patch
        interior_ellip = RotatedPatch(
            interior_ellip, np.deg2rad(-90), axis="z", point=TT_mid
        )

        # Append to patch_dict
        self.patches["wing_close_top_patch0"] = interior_top_patch0
        self.patches["wing_close_top_patch1"] = interior_top_patch1
        self.patches["wing_close_bot_patch0"] = interior_bot_patch0
        self.patches["wing_close_bot_patch1"] = interior_bot_patch1
        self.patches["interior_ellip"] = interior_ellip

        # Interior TE
        TE_point = Vector3(
            x=self.TE_mean_line(0).x
            - self.params["FLAP_LENGTH"] * np.cos(self.params["FLAP_ANGLE"]),
            y=self.TE_mean_line(0).y,
            z=self.TE_mean_line(0).z
            + self.params["FLAP_LENGTH"] * np.sin(self.params["FLAP_ANGLE"]),
        )

        interior_flap_patch = CoonsPatch(
            north=Line(p0=self.TE_top(0), p1=TE_point),
            east=Line(p0=self.TE_bot(0), p1=TE_point),
            south=Line(p0=self.TE_mean_line(0), p1=self.TE_bot(0)),
            west=Line(p0=self.TE_mean_line(0), p1=self.TE_top(0)),
        )

        self.patches["interior_flap_patch"] = interior_flap_patch
