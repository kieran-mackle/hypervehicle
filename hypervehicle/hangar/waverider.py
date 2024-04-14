import numpy as np
from copy import deepcopy
from hypervehicle import Vehicle
from scipy.optimize import bisect
from hypervehicle.generator import Generator
from hypervehicle.transformations import CART3D
from hypervehicle.components import Wing, RevolvedComponent, Fin
from hypervehicle.geometry import Vector3, Bezier, RobertsFunction
from hypervehicle.components.common import uniform_thickness_function


class ParametricWaverider(Generator):
    """Parameterised hypersonic waverider."""

    zero_offset = 1
    curve_zero = 5

    def __init__(self, **kwargs) -> None:
        """Instantiate the parametric waverider.

        Note that the fuselage is a 'ghost' component, meaning that it will not
        be written to STL, but will be included in any analyses and studies.

        Parameters
        ----------
        length : float, optional
            The vehicle length. The default is 1.

        width : float, optional
            The vehicle width. The default is 0.35.

        aoa : float, optional
            The vehicle angle of attack, specified in degrees. The default is
            3.0.

        fuse_stl_res : float, optional
            The STL resolution of the fuselage. The default is 30.

        wing_stl_res : float, optional
            The STL resolution of the wings. The default is 5.

        fin_stl_res : float, optional
            The STL resolution of the fins. The default is 4.

        f1x : float, optional
            Fuselage control point.

        f1y : float, optional
            Fuselage control point.

        f2y : float, optional
            Fuselage control point.

        p2x : float, optional
            Planform control point.

        p2y : float, optional
            Planform control point.

        p3x : float, optional
            Planform control point.

        p3y : float, optional
            Planform control point.

        u1y : float, optional
            Upper surface control point.

        u2x : float, optional
            Upper surface control point.

        u3x : float, optional
            Upper surface control point.

        u3y : float, optional
            Upper surface control point.

        flap_angle : float, optional
            The angle of the wing flap, specified in radians. The default
            is 0.

        flap_height : float, optional
            The height (length) of the flaps.

        a0_t : float, optional
            Wing start control point.

        tt_t : float, optional
            Wing end control point.

        w1x : float, optional
            Wing control point.

        w1y : float, optional
            Wing control point.

        w2x : float, optional
            Wing control point.

        w2y : float, optional
            Wing control point.

        x_curve_mult : float, optional
            Curvature parameter.

        y_curve_mult : float, optional
            Curvature parameter.

        densities : dict, optional
            A dictionary containing the density of each component ('fuselage',
            'wings', 'right_flap', 'left_flap', and 'fin'). If None provided,
            the inertial analysis will not be conducted.
        """
        # Global parameters
        self.length = 1
        self.width = 0.35
        self.le_width = 0.003  # width of LE
        self.LE_thickness = 0.003
        self.aoa = 3.0
        self.fuse_stl_res = 30
        self.wing_stl_res = 5
        self.fin_stl_res = 4

        # Fuselage control points
        body_radius = 0.075  # For manual control/development
        self.f1x = 0.25 * self.length
        self.f1y = body_radius
        self.f2y = body_radius
        self._cf = RobertsFunction(True, True, 1.025)

        # Wing-body planform control points
        self.p2x = 1 * self.length / 3
        self.p2y = 2 * self.width / 2 / 3
        self.p3x = 2.75 * self.length / 3
        self.p3y = 1.5 * self.width / 2 / 3

        # Upper Bezier control points
        self.u1y = 0.1
        self.u2x = self.width / 2 / 3
        self.u3x = 2 * self.width / 2 / 3
        self.u3y = self.u1y / 10

        # Wing flaps
        self.flap_angle = 0  # flap deflection
        self.flap_height = 0.1  # wing span
        self.a0_t = 0.065  # wing connection start
        self.tt_t = 0.35  # wing connection end
        self.w1x = 0.1
        self.w1y = 0.0 + self.zero_offset
        self.w2x = 0.1
        self.w2y = 0.0 + self.zero_offset

        # Curvature
        self.x_curve_mult = 0.001 + self.curve_zero
        self.y_curve_mult = 0.001 + self.curve_zero
        self.curvatures = [
            ("x", self._curv_x, self._curv_xd),
            ("y", self._curv_y, self._curv_yd),
        ]

        # Initialise densities
        self.densities = None

        # Complete vehicle instantiation
        super().__init__(**kwargs)

    def create_instance(self) -> Vehicle:
        # Instantiate Vehicle
        waverider = Vehicle()
        waverider.configure(name="waverider", verbosity=1)

        # Create fuselage
        self._create_fuselage(waverider)

        # Create wings
        self._create_wings(waverider)

        # Create flaps
        self._create_flaps(waverider)

        # Create tail fin
        self._create_fin(waverider)

        # Transform to Cart3D coordinates
        waverider.add_vehicle_transformations(CART3D)

        # Also rotate vehicle to impose AoA
        adjusted_aoa = self._adjust_aoa(self.aoa)
        waverider.add_vehicle_transformations(("rotate", -adjusted_aoa, "z"))

        # Also run analysis after generating patches
        if self.densities:
            waverider.analyse_after_generating(self.densities)

        return waverider

    def _create_fuselage(self, waverider: Vehicle) -> None:
        # Create nominal revolved fuselage
        self.fuselage_revolve_line = Bezier(
            [
                Vector3(x=0.0, y=0.0),
                Vector3(x=self.f1x, y=self.f1y),
                Vector3(x=self.length / 2, y=self.f2y),
                Vector3(x=self.length - self.f1x, y=self.f1y),
                Vector3(x=self.length, y=0.0),
            ]
        )
        fuselage = RevolvedComponent(
            self.fuselage_revolve_line, stl_resolution=self.fuse_stl_res
        )

        # Define offset function to collapse underside
        def modify_underside(x, y, z):
            if z > 0:
                # This is the underside, collapse it
                return -Vector3(0, 0, z)
            else:
                # This is the top side, do nothing
                return Vector3(0, 0, 0)

        # Add fuselage component to vehicle
        waverider.add_component(
            fuselage,
            name="fuselage",
            clustering={"i_clustering_func": self._cf},
            curvatures=self.curvatures,
            modifier_function=modify_underside,
            # ghost=True,
        )

    def _create_wings(self, waverider: Vehicle) -> None:
        # Create body planform line
        A1 = Vector3(x=0.5 * self.length, y=0)
        TT = Vector3(x=self.length, y=0)
        B0 = Vector3(x=0, y=self.width / 2)

        # Need to limit p2y based on fuselage width
        p2y = max(self.p2y, self._get_local_height(self.p2x))
        p3y = max(self.p3y, self._get_local_height(self.p2x))
        Line_B0TT = Bezier(
            [
                B0,
                Vector3(self.p2x, p2y),
                Vector3(self.p3x, p3y),
                TT,
            ]
        )

        # Define body cross-sectional thickness line
        self.Line_B0TT = Line_B0TT
        self.thickness_contour = Bezier(
            [
                Vector3(0.0, self.u1y),
                Vector3(self.u2x, self.u1y),
                Vector3(self.u3x, self.u3y),
                Vector3(self.width / 2, 0.0),
            ]
        )

        def wing1_tf_bot(x, y, z=0):
            # Get local parametric width
            local_width = self._get_local_width(x)
            if local_width == 0:
                s = 0
            else:
                s = y / local_width

            # Get height
            z_val = 0

            return Vector3(x=0, y=0, z=-z_val + self.LE_thickness / 2)

        # Create wing component
        wing = Wing(
            A1=A1,
            TT=TT,
            B0=B0,
            Line_B0TT=Line_B0TT,
            top_tf=self._wing1_tf_top,
            bot_tf=wing1_tf_bot,
            LE_wf=self._lewf,
            stl_resolution=self.wing_stl_res,
        )

        waverider.add_component(
            wing,
            name="wings",
            reflection_axis="y",
            curvatures=self.curvatures,
        )

    def _lewf(self, r):
        """Constant leading edge width function."""
        return self.le_width

    def _get_local_width(self, x):
        """Returns the absolute lateral vehicle width at a given x."""
        func = lambda t: self.Line_B0TT(t).x - x
        t = bisect(func, 0.0, 1.0)
        return self.Line_B0TT(t).y

    def _get_local_height(self, x):
        func = lambda t: self.fuselage_revolve_line(t).x - x
        t = bisect(func, 0.0, 1.0)
        return self.fuselage_revolve_line(t).y

    def _wing1_tf_top(self, x, y, z=0):
        # Get local parametric width
        local_width = self._get_local_width(x)
        if local_width == 0:
            s = 0
        else:
            s = y / local_width

        # Get height
        z_val = self.thickness_contour(s).y

        # Adjust height by local fuselage height
        z_val *= abs(self._get_local_height(x)) / self.u1y

        return Vector3(x=0, y=0, z=-z_val - self.LE_thickness / 2)

    # Define curvatures
    def _curv_x(self, x, y):
        "Curvature in x-direction (about y-axis)"
        return 5 * (self.x_curve_mult - self.curve_zero) * 0.05 * x**2

    def _curv_xd(self, x, y):
        return 5 * (self.x_curve_mult - self.curve_zero) * 0.05 * 2 * x

    def _curv_y(self, x, y):
        "Curvature in y-direction (about x-axis)"
        return 5 * (self.y_curve_mult - self.curve_zero) * 0.1 * y**2

    def _curv_yd(self, x, y):
        return 5 * (self.y_curve_mult - self.curve_zero) * 0.1 * 2 * y

    def _adjust_aoa(self, nominal_aoa):
        """Adjusts the nominal AoA based on the curvature function."""
        angle = self._curv_xd(x=self.length, y=0)
        curve_rotation = np.rad2deg(angle)
        return nominal_aoa + curve_rotation

    def _create_flaps(self, waverider: Vehicle):
        # Define flap planform
        A0 = self.Line_B0TT(self.a0_t)
        A1 = self.Line_B0TT(self.a0_t + (self.tt_t - self.a0_t) / 2)
        TT = self.Line_B0TT(self.tt_t)
        B0 = A0 + Vector3(x=0.0, y=self.flap_height)

        # Define flap leading edge profile
        cp1 = B0 + Vector3(x=self.w1x, y=self.w1y - self.zero_offset)
        cp2 = TT - Vector3(x=self.w2x, y=self.w2y - self.zero_offset)
        line_B0TT = Bezier([B0, cp1, cp2, TT])

        flap_length = A0.x

        wing = Wing(
            A0=A0,
            A1=A1,
            TT=TT,
            B0=B0,
            Line_B0TT=line_B0TT,
            top_tf=uniform_thickness_function(thickness=self.LE_thickness, side="top"),
            bot_tf=uniform_thickness_function(thickness=self.LE_thickness, side="bot"),
            flap_length=flap_length,
            flap_angle=self.flap_angle,
            LE_wf=self._lewf,
            close_wing=True,
            stl_resolution=self.wing_stl_res,
        )

        waverider.add_component(
            wing,
            name="right_flap",
            curvatures=self.curvatures,
        )
        waverider.add_component(
            deepcopy(wing),
            name="left_flap",
            reflection_axis="y",
            append_reflection=False,
            curvatures=self.curvatures,
        )

    def _create_fin(self, waverider: Vehicle):
        fin_thickness = 0.005
        fin_height = 0.1
        fin_length = 0.3

        p0 = Vector3(x=0.0, y=0.0)
        p1 = Vector3(x=0.1 * fin_length, y=0.7 * fin_height)
        p2 = Vector3(x=0.6 * fin_length, y=fin_height)
        p3 = Vector3(x=fin_length, y=0.0)

        fin = Fin(
            p0=p0,
            p1=p1,
            p2=p2,
            p3=p3,
            fin_thickness=fin_thickness,
            fin_angle=np.deg2rad(-90),
            top_thickness_function=uniform_thickness_function(fin_thickness, "top"),
            bot_thickness_function=uniform_thickness_function(fin_thickness, "bot"),
            LE_wf=lambda r: 0.01,
            pivot_point=Vector3(x=0, y=0),
            stl_resolution=self.fin_stl_res,
        )
        waverider.add_component(
            fin,
            name="fin",
            curvatures=self.curvatures,
        )


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_generator = ParametricWaverider()
    vchicle = parametric_generator.create_instance()
    vchicle.generate()
    vchicle.to_stl(merge=True)
