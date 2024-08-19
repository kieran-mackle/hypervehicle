import numpy as np
from hypervehicle import Vehicle
from scipy.optimize import bisect
from hypervehicle.generator import Generator
from hypervehicle.components import Wing, Fin, RevolvedComponent
from hypervehicle.geometry import Vector3, Bezier, Line, Polyline


class ParametricD21(Generator):
    """Parametric generator for mock-up of the Lockheed D-21 drone.

    Dimensions have been approximated based on vehicle's visual proportions.

    References
    ----------
    https://en.wikipedia.org/wiki/Lockheed_D-21
    """

    def __init__(self, **kwargs) -> None:
        # Vehicle parameters
        self.R_b = 1  # Body radius
        self.L_b = 20  # Body length
        self.L_w = 16  # Wing length
        self.W_w = 5  # Wing width,
        self.flap_angle = 0

        # STL resolutions
        self.wing_stl_res = 3
        self.te_fin_stl_res = 3

        # Complete instantiation
        super().__init__(**kwargs)

    def create_instance(self) -> Vehicle:
        # Initialise
        d21 = Vehicle()
        d21.configure(
            name="Lockhead Martin D-21",
            verbosity=1,
        )

        # Construct fuselage
        # ------------------------
        n = Vector3(x=0, y=0)
        b0 = Vector3(x=0, y=self.R_b)
        b1 = b0 - Vector3(x=self.L_b, y=0)
        b2 = b1 - Vector3(x=0, y=self.R_b)
        bL0 = Line(n, b0)
        bL1 = Line(b0, b1)
        bL2 = Line(b1, b2)

        fuseline = Polyline([bL0, bL1, bL2])
        fuselage = RevolvedComponent(revolve_line=fuseline, stl_resolution=20)
        d21.add_component(fuselage)

        # Construct Aerospike nose
        # --------------------------
        shift = Vector3(x=-0.1, y=0.0)
        tip = Vector3(x=0.15 * self.L_b, y=0) + shift
        top = Vector3(x=0.0, y=1 * self.R_b) + shift
        mid_cp = Vector3(x=0.0, y=0.3 * self.R_b) + shift

        front_bz = Bezier([tip, mid_cp, top])
        back_cap = Line(p0=top, p1=Vector3(x=-0.5, y=0.0) + shift)

        noseline = Polyline([front_bz, back_cap])
        nose = RevolvedComponent(revolve_line=noseline, stl_resolution=20)
        d21.add_component(nose)

        # Construct wings
        # ------------------------
        #    ---   |---B0------________________B1
        #     |    |   |                       ----____
        #     |    |   |                                ----_____
        #     |    |   |                                         --____B2
        #    ---   |---A0----------------------A1----------------------TT
        #
        #             |------------------- L_w ---------------------|

        wing_thickness = 0.15 * self.R_b
        wing_span = self.W_w
        flap_length = 0.1 * self.L_w
        wing_LE_shift = Vector3(x=0.3, y=0)

        # Required points
        wing_tip = Vector3(x=0.0, y=0.0)  # Tip of wing reference point
        A0 = wing_tip - Vector3(x=self.L_w, y=0)
        A1 = Vector3(x=A0.x + 0.6 * self.L_w, y=0)
        TT = Vector3(x=A0.x + self.L_w, y=0) - wing_LE_shift
        B0 = Vector3(x=A0.x, y=wing_span)

        # Construction points
        B1 = Vector3(x=A1.x, y=0.5 * B0.y)
        B2 = Vector3(x=TT.x, y=0.15 * wing_span)

        # Leading edge line
        B0B1 = Line(p0=B0, p1=B1)
        # B1B2 = Line(p0=B1, p1=B2)

        # Create Bezier for front edge
        a = B1
        b = Vector3(x=0.5 * (B1.x + B2.x), y=B1.y)
        c = B2
        B1B2 = Bezier([a, b, c])

        B2TT = Line(p0=B2, p1=TT)
        Line_B0TT = Polyline([B0B1, B1B2, B2TT])

        def local_ws(x):
            """Calculates the local wingspan."""
            func = lambda t: Line_B0TT(t).x - x
            t = bisect(func, 0.0, 1.0)
            return Line_B0TT(t).y

        def wing_tf_top(x, y, z=0):
            # Get the local wingspan
            lws = local_ws(x)
            if lws == 0:
                lws = 1e-3

            # Define a thickness scalar which
            thickener = 0.5 * (1 - y / lws)

            return Vector3(x=0, y=0, z=-wing_thickness / 2 - thickener)

        def wing_tf_bot(x, y, z=0):
            return Vector3(x=0, y=0, z=wing_thickness / 2)

        wing = Wing(
            A0=A0,
            A1=A1,
            TT=TT,
            B0=B0,
            Line_B0TT=Line_B0TT,
            top_tf=wing_tf_top,
            bot_tf=wing_tf_bot,
            flap_length=flap_length,
            flap_angle=np.deg2rad(self.flap_angle),
            stl_resolution=self.wing_stl_res,
        )
        d21.add_component(wing, reflection_axis="y")

        # Construct tail
        # ------------------------
        #   |--p1-----p2
        #   |  |         \
        #   |  |           \
        #   |  |             \
        #   |--p0______________p3

        fin_height = 0.15 * self.W_w
        fin_thickness = wing_thickness
        fin_length = self.L_b - self.L_w

        p0 = b2
        p1 = p0 + Vector3(x=0, y=fin_height)
        p2 = p1 + Vector3(x=0.4 * fin_length, y=0)
        p3 = p0 + Vector3(x=fin_length, y=0)

        # Thickness functions
        def fin_thickness_function_top(x, y, z=0):
            return Vector3(x=0.0, y=0.0, z=-fin_thickness / 2)

        def fin_thickness_function_bot(x, y, z=0):
            return Vector3(x=0.0, y=0.0, z=fin_thickness / 2)

        offset_func = lambda x, y, z: Vector3(x=0, y=0, z=-0.95 * self.R_b)
        tail = Fin(
            p0=p0,
            p1=p1,
            p2=p2,
            p3=p3,
            offset_func=offset_func,
            fin_thickness=fin_thickness,
            fin_angle=np.deg2rad(-90),
            top_thickness_function=fin_thickness_function_top,
            bot_thickness_function=fin_thickness_function_bot,
            stl_resolution=self.te_fin_stl_res,
        )
        d21.add_component(tail)

        # Construct trailing wings
        # ------------------------
        # Straight wings at base of vehicle
        # Construct as rectangle fins
        fin_height = 0.45 * self.W_w
        fin_thickness = wing_thickness
        fin_length = 1.3 * (self.L_b - self.L_w)

        te_wing_o = b2
        p0 = te_wing_o
        p1 = p0 + Vector3(x=0, y=fin_height)
        p2 = p1 + Vector3(x=fin_length, y=0)
        p3 = p0 + Vector3(x=fin_length, y=0)

        tew1 = Fin(
            p0=p0,
            p1=p1,
            p2=p2,
            p3=p3,
            fin_thickness=fin_thickness,
            fin_angle=np.deg2rad(0),
            top_thickness_function=fin_thickness_function_top,
            bot_thickness_function=fin_thickness_function_bot,
            stl_resolution=self.te_fin_stl_res,
        )
        d21.add_component(tew1)

        tew2 = Fin(
            p0=p0,
            p1=p1,
            p2=p2,
            p3=p3,
            fin_thickness=fin_thickness,
            fin_angle=np.deg2rad(180),
            top_thickness_function=fin_thickness_function_top,
            bot_thickness_function=fin_thickness_function_bot,
            stl_resolution=self.te_fin_stl_res,
        )
        d21.add_component(tew2)

        return d21


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_generator = ParametricD21()
    d21 = parametric_generator.create_instance()
    d21.generate()
    d21.to_stl("d21")
