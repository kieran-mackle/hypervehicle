import numpy as np
from hypervehicle import Vehicle
from hypervehicle.generator import Generator
from hypervehicle.transformations import CART3D
from hypervehicle.components.common import uniform_thickness_function, OgiveNose
from hypervehicle.components import Wing, RevolvedComponent, Fin, SweptComponent
from hypervehicle.geometry import Vector3, Line, Polyline, Arc, CoonsPatch, Bezier
from hypervehicle.geometry.geometry import ReversedPath


def leading_edge_width_function(r):
    temp = Bezier(
        [
            Vector3(x=0.0, y=0.01),
            Vector3(x=0.5, y=0.05),
            Vector3(x=1.0, y=0.01),
        ]
    )
    le_width = temp(r).y
    return le_width


class ParametricReFEX(Generator):
    """Parametric generator for mock-up of the DLR ReFEX.

    Dimensions have been approximated based on vehicle's visual proportions.

    References
    ----------
    https://www.dlr.de/irs/en/desktopdefault.aspx/tabid-15435/
    """

    def __init__(self, **kwargs) -> None:
        # STL Generation Toggles
        self.generate_fuselage = True
        self.generate_wings = True
        self.generate_canards = True
        self.generate_tail = True

        # Inputs
        self.h = 0.225  # Ogive radial height (R)
        self.r_n = 0.075  # Nose radius
        self.r_o = 1.5  # Ogive radius
        self.L_o = 0.05  # Nose-body fairing
        self.L_b = 2.5  # Body length
        self.fin_thickness = 0.02  # Thickness of fin
        self.wing_length = 0.9  # Wing chord property
        self.wing_span = 0.35  # Wing span
        self.tail_height = 0.35  # Tail fin height
        self.tail_thickness = 0.02  # Tail fin thickness
        self.canard_angle = 0  # degrees

        # Below defines axial location of canards, referenced from base of nose cone
        self.cannard_shift = Vector3(x=0.15, y=0)

        # Complete initialisation
        super().__init__(**kwargs)

    def create_instance(self):
        # Create vehicle object
        refex = Vehicle()
        refex.configure(name="ReFEX", verbosity=1)

        # Ogive Nose
        # --------------------------------------
        nose = OgiveNose(
            h=self.h, r_n=self.r_n, r_o=self.r_o, L_o=self.L_o, stl_resolution=50
        )
        if self.generate_fuselage:
            refex.add_component(nose)

        # Vehicle body
        # --------------------------------------
        f1 = Vector3(0, self.h) - Vector3(self.L_o, 0)
        b00 = Vector3(0, 0)
        b0 = Vector3(0, f1.y)
        b1 = b0 - Vector3(self.L_b, 0)
        body_cap_line = Line(b00, b0)
        body_top_line = Line(b0, b1)
        bb0 = b1  # Bottom outside of body
        bb1 = Vector3(bb0.x, 0)  # Body base axis point
        base_line = Line(bb0, bb1)

        body_line = Polyline([body_cap_line, body_top_line, base_line])
        body_fuse = RevolvedComponent(revolve_line=body_line, stl_resolution=50)

        # Add revolved surface
        if self.generate_fuselage:
            refex.add_component(body_fuse)

        # Cannards
        # --------------------------------------
        #   p1-----p2
        #    \        \
        #     \         \
        #      \          \
        #       p0_________p3

        fin_height = 1.3 * self.h
        fin_length = 1.3 * fin_height
        shift_in = Vector3(x=0, y=-0.02 * fin_height)

        p0 = f1 - Vector3(x=fin_length, y=0) + shift_in + self.cannard_shift
        p1 = p0 + Vector3(x=0.0, y=fin_height) + shift_in
        p2 = p1 + Vector3(x=0.3 * fin_length, y=0) + shift_in
        p3 = f1 + shift_in + self.cannard_shift
        pivot_point = Vector3(x=0.5 * (p0.x + p3.x), y=p0.y)

        if self.generate_canards:
            # Add canards
            for i in range(2):
                angle = np.deg2rad((i / 2) * 360)
                fin = Fin(
                    p0=p0,
                    p1=p1,
                    p2=p2,
                    p3=p3,
                    fin_thickness=self.fin_thickness,
                    fin_angle=angle,
                    top_thickness_function=uniform_thickness_function(
                        self.fin_thickness, "top"
                    ),
                    bot_thickness_function=uniform_thickness_function(
                        self.fin_thickness, "bot"
                    ),
                    LE_wf=leading_edge_width_function,
                    pivot_angle=np.deg2rad((-1) ** i * self.canard_angle),
                    pivot_point=pivot_point,
                    rudder_type="sharp",
                    rudder_length=self.fin_thickness,
                    stl_resolution=3,
                )
                refex.add_component(fin)

        # Wings
        # --------------------------------------
        wing_thickness = 3 * self.fin_thickness
        flap_length = 0.1 * self.wing_length

        A0 = Vector3(x=bb0.x + flap_length, y=0)
        TT = A0 + Vector3(x=self.wing_length, y=0)
        A1 = Vector3(x=0.5 * (A0.x + TT.x), y=0)

        B0 = A0 + Vector3(x=0, y=self.wing_span + self.h)
        B1 = B0 + Vector3(x=0.5 * self.wing_length - flap_length, y=0)

        B0B1 = Line(p0=B0, p1=B1)
        B1TT = Line(p0=B1, p1=TT)

        Line_B0TT = Polyline([B0B1, B1TT])

        def wing2_tf_top(x, y, z=0):
            return Vector3(x=0, y=0, z=self.h - wing_thickness)

        def wing2_tf_bot(x, y, z=0):
            return Vector3(x=0, y=0, z=self.h)

        if self.generate_wings:
            wing = Wing(
                A0=A0,
                A1=A1,
                TT=TT,
                B0=B0,
                Line_B0TT=Line_B0TT,
                top_tf=wing2_tf_top,
                bot_tf=wing2_tf_bot,
                LE_wf=leading_edge_width_function,
                flap_length=flap_length,
                stl_resolution=3,
            )
            refex.add_component(wing, reflection_axis="y")

        # Wing-to-body transition
        # --------------------------------------
        if self.generate_fuselage:
            body_transition = self._define_fuselage(x_ref=bb1.x)
            refex.add_component(body_transition)

        # Tail rudder/fin
        # --------------------------------------
        tail_length = 1.20 * self.tail_height
        rudder_length = self.tail_thickness
        shift_in = Vector3(x=0, y=-0.02 * self.tail_height)

        t0 = bb0 + shift_in + Vector3(x=2 * rudder_length, y=0)
        t1 = t0 + Vector3(x=0, y=self.tail_height) + shift_in
        t2 = t1 + Vector3(x=0.4 * tail_length, y=0) + shift_in
        t3 = t0 + Vector3(x=tail_length, y=0) + shift_in

        if self.generate_tail:
            # Add tail fin
            tail = Fin(
                p0=t0,
                p1=t1,
                p2=t2,
                p3=t3,
                fin_thickness=self.tail_thickness,
                fin_angle=np.deg2rad(-90),
                top_thickness_function=uniform_thickness_function(
                    self.tail_thickness, "top"
                ),
                bot_thickness_function=uniform_thickness_function(
                    self.tail_thickness, "bot"
                ),
                LE_wf=leading_edge_width_function,
                rudder_type="sharp",
                rudder_length=rudder_length,
                stl_resolution=3,
            )
            refex.add_component(tail)

        # Apply Cart3D transformation
        refex.add_vehicle_transformations(CART3D)

        return refex

    def _define_fuselage(self, x_ref: float):
        x1 = x_ref
        n1 = Line(
            p0=Vector3(x=x1, y=self.h, z=0.0),
            p1=Vector3(x=x1, y=self.h, z=self.h),
        )
        s1 = Line(
            p0=Vector3(x=x1, y=-self.h, z=0.0),
            p1=Vector3(x=x1, y=-self.h, z=self.h),
        )
        e1 = Line(
            p0=Vector3(x=x1, y=-self.h, z=self.h),
            p1=Vector3(x=x1, y=self.h, z=self.h),
        )
        w1 = Line(
            p0=Vector3(x=x1, y=-self.h, z=0.0),
            p1=Vector3(x=x1, y=self.h, z=0.0),
        )

        cs1 = [n1, ReversedPath(e1), ReversedPath(s1), w1]

        # x2 = -2.38
        x2 = x_ref + 0.9 * self.wing_length
        n2 = Line(
            p0=Vector3(x=x2, y=self.h, z=0.0),
            p1=Vector3(x=x2, y=self.h, z=self.h),
        )
        s2 = Line(
            p0=Vector3(x=x2, y=-self.h, z=0.0),
            p1=Vector3(x=x2, y=-self.h, z=self.h),
        )
        e2 = Line(
            p0=Vector3(x=x2, y=-self.h, z=self.h),
            p1=Vector3(x=x2, y=self.h, z=self.h),
        )
        w2 = Line(
            p0=Vector3(x=x2, y=-self.h, z=0.0),
            p1=Vector3(x=x2, y=self.h, z=0.0),
        )

        cs2 = [n2, ReversedPath(e2), ReversedPath(s2), w2]

        # x3 = -2.
        x3 = x_ref + 1.3 * self.wing_length
        centre = Vector3(x=x3, y=0, z=0)
        arc_s = Arc(
            a=Vector3(x=x3, y=-self.h, z=0),
            b=Vector3(
                x=x3,
                y=-self.h * np.cos(np.deg2rad(60)),
                z=self.h * np.sin(np.deg2rad(60)),
            ),
            c=centre,
        )
        arc_e = Arc(
            a=Vector3(
                x=x3,
                y=-self.h * np.cos(np.deg2rad(60)),
                z=self.h * np.sin(np.deg2rad(60)),
            ),
            b=Vector3(
                x=x3,
                y=self.h * np.cos(np.deg2rad(60)),
                z=self.h * np.sin(np.deg2rad(60)),
            ),
            c=centre,
        )
        arc_n = Arc(
            a=Vector3(x=x3, y=self.h, z=0),
            b=Vector3(
                x=x3,
                y=self.h * np.cos(np.deg2rad(60)),
                z=self.h * np.sin(np.deg2rad(60)),
            ),
            c=centre,
        )
        west = Line(p0=arc_s.a, p1=arc_n.a)

        cs3 = [arc_n, ReversedPath(arc_e), ReversedPath(arc_s), west]

        sections = [cs1, cs2, cs3]
        body_transition = SweptComponent(
            cross_sections=sections,
            stl_resolution=20,
        )
        return body_transition


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_generator = ParametricReFEX()
    refex = parametric_generator.create_instance()
    refex.generate()
    refex.transform(transformations=CART3D)
    refex.to_stl(prefix="refex")
