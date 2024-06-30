import numpy as np
from hypervehicle import Vehicle
from hypervehicle.generator import Generator
from hypervehicle.geometry import Vector3, Line, Polyline, Bezier
from hypervehicle.components import (
    Wing,
    Fin,
    RevolvedComponent,
    CompositeComponent,
    common,
)


def leading_edge_width_function(r):
    temp = Bezier(
        [
            Vector3(x=0.0, y=0.01),
            Vector3(x=0.5, y=0.1),
            Vector3(x=1.0, y=0.01),
        ]
    )
    le_width = temp(r).y
    return le_width


class ParametricHIFiRE8(Generator):
    """Parametric generator for mock-up of the HIFiRE 8.

    Dimensions have been approximated based on vehicle's visual proportions.
    """

    def __init__(self, **kwargs) -> None:
        # Vehicle parameters
        self.L = 1.2
        self.R_base = 0.175
        self.R_nose = 0.1
        self.nozzle_length = 0.1

        # Complete instantiation
        super().__init__(**kwargs)

    def create_instance(self) -> Vehicle:
        # Instantiate hypervehicle and add all components
        hifire8 = Vehicle()
        hifire8.configure(
            name="HIFiRE-8",
            verbosity=1,
        )

        # Construct fuselage
        # ====================

        fuse_slope = np.arctan((self.R_base - self.R_nose) / self.L)

        Xn = self.L
        X1 = 1 * self.L
        X2 = 0.0
        X3 = -self.nozzle_length  # Nozzle length

        Rn = self.R_nose
        R1 = self.R_base - X1 * np.tan(fuse_slope)
        R2 = self.R_base - X2 * np.tan(fuse_slope)
        R3 = R2  # R_base - X3*np.tan(fuse_slope)

        fuse_line = Polyline(
            [
                Line(p0=Vector3(X1, R1), p1=Vector3(X2, R2)),
                Line(p0=Vector3(X2, R2), p1=Vector3(X3, R3)),
                Line(p0=Vector3(X3, R3), p1=Vector3(X3, 0)),
            ]
        )
        fuselage_0 = RevolvedComponent(fuse_line)

        fuse_base_line = Line(p0=Vector3(X1, 0), p1=Vector3(X1, R1))
        fuselage_1 = RevolvedComponent(fuse_base_line)

        # Create composite component for fuselage
        fuselage = CompositeComponent(stl_resolution=8)
        fuselage.add_component(fuselage_0)
        fuselage.add_component(fuselage_1)

        # Construct fins
        # ====================
        #   |--p1-----p2
        #   |  |         \
        #   |  |           \
        #   |  |             \
        #   |--p0______________p3

        fin_length = 0.4  # length from p0 to p3
        fin_height = 0.4 * fin_length
        fin_thickness = 0.02 * fin_length
        fin_angle = np.deg2rad(90 - 45)

        def eval_r_at_x(x):
            """Returns the R value at a given x."""
            return self.R_base - x * np.tan(fuse_slope)

        p0 = Vector3(x=0, y=eval_r_at_x(0))
        p1 = Vector3(x=0.2 * fin_length, y=eval_r_at_x(0.2 * fin_length) + fin_height)
        p2 = Vector3(x=0.7 * fin_length, y=eval_r_at_x(0.7 * fin_length) + fin_height)
        p3 = Vector3(x=fin_length, y=eval_r_at_x(fin_length))

        fin1 = Fin(
            p0=p0,
            p1=p1,
            p2=p2,
            p3=p3,
            fin_thickness=fin_thickness,
            fin_angle=fin_angle,
            top_thickness_function=common.uniform_thickness_function(
                fin_thickness, "top"
            ),
            bot_thickness_function=common.uniform_thickness_function(
                fin_thickness, "bot"
            ),
            LE_wf=leading_edge_width_function,
            mirror=False,
            stl_resolution=3,
        )

        # Copy for second fin
        fin2 = Fin(
            p0=p0,
            p1=p1,
            p2=p2,
            p3=p3,
            fin_thickness=fin_thickness,
            fin_angle=fin_angle + np.deg2rad(90),
            top_thickness_function=common.uniform_thickness_function(
                fin_thickness, "top"
            ),
            bot_thickness_function=common.uniform_thickness_function(
                fin_thickness, "bot"
            ),
            LE_wf=leading_edge_width_function,
            mirror=False,
            stl_resolution=3,
        )

        # Copy for third fin - this should be improved. Maybe
        # make a fin geom func and modify fin height
        fin3 = Fin(
            p0=p0,
            p1=0.5 * p1,
            p2=0.5 * p2,
            p3=p3,
            fin_thickness=fin_thickness,
            fin_angle=np.deg2rad(-90),
            top_thickness_function=common.uniform_thickness_function(
                fin_thickness, "top"
            ),
            bot_thickness_function=common.uniform_thickness_function(
                fin_thickness, "bot"
            ),
            LE_wf=leading_edge_width_function,
            mirror=False,
            stl_resolution=3,
        )

        # Construct wing
        # ====================

        # |---B0-------B1__
        # |   |            ----____
        # |   |                    ----_____
        # |   |                              B2-----------____B3
        # |---A0-----------------------------A1---------------TT
        #
        #     |------------------- wing_L ---------------------|

        wing_L = 1.3
        wing_thickness = 0.04
        wing_width = 0.4
        wing_origin = 0.2
        flap_length = 0.2

        # Required points
        A0 = Vector3(x=wing_origin, y=0)
        A1 = Vector3(x=A0.x + 0.5 * wing_L, y=0)
        TT = Vector3(x=A0.x + wing_L, y=0)
        B0 = Vector3(x=A0.x, y=wing_width)

        # Construction points
        B1 = Vector3(x=A0.x + 0.1 * wing_L, y=B0.y)
        B2 = Vector3(x=A1.x, y=0.5 * wing_width)
        B3 = Vector3(x=TT.x, y=0.13 * wing_width)

        # Leading edge line
        B0B1 = Line(p0=B0, p1=B1)
        B1B2 = Line(p0=B1, p1=B2)
        B2B3 = Line(p0=B2, p1=B3)
        B3TT = Line(p0=B3, p1=TT)
        Line_B0TT = Polyline([B0B1, B1B2, B2B3, B3TT])

        wing = Wing(
            A0=A0,
            A1=A1,
            TT=TT,
            B0=B0,
            Line_B0TT=Line_B0TT,
            top_tf=common.uniform_thickness_function(wing_thickness, "top"),
            bot_tf=common.uniform_thickness_function(wing_thickness, "bot"),
            LE_wf=leading_edge_width_function,
            flap_length=flap_length,
            stl_resolution=4,
        )

        # Construct nose cap
        # ====================
        L_n = 0.5
        r_b = eval_r_at_x(Xn)
        t_n = 0.005
        w_n = 0.15

        # Side width angle
        side_theta = np.arctan((r_b - 0.5 * w_n) / L_n)

        # Points
        A0n = Vector3(x=Xn, y=0)
        A1n = Vector3(x=Xn + 0.5 * L_n, y=0)
        TTn = Vector3(x=Xn + L_n, y=0)

        B0n = Vector3(x=A0n.x, y=r_b)
        B1n = Vector3(x=A1n.x, y=r_b - (A1n.x - Xn) * np.tan(side_theta))
        B2n = Vector3(x=TTn.x, y=0.5 * w_n)

        B0B1n = Line(p0=B0n, p1=B1n)
        B1B2n = Line(p0=B1n, p1=B2n)
        B2TTn = Line(p0=B2n, p1=TTn)

        Line_B0TTn = Polyline([B0B1n, B1B2n, B2TTn])

        def nose_tf_top(x, y, z=0):
            # Calculate function weightings for linear blending
            circular_weighting = 1 - (1 / L_n) * (x - Xn)
            flat_weighting = 1 - circular_weighting

            # Calculate functions at x
            z_circle = np.sqrt(r_b**2 - y**2)
            z_flat = t_n
            # z_flat = t_n*np.sqrt(w_n - y**2)

            # Weighted sum of function contributions
            z = circular_weighting * z_circle + flat_weighting * z_flat
            # z = t_n
            return Vector3(x=0, y=0, z=-z)

        def nose_tf_bot(x, y, z=0):
            # Calculate function weightings for linear blending
            circular_weighting = 1 - (1 / L_n) * (x - Xn)
            flat_weighting = 1 - circular_weighting

            # Calculate functions at x
            z_circle = np.sqrt(r_b**2 - y**2)
            z_flat = t_n
            # z_flat = np.sqrt(w_n - y**2)

            # Weighted sum of function contributions
            z = circular_weighting * z_circle + flat_weighting * z_flat
            # z = t_n
            return Vector3(x=0, y=0, z=z)

        nose = Wing(
            A0=A0n,
            A1=A1n,
            TT=TTn,
            B0=B0n,
            Line_B0TT=Line_B0TTn,
            top_tf=nose_tf_top,
            bot_tf=nose_tf_bot,
            LE_wf=leading_edge_width_function,
            LE_type="FLAT",
            stl_resolution=4,
        )

        # Add components
        hifire8.add_component(fuselage, name="fuselage")
        hifire8.add_component(wing, reflection_axis="y", name="wing")
        hifire8.add_component(nose, reflection_axis="y", name="nose")
        hifire8.add_component(fin1, name="fin1")
        hifire8.add_component(fin2, name="fin2")
        hifire8.add_component(fin3, name="fin3")

        return hifire8


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_generator = ParametricHIFiRE8()
    hifire8 = parametric_generator.create_instance()
    hifire8.generate()
    hifire8.to_stl()
