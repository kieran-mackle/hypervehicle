import numpy as np
from copy import deepcopy
from hypervehicle import Vehicle
from scipy.optimize import bisect
from hypervehicle.components import Wing
from hypervehicle.generator import Generator
from hypervehicle.geometry import Vector3, Line, Polyline


class ParametricHTV(Generator):
    """Parametric generator for mock-up of the Hypersonic Technology Vehicle 2.

    Dimensions have been approximated based on vehicle's visual proportions.

    References
    ----------
    https://en.wikipedia.org/wiki/Hypersonic_Technology_Vehicle_2
    """

    def __init__(self, **kwargs) -> None:
        # Vehicle parameters
        self.L = 1
        self.body_width = 0.5
        self.h = 0.1
        self.t_LE = 0.01
        self.flap_angle = 15

        # Nominal parameters for proportional scaling
        self.L_nom = 1
        self.body_width_nom = 0.8
        self.h_nom = 0.2
        self.t_LE_nom = 0.025

        # Complete instantiation
        super().__init__(**kwargs)

    def create_instance(self) -> Vehicle:
        # Create Vehicle instance
        htv = Vehicle()
        htv.configure(
            name="Hypersonic Technology Vehicle",
            verbosity=1,
        )

        # Create main body
        # ================
        A0 = Vector3(x=0, y=0)
        A1 = Vector3(x=0.15 * self.L, y=0)
        TT = Vector3(x=self.L, y=0)
        B0 = Vector3(x=0, y=self.body_width / 2)

        B1 = Vector3(x=A1.x, y=B0.y)
        B0B1 = Line(p0=B0, p1=B1)
        B1TT = Line(p0=B1, p1=TT)
        Line_B0TT = Polyline([B0B1, B1TT])

        def get_local_width(x):
            """Returns the vehicle width at a given x."""
            func = lambda t: Line_B0TT(t).x - x
            t = bisect(func, 0.0, 1.0)
            return 2 * Line_B0TT(t).y

        def wing1_tf_top(x, y, z=0):
            """Thickness function to create the top surface shape
            of the vehicle."""
            local_width = get_local_width(x)

            if y < 0.5 * (local_width - 2 * self.t_LE):
                z_val = 0.5 * (self.h - self.t_LE) * np.cos(
                    2 * np.pi * y / (local_width - 2 * self.t_LE)
                ) + 0.5 * (self.h + self.t_LE)
            elif y == 0.5 * (local_width - 2 * self.t_LE):
                z_val = self.t_LE
            else:
                z_val = self.t_LE

            # Apply axial tapering
            z_val *= (self.L - x) + 0.05

            return Vector3(x=0, y=0, z=-z_val)

        def wing1_tf_bot(x, y, z=0):
            """Bottom thickness function."""
            z_val = 0.2 * self.t_LE
            return Vector3(x=0, y=0, z=z_val)

        def lewf(r):
            """Constant leading edge width function."""
            le_width = 0.01
            return le_width

        # Add wing
        wing = Wing(
            A0=A0,
            A1=A1,
            TT=TT,
            B0=B0,
            Line_B0TT=Line_B0TT,
            top_tf=wing1_tf_top,
            bot_tf=wing1_tf_bot,
            LE_wf=lewf,
            stl_resolution=5,
        )
        htv.add_component(wing, reflection_axis="y")

        # Create flaps
        # ================
        length_scaler = self.L / self.L_nom
        width_scaler = self.body_width / self.body_width_nom

        flap_length = 0.2 * length_scaler
        flap_gap = 0.01 * width_scaler

        offset = 0.1 * self.t_LE

        A0f = Vector3(x=flap_length, y=0 + 0.5 * flap_gap)
        A1f = Vector3(x=flap_length + 0.05 * self.L, y=0 + 0.5 * flap_gap)
        TTf = Vector3(x=flap_length + 0.1 * self.L, y=0 + 0.5 * flap_gap)
        B0f = Vector3(x=flap_length, y=0.8 * self.body_width / 2 + 0.5 * flap_gap)

        B1f = Vector3(x=A1f.x, y=0.5 * B0f.y)
        B0B1f = Line(p0=B0f, p1=B1f)
        B1TTf = Line(p0=B1f, p1=TTf)
        Line_B0TTf = Polyline([B0B1f, B1TTf])

        def wing2_tf_top(x, y, z=0):
            return Vector3(x=0, y=0, z=-self.t_LE - offset)

        def wing2_tf_bot(x, y, z=0):
            return Vector3(x=0, y=0, z=0.2 * self.t_LE - offset)

        flap_wing = Wing(
            A0=A0f,
            A1=A1f,
            TT=TTf,
            B0=B0f,
            Line_B0TT=Line_B0TTf,
            top_tf=wing2_tf_top,
            bot_tf=wing2_tf_bot,
            flap_length=flap_length,
            flap_angle=np.deg2rad(self.flap_angle),
            stl_resolution=5,
        )
        htv.add_component(deepcopy(flap_wing))
        htv.add_component(flap_wing, reflection_axis="y")

        return htv


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_generator = ParametricHTV()
    htv = parametric_generator.create_instance()
    htv.generate()
    htv.to_stl(prefix="htv")
