import numpy as np
from copy import deepcopy
from hypervehicle import Vehicle
from hypervehicle.generator import Generator
from hypervehicle.components import Wing, common, Fin
from hypervehicle.geometry import Vector3, Bezier, Line, Polyline


class ParametricX43(Generator):
    """Parametric generator for mock-up of the NASA X43-A demonstrator.

    Dimensions have been approximated based on vehicle's visual proportions.

    References
    -----------
    https://en.wikipedia.org/wiki/NASA_X-43
    """

    def __init__(self, **kwargs) -> None:
        # Vehicle parameters
        self.body_length = 3.7  # From body rear to nose
        self.body_width = 1  # Body width
        self.body_angle = 0  # Body slant angle (+ve nose down) (degrees)

        self.wing_span = 0.4  # Span of each wing
        self.wing_length = 1.00  # Wing length
        self.wing_TE_back = 0.25
        self.flap_length = 0.3  # Flap length

        self.engine_h = 0.1  # Height of engine section

        self.fin_height = 0.3  # Height of tail fin
        self.fin_length = 1.0  # Length of fin
        self.rudder_length = 0.2  # Length of tail fin rudder

        # Configuration parameters
        self.flap_angle = 0  # Elevon angle (+ve up) (degrees)
        self.rudder_angle = 0  # Rudder angle (+ve to +y) (degrees)

        # Complete instantiation
        super().__init__(**kwargs)

    def create_instance(self) -> Vehicle:
        # Create Vehicle instance
        x43 = Vehicle()
        x43.configure(name="X-43A", verbosity=1)

        #####################################################
        ##                  WING GEOMETRY                  ##
        #####################################################

        # WING 1
        #     B0------------------------ B1
        #     |                             ---___
        #     |                                    ---__ B2
        #     |                                           |
        #     A0-------------------------A1---------------TT

        # Define nominal parameters
        L_nom = 3.7

        L = self.body_length
        beta = np.deg2rad(self.body_angle)
        LE_height = 0.05 * L / L_nom
        TE_height = 0.2 * L / L_nom

        A0 = Vector3(x=0, y=0)
        A1 = Vector3(x=1.8 * L / L_nom, y=0)
        TT = Vector3(x=L, y=0)
        B0 = Vector3(x=0, y=self.body_width / 2)
        B1 = Vector3(x=A1.x, y=B0.y)
        B2 = Vector3(x=TT.x, y=0.4 * B0.y)

        B0B1 = Line(p0=B0, p1=B1)
        B1B2 = Line(p0=B1, p1=B2)
        B2TT = Line(p0=B2, p1=TT)

        Line_B0TT = Polyline([B0B1, B1B2, B2TT])

        # Top rounding Bezier points
        rounding_thickness = 0.025
        p1 = Vector3(x=0, y=0, z=rounding_thickness)
        p2 = Vector3(x=0, y=0.9 * B0.y, z=p1.z)
        p3 = Vector3(x=0, y=B0.y, z=0)
        rounding_bez = Bezier([p1, p2, p3])

        def get_local_y(x):
            if x < B1.x:
                local_y = B0.y
            else:
                local_y = (x - B2.x) * (B2.y - B1.y) / (B2.x - B1.x) + B2.y
            return local_y

        def wing1_tf_top(x, y, z=0):
            # Nominal thickness
            z_m = -L * np.tan(beta)
            z_u = z_m - 0.5 * TE_height
            beta_u = -np.arctan((z_u + 0.5 * LE_height) / L)
            z_1 = (x - L) * np.tan(beta_u) - LE_height / 2

            # Rounding thickness
            local_y = get_local_y(x)
            z_2 = -(L - x) * rounding_bez(y / local_y).z

            # Sum
            z_val = z_1 + z_2

            return Vector3(x=0, y=0, z=z_val)

        def wing1_tf_bot(x, y, z=0):
            z_m = -L * np.tan(beta)
            z_l = z_m + 0.5 * TE_height
            beta_l = -np.arctan((z_l - 0.5 * LE_height) / L)
            z_val = (x - L) * np.tan(beta_l) + LE_height / 2
            return Vector3(x=0, y=0, z=z_val)

        def leading_edge_width_function(r):
            temp = Bezier(
                [
                    Vector3(x=0.0, y=0.02),
                    Vector3(x=0.75, y=0.05),
                    Vector3(x=1.0, y=0.05),
                ]
            )
            le_width = temp(r).y
            return le_width

        body = Wing(
            A0=A0,
            A1=A1,
            TT=TT,
            B0=B0,
            Line_B0TT=Line_B0TT,
            top_tf=wing1_tf_top,
            bot_tf=wing1_tf_bot,
            LE_wf=leading_edge_width_function,
            stl_resolution=4,
        )

        # WING 2 (flaps)
        # --------------

        #  fB0--__fB1
        #   \      \_
        #    \       \
        #     B0-----fTT----------------- B1
        #     |                             ---___
        #     |                                    ---__ B2
        #     |                                           |
        #     A0-------------------------A1---------------TT

        flap_thickness = 0.03 * L / L_nom

        fA0 = Vector3(x=B0.x + self.flap_length, y=B0.y)
        fA1 = Vector3(
            x=A0.x + 0.5 * self.wing_length,
            y=get_local_y(A0.x + 0.5 * self.wing_length),
        )
        fTT = Vector3(x=A0.x + self.wing_length, y=get_local_y(A0.x + self.wing_length))

        fB0 = Vector3(
            x=A0.x - self.wing_TE_back + self.flap_length, y=B0.y + self.wing_span
        )
        fB1 = Vector3(x=A0.x + 0.4 * self.wing_length, y=B0.y + 0.75 * self.wing_span)

        fB0B1 = Line(p0=fB0, p1=fB1)
        fB1TT = Line(p0=fB1, p1=fTT)

        Line_fB0TT = Polyline([fB0B1, fB1TT])

        def wing2_tf_top(x, y, z=0):
            z_ba = -(x - L) * np.tan(beta)
            return Vector3(x=0, y=0, z=-flap_thickness / 2 - z_ba)

        def wing2_tf_bot(x, y, z=0):
            z_ba = -(x - L) * np.tan(beta)
            return Vector3(x=0, y=0, z=flap_thickness / 2 - z_ba)

        wing = Wing(
            A0=fA0,
            A1=fA1,
            TT=fTT,
            B0=fB0,
            Line_B0TT=Line_fB0TT,
            top_tf=wing2_tf_top,
            bot_tf=wing2_tf_bot,
            flap_length=self.flap_length,
            flap_angle=self.flap_angle,
            LE_wf=leading_edge_width_function,
            close_wing=True,
        )

        # WING 3 (Inlet/exit)
        #     B0------------------------ B1
        #     |                            --__
        #     |                                 --_ B2
        #     |                                       |
        #     A0-------------------------A1-----------TT
        iA0 = A0
        iA1 = A1
        iTT = Vector3(x=0.5 * (TT.x + B1.x), y=0)
        iB0 = Vector3(x=B0.x, y=B0.y)
        iB1 = Vector3(x=B1.x, y=B1.y)
        iB2 = Vector3(
            x=iTT.x, y=((iTT.x - B2.x) * (B2.y - B1.y) / (B2.x - B1.x) + B2.y)
        )

        iB0B1 = Line(p0=iB0, p1=iB1)
        iB1B2 = Line(p0=iB1, p1=iB2)
        iB2TT = Line(p0=iB2, p1=iTT)

        Line_iB0TT = Polyline([iB0B1, iB1B2, iB2TT])

        inlet_start = 0.8 * B1.x
        exit_start = 0.3 * (B1.x - B0.x)

        def inlet_tf_top(x, y, z=0):
            vehicle_z = wing1_tf_bot(x, y, z).z
            return Vector3(x=0, y=0, z=vehicle_z - 0.05)

        def inlet_tf_bot(x, y, z=0):
            # Get z-location of vehicle body
            vehicle_body = wing1_tf_bot(x, y, z).z

            if x < exit_start:
                z_val = (self.engine_h / (exit_start - A0.x)) * x
            elif x > inlet_start:
                z_val = (-self.engine_h / (iTT.x - inlet_start)) * (x - iTT.x)
            else:
                z_val = self.engine_h

            # Calculate taper z
            taper_start_pc = 0.8  # Percentage of y_local where taper begins
            y_local = get_local_y(x)
            if y > taper_start_pc * y_local:
                z_taper = (5 * y / y_local - 4) * z_val
            else:
                z_taper = 0

            return Vector3(x=0, y=0, z=z_val + vehicle_body - z_taper + 0.00001)

        def leading_edge_width_function2(r):
            temp = Bezier(
                [
                    Vector3(x=0.0, y=0.001),
                    Vector3(x=0.5, y=0.001),
                    Vector3(x=1.0, y=0.001),
                ]
            )
            le_width = temp(r).y
            return le_width

        inlet = Wing(
            A0=iA0,
            A1=iA1,
            TT=iTT,
            B0=iB0,
            Line_B0TT=Line_iB0TT,
            top_tf=inlet_tf_top,
            bot_tf=inlet_tf_bot,
            LE_wf=leading_edge_width_function2,
            stl_resolution=4,
        )

        #####################################################
        ##                     FINS                        ##
        #####################################################
        #   |--p1-----p2
        #   |  |         \
        #   |  |           \
        #   |  |             \
        #   |--p0______________p3

        fin_p3x = self.fin_length
        fin_thickness = 0.02 * L / L_nom
        fin_offset = -(fin_p3x - L) * np.tan(beta)  # offset upwards
        y_shift = B0.y

        p0 = Vector3(x=self.rudder_length, y=fin_offset)
        p1 = Vector3(x=self.rudder_length, y=fin_offset + self.fin_height)
        p2 = Vector3(x=0.5 * fin_p3x, y=fin_offset + self.fin_height)
        p3 = Vector3(x=fin_p3x, y=fin_offset)

        def fin_offset_function(x, y, z):
            return Vector3(x=0, y=y_shift, z=0)

        # Define the fin angles: this is the angle to rotate the fin, constructed in
        # the x-y plane, to its final position, by rotating about the x axis
        fin_angle = np.deg2rad(-90)

        # Rudder angle (fin flap)
        rudder_angle = np.deg2rad(self.rudder_angle)
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
            rudder_type="sharp",
            rudder_length=self.rudder_length,
            rudder_angle=rudder_angle,
            pivot_angle=0,
            pivot_point=Vector3(x=0, y=0),
            offset_func=fin_offset_function,
        )

        # Make second fin to account for rudder angle
        def fin_offset_function2(x, y, z):
            return Vector3(x=0, y=-y_shift, z=0)

        fin2 = Fin(
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
            rudder_type="sharp",
            rudder_length=self.rudder_length,
            rudder_angle=rudder_angle,
            pivot_angle=0,
            pivot_point=Vector3(x=0, y=0),
            offset_func=fin_offset_function2,
        )

        # Add all components
        x43.add_component(body, reflection_axis="y", name="body")
        x43.add_component(deepcopy(wing), name="wing1")
        x43.add_component(wing, reflection_axis="y", name="wing2")
        x43.add_component(inlet, reflection_axis="y", name="inlet")
        x43.add_component(fin1, name="fin1")
        x43.add_component(fin2, name="fin2")

        return x43


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_generator = ParametricX43()
    x43 = parametric_generator.create_instance()
    x43.generate()
    x43.to_stl("x43")

    # Assess inertial properties
    # densities = {
    #     "wing_1": 1680,
    #     "wing_2": 5590,
    #     "wing_3": 1680,
    #     "fin_1": 5590,
    #     "fin_2": 5590,
    # }
    # volume, mass, cog, inertia = x43.analyse(densities)
