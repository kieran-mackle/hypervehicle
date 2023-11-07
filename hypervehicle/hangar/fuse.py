import numpy as np
from hypervehicle import Vehicle
from hypervehicle.generator import Generator
from hypervehicle.transformations import CART3D
from hypervehicle.components.common import uniform_thickness_function, OgiveNose
from hypervehicle.components import Wing, RevolvedComponent, Fin, SweptComponent 
from hypervehicle.geometry import Vector3, Line, Polyline, Arc, CoonsPatch, Bezier
import bezier
import matplotlib.pyplot as plt

def leading_edge_width_function(r):
    #temp = Bezier(
    #    [
    #        Vector3(x=0.0, y=0.002),
    #        #Vector3(x=0.5, y=0.02),
    #        Vector3(x=1.0, y=0.002),
    #    ]
    #)
    #le_width = temp(r).y
    le_width = 2e-4
    return le_width

class ParametricMdof(Generator):
    def __init__(self, **kwargs) -> None:
        # STL Generation Toggles
        self.generate_fuselage = True
        self.generate_wings = True
        self.generate_canards = True
        self.generate_tail = False

        self.body_angle = 0                         # deg, Body slant angle (+ve nose down) (degrees)
        self.body_length = 0.19                     # m, Body total length
        self.height = 0.045/2                       # m, Body height (half)
        self.width = 0.052/2                        # m, Body Width
        self.radius = 0.014                         # m, Large body radius
        self.nose_dia = 0.0015                      # m, Front nose diameter
        self.nose_position = 0.1                    # m, Axial position along body where nose transition begins

        self.wing_length = 0.05                     # m, rear wing length
        self.wing_span = 0.025                      # m, rear wing span
        self.wing_thickness = 0.005                 # m, wing max thickness
        self.wing_le = 0.0002                       # m, wing leading edge radius
        self.wing_maxT = 0.5                        # point of maximum wing thickness, pervcent of chord
        self.wing_angle = 0.0                       # wing angle (don't change)

        self.canard_maxT = 0.5                      # point of maximum wing thickness, pervcent of chord
        self.canard_thickness = 0.005              # m, canard thickness
        self.canard_le = 0.0002                     # m, canard leading edge radius
        self.canard_span = 0.025                    # m, canard span
        self.canard_length = 0.025                  # m, canard length
        self.canard_angle = 15.0                     # deg, canard angle - pivots about point defined below
        self.canard_pivot = 0.55                     # canard pivot point, percentage of chord (length)
        self.canard_position = 0.11                 # Below defines axial location of canards, referenced from base of nose cone
        self.L_o = 0.035                            # m, Canard position from tip of nose

        # Complete initialisation
        super().__init__(**kwargs)

    def create_instance(self):
        # Create vehicle object
        mdof = Vehicle()
        mdof.configure(name="mdof", verbosity=1)

        # Vehicle body
        # --------------------------------------
        beta = np.deg2rad(self.body_angle)
        f1 = Vector3(0, self.height) - Vector3(self.L_o, 0)
        # print(f1)
        b00 = Vector3(0, 0)
        b0 = Vector3(0, f1.y)
        b1 = b0 - Vector3(self.body_length, 0)
        # body_cap_line = Line(b00, b0)
        # body_top_line = Line(b0, b1)
        bb0 = b1  # Bottom outside of body
        bb1 = Vector3(bb0.x, 0)  # Body base axis point
        base_line = Line(bb0, bb1)
        print('b00', b00)
        print('bb0', bb0)
        print('bb1', bb1)
        print('base line',base_line)


        def makePatch(x1, height, radius):
            width = 1.1555555555 * height
            # north
            p0=Vector3(x=x1, y=0, z=height)
            p1=Vector3(x=x1, y=(width-radius), z=height)
            p2=Vector3(x=x1, y=width, z=height-radius)
            p3=Vector3(x=x1, y=width, z=0) 
            centre = Vector3(x=x1, y=width-radius, z=height-radius)
            line1 = Line(p0,p1)
            arc = Arc(p1,p2,centre)
            line2 = Line(p2,p3)
            n1 = Polyline(segments=(line1, arc, line2))

            # south
            p0s=Vector3(x=x1, y=-width, z=0)
            p1s=Vector3(x=x1, y=-width, z=-(height-radius))
            p2s=Vector3(x=x1, y=-(width-radius), z=-height)
            p3s=Vector3(x=x1, y=0, z=-height) 
            centres = Vector3(x=x1, y=-(width-radius), z=-(height-radius))
            line1s = Line(p0s,p1s)
            arcs = Arc(p1s,p2s,centres)
            line2s = Line(p2s,p3s)
            s1 = Polyline(segments=(line1s, arcs, line2s))

            # east
            p0=Vector3(x=x1, y=0, z=-height)
            p1=Vector3(x=x1, y=width-radius, z=-height)
            p2=Vector3(x=x1, y=width, z=-(height-radius))
            p3=Vector3(x=x1, y=width, z=0) 
            centre = Vector3(x=x1, y=(width-radius), z=-(height-radius))
            line1 = Line(p0,p1)
            arc = Arc(p1,p2,centre)
            line2 = Line(p2,p3)
            e1 = Polyline(segments=(line1, arc, line2))

            p0=Vector3(x=x1, y=-width, z=0)
            p1=Vector3(x=x1, y=-width, z=height-radius)
            p2=Vector3(x=x1, y=-(width-radius), z=height)
            p3=Vector3(x=x1, y=0, z=height) 
            centre = Vector3(x=x1, y=-(width-radius), z=height-radius)
            line1 = Line(p0,p1)
            arc = Arc(p1,p2,centre)
            line2 = Line(p2,p3)
            w1 = Polyline(segments=(line1, arc, line2))
            c1 = CoonsPatch(north=n1, south=s1, east=e1, west=w1)

            return c1

        # exterior profile - straight line portion
        # bezier points
        bez1 = Vector3(x=0.14, y=0.0, z=-self.height)
        bez2 = Vector3(0.161, y=0.0, z=-0.020)
        bez3 = Vector3(0.178, y=0.0, z=-0.0182)
        bez4 = Vector3(self.body_length, y=0.0, z=-self.nose_dia/2)

        p0=Vector3(x=bb1.x,             y=0,    z=-self.height)
        p1=Vector3(x=bb1.x + bez1.x,    y=0,    z=bez1.z)
        p2=Vector3(x=bb1.x + bez2.x,    y=0,    z=bez2.z)
        p3=Vector3(x=bb1.x + bez3.x,    y=0,    z=bez3.z)
        p4=Vector3(x=bb1.x + bez4.x,    y=0,    z=bez4.z)
        # line = Line(p0,p1)
        # nose = Bezier([p1, p2, p3, p4])
        # ext_profile = Polyline(segments=(line,nose))



        # bezier curve
        nodes = np.array([[bez1.x, bez2.x, bez3.x, bez4.x], [bez1.z, bez2.z, bez3.z, bez4.z]])     # x values, z values of bezier points
        curve = bezier.Curve(nodes, degree=3)
        
        s_vals = np.linspace(0, 1, 10)            # define number of bezier curve evalualtions
        xx, zz = curve.evaluate_multi(s_vals)       # evaluate bezier at above points

        c_b = []       
        c1 = makePatch(x1 = bb1.x ,         height = self.height,       radius = self.radius)   # make patch for rear section with constant cross section
        c2 = makePatch(x1 = bb1.x + self.nose_position,    height = self.height,       radius = self.radius)   

        xpos = []
        rads = []

        # make patch for front section along bezier curve
        for x, z in zip(xx, zz):
            corner_radius = ( self.radius - (x - (self.body_length - self.nose_position))*10*(self.radius-self.nose_dia/2) )
            xpos.append(x)
            rads.append(corner_radius)
            c_b.append(makePatch(x1 = bb1.x + x,  height = -z,   radius = corner_radius-0.00001 ))
            
            # print(f"x={x}; x1={bb1.x + x}; height={-z}; radius={corner_radius}")

        # plt.plot(xpos, rads)

        # plot points and bezier evaluations
        plt.scatter(xx-self.body_length, zz, label='evals')
        plt.scatter(p0.x, p0.z, label='bezier p0')
        plt.scatter(p1.x, p1.z, label='bezier p1')
        plt.scatter(p2.x, p2.z, label='bezier p2')
        plt.scatter(p3.x, p3.x, label='bezier p3')
        plt.scatter(p4.x, p4.x, label='bezier p4')
        plt.grid()
        plt.legend()

        # plt.show()

        sections = [c1, c2]
        for c in c_b:
            sections.append(c)
        #print(sections)

        # print(sections)
        if self.generate_fuselage:
            body_transition = SweptComponent(
                cross_sections=sections,
                sweep_axis="x",
                stl_resolution=30,
            )
            mdof.add_component(body_transition)

        # Canards
        # --------------------------------------
        #   p1-----p2
        #    \        \
        #     \         \
        #      \          \
        #       p0_________p3

        shift_in = Vector3(x=0, y=-0.02 * self.canard_thickness)

        p0 = bb0 + Vector3(x=self.canard_position, y=0.0) + shift_in
        p1 = p0 + Vector3(x=0.0, y=self.canard_span) + shift_in
        p2 = p1 + Vector3(x=self.canard_length, y=0) + shift_in
        p3 = p0 + Vector3(x=1.5*self.canard_length, y=0) + shift_in
        pivot_point = Vector3(x=self.canard_pivot * (p0.x + p3.x), y=p0.y)

        print(f"p0={p0}, p1={p1}, p2={p2}, p3={p3}")
        # print('p0:',p0, 'p1:', p1, 'p2:',p2, 'p3:',p3)        
        # print('pivot:', pivot_point)

        can_slope_rear_in = (self.canard_thickness/2 - self.canard_le) / (abs(pivot_point.x - p0.x))
        can_slope_rear_out = (self.canard_thickness/2 - self.canard_le) / (abs(pivot_point.x - p1.x))        
        can_slope_front_in = (self.canard_thickness/2 - self.canard_le) / (abs(pivot_point.x - p3.x))
        can_slope_front_out = (self.canard_thickness/2 - self.canard_le) / (abs(pivot_point.x - p2.x))
        print(f"can_slope_front_in={can_slope_front_in}; can_slope_rear_out={can_slope_rear_out}; can_slope_rear_in={can_slope_rear_in}; can_slope_front_out={can_slope_front_out}")


        def canard_thickness_function(thickness: float, side: str):
            """Returns a function handle."""
            m = 1 if side == "top" else -1

            def tf(x: float, y: float, z: float = 0):
                x_rel = x - pivot_point.x
                y_factor = (y-p0.y)/self.canard_span
                if x_rel <= 0:
                    canard_slope = can_slope_rear_in*(1-y_factor) + can_slope_rear_out*y_factor
                    z_local = m * (thickness/2 - canard_slope * abs(x_rel))
                else:
                    canard_slope = can_slope_front_in*(1-y_factor) + can_slope_front_out*y_factor
                    z_local = m * (thickness/2 - canard_slope * abs(x_rel))

                if z_local > self.canard_maxT/2:
                    z_local = self.canard_maxT/2
                elif z_local < -self.canard_maxT/2:
                    z_local = -self.canard_maxT/2 

                return Vector3(x=0.0, y=0.0, z=z_local)

            return tf



        if self.generate_canards:
            # Add canards
            for i in range(2):
                angle = np.deg2rad((i / 2) * 360)
                fin = Fin(
                    p0=p0,
                    p1=p1,
                    p2=p2,
                    p3=p3,
                    fin_thickness=self.canard_thickness,
                    fin_angle=angle,
                    top_thickness_function=canard_thickness_function(
                        self.canard_thickness, "top"
                    ),
                    bot_thickness_function=canard_thickness_function(
                        self.canard_thickness, "bot"
                    ),
                    LE_wf=leading_edge_width_function,
                    pivot_angle=np.deg2rad((-1) ** i * self.canard_angle),
                    pivot_point=pivot_point,
                    # rudder_type="sharp",
                    # rudder_length=self.fin_thickness,
                    stl_resolution=10,
                )
                mdof.add_component(fin)

        ###### WINGS using fin geom type
        # --------------------------------------
        #   p1-----p2
        #    \        \
        #     \         \
        #      \          \
        #       p0_________p3

        p0 = bb0 
        p1 = p0 + Vector3(x=0.0, y=self.wing_span) 
        p2 = p1 + Vector3(x=self.wing_length, y=0) 
        p3 = p0 + Vector3(x=1.2*self.wing_length, y=0)
        wing_max_thickness = Vector3(x=self.wing_maxT * (p0.x + p3.x), y=p0.y)
        # print('p0:',p0, 'p1:', p1, 'p2:',p2, 'p3:',p3)        
        # print('wing max t', wing_max_thickness)
        # print('pivot:', pivot_point)

        wing_slope_rear_in = (self.wing_thickness/2 - self.wing_le) / (abs(wing_max_thickness.x-p0.x))
        wing_slope_rear_out = (self.wing_thickness/2 - self.wing_le) / (abs(wing_max_thickness.x-p1.x))        
        wing_slope_front_in = (self.wing_thickness/2 - self.wing_le) / (abs(wing_max_thickness.x-p3.x))
        wing_slope_front_out = (self.wing_thickness/2 - self.wing_le) / (abs(wing_max_thickness.x-p2.x))


        def wing_thickness_function(thickness: float, side: str):
            """Returns a function handle."""
            m = 1 if side == "top" else -1
            
            def tf(x: float, y: float, z: float = 0):
                x_rel = x - wing_max_thickness.x
                # print(x_rel)
                y_factor = (y - p0.y) / self.wing_span
                if x_rel <= 0:
                    wing_slope = wing_slope_rear_in*(1-y_factor) + wing_slope_rear_out * y_factor
                    z_local = m * (thickness/2 - wing_slope * abs(x_rel))
                else:
                    wing_slope = wing_slope_front_in*(1-y_factor) + wing_slope_front_out * y_factor
                    z_local = m * (thickness/2 - wing_slope * abs(x_rel))

                # if z_local > 2.e-3:
                #     z_local = 2.e-3
                # elif z_local < -2.e-3:
                #     z_local = -2.e-3 

                return Vector3(x=0.0, y=0.0, z=z_local)

            return tf



        if self.generate_wings:
            # Add wings using fin type
            for i in range(2):
                angle = np.deg2rad((i / 2) * 360)
                wing = Fin(
                    p0=p0,
                    p1=p1,
                    p2=p2,
                    p3=p3,
                    fin_thickness = self.wing_thickness,
                    fin_angle = angle,
                    top_thickness_function=wing_thickness_function(
                        self.wing_thickness, "top",
                    ),
                    bot_thickness_function=wing_thickness_function(
                        self.wing_thickness, "bot",
                    ),
                    LE_wf = leading_edge_width_function,
                    pivot_angle = np.deg2rad((-1) ** i * self.wing_angle),
                    pivot_point = wing_max_thickness,
                    # rudder_type="sharp",
                    # rudder_length=self.fin_thickness,
                    stl_resolution=10,
                )

                mdof.add_component(wing)


        return mdof


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_generator = ParametricMdof()
    mdof = parametric_generator.create_instance()
    mdof.generate()
    mdof.transform(transformations=CART3D)
    mdof.to_stl("parts/fuse-c15")

    plt.show()

    # # To run sensitivity study
    # from hypervehicle.utilities import SensitivityStudy

    # # Construct sensitivity study
    # ss = SensitivityStudy(ParametricMdof)

    # # Define parameters to get sensitivities to
    # parameters = {'wing_length': 0.6}

    # # Perform study
    # sensitivities = ss.dvdp(parameters)

    # # Save to CSV
    # ss.to_csv()
