import numpy as np
from hypervehicle import Vehicle
from hypervehicle.generator import Generator
from hypervehicle.transformations import CART3D
from hypervehicle.components.common import uniform_thickness_function, OgiveNose
from hypervehicle.components import Wing, RevolvedComponent, Fin, SweptComponent, Canard
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
        self.generate_fuselage = False #True
        self.generate_wings = True
        self.generate_canards = True

        self.body_angle = 0                         # deg, Body slant angle (+ve nose down) (degrees)
        self.body_length = 0.20                     # m, Body total length
        self.height = 0.045/2                       # m, Body height (half)
        self.width = 0.052/2                        # m, Body Width
        self.radius = 0.003                         # m, Large body radius
        self.nose_dia = 0.0015                      # m, Front nose diameter
        # self.nose_position = 0.1                    # m, Axial position along body where nose transition begins

        # wing parameters
        self.wing_in_length = 0.085                 # m, wing inside length
        self.wing_out_length = 0.04                 # m, wing outside length
        self.wing_span = 0.035                      # m, rear wing span
        self.wing_le = 0.0001                       # m, wing leading edge radius
        self.wing_thickness = 0.002                 # m, wing max thickness
        self.wing_angle = 0.0                       # deg, wing angle (don't change)
        self.wing_chamfer = 10                      # deg, wing chamfer angle 

        # canard parameters
        self.canard_thickness = 0.0025              # m, half of canard thickness
        self.canard_le = 0.0001                     # m, canard leading edge radius
        self.canard_span = 0.03                    # m, canard span
        self.canard_out_length = 0.024                  # m, canard outer length
        self.canard_in_length = 0.04               # m, canard inner length
        self.canard_angle = 15                     # deg, canard angle - pivots about point defined below
        self.canard_pivot = 0.022                     # canard pivot point, distance from trailing edge
        self.canard_position = 0.14                 # Below defines axial location of canards, referenced from base of nose cone
        self.L_o = 0.04                            # m, Canard position from tip of nose
        self.canard_chamfer = 20                    # deg, canard chamfer angle 
        self.Xsetback = 0.000

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
            # print(x1)
            width = ((0.0275-0.020516)/-0.2  * x1) + 0.020516
            #print(width)
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
        # bez0 = Vector3(x=0., y=0.0, z=-self.height)
        bez1 = Vector3(x=0.098457, y=0.0, z=-0.02141)
        # bez2 = Vector3(x=0.142895, y=0.0, z=-0.015999)
        bez2 = Vector3(x=0.142895, y=0.0, z=-0.02009)
        bez3 = Vector3(x=0.174117, y=0.0, z=-0.018895)
        bez4 = Vector3(x=self.body_length, y=0.0, z=-0.001)

        p0=Vector3(x=bb1.x,             y=0,    z=-self.height)
        p1=Vector3(x=bb1.x + bez1.x,    y=0,    z=bez1.z)
        p2=Vector3(x=bb1.x + bez2.x,    y=0,    z=bez2.z)
        p3=Vector3(x=bb1.x + bez3.x,    y=0,    z=bez3.z)
        p4=Vector3(x=bb1.x + bez4.x,    y=0,    z=bez4.z)
        # p5=Vector3(x=bb1.x + bez5.x,    y=0,    z=bez5.z)

        # line = Line(p0,p1)
        # nose = Bezier([p1, p2, p3, p4])
        # ext_profile = Polyline(segments=(line,nose))

        # bezier curve
        nodes = np.array([[bez1.x, bez2.x, bez3.x, bez4.x], [bez1.z, bez2.z, bez3.z, bez4.z]])     # x values, z values of bezier points
        curve = bezier.Curve(nodes, degree=3)
        
        s_vals = np.linspace(0, 1, 30)            # define number of bezier curve evalualtions
        xx, zz = curve.evaluate_multi(s_vals)       # evaluate bezier at above points

        c_b = []       
        c1 = makePatch(x1 = bb1.x ,         height = self.height,       radius = self.radius)   # make patch for rear section with constant cross section
        # c1 = makePatch(x1 = bb1.x,    height = self.height,       radius = self.radius)   

        xpos = []
        rads = []

        # make patch for front section along bezier curve
        for x, z in zip(xx, zz):
            # corner_radius = ( self.radius - (x - (self.body_length - self.nose_position))*5*(self.radius-self.nose_dia/2) )
            corner_radius = 0.0015
            xpos.append(x)
            rads.append(corner_radius)
            c_b.append(makePatch(x1 = bb1.x + x,  height = -z,   radius = corner_radius ))
            print(f"x={x}; x1={bb1.x + x}; height={-z}; radius={corner_radius}")

        # plt.plot(xpos, rads)

        # plot points and bezier evaluations
        plt.scatter(xx-self.body_length, zz, label='evals')
        plt.scatter(p0.x, p0.z, label='bezier p0')
        plt.scatter(p1.x, p1.z, label='bezier p1')
        plt.scatter(p2.x, p2.z, label='bezier p2')
        plt.scatter(p3.x, p3.z, label='bezier p3')
        plt.scatter(p4.x, p4.z, label='bezier p4')
        # plt.scatter(p5.x, p5.z, label='bezier p5')
        plt.grid()
        plt.legend()

        # plt.show()

        sections = [c1]
        for c in c_b:
            sections.append(c)
        #print(sections)

        # print(sections)
        if self.generate_fuselage:
            body_transition = SweptComponent(
                cross_sections=sections,
                sweep_axis="x",
                stl_resolution=80,
            )
            mdof.add_component(body_transition)



        # Canards
        # --------------------------------------
        #          p1--p4------p5--p2
        #         /    /         \   \    ---->
        #   back  /    /          \    \    front
        #         /    /           \     \    --->
        #        p0____p7___o______p6_____p3
        #                 pivot
        #   
        # z = thickness

        shift_in = Vector3(x=0, y=-0.02 * self.canard_thickness)

        p0 = bb0 + Vector3(x=self.canard_position, y=0.0)                       #intersection of lines forming trailing edge
        p1 = p0 + Vector3(x=self.Xsetback, y=self.canard_span, z=0.0)          
        p2 = p1 + Vector3(x=self.canard_out_length, y=0.0, z=0.0) 
        p3 = p0 + Vector3(x=self.canard_in_length, y=0.0, z=0.0)                #intersection of lines forming leading edge
        pp = p0 + Vector3(x=self.canard_pivot, y=0.0, z=0.0) 

        
        if self.generate_canards:
            for i in range(2):
                angle = np.deg2rad((i / 2) * 360)
                canard = Canard(
                    p0=p0,
                    p1=p1,
                    p2=p2,
                    p3=p3,
                    theta1=np.deg2rad(self.canard_chamfer),
                    theta2=np.deg2rad(self.canard_chamfer),
                    canard_thickness = self.canard_thickness,
                    canard_in_length = self.canard_in_length,
                    canard_out_length = self.canard_out_length,
                    canard_le = self.canard_le,
                    pivot_angle = np.deg2rad((-1) ** i * self.canard_angle),
                    pivot_point=pp,                     # point on the canard about which it deflects
                    fin_angle = angle,                  # rotates the second canard to the other side of the body
                    body_angle = np.deg2rad(-2),        # aligns canards with body of fuselage
                    stl_resolution=20,
                    symmetric_canard_def = False,
                )
                mdof.add_component(canard)

        # WINGS
        # --------------------------------------
        #          p1--p4------p5--p2
        #         /    /         \   \    ---->
        #   back  /    /          \    \    front
        #         /    /           \     \    --->
        #        p0____p7___o______p6_____p3
        #                 pivot
        #   
        # z = thickness         

        p0 = bb0 
        p1 = p0 + Vector3(x=0.0, y=self.wing_span, z=0.0) 
        p2 = p1 + Vector3(x=self.wing_out_length, y=0.0, z=0.0) 
        p3 = p0 + Vector3(x=self.wing_in_length, y=0.0, z=0.0)
        
        if self.generate_wings:
            for i in range(1):
                angle = np.deg2rad((i / 2) * 360)
                wing = Canard(
                    p0=p0,
                    p1=p1,
                    p2=p2,
                    p3=p3,
                    theta1=np.deg2rad(self.wing_chamfer),
                    theta2=np.deg2rad(self.wing_chamfer),
                    canard_thickness = self.wing_thickness,
                    canard_in_length = self.wing_in_length,
                    canard_out_length = self.wing_out_length,
                    canard_le = self.wing_le,
                    pivot_angle = np.deg2rad((-1) ** i * self.wing_angle),
                    fin_angle = angle,
                    stl_resolution=20,
                )
                mdof.add_component(wing)
        
        return mdof

if __name__ == "__main__":
    # To create the nominal geometry
    parameters = {}
    parametric_generator = ParametricMdof(**parameters)
    mdof = parametric_generator.create_instance()
    # print(parametric_generator.canard_angle)
    mdof.generate()
    mdof.transform(transformations=CART3D)
    mdof.to_stl("parts/fuse2-c0")
