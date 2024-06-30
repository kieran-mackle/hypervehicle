import numpy as np
from hypervehicle import Vehicle
from hypervehicle.generator import Generator
from hypervehicle.components import RevolvedComponent
from hypervehicle.geometry import Vector3, Line, Polyline, Arc


class ParametricFalcon9(Generator):
    """Parametric generator for mock-up of the SpaceX Falcon 9.

    References
    ----------
    https://www.spacex.com/vehicles/falcon-9/
    """

    def __init__(self, **kwargs) -> None:
        # Vehicle parameters
        self.D = 3.75
        self.L_b = 60
        self.h = 5 / 2
        self.r_n = 0.75
        self.r_o = 7
        self.f_L = 6

        # Complete instantiation
        super().__init__(**kwargs)

    def create_instance(self) -> Vehicle:
        # Initialise
        falcon9 = Vehicle()
        falcon9.configure(
            name="SpaceX Falcon 9",
            verbosity=1,
        )

        # Dependencies
        x_o = -np.sqrt((self.r_o - self.r_n) ** 2 - (self.r_o - self.h) ** 2)
        y_t = self.r_n * (self.r_o - self.h) / (self.r_o - self.r_n)
        x_t = x_o - np.sqrt(self.r_n**2 - y_t**2)
        x_a = x_o - self.r_n

        # Ogive arc
        a_o = Vector3(-x_t, y_t)
        b_o = Vector3(0, self.h)
        c_o = Vector3(0, -self.r_o + self.h)
        ogive_arc = Arc(a_o, b_o, c_o)

        # Nose arc
        a_n = Vector3(-x_a, 0)
        b_n = a_o
        c_n = Vector3(-x_o, 0)
        nose_arc = Arc(a_n, b_n, c_n)

        # Fairing
        f0 = b_o
        f1 = f0 - Vector3(self.f_L, 0)
        fairing_line = Line(f0, f1)

        # Body radius
        r_b = self.D / 2

        # Fairing to body
        fb0 = f1
        fb1 = Vector3(f1.x - 0.1 * self.f_L, r_b)
        fb2 = Vector3(f1.x - 0.3 * self.f_L, 0)
        fb_line = Polyline([Line(fb0, fb1), Line(fb1, fb2)])

        # Fairing component
        fairing_line = Polyline([nose_arc, ogive_arc, fairing_line, fb_line])
        fairing = RevolvedComponent(revolve_line=fairing_line, stl_resolution=100)
        falcon9.add_component(fairing)

        # Body
        b00 = Vector3(f1.x, 0)
        b0 = Vector3(f1.x, r_b)
        b1 = b0 - Vector3(self.L_b, 0)
        body_cap_line = Line(b00, b0)
        body_line = Line(b0, b1)

        # Base
        bb0 = b1
        bb1 = Vector3(bb0.x, 0)
        base_line = Line(bb0, bb1)

        # Join
        line = Polyline([body_cap_line, body_line, base_line])

        fuselage = RevolvedComponent(revolve_line=line, stl_resolution=50)
        falcon9.add_component(fuselage)

        return falcon9


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_generator = ParametricFalcon9()
    falcon9 = parametric_generator.create_instance()
    falcon9.generate()
    falcon9.to_stl("falcon9")
