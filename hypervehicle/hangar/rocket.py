import numpy as np
from hypervehicle import Vehicle
from hypervehicle.geometry import Vector3
from hypervehicle.generator import Generator
from hypervehicle.components import Fuselage, Fin, common


class ParametricRocket(Generator):
    def __init__(self, **kwargs) -> None:
        # Vehicle parameters
        self.fin_height = 0.1
        self.fin_thickness = 0.01
        self.no_fins = 3

        self.Xn = 2.0
        self.X1 = 1.8
        self.X2 = 1.5
        self.X3 = 0.0

        self.R1 = 0.1
        self.R2 = 0.15
        self.R3 = 0.15

        # Complete instantiation
        super.__init__(**kwargs)

    def create_instance(self) -> Vehicle:
        rocket = Vehicle()
        rocket.configure(name="rocket", verbosity=1)

        # Construct fuselage
        # ====================
        #
        #  R3--------------------------------R2____
        #  |                                       R1__
        #  |                                           --_
        #  X3--------------------------------X2----X1----Xn

        body = Fuselage(
            Xn=self.Xn,
            X1=self.X1,
            X2=self.X2,
            X3=self.X3,
            R1=self.R1,
            R2=self.R2,
            R3=self.R3,
        )
        rocket.add_component(body)

        # Construct fins
        # ====================
        #   p1-----p2
        #    \        \
        #     \         \
        #      \          \
        #       p0_________p3

        p0 = Vector3(x=0.1, y=0.15)
        p1 = Vector3(x=0, y=0.15 + self.fin_height)
        p2 = Vector3(x=0.2, y=0.15 + self.fin_height)
        p3 = Vector3(x=0.4, y=0.15)

        # Add fins around body
        for i in range(self.no_fins):
            angle = np.deg2rad((i / self.no_fins) * 360)
            fin = Fin(
                p0=p0,
                p1=p1,
                p2=p2,
                p3=p3,
                fin_thickness=self.fin_thickness,
                fin_angle=angle,
                top_thickness_function=common.uniform_thickness_function(
                    self.fin_thickness, "top"
                ),
                bot_thickness_function=common.uniform_thickness_function(
                    self.fin_thickness, "bot"
                ),
            )
            rocket.add_component(fin)

        return rocket


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_generator = ParametricRocket()
    rocket = parametric_generator.create_instance()
    rocket.generate()
    rocket.to_stl("rocket")
