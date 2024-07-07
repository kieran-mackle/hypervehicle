from copy import copy, deepcopy
from hypervehicle import Vehicle
from hypervehicle.generator import Generator
from hypervehicle.geometry import Vector3, Line
from hypervehicle.components import (
    RevolvedComponent,
    SweptComponent,
    CompositeComponent,
)


class ParametricFinner(Generator):
    """Parametric generator for generic finned rocket."""

    def __init__(self, **kwargs) -> None:
        """Create a parametric generator for the basic finner.

        Parameters
        ----------
        diameter : float, optional
            The diameter of the body. The default is 1.

        total_length : float, optional
            The total length of the rocket. The default is 10.

        nose_length : float, optional
            The length of the nose. The default is 2.84.

        fin_height : float, optional
            The height of the fins. The default is 1.0.

        fin_length : float, optional
            The length of the fins. The default is 1.0.

        max_fin_thickness : float, optional
            The maximum thickness of the fins. The default is 0.08.

        References
        ----------
        https://hisa.gitlab.io/archive/asc/basicFinner/notes/basicFinner.html
        """
        # Vehicle parameters
        self.diameter = 1.0
        self.total_length = 10.0
        self.nose_length = 2.84
        self.fin_height = 1.0
        self.fin_length = 1.0
        self.max_fin_thickness = 0.08

        # Complete instantiation
        super().__init__(**kwargs)

    def create_instance(self) -> Vehicle:
        # Instantiate Vehicle
        finner = Vehicle()
        finner.configure(
            name="Finner",
            verbosity=1,
        )

        # Define key points
        origin = Vector3(0, 0, 0)
        nose_base = origin + Vector3(x=self.nose_length, y=self.diameter / 2)
        rocket_base = origin + Vector3(x=self.total_length, y=self.diameter / 2)

        # Construct nose-body revolve lines
        nose_line = Line(p0=origin, p1=nose_base)
        fuselage_line = Line(p0=nose_base, p1=rocket_base)
        fuselage_base_line = Line(
            p0=rocket_base, p1=rocket_base - Vector3(x=0, y=rocket_base.y)
        )

        # Construct revolved surfaces
        nose = RevolvedComponent(nose_line)
        fuselage_outer = RevolvedComponent(fuselage_line)
        fuselage_base = RevolvedComponent(fuselage_base_line)

        # Define fuselage component
        fuselage = CompositeComponent(stl_resolution=8, name="fuselage")
        fuselage.add_component(nose)
        fuselage.add_component(fuselage_outer)
        fuselage.add_component(fuselage_base)

        # Define nominal fin and transformations
        a = Vector3(x=0, y=self.fin_length)
        b = Vector3(x=0, y=0)
        c = Vector3(x=-0.5 * self.max_fin_thickness, y=0)
        d = Vector3(x=0.5 * self.max_fin_thickness, y=0)

        # Create cross section paths for swept component
        cs1 = [
            Line(a, d),
            Line(d, b),
            Line(b, c),
            Line(c, a),
        ]
        cs2 = [line + Vector3(x=0, y=0, z=self.fin_height) for line in cs1]

        # Create swept fin component
        fin = SweptComponent(cross_sections=[cs1, cs2], stl_resolution=4)

        tf = [
            ("rotate", 90, "z"),  # align
            ("translate", Vector3(x=self.total_length, y=0, z=self.diameter / 2)),
            None,
        ]

        # Add components
        finner.add_component(fuselage)
        for i in range(4):
            angle = i * 90
            tf[2] = ("rotate", angle, "x")
            finner.add_component(
                deepcopy(fin), transformations=copy(tf), name=f"fin_{i+1}"
            )

        return finner


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_generator = ParametricFinner()
    finner = parametric_generator.create_instance()
    finner.generate()
    finner.to_stl()
