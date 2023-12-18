from hypervehicle import Vehicle
from hypervehicle.generator import Generator
from hypervehicle.components import SweptComponent
from hypervehicle.geometry import Vector3, Line, CoonsPatch


class ParametricWedge(Generator):
    def __init__(self, **kwargs) -> None:
        # Wedge parameters
        self.wingspan = 1
        self.chord = 1
        self.thickness = 0.1

        # Complete instantiation
        super().__init__(**kwargs)

    def create_instance(self) -> Vehicle:
        # Create vehicle object
        wedge_tags = Vehicle()
        wedge_tags.configure(name="Wedge_Tags", verbosity=1)

        # Define wedge cross-section points
        #                   ^ +y
        #                   |
        #            W    _ - _   N        ___
        #            _ -    |   - _         |
        #  +x <--- < -------------- >       | thickness
        #            - _    |   _ -         |
        #            S   -  _ -     E      ___
        #
        #          |-----------------|
        #                wingspan

        NW = Vector3(x=0, y=0.5 * self.thickness)
        NE = Vector3(
            x=-0.5 * self.chord,
            y=0,
        )
        SE = Vector3(x=0, y=-0.5 * self.thickness)
        SW = Vector3(x=0.5 * self.chord, y=0)

        # Define patches forming wedge
        sections = []
        for i in [-1, 1]:
            z_loc = 0.5 * i * self.wingspan
            axial_shift = Vector3(x=0, y=0, z=z_loc)

            N = Line(p0=NW + axial_shift, p1=NE + axial_shift)
            S = Line(p0=SW + axial_shift, p1=SE + axial_shift)
            E = Line(p0=SE + axial_shift, p1=NE + axial_shift)
            W = Line(p0=SW + axial_shift, p1=NW + axial_shift)

            patch = CoonsPatch(north=N, south=S, east=E, west=W)
            sections.append(patch)

        fuselage_tags = {'swept_tag': 'FreeStream', 'end_0_tag': 'Inlet', 'end_1_tag': 'Outlet'}
        fuselage = SweptComponent(
            cross_sections=sections,
            sweep_axis="z",
            stl_resolution=10,
            tags=fuselage_tags,
        )
        wedge_tags.add_component(fuselage)

        # Generate STL
        return wedge_tags


if __name__ == "__main__":
    # To create the nominal geometry
    parametric_wedge_generator = ParametricWedge()
    wedge_tags = parametric_wedge_generator.create_instance()
    wedge_tags.generate()
    wedge_tags.to_vtk("wedge_tags")

    # # To run sensitivity study
    # from hypervehicle.utilities import SensitivityStudy

    # # Construct sensitivity study
    # ss = SensitivityStudy(ParametricWedge)

    # # Define parameters to get sensitivities to
    # parameters = {'thickness': 0.1, 'chord': 1, 'wingspan': 1}

    # # Perform study
    # sensitivities = ss.dvdp(parameters)

    # # Save to CSV
    # ss.to_csv()
