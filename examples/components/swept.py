from hypervehicle.components import SweptComponent
from hypervehicle.geometry import Vector3, Line, CoonsPatch


# We will generate the wedge shown below by sweeping the
# cross section shown below through the width of the geometry
#                   ^ +y
#                   |
#            W    _ - _   N        ___
#            _ -    |   - _         |
#  +x <--- < -------------- >       | thickness
#            - _    |   _ -         |
#            S   -  _ -     E      ___
#          |-----------------|
#                length
thickness = 0.25
length = 1
width = 1

# Define the patches to sweep through
NW = Vector3(x=0, y=0.5 * thickness)
NE = Vector3(
    x=-0.5 * width,
    y=0,
)
SE = Vector3(x=0, y=-0.5 * thickness)
SW = Vector3(x=0.5 * width, y=0)

# Define two patches forming wedge
sections = []
for i in [-1, 1]:
    z_loc = 0.5 * i * length
    axial_shift = Vector3(x=0, y=0, z=z_loc)

    N = Line(p0=NW + axial_shift, p1=NE + axial_shift)
    S = Line(p0=SW + axial_shift, p1=SE + axial_shift)
    E = Line(p0=SE + axial_shift, p1=NE + axial_shift)
    W = Line(p0=SW + axial_shift, p1=NW + axial_shift)

    patch = CoonsPatch(north=N, south=S, east=E, west=W)
    sections.append(patch)

# Create the component
component = SweptComponent(
    cross_sections=sections,
    sweep_axis="z",
    stl_resolution=10,
)
component.generate_patches()
component.to_stl("swept.stl")
