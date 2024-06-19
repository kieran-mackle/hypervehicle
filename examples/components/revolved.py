from hypervehicle.components import RevolvedComponent
from hypervehicle.geometry import Vector3, Bezier, RobertsFunction


# Define the curve to be revolved
curve = Bezier(
    [
        Vector3(x=0.0, y=0.0),
        Vector3(x=0.5, y=0.1),
        Vector3(x=0.9, y=0.5),
        Vector3(x=1.0, y=0.0),
    ]
)

# Define clustering function
clustering = RobertsFunction(False, True, 1.01)

# Create the component
component = RevolvedComponent(revolve_line=curve, stl_resolution=50)
component.add_clustering_options(i_clustering_func=clustering)
component.generate_patches()
component.to_stl("revolve.stl")
