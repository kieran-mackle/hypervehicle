from hypervehicle.components import Sphere


sphere = Sphere(r=1, stl_resolution=20)
sphere.generate_patches()
sphere.to_stl(outfile="sphere.stl")
