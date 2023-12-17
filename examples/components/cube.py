from hypervehicle.components import Cube


cube = Cube(a=1, stl_resolution=5)
cube.generate_patches()
cube.to_stl(outfile="cube.stl")
