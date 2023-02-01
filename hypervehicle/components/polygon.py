from scipy.optimize import bisect
from gdtk.geom.vector3 import Vector3
from hypervehicle.geometry import CubePatch, SpherePatch


def create_cube(a, centre):
    """Function generates a cube by calling
    the cube patch fn for each of the 6
    faces.
    Params:
        a - float (cube side length 2a)
        centre - Vector3 (centre point of cube)
    """
    patches = {}
    faces = ["east", "west", "south", "north", "bottom", "top"]

    for face in faces:
        patches[face] = CubePatch(a, centre, face)

    return patches


def create_sphere(r, centre):
    """Function generates a sphere surface by calling
    the sphere patch fn for 6 separate faces.
    Params:
        r - float (sphere radius)
        centre - Vector3 (centre point of sphere)
    """
    patches = {}
    faces = ["east", "west", "south", "north", "bottom", "top"]

    for face in faces:
        patches[face] = SpherePatch(r, centre, face)

    return patches


def poly_gen_main(polygon_geometries: list, verbosity: int = 1) -> list:
    """
    Polygon Generator for use in hypersonic simulations.

    Parameters
    ----------
    polygon_geometries : list
        A list containing polygon definition dictionaries.
    verbosity : int, optional
        The verbosity of the code output. The default is 1.

    Raises
    ------
    Exception
        When an invalid trailing edge option is provided.

    Returns
    -------
    patches : list
        A list of the polygon surface patches.

    References
    ----------
    This code was authored by Flynn Hack as a module to the
    hyper_vehicle code created by Ingo Jahn and Kieran Mackle.
    """

    poly_list = []

    if len(polygon_geometries) > 0:
        for poly_dict in polygon_geometries:
            assert len(list(poly_dict)) == 1, "Error in POLYGON_GEOM_LIST"

            poly_type = list(poly_dict)[0]
            params = poly_dict[poly_type]

            if poly_type == "cube":  # Call create_cube
                # First check the correct params are present.

                assert ("a" in params) and isinstance(params["a"], float), (
                    "Incorrect sphere input parameter: centre. Please ensure"
                    "it exists in params and is a Vector3 object."
                )
                assert ("centre" in params) and isinstance(params["centre"], Vector3), (
                    "Incorrect sphere input parameter: centre. Please ensure"
                    "it exists in params and is a Vector3 object."
                )

                cube_patches = create_cube(params["a"], params["centre"])
                poly_list.append(cube_patches)

            elif poly_type == "sphere":  # Call create_sphere
                # First check the correct params are present.

                assert ("r" in params) and isinstance(params["r"], float), (
                    "Incorrect sphere input parameter: r. Please ensure"
                    "it exists in params and is a float."
                )
                assert ("centre" in params) and isinstance(params["centre"], Vector3), (
                    "Incorrect sphere input parameter: centre. Please ensure"
                    "it exists in params and is a Vector3 object."
                )

                sphere_patches = create_sphere(params["r"], params["centre"])
                poly_list.append(sphere_patches)

            else:
                raise ValueError(
                    "Incorrect polygon type. Supported types are: cube or sphere"
                )

    else:
        raise ValueError(
            "No polygons to create."
            + "Please check jobfile and ensure poly_geom_dict correct"
        )

    return poly_list
