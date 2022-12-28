from hypervehicle.geometry import Vector3, Bezier


def leading_edge_width_function(r):
    temp = Bezier(
        [
            Vector3(x=0.0, y=0.02),
            Vector3(x=0.75, y=0.1),
            Vector3(x=1.0, y=0.3),
        ]
    )
    le_width = temp(r).y
    return le_width


def uniform_thickness_function(thickness: float, side: str):
    """Returns a function handle."""
    m = -1 if side == "top" else 1

    def tf(x: float, y: float, z: float = 0):
        return Vector3(x=x, y=y, z=m * thickness / 2)

    return tf
