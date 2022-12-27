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
