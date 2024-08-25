import numpy as np
from hypervehicle.geometry import Vector3, Bezier


def test_bezier():
    bez = Bezier(
        [
            Vector3(x=0.0, y=0.02),
            Vector3(x=0.75, y=0.05),
            Vector3(x=1.0, y=0.05),
        ]
    )

    t = np.linspace(0, 1, 11)

    x = [bez(t).x for t in t]
    y = [bez(t).y for t in t]
    z = [bez(t).z for t in t]

    x_ref = [
        np.float64(0.0),
        np.float64(0.14500000000000002),
        np.float64(0.28),
        np.float64(0.405),
        np.float64(0.52),
        np.float64(0.625),
        np.float64(0.7200000000000002),
        np.float64(0.805),
        np.float64(0.88),
        np.float64(0.945),
        np.float64(1.0),
    ]

    y_ref = [
        np.float64(0.02),
        np.float64(0.025700000000000004),
        np.float64(0.030800000000000004),
        np.float64(0.0353),
        np.float64(0.0392),
        np.float64(0.0425),
        np.float64(0.045200000000000004),
        np.float64(0.0473),
        np.float64(0.04880000000000001),
        np.float64(0.04970000000000001),
        np.float64(0.05),
    ]

    z_ref = [
        np.float64(0.0),
        np.float64(0.0),
        np.float64(0.0),
        np.float64(0.0),
        np.float64(0.0),
        np.float64(0.0),
        np.float64(0.0),
        np.float64(0.0),
        np.float64(0.0),
        np.float64(0.0),
        np.float64(0.0),
    ]

    assert x == x_ref
    assert y == y_ref
    assert z == z_ref
