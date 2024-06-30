import numpy as np
from hypervehicle.geometry import Vector3, Bezier
from hypervehicle.components import Wing, RevolvedComponent, Fin
from hypervehicle.geometry import Vector3, Bezier, Line, Polyline, Arc, CoonsPatch


def leading_edge_width_function(r):
    temp = Bezier(
        [
            Vector3(x=0.0, y=0.01),
            Vector3(x=0.5, y=0.1),
            Vector3(x=1.0, y=0.01),
        ]
    )
    le_width = temp(r).y
    return le_width


def uniform_thickness_function(thickness: float, side: str):
    """Returns a function handle."""
    m = -1 if side == "top" else 1

    def tf(x: float, y: float, z: float = 0):
        return Vector3(x=0.0, y=0.0, z=m * thickness / 2)

    return tf


def circle_patch(centre: Vector3, r: float, plane: str = "xy") -> CoonsPatch:
    """Returns a parametric surface of a circle."""
    # TODO - planes other than xy
    tr = Vector3(x=centre.x + r * np.cos(np.pi / 4), y=centre.y + r * np.sin(np.pi / 4))
    tl = Vector3(x=centre.x - r * np.cos(np.pi / 4), y=centre.y + r * np.sin(np.pi / 4))
    br = Vector3(x=centre.x + r * np.cos(np.pi / 4), y=centre.y - r * np.sin(np.pi / 4))
    bl = Vector3(
        x=centre.x - r * np.cos(np.pi / 4), y=centre.y + -r * np.sin(np.pi / 4)
    )

    n = Arc(a=tl, b=tr, c=centre)
    e = Arc(a=br, b=tr, c=centre)
    s = Arc(a=bl, b=br, c=centre)
    w = Arc(a=bl, b=tl, c=centre)

    patch = CoonsPatch(north=n, east=e, south=s, west=w)

    return patch


class OgiveNose(RevolvedComponent):
    def __init__(
        self,
        h: float,
        r_n: float,
        r_o: float,
        L_o: float,
        stl_resolution: int = 3,
        name: str = None,
        **kwargs,
    ) -> None:
        """A Fuselage wrapper to create an Ogive nose component.
        Parameters
        ----------
        h : float
            The ogive height.

        r_n : float
            The ogive nose radius.

        r_o : float
            The ogive radius.

        L_o : float
            The ogive length.
        """

        # TODO - update docstrings, and tidy code, have done this
        # elsewhere already

        # TODO - think about locating nose, is the tip at (0,0,0)?
        # Document this

        # Ogive Dependencies
        x_o = -np.sqrt((r_o - r_n) ** 2 - (r_o - h) ** 2)
        y_t = r_n * (r_o - h) / (r_o - r_n)
        x_t = x_o - np.sqrt(r_n**2 - y_t**2)
        x_a = x_o - r_n

        # Ogive arc
        a_o = Vector3(-x_t, y_t)
        b_o = Vector3(0, h)
        c_o = Vector3(0, -r_o + h)
        ogive_arc = Arc(a_o, b_o, c_o)

        # Nose arc
        a_n = Vector3(-x_a, 0)
        b_n = a_o
        c_n = Vector3(-x_o, 0)
        nose_arc = Arc(a_n, b_n, c_n)

        # Nose body
        f0 = b_o
        f1 = f0 - Vector3(L_o, 0)
        fairing_line = Line(f0, f1)

        # Nose body base
        fb0 = f1
        fb1 = Vector3(f1.x, 0)
        fb_line = Line(fb0, fb1)

        fairing = Polyline([nose_arc, ogive_arc, fairing_line, fb_line])

        super().__init__(
            revolve_line=fairing, stl_resolution=stl_resolution, name=name, **kwargs
        )

    def __repr__(self):
        s = "Ogive nose component"
        if self.name:
            s += f" (tagged '{self.name}')"
        return s
