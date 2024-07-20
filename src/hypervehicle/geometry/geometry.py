import numpy as np
from hypervehicle.geometry.vector import Vector3
from hypervehicle.geometry.surface import CoonsPatch, ParametricSurface
from hypervehicle.geometry.path import Line, Path, ArcLengthParameterizedPath


class SubRangedPath(Path):
    """
    A Path reparameterized to a subset of the original between t0 and t1.
    If t1 < t0 the path will be traveresed in the reverse direction.
    """

    __slots__ = ["underlying_path", "t0", "t1"]

    def __init__(self, underlying_path, t0, t1):
        if isinstance(underlying_path, Path):
            self.underlying_path = underlying_path
            self.t0 = t0
            self.t1 = t1
        else:
            raise NotImplementedError("underlying_path should be a type of Path")

    def __repr__(self) -> str:
        return "ArcLengthParameterizedPath(underlying_path={}, t0={}, t1={})".format(
            self.underlying_path, self.t0, self.t1
        )

    def __str__(self) -> str:
        return "ArcLengthParameterizedPath"

    def __call__(self, t):
        t_dash = self.t0 + t * (self.t1 - self.t0)
        return self.underlying_path(t_dash)

    def length(self):
        return self.underlying_path.length()


class ReversedPath(SubRangedPath):
    def __init__(self, underlying_path: Path) -> None:
        self.underlying_path = underlying_path
        self.t0 = 1
        self.t1 = 0


class ElipsePath(Path):
    """
    A path following a quarter elipse from a -> b, around c

    b - _
          -_
            \
    c       a

    """

    __slots__ = ["centre", "thickness", "LE_width", "side"]

    def __init__(self, centre, thickness, LE_width, side):
        self.centre = centre
        self.thickness = thickness
        self.LE_width = LE_width
        self.side = side

    def __repr__(self):
        return "ElipsePath"

    def __call__(self, r):
        # establish local elipse shapes
        a = self.LE_width
        b = abs(self.thickness)
        # a = b*self.LE_ratio(t).y

        # construct elipse
        if self.side == "bot":
            theta = 0 + r * np.pi / 2
        elif self.side == "top":
            theta = -np.pi / 2 + r * np.pi / 2
        else:
            raise Exception("Value of 'side' not supported")

        x_elipse = a * b / np.sqrt(b * b + (a * np.tan(theta)) ** 2)
        y_elipse = x_elipse * np.tan(theta)
        # set elipse angle
        angle = np.pi / 2
        # add elipse to overall shape
        x = x_elipse * np.cos(angle)
        y = x_elipse * np.sin(angle)
        z = y_elipse
        elipse_base = self.centre
        return elipse_base + Vector3(x=x, y=y, z=z)


class OffsetPathFunction(Path):
    """
    Creates a line with an offset applied
    """

    __slots__ = ["underlying_line", "offset_function"]

    def __init__(self, underlying_line, offset_function):
        self.underlying_line = underlying_line
        self.offset_function = offset_function

    def __repr__(self):
        return "Offset path function"

    def __call__(self, t):
        # calculate postion
        pos = self.underlying_line(t)

        # calculate local thickness
        offset = self.offset_function(x=pos.x, y=pos.y)

        return pos + offset


class GeometricMeanPathFunction(Path):
    """
    Creates a line which is the geometric mean of two underlying lines.
    """

    __slots__ = ["underlying_line_1", "underlying_line_2"]

    def __init__(self, underlying_line_1, underlying_line_2):
        self.underlying_line_1 = underlying_line_1
        self.underlying_line_2 = underlying_line_2

    def __repr__(self):
        return "Mean path function"

    def __call__(self, t):
        # calculate postion
        pos_1 = self.underlying_line_1(t)
        pos_2 = self.underlying_line_2(t)

        # calculate local thickness
        mean_point = 0.5 * (pos_1 + pos_2)

        return mean_point


class OffsetPatchFunction(ParametricSurface):
    """
    Creates a patch with an offset applied.
    """

    __slots__ = ["underlying_surf", "function"]

    def __init__(self, underlying_surf, function):
        self.underlying_surf = underlying_surf
        self.function = function

    def __repr__(self):
        str = " + offset function"
        str = self.underlying_surf.__repr__() + str
        return str

    def __call__(self, r, s):
        pos = self.underlying_surf(r, s)
        offset = self.function(pos.x, pos.y, pos.z)
        return self.underlying_surf(r, s) + offset


class LeadingEdgePatchFunction(ParametricSurface):
    """Creates Leading Edge by pair of guiding lines and LE_width function."""

    __slots__ = [
        "centralLine",
        "thickness_function",
        "LE_width_function",
        "t0",
        "t1",
        "side",
    ]

    def __init__(
        self, centralLine, thickness_function, LE_width_function, t0, t1, side="top"
    ):
        self.centralLine = centralLine
        self.thickness_function = thickness_function
        self.t0 = t0
        self.t1 = t1
        self.LE_width_function = LE_width_function
        self.side = side

    def __repr__(self):
        return "Leading edge surface"

    def __call__(self, t_raw, r):
        # convert to global t
        # t = self.t0 + t_raw * (self.t1 - self.t0)
        t = self.t1 - t_raw * (self.t1 - self.t0)
        # calculate local thickness
        pos = self.centralLine(t)
        thickness = self.thickness_function(
            x=pos.x, y=pos.y
        ).z  # we only need to get z-value
        LE_width = self.LE_width_function(t)

        # establish local elipse shapes
        elipse = ElipsePath(
            centre=Vector3(x=0.0, y=0.0, z=0),
            thickness=thickness,
            LE_width=LE_width,
            side=self.side,
        )
        elipse_norm = ArcLengthParameterizedPath(underlying_path=elipse)
        pos_elipse = elipse_norm(r)
        x_elipse = pos_elipse.y
        # y_elipse = pos_elipse.x
        z_elipse = pos_elipse.z

        # set elipse angle
        if t == 0:
            angle = np.pi / 2
        elif t == 1:
            angle = 0
        else:
            dt = min([0.001, abs(1.0 - t), abs(t - 0.0)])
            plus = self.centralLine(t + dt)
            minus = self.centralLine(t - dt)
            angle = np.arctan2((plus.y - minus.y), (plus.x - minus.x)) + np.pi / 2

        # print(angle, x_elipse)

        # add elipse to overall shape
        x = x_elipse * np.cos(angle)
        y = x_elipse * np.sin(angle)
        z = z_elipse
        elipse_base = self.centralLine(t)
        return elipse_base + Vector3(x=x, y=y, z=z)


class MeanLeadingEdgePatchFunction(ParametricSurface):
    """Creates Leading Edge by mean line and guiding line, and LE_width function."""

    __slots__ = ["mean_line", "guide_line", "LE_width_function", "t0", "t1", "side"]

    def __init__(self, mean_line, guide_line, LE_width_function, t0, t1, side="top"):
        self.mean_line = mean_line
        self.guide_line = guide_line
        self.t0 = t0
        self.t1 = t1
        self.LE_width_function = LE_width_function
        self.side = side

    def __repr__(self):
        return "Leading edge surface patch"

    def __call__(self, t_raw, r):
        # Convert to global t
        t = self.t1 - t_raw * (self.t1 - self.t0)

        # Calculate local thickness
        mean_point = self.mean_line(t)
        guide_point = self.guide_line(t)
        thickness = mean_point.z - guide_point.z
        LE_width = self.LE_width_function(t)

        # Establish local elipse shapes
        elipse = ElipsePath(
            centre=Vector3(x=0.0, y=0.0, z=0),
            thickness=thickness,
            LE_width=LE_width,
            side=self.side,
        )
        elipse_norm = ArcLengthParameterizedPath(underlying_path=elipse)
        pos_elipse = elipse_norm(r)
        x_elipse = pos_elipse.y
        z_elipse = pos_elipse.z

        # set elipse angle
        if t == 0:
            angle = np.pi / 2
        elif t == 1:
            angle = 0
        else:
            dt = min([0.001, abs(1.0 - t), abs(t - 0.0)])
            plus = self.mean_line(t + dt)
            minus = self.mean_line(t - dt)
            angle = np.arctan2((plus.y - minus.y), (plus.x - minus.x)) + np.pi / 2

        # Add elipse to overall shape
        x = x_elipse * np.cos(angle)
        y = x_elipse * np.sin(angle)
        z = z_elipse

        return mean_point + Vector3(x=x, y=y, z=z)


class FlatLeadingEdgePatchFunction(ParametricSurface):
    """
    Creates a flat leading edge between two paths.

    TODO - adapt this to allow sharp LE
    """

    __slots__ = ["path1", "path2", "t0", "t1"]

    def __init__(self, path1, path2, t0, t1):
        self.path1 = path1
        self.path2 = path2

        # Parametric section of mean and guide line
        self.t0 = t0
        self.t1 = t1

    def __repr__(self):
        return "Flat leading edge surface patch"

    def __call__(self, t_raw, r):
        # Convert to global t
        t = self.t1 - t_raw * (self.t1 - self.t0)

        # Calculate local thickness
        point1 = self.path1(t)
        point2 = self.path2(t)

        return Vector3(x=point1.x, y=point1.y, z=(1 - r) * point1.z + r * point2.z)


class TrailingEdgePath(Path):
    """
    A path following rear of wing
    """

    __slots__ = ["A0", "B0", "thickness_function"]

    def __init__(self, A0, B0, thickness_function):
        self.A0 = A0
        self.B0 = B0
        self.thickness_function = thickness_function
        self.Line = Line(p0=A0, p1=B0)

    def __repr__(self):
        return "Trailing Egde Path"

    def __call__(self, r):
        # calculate postion
        pos = self.Line(r)
        # calculate local thickness
        offset = self.thickness_function(x=pos.x, y=pos.y)
        return pos + offset


class TrailingEdgePatch(ParametricSurface):
    """
    Fucntion to create a trailing edge patch
    """

    __slots__ = ["A0", "B0", "TE_path", "flap_length", "flap_angle", "side"]

    def __init__(self, A0, B0, TE_path, flap_length, flap_angle=0.0, side="top"):
        self.A0 = A0
        self.B0 = B0
        self.TE_path = TE_path
        self.flap_length = flap_length
        self.flap_angle = flap_angle
        self.side = side
        self.preparation()

    def __repr__(self):
        return "Trailing Edge for '{0}' with flap_angle={1} deg".format(
            self.side, np.rad2deg(self.flap_angle)
        )

    def __call__(self, r, s):
        return self.patch(r, s)

    def preparation(self):
        if self.side == "top":
            north = self.TE_path
            south = Line(
                p0=Vector3(
                    x=self.A0.x - self.flap_length,
                    y=self.A0.y,
                    z=0.0 + self.flap_length * np.sin(self.flap_angle),
                ),
                p1=Vector3(
                    x=self.B0.x - self.flap_length,
                    y=self.B0.y,
                    z=0.0 + self.flap_length * np.sin(self.flap_angle),
                ),
            )
            west = Line(p0=south(0.0), p1=north(0.0))
            east = Line(p0=south(1.0), p1=north(1.0))
        elif self.side == "bot":
            south = self.TE_path
            north = Line(
                p0=Vector3(
                    x=self.A0.x - self.flap_length,
                    y=self.A0.y,
                    z=0.0 + self.flap_length * np.sin(self.flap_angle),
                ),
                p1=Vector3(
                    x=self.B0.x - self.flap_length,
                    y=self.B0.y,
                    z=0.0 + self.flap_length * np.sin(self.flap_angle),
                ),
            )
            west = Line(p0=south(0.0), p1=north(0.0))
            east = Line(p0=south(1.0), p1=north(1.0))
        else:
            raise Exception("Value of 'side' not supported")
        self.patch = CoonsPatch(south=south, north=north, west=west, east=east)


class MeanTrailingEdgePatch(ParametricSurface):
    """
    Fucntion to create a trailing edge patch, with flap angle defined
    from the geometric mean of upper and lower surfaces.
    """

    __slots__ = ["mean_line", "TE_path", "flap_length", "flap_angle", "side"]

    def __init__(self, mean_line, TE_path, flap_length, flap_angle=0, side="top"):
        self.mean_line = mean_line
        self.TE_path = TE_path
        self.flap_length = flap_length
        self.flap_angle = flap_angle
        self.side = side
        self.preparation()

    def __repr__(self):
        return f"Trailing Edge for '{self.side}' with flap_angle={np.rad2deg(self.flap_angle)} deg"

    def __call__(self, r, s):
        return self.patch(r, s)

    def preparation(self):
        if self.side == "top":
            north = self.TE_path
            south = Line(
                p0=Vector3(
                    x=self.mean_line(0).x - self.flap_length * np.cos(self.flap_angle),
                    y=self.mean_line(0).y,
                    z=self.mean_line(0).z + self.flap_length * np.sin(self.flap_angle),
                ),
                p1=Vector3(
                    x=self.mean_line(1).x - self.flap_length * np.cos(self.flap_angle),
                    y=self.mean_line(1).y,
                    z=self.mean_line(1).z + self.flap_length * np.sin(self.flap_angle),
                ),
            )

            west = Line(p0=south(0.0), p1=north(0.0))
            east = Line(p0=south(1.0), p1=north(1.0))

        elif self.side == "bot":
            south = self.TE_path
            north = Line(
                p0=Vector3(
                    x=self.mean_line(0).x - self.flap_length * np.cos(self.flap_angle),
                    y=self.mean_line(0).y,
                    z=self.mean_line(0).z + self.flap_length * np.sin(self.flap_angle),
                ),
                p1=Vector3(
                    x=self.mean_line(1).x - self.flap_length * np.cos(self.flap_angle),
                    y=self.mean_line(1).y,
                    z=self.mean_line(1).z + self.flap_length * np.sin(self.flap_angle),
                ),
            )
            west = Line(p0=south(0.0), p1=north(0.0))
            east = Line(p0=south(1.0), p1=north(1.0))

        else:
            raise Exception("Value of 'side' not supported")

        self.patch = CoonsPatch(south=south, north=north, west=west, east=east)


class CurvedPatch(ParametricSurface):
    """Adds curvature in x or y direction to an existing patch"""

    __slots__ = ["underlying_surf", "direction", "fun", "fun_dash"]

    def __init__(self, underlying_surf, direction=None, fun=None, fun_dash=None):
        self.underlying_surf = underlying_surf
        self.direction = direction
        self.fun = fun
        self.fun_dash = fun_dash

    def __repr__(self):
        return self.underlying_surf.__repr__() + " with added curvature"

    def __call__(self, r, s):
        if self.fun == None or self.fun_dash == None:
            raise Exception("Both 'fun' and 'fun_dash' need to be specified.")
        pos = self.underlying_surf(r, s)
        offset = self.fun(pos.x, pos.y)
        slope = self.fun_dash(pos.x, pos.y)
        angle = np.arctan2(slope, 1)
        if self.direction == "x":
            pos_new = Vector3(
                x=pos.x - pos.z * np.sin(angle),
                y=pos.y,
                z=pos.z * np.cos(angle) + offset,
            )
            return pos_new
        if self.direction == "y":
            pos_new = Vector3(
                x=pos.x,
                y=pos.y - pos.z * np.sin(angle),
                z=offset + pos.z * np.cos(angle),
            )
            return pos_new


class ConePatch(ParametricSurface):
    """
    Creates a patch describing a cone (or cylinder) between two rings.
    """

    __slots__ = ["x0", "x1", "r0", "r1", "angle0", "angle1"]

    def __init__(self, x0, x1, r0, r1, angle0, angle1):
        self.x0 = x0
        self.x1 = x1
        self.r0 = r0
        self.r1 = r1
        self.angle0 = angle0
        self.angle1 = angle1

    def __repr__(self):
        str = "Cone Patch"
        return str

    def __call__(self, r, s):
        angle = self.angle0 * (1 - s) + self.angle1 * s
        x0 = self.x0
        y0 = self.r0 * np.cos(angle)
        z0 = self.r0 * np.sin(angle)
        x1 = self.x1
        y1 = self.r1 * np.cos(angle)
        z1 = self.r1 * np.sin(angle)
        x = x0 * (1 - r) + x1 * r
        y = y0 * (1 - r) + y1 * r
        z = z0 * (1 - r) + z1 * r
        return Vector3(x=x, y=y, z=z)


class RevolvedPatch(ParametricSurface):
    """Creates a path by revolving a line about a central
    axis.
    """

    def __init__(self, line, angle0=0, angle1=2 * np.pi):
        self.line = line
        self.angle0 = angle0
        self.angle1 = angle1

    def __repr__(self):
        return "Revolved Patch"

    def __call__(self, r, s):
        # Map s to angular coordinate
        angle = self.angle0 * (1 - s) + self.angle1 * s

        # Map r to distance along line
        point = self.line(r)

        # Calculate points
        x = point.x
        y = point.y * np.cos(angle) + point.z * np.sin(angle)
        z = point.z * np.cos(angle) - point.y * np.sin(angle)

        return Vector3(x=x, y=y, z=z)


class BluntConePatch(ParametricSurface):
    """
    Creates a patch describing a blunt cone.

    Parameters:
        x0: x coordinate of base of cone
        y0: y coordinate of base of cone
        rn: spherical nose radius
        rb: radius of cone at base of cone
        L: length of nominal cone (base to tip before spherical blunting)
        angle0: start revolve angle
        angle1: end revolve angle

    """

    __slots__ = ["x0", "y0", "rn", "rb", "L", "angle0", "angle1"]

    def __init__(self, x0, y0, rn, rb, L, angle0, angle1):
        self.L = L
        self.x0 = x0
        self.y0 = y0
        self.rn = rn
        self.rb = rb
        self.angle0 = angle0
        self.angle1 = angle1

    def __repr__(self):
        str = "Blunt Cone Patch"
        return str

    def __call__(self, r, s):
        angle = self.angle0 * (1 - s) + self.angle1 * s
        x0 = self.x0
        y0 = self.r0 * np.cos(angle)
        z0 = self.r0 * np.sin(angle)
        x1 = self.x1
        y1 = self.r1 * np.cos(angle)
        z1 = self.r1 * np.sin(angle)
        x = x0 * (1 - r) + x1 * r
        y = y0 * (1 - r) + y1 * r
        z = z0 * (1 - r) + z1 * r
        return Vector3(x=x, y=y, z=z)


class SweptPatch(ParametricSurface):
    """Creates a swept patch from a series of cross sections."""

    __slots__ = ["cross_sections", "section_origins"]

    def __init__(self, cross_sections: list, sweep_axis: str = "z") -> None:
        """Construct the SweptPatch object.

        Parameters
        -----------
        cross_sections : list
            A list containing the cross sections to be blended. These must
            be defined in the x-y plane.
        sweep_axis : str, optional
            The axis to sweep the cross sections along. Note that each cross
            section should vary along this axis. The default is "z".
        """
        self.cross_sections = cross_sections
        self.section_origins = [getattr(cs(0, 0), sweep_axis) for cs in cross_sections]
        self._sweep_axis = sweep_axis
        self._other_axes = {"x", "y", "z"} - set(sweep_axis)

        if min(self.section_origins) == max(self.section_origins):
            raise Exception(
                "There is no axial variation in the cross " + "sections provided!"
            )

        self.perimeters = [SurfacePerimeter(s) for s in cross_sections]
        self.min_origin = min(self.section_origins)
        self.origin_dist = max(self.section_origins) - self.min_origin

    def __repr__(self):
        return "Swept Patch"

    def __call__(self, r, s) -> Vector3:
        # Calculate physical axial distance
        dist = self.min_origin + r * self.origin_dist

        # Find index of bounding cross sections
        for i, lv in enumerate(self.section_origins):
            if dist == self.section_origins[-1]:
                i = len(self.section_origins) - 2
                break
            elif lv <= dist < self.section_origins[i + 1]:
                break

        # Get upper and lower perimeter
        lps = self.perimeters[i](s)
        ups = self.perimeters[i + 1](s)

        # Get upper and lower bounding sections
        ls = self.section_origins[i]
        us = self.section_origins[i + 1]

        # Linearly interpolate between cross-sectional perimeters
        args = {
            a: getattr(lps, a)
            + (dist - ls) * (getattr(ups, a) - getattr(lps, a)) / (us - ls)
            for a in self._other_axes
        }
        args[self._sweep_axis] = dist

        return Vector3(**args)


class SweptPatchfromEdges(ParametricSurface):
    """Creates a swept patch from a series of cross sections.
    Cross sections do not need to be parallel or aligned."""

    __slots__ = ["edges"]

    def __init__(self, edges: list) -> None:
        """Construct the SweptPatch object.

        Parameters
        -----------
        edges : list
            A list containing the edges through which to sweep.
        """
        self.edges = edges
        self.n_edges = len(edges)

    def __repr__(self):
        return "Swept Patch"

    def __call__(self, r, s) -> Vector3:
        r_slices = np.linspace(0, 1, self.n_edges)
        if r in r_slices:
            # can evaluate exactly
            i = np.where(r_slices == r)[0][0]
            return self.edges[i](s)
        else:
            # need to interpolate
            index_above = np.where(r_slices > r)[0][0]
            index_below = index_above - 1
            above = self.edges[index_above](s)
            below = self.edges[index_below](s)
            alpha = (r - r_slices[index_below]) / (
                r_slices[index_above] - r_slices[index_below]
            )
            return (1 - alpha) * below + alpha * above


class RotatedPatch(ParametricSurface):
    """
    Rotates a surface about a point in an axis-specified direction.
    """

    __slots__ = ["underlying_surf", "angle", "axis", "point"]

    def __init__(self, underlying_surf, angle, axis="x", point=Vector3(x=0, y=0, z=0)):
        self.underlying_surf = underlying_surf
        self.angle = angle
        self.axis = axis.lower()
        self.point = point

    def __repr__(self):
        str = f" (rotated by {np.rad2deg(self.angle)} degrees)"
        return self.underlying_surf.__repr__() + str

    def __call__(self, r, s):
        pos = self.underlying_surf(r, s) - self.point

        if self.axis == "x":
            x = pos.x
            y = pos.y * np.cos(self.angle) - pos.z * np.sin(self.angle)
            z = pos.y * np.sin(self.angle) + pos.z * np.cos(self.angle)
        elif self.axis == "y":
            x = pos.x * np.cos(self.angle) + pos.z * np.sin(self.angle)
            y = pos.y
            z = -pos.x * np.sin(self.angle) + pos.z * np.cos(self.angle)

        elif self.axis == "z":
            x = pos.x * np.cos(self.angle) - pos.y * np.sin(self.angle)
            y = pos.x * np.sin(self.angle) + pos.y * np.cos(self.angle)
            z = pos.z

        return Vector3(x=x, y=y, z=z) + self.point


class MirroredPatch(ParametricSurface):
    """Mirrors a surface in an axis-specified direction."""

    __slots__ = ["underlying_surf", "axis"]

    def __init__(self, underlying_surf, axis="x"):
        self.underlying_surf = underlying_surf
        self.axis = axis.lower()

    def __repr__(self):
        return self.underlying_surf.__repr__() + f" mirrored along {self.axis}-axis"

    def __call__(self, r, s):
        pos = self.underlying_surf(r, s)
        x = pos.x
        y = pos.y
        z = pos.z
        if self.axis == "x":
            # Mirror about y-z plane
            x *= -1
        elif self.axis == "y":
            # Mirror about x-z plane
            y *= -1
        elif self.axis == "z":
            # Mirror about x-y plane
            z *= -1

        return Vector3(x=x, y=y, z=z)


class CubePatch(ParametricSurface):
    """Creates a cube face patch for a cube of length
    2a about the Vector3 centre.
    """

    __slots__ = ["a", "centre", "face"]

    def __init__(self, a, centre, face):
        self.a = a
        self.face = face
        self.centre = centre

    def __repr__(self):
        return "Cube: a = {}, centre = {}".format(
            self.r, self.centre
        ) + "face = {}".format(self.face)

    def __call__(self, r, s):
        if self.face == "east":
            x = 1.0
            y = -1.0 + 2.0 * r
            z = -1.0 + 2.0 * s
        elif self.face == "west":
            x = -1.0
            y = -1.0 + 2.0 * r
            z = -1.0 + 2.0 * s
        elif self.face == "south":
            x = -1.0 + 2.0 * r
            y = -1.0
            z = -1.0 + 2.0 * s
        elif self.face == "north":
            x = -1.0 + 2.0 * r
            y = 1.0
            z = -1.0 + 2.0 * s
        elif self.face == "bottom":
            x = -1.0 + 2.0 * r
            y = -1.0 + 2.0 * s
            z = -1.0
        elif self.face == "top":
            x = -1.0 + 2.0 * r
            y = -1.0 + 2.0 * s
            z = 1.0

        else:
            raise ValueError(
                "Incorrect face name."
                + "Allowable faces are: east, west, south, north, bottom or top."
            )

        x_cube = x * self.a + self.centre.x
        y_cube = y * self.a + self.centre.y
        z_cube = z * self.a + self.centre.z

        return Vector3(x=x_cube, y=y_cube, z=z_cube)


class SpherePatch(ParametricSurface):
    """Creates a sphere face patch for a cube of length
    2a about the Vector3 centre.
    """

    __slots__ = ["r", "centre", "face"]

    def __init__(self, r, centre, face):
        self.r = r
        self.face = face
        self.centre = centre

    def __repr__(self):
        return "Sphere: r = {}, centre = {}".format(
            self.r, self.centre
        ) + "face = {}".format(self.face)

    def __call__(self, r, s):
        # First create a cube face, then map to a sphere.
        # Can change below to call a cube patch rather than
        # computing cube face here, it just adds a dependency.

        if self.face == "east":
            x = 1.0
            y = -1.0 + 2.0 * r
            z = -1.0 + 2.0 * s
        elif self.face == "west":
            x = -1.0
            y = -1.0 + 2.0 * r
            z = -1.0 + 2.0 * s
        elif self.face == "south":
            x = -1.0 + 2.0 * r
            y = -1.0
            z = -1.0 + 2.0 * s
        elif self.face == "north":
            x = -1.0 + 2.0 * r
            y = 1.0
            z = -1.0 + 2.0 * s
        elif self.face == "bottom":
            x = -1.0 + 2.0 * r
            y = -1.0 + 2.0 * s
            z = -1.0
        elif self.face == "top":
            x = -1.0 + 2.0 * r
            y = -1.0 + 2.0 * s
            z = 1.0

        else:
            raise ValueError(
                "Incorrect face name."
                + "Allowable faces are: east, west, south, north, bottom or top."
            )

        x_dash = x * np.sqrt(1.0 - 0.5 * z * z - 0.5 * y * y + y * y * z * z / 3.0)
        y_dash = y * np.sqrt(1.0 - 0.5 * z * z - 0.5 * x * x + x * x * z * z / 3.0)
        z_dash = z * np.sqrt(1.0 - 0.5 * y * y - 0.5 * x * x + x * x * y * y / 3.0)

        x_sphere = x_dash * self.r + self.centre.x
        y_sphere = y_dash * self.r + self.centre.y
        z_sphere = z_dash * self.r + self.centre.z

        return Vector3(x=x_sphere, y=y_sphere, z=z_sphere)


class SurfacePerimeter(Path):
    """Returns a path corresponding to the perimiter of an underlying
    surface."""

    __slots__ = ["underlying_surf"]

    def __init__(self, underlying_surf: ParametricSurface) -> None:
        self.underlying_surf = underlying_surf

    def __repr__(self):
        return "Surface Perimeter Path"

    def __call__(self, t) -> Vector3:
        """Returns the point on the perimeter."""
        f = 0.25
        rem = t % f

        # Calculate r,s along perimeter
        if t < 0.25:
            r = round(rem / f, 8)
            s = 0.0
        elif t < 0.5:
            r = 1.0
            s = round(rem / f, 8)
        elif t < 0.75:
            r = round(1 - rem / f, 8)
            s = 1.0
        elif t < 1:
            r = 0.0
            s = round(1 - rem / f, 8)
        else:
            r = 0.0
            s = 0.0

        return self.underlying_surf(r, s)
