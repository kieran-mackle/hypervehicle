import numpy as np
from typing import Optional
from abc import ABC, abstractmethod
from hypervehicle.geometry.vector import Vector3, cross_product


class Path(ABC):
    """Abstract parameteric path."""

    @abstractmethod
    def __repr__(self):
        pass

    @abstractmethod
    def __call__(self, t):
        pass

    def length(self, n=20):
        """Rudimentary evaluation of path length by sampling and summing."""
        length = 0.0
        p0 = self.__call__(0.0)
        dt = 1.0 / n
        for i in range(n):
            p1 = self.__call__((i + 1) * dt)
            length += abs(p1 - p0)
            p0 = p1
        return length


class Line(Path):
    """Straight line path between two points."""

    __slots__ = ["p0", "p1"]

    def __init__(self, p0, p1):
        self.p0 = Vector3(p0)
        self.p1 = Vector3(p1)

    def __repr__(self):
        return "Line(p0={}, p1={})".format(self.p0, self.p1)

    def __call__(self, t: float):
        return self.p0 * (1 - t) + self.p1 * t

    def length(self):
        return abs(self.p1 - self.p0)

    def __add__(self, offset: Vector3):
        if isinstance(offset, Vector3):
            p0 = self.p0 + offset
            p1 = self.p1 + offset
            return Line(p0=p0, p1=p1)
        else:
            raise ValueError(f"Cannot add a {type(offset)} to a line.")


class Bezier(Path):
    """Bezier curve defined on a list of points."""

    __slots__ = ["B"]

    def __init__(self, B):
        try:
            self.B = [Vector3(p) for p in B]
        except Exception as e:
            raise ValueError(
                f"Was expecting to get a list of points for B, but got {B}"
            )

    def __repr__(self):
        return f"Bezier(B={self.B})"

    def __call__(self, t):
        if len(self.B) == 1:
            return self.B[0]
        n_order = len(self.B) - 1
        # Apply de Casteljau's algorithm.
        Q = self.B.copy()  # work array will be overwritten
        for k in range(n_order):
            for i in range(n_order - k):
                Q[i] = Q[i] * (1.0 - t) + Q[i + 1] * t
        return Q[0]


class Polyline(Path):
    """Collection of Path segments."""

    __slots__ = ["segments", "t_values", "isclosed"]

    def __init__(self, segments: list[Path], closed=False, tolerance=1.0e-10):
        self.segments: list[Path] = []
        for seg in segments:
            self.segments.append(seg)
        self.isclosed = closed
        if self.isclosed:
            p0 = self.segments[0](0.0)
            p1 = self.segments[-1](1.0)
            if abs(p1 - p0) > tolerance:
                self.segments.append(Line(p0, p1))
        self.reset_breakpoints()

    def reset_breakpoints(self):
        self.t_values = [0.0]
        t_total = 0.0
        for seg in self.segments:
            t_total += seg.length()
            self.t_values.append(t_total)
        for i in range(len(self.t_values)):
            self.t_values[i] /= t_total

    def __repr__(self):
        text = "Polyline(segments=["
        n = len(self.segments)
        for i in range(n):
            text += "{}".format(self.segments[i])
            text += ", " if i < n - 1 else "]"
        return text

    def __call__(self, t):
        n = len(self.segments)
        if n == 1:
            return self.segments[0](t)

        f = self.segments[0](t)
        f *= 0.0

        for i, tl, tu in zip(range(n), self.t_values[:-1], self.t_values[1:]):
            t_local = (t - tl) / (tu - tl)
            if i == 0:
                isokay = t_local <= 1.0
            elif i == n - 1:
                isokay = t_local > 0.0
            else:
                isokay = np.logical_and(t_local > 0.0, t_local <= 1.0)
            f += self.segments[i](t_local) * isokay
        return f

    def length(self):
        L = 0.0
        for seg in self.segments:
            L += seg.length()
        return L


class Spline(Polyline):
    """Construct a spline of Bezier segments from a sequence of points."""

    def __init__(
        self,
        points,
        closed: Optional[bool] = False,
        tolerance: Optional[float] = 1.0e-10,
    ):
        """Given m+1 interpolation points p, determine the m-segment
        Bezier polyline that interpolates these points as a spline.
        This is done by first determining the array of weight points
        which define the spline and then evaluating the cubic
        Bezier segments.

        For a natural spline, the first and last weight points
        are also the first and last interpolation points.
        And, for the initial guess at the remaining weight points,
        just use the supplied data points.
        This amounts to copying the whole p collection.

        References
        ----------
            G. Engelin & F. Uhlig (1996)
            Numerical Algorithms with C
            Springer, Berlin
            Section 12.3.1
        """
        self.points = [Vector3(p) for p in points]
        if closed and (abs(self.points[0] - self.points[-1]) > tolerance):
            self.points.append(Vector3(points[0]))
        self.closed = closed

        m = len(self.points) - 1
        d = [Vector3(p) for p in self.points]

        # Apply Gauss-Seidel iteration until
        # the internal weight points converge.
        for j in range(50):
            max_diff = 0.0
            for i in range(1, m):
                old_p = Vector3(d[i])
                d[i] = 0.25 * (6.0 * self.points[i] - d[i - 1] - d[i + 1])
                max_diff = max(max_diff, abs(d[i] - old_p))
            if max_diff < tolerance:
                break

        # Final stage; calculate the cubic Bezier segments.
        segments = [
            Bezier(
                [
                    self.points[i],
                    (2.0 * d[i] + d[i + 1]) / 3.0,
                    (d[i] + 2.0 * d[i + 1]) / 3.0,
                    self.points[i + 1],
                ]
            )
            for i in range(m)
        ]
        super().__init__(segments, closed, tolerance)

    def __repr__(self):
        return f"Spline(points={self.points}, closed={self.closed})"


class Arc(Path):
    """An arc from point a to point b, with a centre at point c."""

    __slots__ = ["a", "b", "c"]

    def __init__(self, a: Vector3, b: Vector3, c: Vector3):
        self.a = Vector3(a)
        self.b = Vector3(b)
        self.c = Vector3(c)

    def __repr__(self):
        return f"Arc(a={self.a}, b={self.b}, c={self.c})"

    def __call__(self, t: float):
        p, L = self.evaluate_position_and_length(t)
        return p

    def length(self):
        p, L = self.evaluate_position_and_length(1.0)
        return L

    def evaluate_position_and_length(self, t: float):
        l = 0.0
        ca = self.a - self.c
        ca_mag = abs(ca)
        cb = self.b - self.c
        cb_mag = abs(cb)
        if abs(ca_mag - cb_mag) > 1.0e-5:
            raise Exception(f"Arc: radii do not match |ca|={abs(ca)} |cb|={abs(cb)}")

        # First vector in plane.
        tangent1 = Vector3(ca)
        tangent1.normalize()

        # Compute unit normal to plane of all three points
        n = cross_product(ca, cb)
        if abs(n) > 0.0:
            n.normalize()
        else:
            raise Exception(
                "Arc: cannot find plane of three points. Maybe you are trying "
                + "to define an arc with 180 degrees?"
            )

        # Third (orthogonal) vector is in the original plane
        tangent2 = cross_product(n, tangent1)

        # Now transform to local coordinates to do the calculation of the point along
        # the arc in the local xy-plane, with ca along the x-axis
        cb_local = Vector3(cb)
        cb_local.transform_to_local_frame(tangent1, tangent2, n)
        if np.any(np.absolute(cb_local.z) > 1.0e-6):
            raise Exception(f"Arc: problem with transformation cb_local={cb_local}")

        # Angle of the final point on the arc is in the range -pi < th <= +pi
        theta = np.arctan2(cb_local.y, cb_local.x)

        # The length of the circular arc
        l = theta * cb_mag

        # Move the second point around the arc in the local xy-plane
        theta *= t
        loc = Vector3(np.cos(theta) * cb_mag, np.sin(theta) * cb_mag, 0.0 * theta)

        # Transform back to global xyz coordinates and add the centre coordinates
        loc.transform_to_global_frame(tangent1, tangent2, n, self.c)

        return loc, l


class ArcLengthParameterizedPath(Path):
    """A Path reparameterized such that equal increments in t correspond
    to approximately equal increments in arc length.
    """

    __slots__ = ["underlying_path", "arc_lengths", "t_values", "_n"]

    def __init__(self, underlying_path, n=1000):
        if isinstance(underlying_path, Path):
            self.underlying_path = underlying_path
            if n < 1:
                raise RuntimeError("Should have at least one arc-length sample.")
            self._n = n
            self.set_arc_lengths()
        else:
            raise NotImplementedError("underlying_path should be a type of Path")
        return

    def set_arc_lengths(self):
        """
        Compute the arc lengths for a number of sample points along the Path
        (in equally-spaced values of t) so that these can later be used to do
        a reverse interpolation on the evaluation parameter.
        """
        dt = 1.0 / self._n
        L = 0.0
        self.arc_lengths = [
            0.0,
        ]
        self.t_values = [
            0.0,
        ]
        p0 = self.underlying_path(0.0)
        for i in range(1, self._n + 1):
            p1 = self.underlying_path(dt * i)
            L += abs(p1 - p0)
            self.arc_lengths.append(L)
            self.t_values.append(dt * i)
            p0 = p1
        self.t_values = np.array(self.t_values)
        self.arc_lengths = np.array(self.arc_lengths)
        return

    def underlying_t(self, t):
        """
        Search the pieces of arc length to find the piece containing the
        desired point and then interpolate the local value of t for that piece.
        """
        # The incoming parameter value, t, is proportional to arc_length fraction.
        L_target = t * self.arc_lengths[-1]

        # Do a single variable linear interpolation to approximate an ordinary t value
        ut = np.interp(L_target, self.arc_lengths, self.t_values, left=0.0, right=1.0)
        return ut

    def __repr__(self):
        return "ArcLengthParameterizedPath(underlying_path={}, n={})".format(
            self.underlying_path, self._n
        )

    def __call__(self, t):
        return self.underlying_path(self.underlying_t(t))

    def length(self):
        return self.underlying_path.length()


def roberts(eta: float, alpha: float, beta: float):
    """Computes the stretched coordinate in the range [0.0..1.0]
    using the boundary-layer-like transformation devised by Roberts.

    Parameters
    ------------
    eta : float
        unstretched coordinate, 0 <= eta <= 1.0

    beta : float
        stretching factor (more stretching as beta --> 1.0)

    alpha : float
        Location of stretching
            | alpha = 0.5: clustering of nodes at both extremes of eta
            | alpha = 0.0: nodes will be clustered near eta=1.0
    """
    lmbda = (beta + 1.0) / (beta - 1.0)
    lmbda = np.power(lmbda, ((eta - alpha) / (1.0 - alpha)))
    etabar = (beta + 2.0 * alpha) * lmbda - beta + 2.0 * alpha
    etabar = etabar / ((2.0 * alpha + 1.0) * (1.0 + lmbda))
    return etabar


def clustered_roberts(eta: float, end1: bool, end2: bool, beta: float):
    """Compute a stretched ordinate.

    Parameters
    ------------
    eta : float
        Unstretched ordinate, scalar or array.

    end1 : bool
        Cluster flag for end 1. If False, points are not clustered to end 1.
        If True, points ARE clustered to end 1.

    end2 : bool
        Cluster flag for end 2. If False, points are not clustered to end 2.
        If True, points ARE clustered to end 2.

    beta : float
        Grid stretching parameter:
        1 < beta < +inf : points are clustered
        The closer to 1, the more the clustering.
        beta < 1 for no clustering.
    """
    # Decide on stretching parameters for Robert's transform.
    alpha = 0.5
    reverse = False
    cluster = True
    if ((not end1) and (not end2)) or (beta < 1.0):
        cluster = False
    if end1 and end2:
        alpha = 0.5
    if end1 and (not end2):
        reverse = True
        alpha = 0.0
    if (not end1) and end2:
        reverse = False
        alpha = 0.0
    if cluster:
        if reverse:
            eta = 1.0 - eta
        etabar = roberts(eta, alpha, beta)
        if reverse:
            etabar = 1.0 - etabar
    else:
        etabar = eta
    return etabar


def distribute_points(t1: float, t2: float, n: int, end1, end2, beta):
    """Generate a set of n+1 points nonuniformly distributed from t1 to t2.

    Parameters
    -----------
    t1 : float
        Parameter value 1.

    t2 : float
        Parameter value 2.

    n : int
        Number of intervals (the number of points is n+1).

    end1 : bool
        Cluster flag for end 1. If False, points are not clustered to end 1.
        If True, points ARE clustered to end 1.

    end2 : bool
        Cluster flag for end 2. If False, points are not clustered to end 2.
        If True, points ARE clustered to end 2.

    beta : float
        Grid stretching parameter:
        1 < beta < +inf : points are clustered
        The closer to 1, the more the clustering.
        beta < 1 for no clustering.

    Returns the array of n+1 distributed values.
    """
    # Compute the grid points as an array.
    etabar = clustered_roberts(np.linspace(0.0, 1.0, n + 1), end1, end2, beta)

    # Compute the parameter value within the given end-points.
    x = (1.0 - etabar) * t1 + etabar * t2
    return x


class ClusterFunction(ABC):
    @abstractmethod
    def __repr__(self):
        pass

    @abstractmethod
    def __call__(self, x):
        pass

    @abstractmethod
    def distribute_parameter_values(self, nv):
        pass


class LinearFunction(ClusterFunction):
    def __init__(self):
        return

    def __repr__(self):
        return "LinearFunction()"

    def __call__(self, x):
        """
        Simply returns the value, be it scalar or array.
        """
        return x

    def distribute_parameter_values(self, nv):
        """
        Returns an array of uniformly-distributed sample points in range [0.0 ... 1.0].
        """
        return np.linspace(0.0, 1.0, nv)


class RobertsFunction(ClusterFunction):
    """Roberts' boundary-layer-like clustering function.

    Note that, compared with the old definition that we delegate the actual work to,
    the ends are renamed 0, 1 to align with the Eilmer4 notation.
    """

    _slots_ = ["end0", "end1", "beta"]

    def __init__(self, end0, end1, beta):
        """
        Store the cluster parameters for later use.
        """
        self.end0 = end0
        self.end1 = end1
        self.beta = beta
        return

    def __repr__(self):
        return f"RobertsFunction(end0={self.end0}, end1={self.end1}, beta={self.beta})"

    def __call__(self, x):
        return clustered_roberts(x, self.end0, self.end1, self.beta)

    def distribute_parameter_values(self, nv):
        """Returns an array of clustered sample points in range [0.0 ... 1.0]."""
        return distribute_points(0.0, 1.0, nv - 1, self.end0, self.end1, self.beta)
