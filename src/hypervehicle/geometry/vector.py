from __future__ import annotations
import numpy as np
from typing import Union, Optional


class Vector3:
    """A 3-dimensional vector in Cartesian coordinates."""

    PRECISION = 3  # for display purposes only

    def __init__(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
    ):
        """Define a new vector.

        Parameters
        ----------
        x : float, optional
            The x-component of the vector.

        y : float, optional
            The y-component of the vector.

        z : float, optional
            The z-component of the vector.
        """
        # Check input types
        if isinstance(x, Vector3):
            # Vector passed in, inherit coordinates
            self._x = x.x
            self._y = x.y
            self._z = x.z
        elif isinstance(x, (float, int)):
            self._x = x
            self._y = y if y is not None else 0.0
            self._z = z if z is not None else 0.0
        else:
            raise ValueError("Invalid argument type specified.")

    def __str__(self) -> str:
        round_non_none = [
            str(round(i, self.PRECISION))
            for i in [self._x, self._y, self._z]
            if i is not None
        ]
        dimensions = len(round_non_none)
        s = f"{dimensions}-dimensional vector: ({', '.join(round_non_none)})"
        return s

    def __repr__(self) -> str:
        round_non_none = [
            str(round(i, self.PRECISION))
            for i in [self._x, self._y, self._z]
            if i is not None
        ]
        return f"Vector({', '.join(round_non_none)})"

    def __neg__(self):
        """Returns the vector pointing in the opposite direction."""
        return Vector3(x=-1 * self.x, y=-1 * self.y, z=-1 * self.z)

    def __add__(self, other):
        """Element-wise vector addition.

        Parameters
        ----------
        other : Vector
            Another Vector object to be added. This Vector must be of the same
            dimension as the one it is being added to.
        """
        if not isinstance(other, Vector3):
            raise Exception(f"Cannot add a {type(other)} to a vector.")
        return Vector3(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)

    def __sub__(self, other):
        """Element-wise vector subtraction.

        Parameters
        ----------
        other : Vector
            Another Vector object to be added. This Vector must be of the same
            dimension as the one it is being added to.
        """
        if not isinstance(other, Vector3):
            raise Exception(f"Cannot add a {type(other)} to a vector.")
        return Vector3(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)

    def __truediv__(self, denominator: Union[float, int]):
        """Element-wise vector division.

        Parameters
        ----------
        denominator : float | int
            The denominator to use in the division.
        """
        return Vector3(
            x=self.x / denominator, y=self.y / denominator, z=self.z / denominator
        )

    def __mul__(self, multiple: Union[float, int]):
        """Element-wise vector multiplication.

        Parameters
        ----------
        multiple : float | int
            The multiple to use in the multiplication.
        """
        return Vector3(x=self.x * multiple, y=self.y * multiple, z=self.z * multiple)

    def __rmul__(self, other):
        """Multiplication operand for vectors on the right."""
        return self * other

    def __abs__(self):
        """Vector magnitude."""
        return np.sqrt(self.x**2 + self.y**2 + self.z**2)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Vector3):
            raise Exception(f"Cannot compare a {type(other)} to a vector.")
        return (self.x == other.x) & (self.y == other.y) & (self.z == other.z)

    def __imul__(self, other):
        """Returns self *= other number."""
        if isinstance(other, (float, int)) or isinstance(other, np.ndarray):
            self._x *= other
            self._y *= other
            self._z *= other
            return self
        else:
            return NotImplemented

    def __itruediv__(self, other):
        """Returns self /= other number."""
        if isinstance(other, (float, int)) or isinstance(other, np.ndarray):
            self._x /= other
            self._y /= other
            self._z /= other
            return self
        else:
            return NotImplementedError

    def __abs__(self):
        """Returns magnitude."""
        return np.sqrt(self.x**2 + self.y**2 + self.z**2)

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def z(self) -> float:
        return self._z

    @property
    def vec(self) -> np.array:
        """The vector represented as a Numpy array."""
        non_none = [str(i) for i in [self._x, self._y, self._z] if i is not None]

        return np.array([float(i) for i in non_none])

    @property
    def unit(self) -> Vector3:
        """The unit vector associated with the Vector."""
        return self / self.norm

    @property
    def norm(self) -> Vector3:
        """The norm associated with the Vector."""
        return np.linalg.norm(self.vec)

    def normalize(self):
        mag = abs(self)
        self /= mag

    @classmethod
    def from_coordinates(cls, coordinates: np.array) -> Vector3:
        """Constructs a Vector object from an array of coordinates.

        Parameters
        ----------
        coordinates : np.array
            The coordinates of the vector.

        Returns
        -------
        Vector

        Examples
        --------
        >>> Vector.from_coordinates([1,2,3])
        Vector(1, 2, 3)
        """
        return cls(*coordinates)

    def transform_to_global_frame(
        self, n: Vector3, t1: Vector3, t2: Vector3, c: Vector3 = None
    ):
        """Change the coordinates from the local right-handed (RH) system at point c."""
        new_x = self.x * n.x + self.y * t1.x + self.z * t2.x
        new_y = self.x * n.y + self.y * t1.y + self.y * t2.y
        new_z = self.x * n.z + self.y * t1.z + self.z * t2.z
        if c is not None:
            new_x += c.x
            new_y += c.y
            new_z += c.z
        self._x = new_x
        self._y = new_y
        self._z = new_z
        return self

    def transform_to_local_frame(
        self, n: Vector3, t1: Vector3, t2: Vector3, c: Vector3 = None
    ):
        """Change coordinates into the local right-handed (RH) system at point c."""
        if c is not None:
            self -= c

        x = self.dot(n)
        y = self.dot(t1)
        z = self.dot(t2)

        self._x = x
        self._y = y
        self._z = z
        return self

    def dot(self, other: Vector3):
        """Returns dot product of self with another Vector3 object."""
        if isinstance(other, Vector3):
            return self.x * other.x + self.y * other.y + self.z * other.z
        else:
            raise Exception(f"dot() not implemented for {type(other)}")


def approximately_equal_vectors(
    v1: Vector3,
    v2: Vector3,
    rel_tol: Optional[float] = 1.0e-2,
    abs_tol: Optional[float] = 1.0e-5,
):
    """Returns a boolean indicating whether v1 and v2 are approximately equal."""
    return np.all(
        [
            np.isclose(v1.x, v2.x, rtol=rel_tol, atol=abs_tol),
            np.isclose(v1.y, v2.y, rtol=rel_tol, atol=abs_tol),
            np.isclose(v1.z, v2.z, rtol=rel_tol, atol=abs_tol),
        ]
    )


def cross_product(v1: Vector3, v2: Vector3):
    """Returns the cross product between two vectors."""
    x = v1.y * v2.z - v2.y * v1.z
    y = v2.x * v1.z - v1.x * v2.z
    z = v1.x * v2.y - v2.x * v1.y
    return Vector3(x, y, z)
