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

        self._non_none = [str(i) for i in [self._x, self._y, self._z] if i is not None]
        self._round_non_none = [
            str(round(i, self.PRECISION))
            for i in [self._x, self._y, self._z]
            if i is not None
        ]
        self._dimensions = len(self._non_none)

    def __str__(self) -> str:
        s = f"{self._dimensions}-dimensional vector: ({', '.join(self._round_non_none)})"
        return s

    def __repr__(self) -> str:
        return f"Vector({', '.join(self._round_non_none)})"

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
        return np.array([float(i) for i in self._non_none])

    @property
    def unit(self) -> Vector3:
        """The unit vector associated with the Vector."""
        return self / self.norm

    @property
    def norm(self) -> Vector3:
        """The norm associated with the Vector."""
        return np.linalg.norm(self.vec)

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
