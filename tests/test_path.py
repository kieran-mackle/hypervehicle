import pytest
import numpy as np
from hypervehicle.geometry.vector import Vector3
from hypervehicle.geometry.path import Line, Polyline, Arc
from hypervehicle.geometry.vector import approximately_equal_vectors


def test_line():
    # Create line
    p0 = Vector3(0, 0, 0)
    p1 = Vector3(2, 2, 2)
    line = Line(p0=p0, p1=p1)

    # Test attributes
    assert line.p0 == p0
    assert line.p1 == p1
    assert line.length() == (2**2 + 2**2 + 2**2) ** 0.5

    # Test call
    assert line(0.0) == p0
    assert line(0.5) == Vector3(1, 1, 1)
    assert line(1.0) == p1

    # Test adding a vector to a line
    shifted_line = line + p1
    assert shifted_line.p0 == p0 + p1
    assert shifted_line.p1 == p1 + p1


def test_polyline():
    # Create lines
    p0 = Vector3(0, 0, 0)
    p1 = Vector3(2, 2, 2)
    p2 = Vector3(4, 4, 4)
    line1 = Line(p0=p0, p1=p1)
    line2 = Line(p0=p1, p1=p2)

    # Create polyline
    pl = Polyline([line1, line2])

    # Test
    assert pl.length() == line1.length() + line2.length()
    assert pl(0) == p0
    assert pl(0.5) == p1
    assert pl(1.0) == p2


def test_arc():
    # Create arc
    a = Vector3(-1, 0)
    b = Vector3(0, 1)
    c = Vector3(0, 0)
    arc = Arc(a, b, c)

    # Test
    assert approximately_equal_vectors(arc(1), b)
    assert approximately_equal_vectors(arc(1), b)

    # Create a bad arc, eg. with different radii
    a = Vector3(-1.33, -0.225, 0)
    b = Vector3(-1.33, -0.113, 0.195)
    c = Vector3(-1.33, 0, 0)
    arc = Arc(a, b, c)

    with pytest.raises(Exception) as e_info:
        arc(0.0)
