import pytest
from hypervehicle.geometry.vector import Vector3


def test_vector():
    # Test 1D vector
    v = Vector3(1.0)
    assert v.x == 1.0
    assert v.y == 0.0
    assert v.z == 0.0

    # Test 3D vector
    v1 = Vector3(1, 2, 3)
    assert v1.x == 1.0
    assert v1.y == 2.0
    assert v1.z == 3.0

    # Test instantiation from existing vector
    v2 = Vector3(v1)

    # Test equality check
    assert v1 == v2

    # Test inplace normalisation
    assert v1.norm > 1
    v1.normalize()
    assert v1.norm == 1


def test_invalid():
    with pytest.raises(ValueError) as e_info:
        v = Vector3()
