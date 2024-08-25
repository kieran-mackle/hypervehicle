import pytest
from hypervehicle.geometry.vector import Vector3
from hypervehicle.geometry.surface import CoonsPatch


def test_coons_patch():
    # Test instantiation by corners
    p00 = Vector3(0, 0, 0)
    p10 = Vector3(1, 0, 0)
    p01 = Vector3(0, 1, 0)
    p11 = Vector3(1, 1, 0)
    patch = CoonsPatch(p00=p00, p10=p10, p11=p11, p01=p01)

    assert patch.defined_by_corners == True

    # TODO - Test instantiation by edges

    # Test instantiation by mix of corners and edges
    with pytest.raises(Exception) as e_info:
        CoonsPatch()

    # Test interpolation
    assert patch(0, 0) == p00
    assert patch(1, 1) == p11
