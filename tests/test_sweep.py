import os
import numpy as np
from stl import Mesh
from hypervehicle.utilities import surfce_to_stl
from hypervehicle.geometry import SweptPatch, CoonsPatch, Vector3


def test_sweep():
    # Define cross section 1
    p00 = Vector3(0, 0)
    p01 = Vector3(0, 1)
    p11 = Vector3(1, 1)
    p10 = Vector3(1, 0)
    c1 = CoonsPatch(p00=p00, p01=p01, p11=p11, p10=p10)

    # Define cross section 2
    z2 = 1
    p002 = Vector3(0, 0, z2)
    p012 = Vector3(0, 0.5, z2)
    p112 = Vector3(0.5, 0.5, z2)
    p102 = Vector3(0.5, 0, z2)
    c2 = CoonsPatch(p00=p002, p01=p012, p11=p112, p10=p102)

    # Create swept patch
    sections = [c1, c2]
    p = SweptPatch(sections)
    s = surfce_to_stl(p, 20, 20)
    s1 = surfce_to_stl(c1, 20, 20)
    s2 = surfce_to_stl(c2, 20, 20)

    # Get data and mesh
    stl_data = [s.data, s1.data, s2.data]
    m = Mesh(np.concatenate(stl_data))

    # Compare output to reference stl
    tests_dir = os.path.dirname(os.path.realpath(__file__))
    reference = Mesh.from_file(os.path.join(tests_dir, "data", "swept.stl"))

    assert np.all(reference.vectors == m.vectors)
