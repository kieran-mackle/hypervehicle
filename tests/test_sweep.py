import os
import numpy as np
from stl import Mesh
from gdtk.geom.surface import CoonsPatch, Vector3
from hypervehicle.utils import SweptPatch, parametricSurfce2stl


def test_sweep():
    p00 = Vector3(0, 0)
    p01 = Vector3(0, 1)
    p11 = Vector3(1, 1)
    p10 = Vector3(1, 0)

    c1 = CoonsPatch(p00=p00, p01=p01, p11=p11, p10=p10)

    z2 = 1
    p002 = Vector3(0, 0, z2)
    p012 = Vector3(0, 0.5, z2)
    p112 = Vector3(0.5, 0.5, z2)
    p102 = Vector3(0.5, 0, z2)

    c2 = CoonsPatch(p00=p002, p01=p012, p11=p112, p10=p102)

    sections = [c1, c2]
    section_origins = [0, 1]

    p = SweptPatch(sections)

    s = parametricSurfce2stl(p, 20)
    s1 = parametricSurfce2stl(c1, 20)
    s2 = parametricSurfce2stl(c2, 20)

    stl_data = [s.data, s1.data, s2.data]
    m = Mesh(np.concatenate(stl_data))

    # m.save("swept.stl")

    # Compare output to reference stl
    tests_dir = os.path.dirname(os.path.realpath(__file__))
    reference = Mesh.from_file(os.path.join(tests_dir, "data", "swept.stl"))

    assert np.all(reference.vectors == m.vectors)
