import os
import numpy as np
from stl import Mesh
from hypervehicle.vehicle import Vehicle
from hypervehicle.hangar import (
    ParametricD21,
    ParametricFalcon9,
    ParametricFinner,
    ParametricHIFiRE4,
    ParametricHIFiRE8,
    ParametricHTV,
    ParametricReFEX,
    ParametricWaverider,
    ParametricWedge,
    ParametricX43,
)


TEST_DATA = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "hangar")


def compare_meshes(prefix: str, vehicle: Vehicle):
    for name, component in vehicle._named_components.items():
        if component.name:
            filename = f"{prefix}-{component.name}.stl"
        else:
            # Construct reference mesh filename
            if "_" in name:
                comp_type, no = name.split("_")
                no = int(no) - 1
                filename = f"{prefix}-{comp_type}-{no}.stl"
            else:
                filename = f"{prefix}-{name}.stl"

        # Load reference mesh
        reference = Mesh.from_file(os.path.join(TEST_DATA, prefix, filename))

        # Compare to generated mesh
        try:
            assert np.all(
                reference.vectors == component.mesh.vectors
            ), f"Bad mesh: {filename}"
        except AssertionError as e:
            matches = (reference.vectors == component.mesh.vectors).flatten().sum()
            total = len((reference.vectors == component.mesh.vectors).flatten())
            print(f"Match rate: {100 * (matches / total):.2f}%")
            raise e


def test_d21():
    generator = ParametricD21()
    vehicle = generator.create_instance()
    vehicle.generate()

    # Test mesh
    compare_meshes(prefix="d21", vehicle=vehicle)


def test_falcon9():
    generator = ParametricFalcon9()
    vehicle = generator.create_instance()
    vehicle.generate()

    # Test mesh
    compare_meshes(prefix="falcon9", vehicle=vehicle)


def test_finner():
    generator = ParametricFinner()
    vehicle = generator.create_instance()
    vehicle.generate()

    # Test mesh
    compare_meshes(prefix="finner", vehicle=vehicle)


def test_hifire4():
    generator = ParametricHIFiRE4()
    vehicle = generator.create_instance()
    vehicle.generate()

    # Test mesh
    compare_meshes(prefix="hifire4", vehicle=vehicle)


def test_hifire8():
    generator = ParametricHIFiRE8()
    vehicle = generator.create_instance()
    vehicle.generate()

    # Test mesh
    compare_meshes(prefix="hifire8", vehicle=vehicle)


def test_htv():
    generator = ParametricHTV()
    vehicle = generator.create_instance()
    vehicle.generate()

    # Test mesh
    compare_meshes(prefix="htv", vehicle=vehicle)


def test_refex():
    generator = ParametricReFEX()
    vehicle = generator.create_instance()
    vehicle.generate()

    # Test mesh
    compare_meshes(prefix="refex", vehicle=vehicle)


def test_waverider():
    generator = ParametricWaverider()
    vehicle = generator.create_instance()
    vehicle.generate()

    # Test mesh
    compare_meshes(prefix="waverider", vehicle=vehicle)


def test_wedge():
    generator = ParametricWedge()
    vehicle = generator.create_instance()
    vehicle.generate()

    # Test mesh
    compare_meshes(prefix="wedge", vehicle=vehicle)


def test_x43():
    generator = ParametricX43()
    vehicle = generator.create_instance()
    vehicle.generate()

    # Test mesh
    compare_meshes(prefix="x43", vehicle=vehicle)
