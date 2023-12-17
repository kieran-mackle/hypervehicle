import meshio
import numpy as np
from stl import mesh
import multiprocess as mp
from copy import deepcopy
from abc import abstractmethod
from typing import Callable, Union
from hypervehicle.geometry import Vector3
from gdtk.geom.sgrid import StructuredGrid
from hypervehicle.geometry import (
    CurvedPatch,
    RotatedPatch,
    MirroredPatch,
    OffsetPatchFunction,
)
from hypervehicle.utilities import parametricSurfce2stl, parametricSurfce2vtk


class AbstractComponent:
    componenttype = None

    @abstractmethod
    def __init__(self, params: dict, verbosity: int = 1) -> None:
        pass

    @abstractmethod
    def __repr__(self):
        pass

    @abstractmethod
    def __str__(self):
        pass

    @property
    @abstractmethod
    def componenttype(self):
        # This is a placeholder for a class variable defining the component type
        pass

    @abstractmethod
    def generate_patches(self):
        """Generates the parametric patches from the parameter dictionary."""
        pass

    @abstractmethod
    def curve(self):
        """Applies a curvature function to the parametric patches."""
        pass

    @abstractmethod
    def rotate(self, angle: float = 0, axis: str = "y"):
        """Rotates the parametric patches."""
        pass

    @abstractmethod
    def reflect(self):
        """Reflects the parametric patches."""
        pass

    @abstractmethod
    def grid(self):
        """Creates a discrete grid from the parametric patches."""
        pass

    @abstractmethod
    def surface(self):
        """Creates a surface from the parametric patches."""
        pass

    @abstractmethod
    def to_vtk(self):
        """Writes the component to VTK file format."""
        pass

    @abstractmethod
    def to_stl(self):
        """Writes the component to STL file format."""
        pass

    @abstractmethod
    def analyse(self):
        """Evaluates properties of the STL mesh."""
        pass


def AssignTags2Celle(patch, length):
    # Creates a tag vector for a given patch

    tags_definition = {"FreeStream": 1, "Inlet": 2, "Outlet": 3, "Nozzle": 4}

    tags = np.ones(length) * tags_definition[patch.tag]
    return tags


class Component(AbstractComponent):
    def __init__(
        self,
        params: dict = None,
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
        output_file_type: str = "stl",
    ) -> None:
        # Set verbosity
        self.verbosity = verbosity

        # Save parameters
        self.params = params

        # Processed objects
        self.patches = {}  # Parametric patches (continuous)

        # VTK Attributes
        self.cells = None  # Mesh cells

        # STL Attributes
        self.surfaces = None  # STL surfaces for each patch
        self.stl_resolution = stl_resolution  # STL cells per edge
        self._mesh = None  # STL mesh for entire component

        # Curvature functions
        self._curvatures = None

        # Clustering
        self._clustering = {}

        # Transformations
        self._transformations = []

        # Component reflection
        self._reflection_axis = None
        self._append_reflection = True

        # Component name
        self.name = name

        # Output file type
        self.output_file_type = output_file_type

    def __repr__(self):
        s = f"{self.componenttype} component"
        if self.name:
            s += f" (tagged '{self.name}')"

        return s

    def __str__(self):
        if self.name:
            return self.name
        else:
            return f"{self.componenttype} component"

    def __copy__(self):
        cls = self.__class__
        result = cls.__new__(cls)
        result.__dict__.update(self.__dict__)
        return result

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, deepcopy(v, memo))
        return result

    def curve(self):
        if self._curvatures is not None:
            for curvature in self._curvatures:
                for key, patch in self.patches.items():
                    self.patches[key] = CurvedPatch(
                        underlying_surf=patch,
                        direction=curvature[0],
                        fun=curvature[1],
                        fun_dash=curvature[2],
                    )

    @property
    def mesh(self):
        # Check for processed surfaces
        if self.surfaces is None:
            if self.verbosity > 1:
                print(" Generating surfaces for component.")

            # Generate surfaces
            self.surface()

        # Combine all surface data
        surface_data = np.concatenate([s[1].data for s in self.surfaces.items()])

        # Create STL mesh
        self._mesh = mesh.Mesh(surface_data)

        return self._mesh

    @mesh.setter
    def mesh(self, value):
        self._mesh = value

    def rotate(self, angle: float = 0, axis: str = "y"):
        for key, patch in self.patches.items():
            self.patches[key] = RotatedPatch(patch, np.deg2rad(angle), axis=axis)

    def translate(self, offset: Union[Callable, Vector3]):
        if isinstance(offset, Vector3):
            # Translate vector into lambda function
            def offset_function(x, y, z):
                return offset

        else:
            offset_function = offset

        # Could wrap it in a lambda if provided
        for key, patch in self.patches.items():
            self.patches[key] = OffsetPatchFunction(patch, offset_function)

    def transform(self):
        for transform in self._transformations:
            func = getattr(self, transform[0])
            func(*transform[1:])

    def reflect(self, axis: str = None):
        axis = self._reflection_axis if self._reflection_axis is not None else axis
        if axis is not None:
            # Create mirrored patches
            mirrored_patches = {}
            for key, patch in self.patches.items():
                mirrored_patches[f"{key}_mirrored"] = MirroredPatch(patch, axis=axis)

            if self._append_reflection:
                # Append mirrored patches to original patches
                for key, patch in mirrored_patches.items():
                    self.patches[key] = patch
            else:
                # Overwrite existing patches
                self.patches = mirrored_patches

    def grid(self):
        for key in self.patches:
            self.grids[key] = StructuredGrid(
                psurf=self.patches[key],
                niv=self.vtk_resolution,
                njv=self.vtk_resolution,
            )

    def surface(self, resolution: int = None):
        stl_resolution = self.stl_resolution if resolution is None else resolution

        # Check for patches
        if len(self.patches) == 0:
            raise Exception(
                "No patches have been generated. " + "Please call .generate_patches()."
            )

        # Prepare multiprocessing arguments iterable
        def wrapper(key: str, patch):
            flip = True if key.split("_")[-1] == "mirrored" else False
            res = stl_resolution

            if "swept" in key:
                # Swept fuselage component
                res = (
                    int(stl_resolution / 4)
                    if "end" in key
                    else int(stl_resolution / 4) * 4
                )
                flip = True if "1" in key else False

            surface = parametricSurfce2stl(
                patch, res, flip_faces=flip, **self._clustering
            )

            return (key, surface)

        # Initialise surfaces and pool
        self.surfaces = {}
        pool = mp.Pool()

        # Submit tasks single (debug mode)
        # for a in self.patches.items():
        #     result = wrapper(a[0], a[1])
        #     self.surfaces[result[0]] = result[1]

        # Submit tasks multi
        for result in pool.starmap(wrapper, self.patches.items()):
            self.surfaces[result[0]] = result[1]

    def cell(self, resolution: int = None):
        stl_resolution = self.stl_resolution if resolution is None else resolution

        # Check for patches
        if len(self.patches) == 0:
            raise Exception(
                "No patches have been generated. " + "Please call .generate_patches()."
            )

        # Prepare multiprocessing arguments iterable
        def wrapper(key: str, patch):
            flip = True if key.split("_")[-1] == "mirrored" else False
            res = stl_resolution

            if "swept" in key:
                # Swept fuselage component
                res = (
                    int(stl_resolution / 4)
                    if "end" in key
                    else int(stl_resolution / 4) * 4
                )
                flip = True if "1" in key else False

            vertices, cell_ids = parametricSurfce2vtk(
                patch, res, flip_faces=flip, **self._clustering
            )

            # Assign tags to the cells
            tags = AssignTags2Celle(patch, len(cell_ids))

            return (key, vertices, cell_ids, tags)

        # Initialise cells and pool
        self.cells = {}
        pool = mp.Pool()

        # Submit tasks single (debug mode)
        # for a in self.patches.items():
        #     result = wrapper(a[0], a[1])
        #     self.cells[result[0]] = (result[1], result[2], result[3])

        # Submit tasks multi
        for result in pool.starmap(wrapper, self.patches.items()):
            self.cells[result[0]] = (result[1], result[2], result[3])

    def to_stl(self, outfile: str = None):
        if self.verbosity > 1:
            print("Writing patches to STL format. ")
            if outfile is not None:
                print(f"Output file = {outfile}.")

        # Get mesh
        stl_mesh = self.mesh

        if outfile is not None:
            # Write STL to file
            stl_mesh.save(outfile)

    def to_vtk(self, outfile: str = None):
        if self.verbosity > 1:
            print("Writing patches to VTK format. ")
            if outfile is not None:
                print(f"Output file = {outfile}.")

        if self.verbosity > 1:
            print(" Generating cells for component.")

        # Generate cells
        self.cell()

        vertices = np.empty((0, 3))
        cell_ids = np.empty((0, 3), dtype=int)
        tags = np.empty(0, dtype=int)

        # Combine all Cell data
        for sss in self.cells.items():
            cell_ids = np.concatenate([cell_ids, sss[1][1] + len(vertices)])
            vertices = np.concatenate([vertices, sss[1][0]])
            tags = np.concatenate([tags, sss[1][2]])

        # Generate mesh in VTK format
        cell_ids = [("triangle", cell_ids)]
        cell_data = {"tag": [tags]}
        vtk_mesh = meshio.Mesh(vertices, cell_ids, cell_data=cell_data)

        # Write VTK to file
        if outfile is not None:
            vtk_mesh.write(outfile)

    def analyse(self):
        # Get mass properties
        volume, cog, inertia = self.mesh.get_mass_properties()

        # Print results
        print(f"Volume: {volume} m^3")
        print(f"COG location: {cog}")
        print("Moment of intertia metrix at COG:")
        print(inertia)
