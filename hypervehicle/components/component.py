import numpy as np
from stl import mesh
from abc import ABC, abstractmethod
from gdtk.geom.sgrid import StructuredGrid
from hypervehicle.geometry import (
    CurvedPatch,
    RotatedPatch,
    MirroredPatch,
)
from hypervehicle.utilities import parametricSurfce2stl


class AbstractComponent(ABC):
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


# TODO - add general attributes, such as curvature functions, etc

# TODO - need to document adding specs (curve, rotate, etc) to
# individual components, handle implied stuff in Vehicle class


class Component(AbstractComponent):
    def __init__(self, params: dict, verbosity: int = 1) -> None:
        # Set verbosity
        self.verbosity = verbosity

        # Save parameters
        self.params = params

        # Processed objects
        self.patches = {}  # Parametric patches (continuous)

        # VTK Attributes
        self.grids = None  # Structured grids

        # STL Attributes
        self.surfaces = None  # STL surfaces for each patch
        self.mesh = None  # STL mesh for entire component
        self.stl_resolution = 3  # STL cells per edge

        # TODO - tidy the below
        self.x_curv_func = None
        self.x_curv_func_dash = None
        self.y_curv_func = None
        self.y_curv_func_dash = None

        # TODO - nice way to handle component transformations
        self._reflect = False
        self._append_reflection = True

    def __repr__(self):
        return f"{self.componenttype} component"

    def __str__(self):
        return f"{self.componenttype} component"

    def curve(self):

        # TODO - generalise curvature, similar to transformations,
        # with stacked functions and axes

        # Curvature about the x-axis
        if self.x_curv_func is not None:
            # Longitudal Curvature
            for key, patch in self.patches.items():
                self.patches[key] = CurvedPatch(
                    underlying_surf=patch,
                    direction="x",
                    fun=self.x_curv_func,
                    fun_dash=self.x_curv_func_dash,
                )

        # Curvature about the y-axis
        if self.y_curv_func is not None:
            # Spanwise Curvature
            for key, patch in self.patches.items():
                self.patches[key] = CurvedPatch(
                    underlying_surf=patch,
                    direction="y",
                    fun=self.y_curv_func,
                    fun_dash=self.y_curv_func_dash,
                )

    def rotate(self, angle: float = 0, axis: str = "y"):
        for key, patch in self.patches.items():
            self.patches[key] = RotatedPatch(patch, np.deg2rad(angle), axis=axis)

    def reflect(self, axis: str = "y"):
        if self._reflect:
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

        resolution = self.stl_resolution if resolution is None else resolution

        # Check for patches
        if len(self.patches) == 0:
            raise Exception(
                "No patches have been generated. " + "Please call .generate_patches()."
            )

        # Initialise surfaces
        self.surfaces = {}

        # Generate surfaces
        for key, patch in self.patches.items():
            flip = True if key.split("_")[-1] == "mirrored" else False
            self.surfaces[key] = parametricSurfce2stl(
                patch, resolution, flip_faces=flip
            )

    def to_vtk(self):
        # TODO - check for processed grids
        for key, grid in self.grids.items():
            grid.write_to_vtk_file(f"{self.vtk_filename}-wing_{key}.vtk")

    def to_stl(self, outfile: str = "test.stl", stl_resolution: int = None):
        if self.verbosity > 1:
            print(f"Writing {outfile}.")

        # Check for processed surfaces
        if self.surfaces is None or stl_resolution is not None:
            if self.verbosity > 1:
                print(" Generating surfaces for component.")

            # Generate surfaces
            self.surface(resolution=stl_resolution)

        # Combine all surface data
        surface_data = np.concatenate([s[1].data for s in self.surfaces.items()])

        # Create STL mesh
        stl_mesh = mesh.Mesh(surface_data)

        # Save mesh
        self.mesh = stl_mesh

        # Write STL to file
        stl_mesh.save(outfile)
