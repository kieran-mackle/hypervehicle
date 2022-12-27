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


class Component(AbstractComponent):
    def __init__(self, params: dict, verbosity: int = 1) -> None:
        # Set verbosity
        self.verbosity = verbosity

        # Save parameters
        self.params = params

        # Processed objects
        self.patches = {}  # Parametric patches
        self.grids = None  # Structured grids

        # STL Attributes
        self.surfaces = None  # STL surfaces
        self.mesh = None  # STL mesh
        self.stl_resolution = 3
        self.stl_filename = "test"

    def __repr__(self):
        return f"{self.componenttype} component"

    def __str__(self):
        return f"{self.componenttype} component"

    def curve(self):
        # Curvature about the x-axis
        if (
            self.params["FUNC_CURV_X"] is None
            and self.params["FUNC_CURV_X_DASH"] is None
        ):
            if self.verbosity > 0:
                print("    Skipping wing X-curvature.")
        else:
            # (a) Longitudal Curvature
            for key in self.patches:
                self.patches[key] = CurvedPatch(
                    underlying_surf=self.patches[key],
                    direction="x",
                    fun=self.params["FUNC_CURV_X"],
                    fun_dash=self.params["FUNC_CURV_X_DASH"],
                )

        # Curvature about the y-axis
        if (
            self.params["FUNC_CURV_Y"] is None
            and self.params["FUNC_CURV_Y_DASH"] is None
        ):
            if self.verbosity > 0:
                print("    Skipping wing Y-curvature.")
        else:
            # (b) Spanwise Curvature
            for key in self.patches:
                self.patches[key] = CurvedPatch(
                    underlying_surf=self.patches[key],
                    direction="y",
                    fun=self.params["FUNC_CURV_Y"],
                    fun_dash=self.params["FUNC_CURV_Y_DASH"],
                )

    def rotate(self, angle: float = 0, axis: str = "y"):
        for key in self.patches:
            self.patches[key] = RotatedPatch(
                self.patches[key], np.deg2rad(angle), axis=axis
            )

    def reflect(self, axis: str = "y", append: bool = False):

        # Append will add to the existing patches, if False,
        # original patches will be overwritten

        # Reflect the current component patches

        # Create mirrored patches
        mirrored_patches = {}
        for key, patch in self.patches.items():
            mirrored_patches[f"{key}_mirrored"] = MirroredPatch(patch, axis=axis)

        if append:
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

    def surface(self):
        # Initialise surfaces
        self.surfaces = {}

        # Generate surfaces
        for key, patch in self.patches.items():
            flip = True if key.split("_")[-1] == "mirrored" else False
            self.surfaces[key] = parametricSurfce2stl(
                patch, self.stl_resolution, flip_faces=flip
            )

    def to_vtk(self):
        for key, grid in self.grids.items():
            grid.write_to_vtk_file(f"{self.vtk_filename}-wing_{key}.vtk")

    def to_stl(self, outfile: str = "test"):
        # Check for processed surfaces
        if self.surfaces is None:
            if self.verbosity > 1:
                print("Generating surfaces.")

            # Generate surfaces
            self.surface()

        # Combine all surface data
        surface_data = np.concatenate([s[1].data for s in self.surfaces.items()])

        # Create STL mesh
        stl_mesh = mesh.Mesh(surface_data)

        # Save mesh
        self.mesh = stl_mesh

        # Write STL to file
        stl_mesh.save(outfile)
