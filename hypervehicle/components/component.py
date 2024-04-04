import os
import time
import pymeshfix
import numpy as np
from stl import mesh
import multiprocess as mp
from copy import deepcopy
from abc import abstractmethod
from hypervehicle.geometry import Vector3
from gdtk.geom.sgrid import StructuredGrid
from typing import Callable, Union, Optional
from hypervehicle.geometry import (
    CurvedPatch,
    RotatedPatch,
    MirroredPatch,
    OffsetPatchFunction,
)
from hypervehicle.utilities import parametricSurfce2stl


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
        """Creates the discretised surface data from the
        parametric patches."""
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


class Component(AbstractComponent):
    def __init__(
        self,
        params: dict = None,
        edges: list =[],
        stl_resolution: int = 2,
        verbosity: int = 1,
        name: str = None,
    ) -> None:
        # Set verbosity
        self.verbosity = verbosity

        # Save parameters
        self.params = params

        # Save edges
        self.edges = edges

        # Processed objects
        self.patches = {}  # Parametric patches (continuous)

        # VTK Attributes
        self.grids = None  # Structured grids

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

        # Modifier function
        self._modifier_function = None

        # Component reflection
        self._reflection_axis = None
        self._append_reflection = True

        # Ghost component
        self._ghost = False

        # Component name
        self.name = name

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
        if not self._mesh:
            # Check for processed surfaces
            if self.surfaces is None:
                if self.verbosity > 1:
                    print(" Generating surfaces for component.")

                # Generate surfaces
                self.surface()

            # Combine all surface data
            surface_data = np.concatenate([s[1].data for s in self.surfaces.items()])

            # Create nominal STL mesh
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
            offset_function = lambda x, y, z: offset
        else:
            offset_function = offset

        # Could wrap it in a lambda if provided
        for key, patch in self.patches.items():
            self.patches[key] = OffsetPatchFunction(patch, offset_function)

    def transform(self):
        for transform in self._transformations:
            func = getattr(self, transform[0])
            func(*transform[1:])

    def apply_modifier(self):
        if self._modifier_function:
            for key, patch in self.patches.items():
                self.patches[key] = OffsetPatchFunction(patch, self._modifier_function)

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

            #if "swept" in key:
            #    # Swept fuselage component
            #    res = (
            #        int(stl_resolution / 4)
            #        if "end" in key
            #        else int(stl_resolution / 4) * 4
            #    )
            #    flip = True if "1" in key else False

            surface = parametricSurfce2stl(
                patch, res, flip_faces=flip, **self._clustering
            )
            return (key, surface)

        multiprocess = True  # flag to disable multiprocessing for debugging
        self.surfaces = {}
        if multiprocess is True:
            # Initialise surfaces and pool
            pool = mp.Pool()

            # Submit tasks
            print(f"START: Creating stl - multiprocessor run.")
            for result in pool.starmap(wrapper, self.patches.items()):
                self.surfaces[result[0]] = result[1]
            print("  DONE: Creating stl - multiprocess.")
        else:
            for case in self.patches.items():
                k = case[0]
                pat = case[1]
                print(f"START: Creating stl for '{k}'.")
                result = wrapper(k, pat)
                self.surfaces[result[0]] = result[1]
                print("  DONE: Creating stl.")

    def to_vtk(self):
        raise NotImplementedError("This method has not been implemented yet.")
        # TODO - check for processed grids
        for key, grid in self.grids.items():
            grid.write_to_vtk_file(f"{self.vtk_filename}-wing_{key}.vtk")

    def to_stl(self, outfile: str = None):
        if not self._ghost:
            if self.verbosity > 1:
                print("Writing patches to STL format. ")
                if outfile is not None:
                    print(f"Output file = {outfile}.")

            # Get mesh
            stl_mesh = self.mesh

            if outfile is not None:
                # Write STL to file
                stl_mesh.save(outfile)

                # Clean it
                pymeshfix.clean_from_file(outfile, outfile)

    def analyse(self):
        # Get mass properties
        volume, cog, inertia = self.mesh.get_mass_properties()

        # Print results
        print(f"Volume: {volume} m^3")
        print(f"COG location: {cog}")
        print("Moment of intertia metrix at COG:")
        print(inertia)

    def add_clustering_options(
        self,
        i_clustering_func: Optional[Callable] = None,
        j_clustering_func: Optional[Callable] = None,
    ):
        """Add a clustering option to this component.

        Parameters
        -----------
        i_clustering_func : Callable, optional
            The clustering function in the i direction. The default is None.

        j_clustering_func : Callable, optional
            The clustering function in the j direction. The default is None.
        """
        if i_clustering_func:
            self._clustering.update({"i_clustering_func": i_clustering_func})

        if j_clustering_func:
            self._clustering.update({"j_clustering_func": j_clustering_func})

    def stl_check(self, projecteArea=True, matchingLines=True):
        """Check stl mesh."""
        pass_flag = True
        mesh = self.mesh
        print(f"    MESH CHECK")
        print(f"        mesh:{mesh}")
        if projecteArea:
            print(f"    Project Area Check")
            normals = np.asarray(mesh.normals, dtype=np.float64)
            allowed_max_errors = (
                np.abs(normals).sum(axis=0) * np.finfo(np.float32).eps
                )
            sum_normals = normals.sum(axis=0)
            print(f"        allowed error: {allowed_max_errors}")
            print(f"        normals sum: {sum_normals}")
            if ((np.abs(sum_normals)) <= allowed_max_errors).all():
                print(f"      PASS")
            else:
                print(f"      FAIL")
                pass_flag = False
        if matchingLines:
            print(f"    Matching Lines Check")
            reversed_triangles = (np.cross(mesh.v1 - mesh.v0,
                                        mesh.v2 - mesh.v0) * mesh.normals
                                    ).sum(axis=1) < 0
            import itertools
            directed_edges = {
                tuple(edge.ravel() if not rev else edge[::-1, :].ravel())
                for rev, edge in zip(
                    itertools.cycle(reversed_triangles),
                    itertools.chain(
                        mesh.vectors[:, (0, 1), :],
                        mesh.vectors[:, (1, 2), :],
                        mesh.vectors[:, (2, 0), :],
                        ),
                    )
                }
        undirected_edges = {frozenset((edge[:3], edge[3:])) for edge in
                            directed_edges}
        edge_check = len(directed_edges) == 3 * mesh.data.size
        print(f"        len(directed_edges) == 3 * mesh.data.size:{edge_check}")
        if edge_check:
            print(f"      PASS")
        else:
            print(f"      FAIL")
            print(f"        The number of edges should be N_cells*3")
            pass_flag = False
        pair_check = len(directed_edges) == 2 * len(undirected_edges)
        print(f"        len(directed_edges) == 2 * len(undirected_edges):{pair_check}")
        if pair_check:
            print(f"      PASS")
            return pass_flag
        else:
            print(f"      FAIL")
            print(f"        All edges should be in pairs")
            print(f"        len_directed:{len(directed_edges)}")
            print(f"        len_undirect (pairs+leftover):{len(undirected_edges)}")
            pass_flag = False

        edge_list = []
        for edge in directed_edges:
            edge_list.append( frozenset((edge[:3], edge[3:])))
        # print(f"edge_list={edge_list}")

        from collections import Counter
        counter_out = Counter(edge_list)
        #print(f"counter_out={counter_out}")

        k_list = []
        for k, v in counter_out.items():
            if v == 2:
                k_list.append(k)
        for k in k_list:
            del counter_out[k]

        return pass_flag

    def stl_repair(self, small_distance: float=1e-6):
        """Attempts to repair stl mesh issues.

        Parameters
        -----------
        small_distance : Float, optional
            Vectors less than this distance apart will be set to the same value.
        """
        mesh = self.mesh
        N_faces = np.shape(mesh)[0]
        vectors = np.vstack((mesh.v0, mesh.v1, mesh.v2))

        def find_groups(vector, small_distance):
            """find indices for groups of points that within small_distance from one another."""
            sort_i = np.argsort(vector)
            groups = []
            li = []
            count = 0
            for i in range(len(vector)-1):
                if count == 0:
                    x0 = vector[sort_i[i]]
                x1 = vector[sort_i[i+1]]
                if abs(x1-x0) < small_distance:
                    if count == 0:
                        li.append(sort_i[i])
                        li.append(sort_i[i+1])
                        count = 1
                    else:
                        li.append(sort_i[i+1])
                else:
                    groups.append(li)
                    li = []
                    count = 0
                if i == len(vector)-2 and count > 0:
                    groups.append(li)
            return groups
        iix = find_groups(vectors[:,0], small_distance)
        iiy = find_groups(vectors[:,1], small_distance)
        iiz = find_groups(vectors[:,2], small_distance)

        # find intersecting sets and take average
        for ix in iix:
            for iy in iiy:
                for iz in iiz:
                    common = list(set(ix) & set(iy) & set(iz))
                    if common:
                        vectors[common] = np.mean(vectors[common], axis=0)
        mesh.v0 = vectors[0:N_faces,:]
        mesh.v1 = vectors[N_faces:2*N_faces,:]
        mesh.v2 = vectors[2*N_faces:3*N_faces,:]
