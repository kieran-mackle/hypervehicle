import os
import sys
import glob
import numpy as np
import pandas as pd
from stl import mesh
from tqdm import tqdm
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional


def create_cells(
    parametric_surface,
    triangles_per_edge: int,
    si: float = 1.0,
    sj: float = 1.0,
    mirror_y=False,
    flip_faces=False,
):
    """
    Generates a list of vertices and a corrosponding list of index triplets,
    each pinting the vertices of a single cell

    Parameters
    ----------
    parametric_surface : Any
        The parametric surface object.

    si : float, optional
        The clustering in the i-direction. The default is 1.0.

    sj : float, optional
        The clustering in the j-direction. The default is 1.0.

    triangles_per_edge : int
        The resolution for the stl object.

    mirror_y : bool, optional
        Create mirror image about x-z plane. The default is False.

    Returns
    ----------
    vertices: numpy 2D array, each row contains (x,y,z) coordinates of one vertex
    cell_ids: numpy 2D array, each row contains 3 indecies (row number in vertices)
    of a single cell

    """
    # TODO - allow different ni and nj discretisation

    ni = triangles_per_edge
    nj = triangles_per_edge

    def gen_points(lb, ub, steps, spacing=1.0):
        span = ub - lb
        dx = 1.0 / (steps - 1)
        return np.array([lb + (i * dx) ** spacing * span for i in range(steps)])

    # Create list of vertices
    r_list = gen_points(lb=0.0, ub=1.0, steps=ni + 1, spacing=si)
    s_list = gen_points(lb=0.0, ub=1.0, steps=nj + 1, spacing=sj)

    y_mult: int = -1 if mirror_y else 1

    # Create vertices for corner points of each quad cell
    # columns x, y, z for each vertex row
    # quad exterior vertices + quad centres
    vertices: np.ndarray = np.zeros(((ni + 1) * (nj + 1) + ni * nj, 3))
    centre_ix: int = (ni + 1) * (nj + 1)

    # For vertices along the x direction (i)
    for i, r in enumerate(r_list):
        # For vertices along the y direction (j)
        for j, s in enumerate(s_list):
            # Evaluate position
            pos = parametric_surface(r, s)

            # Assign vertex
            vertices[j * (ni + 1) + i] = np.array([pos.x, y_mult * pos.y, pos.z])

            # Create vertices for centre point of each quad cell
            try:
                # Try index to bail before calling surface
                vertices[centre_ix + (j * ni + i)]

                r0 = r_list[i]
                r1 = r_list[i + 1]
                s0 = s_list[j]
                s1 = s_list[j + 1]

                # Get corner points
                pos00 = parametric_surface(r0, s0)
                pos10 = parametric_surface(r1, s0)
                pos01 = parametric_surface(r0, s1)
                pos11 = parametric_surface(r1, s1)

                # Evaluate quad centre coordinate
                pos_x = 0.25 * (pos00.x + pos10.x + pos01.x + pos11.x)
                pos_y = 0.25 * (pos00.y + pos10.y + pos01.y + pos11.y)
                pos_z = 0.25 * (pos00.z + pos10.z + pos01.z + pos11.z)

                # Assign quad centre vertices
                vc = np.array([pos_x, y_mult * pos_y, pos_z])
                vertices[centre_ix + (j * ni + i)] = vc

            except IndexError:
                # Index out of bounds
                pass

    # Create list of cell_ids, defining the face vertices
    cell_ids = []
    for i in range(ni):
        for j in range(nj):
            p00 = j * (nj + 1) + i  # bottom left
            p10 = j * (nj + 1) + i + 1  # bottom right
            p01 = (j + 1) * (ni + 1) + i  # top left
            p11 = (j + 1) * (ni + 1) + i + 1  # top right

            pc = centre_ix + j * min(ni, nj) + i  # vertex at centre of cell

            if mirror_y or flip_faces:
                cell_ids.append([p00, pc, p10])
                cell_ids.append([p10, pc, p11])
                cell_ids.append([p11, pc, p01])
                cell_ids.append([p01, pc, p00])
            else:
                cell_ids.append([p00, p10, pc])
                cell_ids.append([p10, p11, pc])
                cell_ids.append([p11, p01, pc])
                cell_ids.append([p01, p00, pc])

    cell_ids = np.array(cell_ids)

    return (vertices, cell_ids)


def parametricSurfce2stl(
    parametric_surface,
    triangles_per_edge: int,
    si: float = 1.0,
    sj: float = 1.0,
    mirror_y=False,
    flip_faces=False,
) -> mesh.Mesh:
    """
    Function to convert parametric_surface generated using the Eilmer Geometry
    Package into a stl mesh object.

    Parameters
    ----------
    parametric_surface : Any
        The parametric surface object.

    si : float, optional
        The clustering in the i-direction. The default is 1.0.

    sj : float, optional
        The clustering in the j-direction. The default is 1.0.

    triangles_per_edge : int
        The resolution for the stl object.

    mirror_y : bool, optional
        Create mirror image about x-z plane. The default is False.

    Returns
    ----------
    stl_mesh : Mesh
        The numpy-stl mesh.
    """

    vertices, cell_ids = create_cells(
        parametric_surface, triangles_per_edge, si, sj, mirror_y, flip_faces
    )

    # Create the STL mesh object
    stl_mesh = mesh.Mesh(np.zeros(cell_ids.shape[0], dtype=mesh.Mesh.dtype))
    for ix, cell_id in enumerate(cell_ids):
        # For each face
        for c in range(3):
            # For each coordinate x/y/z
            stl_mesh.vectors[ix][c] = vertices[cell_id[c], :]

    return stl_mesh


def parametricSurfce2vtk(
    parametric_surface,
    triangles_per_edge: int,
    si: float = 1.0,
    sj: float = 1.0,
    mirror_y=False,
    flip_faces=False,
):
    """
    Function to convert parametric_surface generated using the Eilmer Geometry
    Package into a vtk cell format.

    Parameters
    ----------
    parametric_surface : Any
        The parametric surface object.

    si : float, optional
        The clustering in the i-direction. The default is 1.0.

    sj : float, optional
        The clustering in the j-direction. The default is 1.0.

    triangles_per_edge : int
        The resolution for the stl object.

    mirror_y : bool, optional
        Create mirror image about x-z plane. The default is False.

    Returns
    ----------
    tuple[vertices, cell_ids]
    """
    # Generate the mesh vertices and cell index list
    vertices, cell_ids = create_cells(
        parametric_surface, triangles_per_edge, si, sj, mirror_y, flip_faces
    )

    # Create the vtk cells format and add patch_tag to each cell
    # cells = []
    # for cell_id in cell_ids:
    #     cells.append(Cell(
    #         p0=Vector(x=vertices[cell_id[0]][0],
    #                   y=vertices[cell_id[0]][1], z=vertices[cell_id[0]][2]),
    #         p1=Vector(x=vertices[cell_id[1]][0],
    #                   y=vertices[cell_id[1]][1], z=vertices[cell_id[1]][2]),
    #         p2=Vector(x=vertices[cell_id[2]][0],
    #                   y=vertices[cell_id[2]][1], z=vertices[cell_id[2]][2]),
    #         face_ids=cell_id
    #     ))
    # cells.attributes[-1] = {'cell_tag': 1}

    return (vertices, cell_ids)


def assess_inertial_properties(vehicle, component_densities: Dict[str, float]):
    """Return the inertial properties of a vehicle.

    Parameters
    ----------
    vehicle : Vehicle
        A hypervehicle Vehicle instance.

    component_densities : Dict[str, float]
        A dictionary containing the effective densities for each component.
        Note that the keys of the dict must match the keys of
        vehicle._named_components.

    Returns
    -------
    total_volume : float
        The total volume.

    total_mass : float
        The toal mass.

    composite_cog : np.array
        The composite center of gravity.

    composite_inertia : np.array
        The composite mass moment of inertia.

    Examples
    --------
    >>> components = {'body': {'type': 'body', 'mesh': body},
                      'wings': {'type': 'wing', 'mesh': wings},
                      'inlet': {'type': 'inlet', 'mesh': inlet},
                      'fin1': {'type': 'fin', 'mesh': fin1},
                      'fin2': {'type': 'fin', 'mesh': fin2}}

    >>> component_densities = {'wing': 5590, 'body': 1680, 'inlet': 1680, 'fin': 5590}

    >>> volume, mass, cog, inertia = utils.assess_inertial_properties(components,
                                                             component_densities)
    """
    # Check if vehicle has been generated
    if not vehicle._generated:
        vehicle.generate()

    volumes = {}
    masses = {}
    cgs = {}
    inertias = {}
    total_mass = 0
    total_volume = 0

    for name, component in vehicle._named_components.items():
        inertia_handle = getattr(component.mesh, "get_mass_properties_with_density")

        volume, vmass, cog, inertia = inertia_handle(component_densities[name])

        volumes[name] = volume
        masses[name] = vmass
        cgs[name] = cog
        inertias[name] = inertia
        total_mass += vmass
        total_volume += volume

    # Composite centre of mass
    composite_cog = 0
    for component in vehicle._named_components:
        m = masses[component]
        composite_cog += m * cgs[component]

    composite_cog *= 1 / total_mass

    # Parallel axis theorem
    shifted_inertias = {}
    composite_inertia = 0
    for component in vehicle._named_components:
        m = masses[component]
        r = cgs[component] - composite_cog
        I_adj = inertias[component] + m * r**2

        shifted_inertias[component] = I_adj
        composite_inertia += I_adj

    return total_volume, total_mass, composite_cog, composite_inertia


class SensitivityStudy:
    """
    Computes the geometric sensitivities using finite differencing.
    """

    def __init__(self, vehicle_constructor, verbosity: int = 1):
        """Vehicle geometry sensitivity constructor.

        Parameters
        ----------
        vehicle_constructor : AbstractGenerator
            The Vehicle instance constructor.

        Returns
        -------
        VehicleSensitivity object.

        """
        self.vehicle_constructor = vehicle_constructor
        self.verbosity = verbosity

        # Parameter sensitivities
        self.parameter_sensitivities = None
        self.component_sensitivities = None
        self.scalar_sensitivities = None

    def __repr__(self):
        return "HyperVehicle sensitivity study"

    def dvdp(
        self,
        parameter_dict: dict,
        perturbation: float = 5,
        write_nominal_stl: bool = True,
        nominal_stl_prefix: str = None,
    ):
        """Computes the sensitivity of the geometry with respect to the
        parameters.

        Parameters
        ----------
        parameter_dict : dict
            A dictionary of the design parameters to perturb, and their
            nominal values.

        perturbation : float, optional
            The design parameter perturbation amount, specified as percentage.
            The default is 20.

        vehicle_creator_method : str, optional
            The name of the method which returns a hypervehicle.Vehicle
            instance, ready for generation. The default is 'create_instance'.

        write_nominal_stl : bool, optional
            A boolean flag to write the nominal geometry STL(s) to file. The
            default is True.

        nominal_stl_prefix : str, optional
            The prefix to append when writing STL files for the nominal geometry.
            If None, no prefix will be used. The default is None.

        Returns
        -------
        sensitivities : TYPE
            A dictionary containing the sensitivity information for all
            components of the geometry, relative to the nominal geometry.

        """
        # TODO - return perturbed instances? After generatation to allow
        # quickly writing to STL
        from hypervehicle.generator import AbstractGenerator

        # Create Vehicle instance with nominal parameters
        if self.verbosity > 0:
            print("Generating nominal geometry...")

        constructor_instance: AbstractGenerator = self.vehicle_constructor(
            **parameter_dict
        )
        nominal_instance = constructor_instance.create_instance()
        nominal_instance.verbosity = 0

        # Generate components
        nominal_instance.generate()
        nominal_meshes = {
            name: component.mesh
            for name, component in nominal_instance._named_components.items()
        }

        if self.verbosity > 0:
            print("  Done.")

        if write_nominal_stl:
            # Write nominal instance to STL files
            nominal_instance.to_stl(prefix=nominal_stl_prefix)

        # Generate meshes for each parameter
        if self.verbosity > 0:
            print("Generating perturbed geometries...")

        sensitivities = {}
        analysis_sens = {}
        for parameter, value in parameter_dict.items():
            sensitivities[parameter] = {}

            # Create copy
            adjusted_parameters = parameter_dict.copy()

            # Adjust current parameter for sensitivity analysis
            adjusted_parameters[parameter] *= 1 + perturbation / 100
            dp = adjusted_parameters[parameter] - value

            # Create Vehicle instance with perturbed parameter
            constructor_instance = self.vehicle_constructor(**adjusted_parameters)
            parameter_instance = constructor_instance.create_instance()
            parameter_instance.verbosity = 0

            # Generate stl meshes
            parameter_instance.generate()
            parameter_meshes = {
                name: component.mesh
                for name, component in parameter_instance._named_components.items()
            }

            # Generate sensitivities for geometric analysis results
            if nominal_instance.analysis_results:
                analysis_sens[parameter] = {}
                for r, v in nominal_instance.analysis_results.items():
                    analysis_sens[parameter][r] = (
                        parameter_instance.analysis_results[r] - v
                    ) / dp

            # Generate sensitivities
            for component, nominal_mesh in nominal_meshes.items():
                parameter_mesh = parameter_meshes[component]
                sensitivity_df = self._compare_meshes(
                    nominal_mesh,
                    parameter_mesh,
                    dp,
                    parameter,
                )

                sensitivities[parameter][component] = sensitivity_df

        if self.verbosity > 0:
            print("  Done.")

        # Return output
        self.parameter_sensitivities = sensitivities
        self.scalar_sensitivities = analysis_sens
        self.component_sensitivities = self._combine(nominal_instance, sensitivities)

        return sensitivities

    def to_csv(self, outdir: Optional[str] = None):
        """Writes the sensitivity information to CSV file.

        Parameters
        ----------
        outdir : str, optional
            The output directory to write the sensitivity files to. If
            None, the current working directory will be used. The default
            is None.
        """
        if self.component_sensitivities is None:
            raise Exception("Sensitivities have not yet been generated.")
        else:
            if outdir is None:
                outdir = os.getcwd()

            if not os.path.exists(outdir):
                # Make the directory
                os.mkdir(outdir)

            for component, df in self.component_sensitivities.items():
                df.to_csv(
                    os.path.join(outdir, f"{component}_sensitivity.csv"), index=False
                )

            # Also save analysis sensitivities
            if self.scalar_sensitivities:
                # Make analysis results directory
                properties_dir = os.path.join(outdir, f"scalar_sensitivities")
                if not os.path.exists(properties_dir):
                    os.mkdir(properties_dir)

                vm = {
                    p: {k: self.scalar_sensitivities[p][k] for k in ["volume", "mass"]}
                    for p in self.scalar_sensitivities
                }
                pd.DataFrame(vm).to_csv(
                    os.path.join(properties_dir, "volmass_sensitivity.csv")
                )

                for param in self.scalar_sensitivities:
                    self.scalar_sensitivities[param]["cog"].tofile(
                        os.path.join(properties_dir, f"{param}_cog_sensitivity.txt"),
                        sep=", ",
                    )
                    self.scalar_sensitivities[param]["moi"].tofile(
                        os.path.join(properties_dir, f"{param}_moi_sensitivity.txt"),
                        sep=", ",
                    )

    @staticmethod
    def _compare_meshes(mesh1, mesh2, dp, parameter_name: str) -> pd.DataFrame:
        """Compares two meshes with each other and applies finite differencing
        to quantify their differences.

        Parameters
        ----------
        mesh1 : None
            The reference mesh.

        mesh1 : None
            The perturbed mesh.

        dp : None

        parameter_name : str
            The name of the parameter.

        Returns
        --------
        df : pd.DataFrame
            A DataFrame of the finite difference results.
        """
        # Take the vector difference
        diff = mesh2.vectors - mesh1.vectors

        # Resize difference array to flatten
        shape = diff.shape
        flat_diff = diff.reshape((shape[0] * shape[2], shape[1]))

        # Also flatten the reference mesh vectors
        vectors = mesh1.vectors.reshape((shape[0] * shape[2], shape[1]))

        # Concatenate all data column-wise
        all_data = np.zeros((shape[0] * shape[2], shape[1] * 2))
        all_data[:, 0:3] = vectors  # Reference locations
        all_data[:, 3:6] = flat_diff  # Location deltas

        # Create DataFrame
        df = pd.DataFrame(data=all_data, columns=["x", "y", "z", "dx", "dy", "dz"])
        df["magnitude"] = np.sqrt(np.square(df[["dx", "dy", "dz"]]).sum(axis=1))

        # Sensitivity calculations
        sensitivities = df[["dx", "dy", "dz"]] / dp
        sensitivities.rename(
            columns={
                "dx": f"dxd{parameter_name}",
                "dy": f"dyd{parameter_name}",
                "dz": f"dzd{parameter_name}",
            },
            inplace=True,
        )

        # Merge dataframes
        df = df.merge(sensitivities, left_index=True, right_index=True)

        # Delete duplicate vertices
        df = df[~df.duplicated()]

        return df

    @staticmethod
    def _combine(nominal_instance, sensitivities):
        """Combines the sensitivity information for multiple parameters."""
        component_names = nominal_instance._named_components.keys()
        params = list(sensitivities.keys())

        allsens = {}
        for component in component_names:
            df = sensitivities[params[0]][component][["x", "y", "z"]]
            for param in params:
                p_s = sensitivities[param][component][
                    [f"dxd{param}", f"dyd{param}", f"dzd{param}"]
                ]
                df = pd.concat([df, p_s], axis=1)

            allsens[component] = df

        return allsens


def append_sensitivities_to_tri(
    dp_filenames: List[str],
    components_filepath: str = "Components.i.tri",
    match_tolerance: float = 1e-5,
    rounding_tolerance: float = 1e-8,
    verbosity: int = 1,
    outdir: Optional[str] = None,
) -> float:
    """Appends shape sensitivity data to .i.tri file.

    Parameters
    ----------
    dp_files : list[str]
        A list of the file names of the sensitivity data.

    components_filepath : str, optional
        The filepath to the .tri file to be appended to. The default is
        'Components.i.tri'.

    match_tolerance : float, optional
        The precision tolerance for matching point coordinates. The
        default is 1e-5.

    rounding_tolerance : float, optional
        The tolerance to round data off to. The default is 1e-8.

    outdir : str, optional
        The output directory to write the combined sensitivity file to. If
        None, the current working directory will be used. The default
        is None.

    Returns
    ---------
    match_fraction : float
        The fraction of cells which got matched. If this is below 100%, try
        decreasing the match tolerance and run again.

    Examples
    ---------
    >>> dp_files = ['wing_0_body_width_sensitivity.csv',
                    'wing_1_body_width_sensitivity.csv']
    """
    # Check outdir
    if outdir is None:
        outdir = os.getcwd()

    else:
        # Desired outdir provided, check it exists
        if not os.path.exists(outdir):
            # Make the directory
            os.mkdir(outdir)

    # TODO - rename to 'combine sensitivity' or "combine_comp_sens"
    # Parse .tri file
    tree = ET.parse(components_filepath)
    root = tree.getroot()
    grid = root[0]
    piece = grid[0]
    points = piece[0]

    points_data = points[0].text

    points_data_list = [el.split() for el in points_data.splitlines()]
    points_data_list = [[float(j) for j in i] for i in points_data_list]

    points_df = pd.DataFrame(points_data_list, columns=["x", "y", "z"]).dropna()

    # Ensure previous components sensitivity file is not included
    try:
        del dp_filenames[dp_filenames.index("all_components_sensitivity.csv")]
    except ValueError:
        # It is not in there
        pass

    # Load and concatenate sensitivity data across components
    dp_df = pd.DataFrame()
    for filename in dp_filenames:
        df = pd.read_csv(filename)
        dp_df = pd.concat([dp_df, df])

    # Extract parameters
    parameters = []
    param_cols = dp_df.columns[3:]
    for i in range(int(len(param_cols) / 3)):
        parameters.append(param_cols[int(i * 3)].split("dxd")[-1])

    # Match points_df to sensitivity df
    if verbosity > 0:
        print("Running coordinate matching algorithm for sensitivities...")
        pbar = tqdm(
            total=len(points_df), position=0, leave=True, desc="Point matching progress"
        )
    data_str = "\n "
    param_data = dict(zip(parameters, ["\n " for _ in parameters]))
    all_data = np.zeros((len(points_df), len(param_cols)), dtype=float)
    matched_points = 0
    for i in range(len(points_df)):
        match_x = (points_df["x"].iloc[i] - dp_df["x"]).abs() < match_tolerance
        match_y = (points_df["y"].iloc[i] - dp_df["y"]).abs() < match_tolerance
        match_z = (points_df["z"].iloc[i] - dp_df["z"]).abs() < match_tolerance

        match = match_x & match_y & match_z
        try:
            # TODO - what if there are multiple matches? (due to intersect perturbations)
            matched_data = dp_df[match].iloc[0][param_cols].values

            # Round off infinitesimally small values
            matched_data[abs(matched_data) < rounding_tolerance] = 0

            # Update data
            all_data[i, :] = matched_data
            p_n = 0
            for parameter, data_str in param_data.items():
                line = ""
                for j in range(3):
                    line += f"\t{matched_data[j+3*p_n]:.14e}"
                line += "\n "

                data_str += line
                param_data[parameter] = data_str
                p_n += 1

            # Count matched points
            matched_points += 1

        except IndexError:
            # No match found, append zeros to maintain order
            line = f"\t{0:.14e}\t{0:.14e}\t{0:.14e}\n "

            # Update data string for each parameter
            p_n = 0
            for parameter, data_str in param_data.items():
                param_data[parameter] = data_str + line
                p_n += 1

        # Update progress bar
        if verbosity > 0:
            pbar.update(1)

    match_fraction = matched_points / len(points_df)
    if verbosity > 0:
        pbar.close()
        print(f"Done - matched {100*match_fraction:.2f}% of points.")

    # Write combined sensitivity data to CSV
    combined_sense = pd.merge(
        left=points_df.reset_index(),
        right=pd.DataFrame(all_data, columns=param_cols),
        left_index=True,
        right_index=True,
    ).drop("index", axis=1)
    combined_sense.to_csv(
        os.path.join(outdir, "all_components_sensitivity.csv"), index=False
    )

    # Write the matched sensitivity df to i.tri file as new xml element
    # NumberOfComponents is how many sensitivity components there are (3 for x,y,z)
    for parameter in parameters:
        attribs = {
            "Name": f"{parameter}_sens",
            "NumberOfComponents": "3",
            "type": "Float64",
            "format": "ascii",
            "TRIXtype": "SHAPE_LINEARIZATION",
        }
        PointData = ET.SubElement(piece, "PointData")
        PointDataArray = ET.SubElement(PointData, "DataArray", attribs)
        PointDataArray.text = param_data[parameter]

    # Save to file
    tree.write(components_filepath)

    return match_fraction


def csv_to_delaunay(filepath: str):
    """Converts a csv file of points to a Delaunay3D surface.

    Parameters
    ------------
    filepath : str
        The filepath to the CSV file.
    """
    # TODO - rename
    from paraview.simple import CSVReader, TableToPoints, Delaunay3D, SaveData

    root_dir = os.path.dirname(filepath)
    prefix = filepath.split(os.sep)[-1].split(".csv")[0]
    savefilename = os.path.join(root_dir, f"{prefix}.vtu")

    # create a new 'CSV Reader'
    fin_0_L_b_sensitivitycsv = CSVReader(FileName=[filepath])

    # create a new 'Table To Points'
    tableToPoints1 = TableToPoints(Input=fin_0_L_b_sensitivitycsv)

    # Properties modified on tableToPoints1
    tableToPoints1.XColumn = "x"
    tableToPoints1.YColumn = "y"
    tableToPoints1.ZColumn = "z"

    # create a new 'Delaunay 3D'
    delaunay3D1 = Delaunay3D(Input=tableToPoints1)

    # save data
    SaveData(savefilename, proxy=delaunay3D1)


def convert_all_csv_to_delaunay(directory: str = ""):
    # TODO - rename
    # TODO - docstrings
    # TODO - specify outdir
    files = glob.glob(os.path.join(directory, "*.csv"))

    if len(files) == 0:
        print(f"No CSV files in directory {directory}.")
        sys.exit()

    for file in files:
        csv_to_delaunay(file)


def assign_tags_to_cell(patch, length):
    """Assign tags to cells."""
    # Creates a tag vector for a given patch
    tags = [patch.tag.value] * length
    return tags
