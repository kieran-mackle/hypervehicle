import os
import sys
import glob
import numpy as np
import pandas as pd
from stl import mesh
from tqdm import tqdm
from art import tprint, art
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .vehicle import Vehicle


def surfce_to_stl(
    parametric_surface,
    triangles_per_edge_r: int,
    triangles_per_edge_s: int,
    si: float = 1.0,
    sj: float = 1.0,
    mirror_y: bool = False,
    i_clustering_func: callable = None,
    j_clustering_func: callable = None,
    verbosity=0,
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

        triangles_per_edge_r : int
            The resolution for the stl object - x/i direction.

        triangles_per_edge_s : int
            The resolution for the stl object - y/j direction.

        mirror_y : bool, optional
            Create mirror image about x-z plane. The default is False.

        i_clustering_func : callable, optional
            A custom clustering function to apply in the i direction.
            The default is None.

        j_clustering_func : callable, optional
            A custom clustering function to apply in the j direction.
            The default is None.

    Returns
    ----------
    stl_mesh : Mesh
        The numpy-stl mesh.
    """
    ni = triangles_per_edge_r
    nj = triangles_per_edge_s

    if verbosity > 1:
        print(f"Triangles per edge: (ni, nj) = ({ni}, {nj})")

    # Create list of vertices
    if i_clustering_func:
        r_list = [i_clustering_func(i) for i in np.linspace(0, 1, ni + 1)]
    else:
        r_list = default_vertex_func(lb=0.0, ub=1.0, steps=ni + 1, spacing=si)

    if j_clustering_func:
        s_list = [j_clustering_func(i) for i in np.linspace(0, 1, ni + 1)]
    else:
        s_list = default_vertex_func(lb=0.0, ub=1.0, steps=nj + 1, spacing=sj)

    y_mult: int = -1 if mirror_y else 1

    if verbosity > 1:
        print(f"r_list = {r_list}")
        print(f"s_list = {s_list}")

    # For stl generation we split each cell into 4x triangles with indece [0, 3]
    #   p01-------p11
    #    | \  2  / |
    #    |   \  /  |
    #    | 3  pc 1 |
    #    |   /  \  |
    #    |  /  0  \ |
    #   p00-------p10

    N_triangles = 4 * (ni) * (nj)
    stl_mesh = mesh.Mesh(np.zeros(N_triangles, dtype=mesh.Mesh.dtype))

    # mesh.vectors contains a list defining three corners of each triangle.
    t = 0

    # For vertices along the x direction (i)
    for i in range(ni):
        # For vertices along the y direction (j)
        for j in range(nj):
            # Get corner points for current cell (quad)
            pos00 = parametric_surface(r_list[i], s_list[j])
            pos10 = parametric_surface(r_list[i + 1], s_list[j])
            pos01 = parametric_surface(r_list[i], s_list[j + 1])
            pos11 = parametric_surface(r_list[i + 1], s_list[j + 1])

            # Get centre for current cell (quad)
            pos_x = 0.25 * (pos00.x + pos10.x + pos01.x + pos11.x)
            pos_y = 0.25 * (pos00.y + pos10.y + pos01.y + pos11.y)
            pos_z = 0.25 * (pos00.z + pos10.z + pos01.z + pos11.z)

            # Add triangle 0 [p00, p10, pc]
            stl_mesh.vectors[t][0] = np.array([pos00.x, y_mult * pos00.y, pos00.z])
            stl_mesh.vectors[t][1] = np.array([pos10.x, y_mult * pos10.y, pos10.z])
            stl_mesh.vectors[t][2] = np.array([pos_x, y_mult * pos_y, pos_z])
            t += 1

            # Add triangle 1 [p10, p11, pc]
            stl_mesh.vectors[t][0] = np.array([pos10.x, y_mult * pos10.y, pos10.z])
            stl_mesh.vectors[t][1] = np.array([pos11.x, y_mult * pos11.y, pos11.z])
            stl_mesh.vectors[t][2] = np.array([pos_x, y_mult * pos_y, pos_z])
            t += 1

            # Add triangle 2 [p11, p01, pc]
            stl_mesh.vectors[t][0] = np.array([pos11.x, y_mult * pos11.y, pos11.z])
            stl_mesh.vectors[t][1] = np.array([pos01.x, y_mult * pos01.y, pos01.z])
            stl_mesh.vectors[t][2] = np.array([pos_x, y_mult * pos_y, pos_z])
            t += 1

            # Add triangle 3 [p01, p00, pc]
            stl_mesh.vectors[t][0] = np.array([pos01.x, y_mult * pos01.y, pos01.z])
            stl_mesh.vectors[t][1] = np.array([pos00.x, y_mult * pos00.y, pos00.z])
            stl_mesh.vectors[t][2] = np.array([pos_x, y_mult * pos_y, pos_z])
            t += 1

    return stl_mesh


def default_vertex_func(lb, ub, steps, spacing=1.0):
    span = ub - lb
    dx = 1.0 / (steps - 1)
    return np.array([lb + (i * dx) ** spacing * span for i in range(steps)])


def assess_inertial_properties(
    vehicle: "Vehicle", component_densities: Dict[str, float]
):
    """

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
    vehicle_properties : dict
        A dictionary containing the vehicle's mass, volume, location of
        CoG and moment of inertia matrix.

    component_properties : dict
        A dictionary containing the same keys as vehicle_properties, but
        the values are now dictionaries for each component of the vehicle.
    """
    # Check if vehicle has been generated
    if not vehicle._generated:
        vehicle.generate()

    volumes = {}
    areas = {}
    masses = {}
    cgs = {}
    inertias = {}
    total_mass = 0
    total_volume = 0
    total_area = 0

    for name, component in vehicle._named_components.items():
        # Estiamte inertial properties
        volume, vmass, cog, inertia = component.mesh.get_mass_properties_with_density(
            component_densities[name]
        )

        # Also calculate surface area
        area = float(sum(component.mesh.areas))

        # Save component result
        volumes[name] = volume
        areas[name] = area
        masses[name] = vmass
        cgs[name] = cog
        inertias[name] = inertia
        total_mass += vmass
        total_volume += volume
        total_area += area

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

    # Prepare output
    vehicle_properties = {
        "mass": total_mass,
        "volume": total_volume,
        "area": total_area,
        "cog": composite_cog,
        "moi": composite_inertia,
    }
    component_properties = {
        "mass": masses,
        "volume": volumes,
        "area": areas,
        "cog": cgs,
        "moi": inertias,
    }

    return vehicle_properties, component_properties


class SensitivityStudy:
    """
    Computes the geometric sensitivities using finite differencing.
    """

    def __init__(self, vehicle_constructor, verbosity: Optional[int] = 1):
        """Vehicle geometry sensitivity constructor.

        Parameters
        ----------
        vehicle_constructor : AbstractGenerator
            The Vehicle instance constructor.

        verbosity : int, optional
            The code verbosity. The default is 1.

        Returns
        -------
        VehicleSensitivity object.
        """
        self.vehicle_constructor = vehicle_constructor
        self.verbosity = verbosity

        # Parameter sensitivities
        self.parameter_sensitivities = None
        self.component_sensitivities = None
        self.component_scalar_sensitivities = None
        self.scalar_sensitivities = None
        self.property_sensitivities = None

        # Nominal vehicle instance
        self.nominal_vehicle_instance = None

        # Combined data file name
        self.combined_fn = "all_components_sensitivity.csv"

    def __repr__(self):
        return "HyperVehicle sensitivity study"

    def dvdp(
        self,
        parameter_dict: dict[str, any],
        overrides: Optional[dict[str, any]] = None,
        perturbation: Optional[float] = 5,
        write_nominal_stl: Optional[bool] = True,
        nominal_stl_prefix: Optional[str] = None,
    ):
        """Computes the sensitivity of the geometry with respect to the
        parameters.

        Parameters
        ----------
        parameter_dict : dict
            A dictionary of the design parameters to perturb, and their
            nominal values.

        overrides : dict, optional
            Optional vehicle generator overrides to provide along with the
            parameter dictionary without variation. The default is None.

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
        sensitivities : dict
            A dictionary containing the sensitivity information for all
            components of the geometry, relative to the nominal geometry.
        """
        # Print banner
        if self.verbosity > 0:
            print_banner()
            print("Running geometric sensitivity study.")

        # TODO - return perturbed instances? After generatation to allow
        # quickly writing to STL
        from hypervehicle.generator import AbstractGenerator

        # Check overrides
        overrides = overrides if overrides else {}

        # Create Vehicle instance with nominal parameters
        if self.verbosity > 0:
            print("  Generating nominal geometry...")

        constructor_instance: AbstractGenerator = self.vehicle_constructor(
            **parameter_dict, **overrides
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
            print("    Done.")

        if write_nominal_stl:
            # Write nominal instance to STL files
            nominal_instance.to_stl(prefix=nominal_stl_prefix)

        # Generate meshes for each parameter
        if self.verbosity > 0:
            print("  Generating perturbed geometries...")
            print("    Parameters: ", parameter_dict.keys())

        sensitivities = {}
        analysis_sens = {}
        component_analysis_sens = {}
        property_sens = {}
        for parameter, value in parameter_dict.items():
            if self.verbosity > 0:
                print(f"    Generating for {parameter}.")

            sensitivities[parameter] = {}

            # Create copy
            adjusted_parameters = parameter_dict.copy()

            # Adjust current parameter for sensitivity analysis
            adjusted_parameters[parameter] *= 1 + perturbation / 100
            dp = adjusted_parameters[parameter] - value

            # Create Vehicle instance with perturbed parameter
            constructor_instance = self.vehicle_constructor(
                **adjusted_parameters, **overrides
            )
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

                # Repeat for components
                component_analysis_sens[parameter] = (
                    parameter_instance._volmass - nominal_instance._volmass
                ) / dp

            # Generate sensitivities for vehicle properties
            if nominal_instance.properties:
                property_sens[parameter] = {}
                for property, v in nominal_instance.properties.items():
                    property_sens[parameter][property] = (
                        parameter_instance.properties[property] - v
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
            print("    Done.")

        # Return output
        self.parameter_sensitivities = sensitivities
        self.scalar_sensitivities = analysis_sens
        self.component_scalar_sensitivities = component_analysis_sens
        self.property_sensitivities = property_sens
        self.component_sensitivities = self._combine(nominal_instance, sensitivities)
        self.nominal_vehicle_instance = nominal_instance

        if self.verbosity > 0:
            print("Sensitivity study complete.")

        return sensitivities

    def to_csv(self, outdir: Optional[str] = None):
        """Writes the sensitivity information to CSV file.

        Parameters
        ----------
        outdir : str, optional
            The output directory to write the sensitivity files to. If
            None, the current working directory will be used. The default
            is None.

        Returns
        -------
        combined_data_filepath : str
            The filepath to the combined sensitivity data.
        """
        # Check if sensitivities have been generated
        if self.component_sensitivities is None:
            raise Exception("Sensitivities have not yet been generated.")

        else:
            # Check output directory
            if outdir is None:
                outdir = os.getcwd()

            if not os.path.exists(outdir):
                # Make the directory
                os.mkdir(outdir)

            # Save sensitivity data for each component
            all_sens_data = pd.DataFrame()
            for component, df in self.component_sensitivities.items():
                df.to_csv(
                    os.path.join(outdir, f"{component}_sensitivity.csv"), index=False
                )
                all_sens_data = pd.concat([all_sens_data, df])

            # Also save the combined sensitivity data
            combined_data_path = os.path.join(outdir, self.combined_fn)
            all_sens_data.to_csv(combined_data_path, index=False)

            # Also save analysis sensitivities
            if self.scalar_sensitivities:
                # Make analysis results directory
                properties_dir = os.path.join(outdir, f"scalar_sensitivities")
                if not os.path.exists(properties_dir):
                    os.mkdir(properties_dir)

                # Save volume and mass
                reformatted_results = {}
                for p, s in self.component_scalar_sensitivities.items():
                    labels = []
                    values = []
                    s: pd.DataFrame
                    for component, comp_sens in s.iterrows():
                        comp_sens: pd.Series
                        for i, j in comp_sens.items():
                            labels.append(f"{component}_{i}")
                            values.append(j)

                    reformatted_results[p] = values

                # Convert to DataFrame and save
                comp_sens = pd.DataFrame(data=reformatted_results, index=labels)
                comp_sens.to_csv(
                    os.path.join(properties_dir, "volmass_sensitivity.csv")
                )

                # Save others
                for param in self.scalar_sensitivities:
                    self.scalar_sensitivities[param]["cog"].tofile(
                        os.path.join(properties_dir, f"{param}_cog_sensitivity.txt"),
                        sep=", ",
                    )
                    self.scalar_sensitivities[param]["moi"].tofile(
                        os.path.join(properties_dir, f"{param}_moi_sensitivity.txt"),
                        sep=", ",
                    )

            # Also save user-defined property sensitivities
            if self.property_sensitivities:
                properties_dir = os.path.join(outdir, f"scalar_sensitivities")
                if not os.path.exists(properties_dir):
                    os.mkdir(properties_dir)

                pd.DataFrame(self.property_sensitivities).to_csv(
                    os.path.join(properties_dir, "property_sensitivity.csv")
                )

            return combined_data_path

    @staticmethod
    def _compare_meshes(mesh1, mesh2, dp, parameter_name: str) -> pd.DataFrame:
        """Compares two meshes with each other and applies finite differencing
        to quantify their differences.

        Parameters
        ----------
        mesh1 : Mesh
            The reference mesh.

        mesh1 : Mesh
            The perturbed mesh.

        dp : float
            The parameter perturbation.

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
    components_filepath: Optional[str] = "Components.i.tri",
    match_tolerance: Optional[float] = 1e-5,
    rounding_tolerance: Optional[float] = 1e-8,
    combined_sens_fn: Optional[str] = "all_components_sensitivity.csv",
    outdir: Optional[str] = None,
    verbosity: Optional[int] = 1,
) -> float:
    """Appends shape sensitivity data to .i.tri file, and writes the sensitivity
    data to csv file too. This step is required for geometries with multiple
    components. The .tri file is used to match individual sensitivity files
    (dp_filenames) to the geometry. The combined sensitivity file is required
    to calculate flow sensitivities with the .plt file, which has local flow
    conditions attached.

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

    combined_sens_fn : str, optional
        The filename of the combined geometry sensitivity data. The default
        is "all_components_sensitivity.csv".

    outdir : str, optional
        The output directory to write the combined sensitivity file to. If
        None, the current working directory will be used. The default
        is None.

    verbosity : int, optional
        The verbosity of the code. The defualt is 1.

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
        del dp_filenames[dp_filenames.index(combined_sens_fn)]
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
    combined_sense.to_csv(os.path.join(outdir, combined_sens_fn), index=False)

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
    from paraview.simple import CSVReader, TableToPoints, Delaunay3D, SaveData  # type: ignore

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


def merge_stls(
    stl_files: List[str], name: Optional[str] = None, verbosity: Optional[int] = 1
) -> str:
    """Merge STL files into a single file. Note that this function
    depends on having PyMesh installed.

    Parameters
    ----------
    stl_files : list[str]
        A list of the STL file names to be merged.

    name : str, optional
        The prefix of the combined STL filename output.

    verbosity : int, optional
        The function verbosity. The default is 1.

    Returns
    -------
    outfile : str
        The filename of the merged STL.
    """
    # Import PyMesh
    try:
        import pymesh  # type: ignore
    except ModuleNotFoundError:
        raise Exception(
            "Could not find pymesh. Please follow the "
            + "installation instructions at "
            + "https://pymesh.readthedocs.io/en/latest/installation.html"
        )

    if verbosity > 0:
        print("")

    # Load STL files
    if verbosity > 1:
        print("Loading STLs...")
    meshes = [pymesh.meshio.load_mesh(f) for f in stl_files]

    # Merge meshes
    if verbosity > 0:
        print("Merging STLs...")
    merged = pymesh.merge_meshes(meshes)

    # Resolve self-intersections
    if verbosity > 1:
        print("Resolving self intersections...")
    merged = pymesh.resolve_self_intersection(merged)

    # Remove degenerate triangles
    if verbosity > 1:
        print("Removing degenerate triangles...")
    merged, info = pymesh.remove_degenerated_triangles(merged)

    # Remove duplicate faces
    if verbosity > 1:
        print("Removing duplicated faces...")
    merged, info = pymesh.remove_duplicated_faces(merged)

    # Remove isolated vertices
    if verbosity > 1:
        print("Removing isolated vertices...")
    merged, info = pymesh.remove_isolated_vertices(merged)

    # Remove obtuse triangles
    if verbosity > 1:
        print("Removing obtuse triangles...")
    merged, info = pymesh.remove_obtuse_triangles(merged)

    # Write to file
    if verbosity > 1:
        print("Saving merged STL mesh...")
    name = "combined_mesh" if name is None else name
    outfile = f"{name}.stl"
    pymesh.meshio.save_mesh(outfile, merged)

    if verbosity > 0:
        print(f"Done. Merged STLs written to '{outfile}'.")

    return outfile


def print_banner():
    """Prints the hypervehicle banner"""
    tprint("Hypervehicle", "tarty4")
    p = art("airplane2")
    print(f" {p}               {p}               {p}               {p}")
