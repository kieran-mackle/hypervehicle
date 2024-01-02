# Generating Geometry Sensitivities

<tt>HyperVehicle</tt> also features the generation of 
geometry sensitivities via method of finite differences.
This page outlines how to use this capability.

## Nomenclature

| Symbol | Description |
| ------ | ----------- |
| $\underline{\theta}$ | The set of design parameters defining a geometry |
| $\mathcal{G}$ | The geometry mesh |


## Description of Implementation

1. A <tt>HyperVehicle</tt> parameterised geometry {py:class}`.Generator` is 
defined, along with the parameters to be varied.

2. Geometry meshes for the nominal geometry $\mathcal{G}_\text{nominal}$ are generated.

3. For each parameter $\theta_i$ to be varied:

    - The perturbed parameter value is defined as $ \theta_i' = \theta_i + \delta\theta_i $.
    
    - Geometry meshes for the parameter set including $\theta_i'$ are generated, yielding $\mathcal{G}_i'$.

    - The sensitivity of each vertex of $\mathcal{G}$ to the parmater perturbation 
    $\theta_i'$ is calculated by comparing the perturbed geometry $\mathcal{G}_i'$ to 
    the nominal geometry $\mathcal{G}_\text{nominal}$. That 
    is,  $\delta \mathcal{G}_i = \mathcal{G}_i' - \mathcal{G}_\text{nominal}$.

    - Finally, the sensitivity is approximated via:

$$ 
\frac{\partial\mathcal{G}}{\partial\theta} \approx \frac{\delta \mathcal{G}}{\delta \theta} 
$$


## Working with Mutliple-Component Geometries


### Merging components with PyMesh
If you have PyMesh installed, simply pass `merge=True` when calling {py:meth}`.Vehicle.to_stl`.
This will merge all components of your Vehicle into a single file.


### Cart3D intersected Components
Cart3D is a component based solver, meaning oftentimes, many individual components are 
stacked to form the final geometry. This final geometry is a single body, made up of 
many combined components. This body has a different mesh to the individual component 
STL meshes.

When working with multiple components, we need to map the component sensitivities onto 
the final combined mesh. The utility function `append_sensitivities_to_tri` contained 
within the `hypervehicle.utilities` module provides this capability. See the code below, 
which first finds the sensitivity files using `glob`, then runs `append_sensitivities_to_tri`. 
The Cart3D file, `Components.i.tri` will be overwritten, with the sensitivity data 
appended. A CSV file called `all_components_sensitivity.csv` will also be written, 
containing the sensitivity information as point data. This file is compatible with
[PySAGAS](https://github.com/kieran-mackle/pysagas).


```python
import glob
from hypervehicle.utilities import append_sensitivities_to_tri

sens_files = glob.glob("*sensitivity*")
append_sensitivities_to_tri(sens_files)
```

You can view this data of the `Components.i.tri` file in ParaView, by first converting the 
file into a Tecplot format using the Cart3D utility function `trix`:

```
trix Components.i.tri -T
```


## Example
Interested in seeing this theory in application? Check out the
[parameter sensitivities](../examples/sensitivity.md) tutorial.


