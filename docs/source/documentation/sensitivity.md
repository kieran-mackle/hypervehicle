# Generating Geometry Sensitivities

<tt>HyperVehicle</tt> also features the generation of 
geometry sensitivities via method of finite differences.
This page outlines how to use this capability.

## Nomenclature

| Symbol | Description |
| ------ | ----------- |
| $\theta$ | The set of design parameters defining a geometry |
| $\mathcal{G}$ | The geometry |



## Theory

1. A <tt>HyperVehicle</tt> parameterised geometry {py:class}`.Generator` is 
supplied, along with the parameters to be varied.
2. STL files for the nominal geometry $G_{nominal}$ are generated.
3. For each parameter to be varied:
    - The perturbed parameter value is calculated as
    $ P_{perturbed} = P \times (1+pertubation) $.
    - The parameter delta is calculated from the 
    perturbed value: $\Delta P = P_{perturbed} - P_{nominal}$.
    - STL files for the perturbed geometry $G_{perturbed}$ are generated.
    - The parameter sensitivities are calculated for
    each vertex of the resulting STL, by comparing the 
    perturbed geometry $G_{perturbed}$ to the nominal 
    geometry $G_{nominal}$, with 
    $\Delta G = G_{perturbed} - G_{nominal}$.

    $$ sensitivity = \frac{\Delta G}{\Delta P} $$


## Cart3D Intersected Components
Cart3D is a component based solver, meaning oftentimes, many individual 
components are stacked to form the final geometry. This final geometry 
is a single body, made up of many combined components. This body has a 
different mesh to the individual component STL meshes.

When working with multiple components, we need to map the component 
sensitivities onto the final combined mesh. The utility function 
`append_sensitivities_to_tri` contained within the 
`hypervehicle.utilities` module provides this capability. See the 
code below, which first finds the sensitivity files using `glob`, 
then runs `append_sensitivities_to_tri`. The Cart3D file,
`Components.i.tri` will be overwritten, with the sensitivity data 
appended. A CSV file called `all_components_sensitivity.csv` will
also be written, containing the sensitivity information as point 
data. This file is compatible with 
[PySAGAS](https://github.com/kieran-mackle/pysagas).



```python
import glob
from hypervehicle.utilities import append_sensitivities_to_tri

sens_files = glob.glob("*sensitivity*")
append_sensitivities_to_tri(sens_files)
```

You can view this data of the `Components.i.tri` file in ParaView, by 
first converting the file into a Tecplot format using the Cart3D utility
function `trix`:

```
trix Components.i.tri -T
```


## Example
Interested in seeing this theory in application? Check out the
[parameter sensitivities](../examples/sensitivity.md) tutorial.


