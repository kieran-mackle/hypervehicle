# Generating Geometry Sensitivities

The *hypervehicle* tool also features the generation of 
geometry sensitivities via method of finite differences.
This page outlines how to use this capability.

![geom](../images/sensitivity/geom.gif)


## Theory

1. A *hypervehicle* parameterised geometry generator is 
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


## Example
Interested in seeing this theory in application? Check out the
[example](../examples/sensitivity.md).


