
# HYPERVEHICLE: Parametric Vehicle Geometry Generation
A suite of tools to rapidly generate geometry files for hypersonic vehicles.

![tests](https://github.com/kieran-mackle/hypervehicle/actions/workflows/tests.yml/badge.svg)

## General Information
The HyperVehicle module of IDMOC revolves around the `Vehicle` class of 
`idmoc.hypervehicle.hyper_vehicle`. This class contains the vehicle 
geometry components as attributes, as well as the methods required to write
them to *.stl* files. The code is flexible in that a vehicle can be constructed
from any number of components, simply by specifying a geometry definition 
dictionary for each new component.


| Component Type | Module | Description |
| -------------- | ------ | ----------- |
|    Wing        | `idmoc.hypervehicle.wingen` | Wing geometry generation. |
|    Fuselage    | `idmoc.hypervehicle.fusgen` | Fuselage geometry generation. |
|    Fin        | `idmoc.hypervehicle.fingen` | Fin geometry generation. |


The `Vehicle` class and the methods it contains are well documented internally.


