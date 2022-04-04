
# HYPERVEHICLE: Parametric Vehicle Geometry Generation
A suite of tools to rapidly generate geometry files for hypersonic vehicles.


## Installation


### From PyPi

(Not available)


### From Repository clone
First clone the repository, then install as editable.

```
git clone https://github.com/kieran-mackle/hypervehicle
cd hypervehicle
python3 -m pip install -e ./
```


### Testing your install
```
python3 -m pytest tests/
```




## General Information
The HyperVehicle pacakge revolves around the `Vehicle` class of 
`hypervehicle.vehicle`. This class contains the vehicle 
geometry components as attributes, as well as the methods required to write
them to *.stl* files. The code is flexible in that a vehicle can be constructed
from any number of components, simply by specifying a geometry definition 
dictionary for each new component.


| Component Type | Module | Description |
| -------------- | ------ | ----------- |
|    Wing        | `hypervehicle.components.wing` | Wing geometry generation. |
|    Fuselage    | `hypervehicle.components.fuselage` | Fuselage geometry generation. |
|    Fin        | `hypervehicle.components.fin` | Fin geometry generation. |


The `Vehicle` class and the methods it contains are well documented internally.


