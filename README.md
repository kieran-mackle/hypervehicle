
# HYPERVEHICLE: Parametric Vehicle Geometry Generation
A suite of tools to rapidly generate geometry files for hypersonic vehicles.

![x43](https://user-images.githubusercontent.com/60687606/168926371-a383434b-3ea5-40ab-989a-93f7a8d7b4ff.png)


## Dependencies
Hypervehicle relies on the [Eilmer](https://github.com/gdtk-uq/gdtk) geometry 
package to function.


## Installation

First clone the repository, then install via pip as an editable install.

```
git clone https://github.com/kieran-mackle/hypervehicle
cd hypervehicle
python3 -m pip install -e ./
```


## Testing your install
```
python3 -m pytest tests/
```



## Documentation
HYPERVEHICLE is well documented in-code. To access more information on a 
specific method, use the `help()` method of Python. To view documentation 
of the entire codebase in html format, run the commands below.

```
cd docs
make html
xdg-open _build/html/index.html
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
|    Fin         | `hypervehicle.components.fin` | Fin geometry generation. |
|    Polygon     | `hypervehicle.components.polygon` | Polygon (cube, sphere) geometry generation. |

The `Vehicle` class and the methods it contains are well documented internally.



## Example Vehicle Generation
Please see the [example geometry generation](docs/example.md) to generate a mockup of the X-43A, shown above.




