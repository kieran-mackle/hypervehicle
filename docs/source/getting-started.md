# Getting Started with HyperVehicle

## Installation

### Installation from PyPI

```
pip install hypervehicle
```

### Installation from source
To install `hypervehicle` from source, use the command below.

```
pip install git+https://github.com/kieran-mackle/hypervehicle
```


### Testing the installation

To check that everything has been installed properly and <tt>HyperVehicle</tt> is 
ready to go, run the command below.

```
python3 -m pytest tests/
```


## Optional Dependencies

### PyMESH

To access more advanced features of <tt>HyperVehicle</tt>, such as compeonent mesh merging
and cleaning, [PyMesh](https://github.com/PyMesh/PyMesh) is required. You do not need to install
this to use <tt>HyperVehicle</tt>, but it can come in handy, especially when extracting 
sensitivities for multi-component geometries.
