# Getting Started with HyperVehicle

## Dependencies

### GDTK
<tt>HyperVehicle</tt> depends on the 
[GDTK](https://github.com/gdtk-uq/gdtk) Python package for its geometric toolkit. 
Note that a full install of GDTK is not required. Instead, simply do a 
[sparse checkout](https://stackoverflow.com/questions/600079/how-do-i-clone-a-subdirectory-only-of-a-git-repository)
of the relevant files, using the commands below.

```
mkdir gdtk
cd gdtk/
git init
git remote add -f origin https://github.com/gdtk-uq/gdtk.git
git config core.sparseCheckout true
echo "src/lib/" >> .git/info/sparse-checkout
git pull origin master
cd src/lib
python3 -m pip install .
cd ../../../
```

## Optional Dependencies

### PyMESH

To access more advanced features of <tt>HyperVehicle</tt>, such as compeonent mesh merging
and cleaning, [PyMesh](https://github.com/PyMesh/PyMesh) is required. You do not need to install
this to use <tt>HyperVehicle</tt>, but it can come in handy, especially when extracting 
sensitivities for multi-component geometries.


## Installation


### Installation from PyPi

```
pip install hypervehicle
```

### Installation from source
First clone the repository, then install via pip.

```
pip install git+https://github.com/kieran-mackle/hypervehicle
```


### Testing the installation

To check that everything has been installed properly and <tt>HyperVehicle</tt> is 
ready to go, run the command below.

```
python3 -m pytest tests/
```
