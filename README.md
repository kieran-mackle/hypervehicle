[![x43](docs/source/images/logo-dark.png)](docs/hangar.md)

A Python package to rapidly generate parametric geometries
defined by hyperpatches. Check out the 
[hangar](docs/source/hangar.md) for some examples.


## Getting Started

### Prerequisites
Hypervehicle relies on the [GDTK](https://github.com/gdtk-uq/gdtk) geometry 
package. Note that a full install of GDTK is not required. Instead, you can simply do a 
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
python3 -m pip install -e ./
cd ../../../
```

#### PyMesh

Having [PyMesh](https://github.com/PyMesh/PyMesh) installed can greatly enhance the capabilities
offered by `HyperVehicle`. However, it can be difficult to install. Troubleshooting 
guide coming soon.


### Installation

To install `hypervehicle` from source, use the command below.

```
pip install git+https://github.com/kieran-mackle/hypervehicle
```

## Usage
Please see the [example geometry generation](docs/source/examples/x43.md) 
to generate a mockup of the X-43A, shown above.
