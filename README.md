<a name="readme-top"></a>

<!-- <h1 align="center"><em>HyperVehicle</em></h1> -->
[![x43](docs/source/images/logo-dark.png)](docs/hangar.md)

A Python package to rapidly generate parametric geometries
defined by hyperpatches. Check out the 
[hangar](docs/source/hangar.md) for some examples.


<!-- [![x43](docs/source/images/x43.gif)](docs/hangar.md) -->


<!-- TABLE OF CONTENTS -->
<details>
  <summary><h2>Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#environment">Working Environment</a></li>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
  </ol>
</details>



## Getting Started

<a name="environment"></a>

### Working in a dedicated environment
It is *highly* recommended that you work in a dedicated environment. This 
is especially important when running simulations on UQ's HPC's, which may 
not have the latest version of Python. To avoid any conflicts, 
[Anaconda](https://www.anaconda.com/) is recommended, with a Python3.9 
environment.


### Prerequisites
Hypervehicle relies on the [Eilmer](https://github.com/gdtk-uq/gdtk) geometry 
package. Note that a full Eilmer install is not required. Instead, do a 
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


<p align="right">[<a href="#readme-top">back to top</a>]</p>


## Usage
Please see the [example geometry generation](docs/source/examples/x43.md) 
to generate a mockup of the X-43A, shown above.

<p align="right">[<a href="#readme-top">back to top</a>]</p>
