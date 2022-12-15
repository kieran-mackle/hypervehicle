# HYPERVEHICLE: Parametric Vehicle Geometry Generation
A suite of tools to rapidly generate parametric geometry 
for hypersonic vehicles. 
Check out the [hypervehicle hangar](docs/source/hangar.md)
for some examples.

[![x43](https://user-images.githubusercontent.com/60687606/168926371-a383434b-3ea5-40ab-989a-93f7a8d7b4ff.png)](docs/hangar.md)




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

### Installation

First clone the repository, then install via pip as an editable install.

```
git clone https://github.com/kieran-mackle/hypervehicle
cd hypervehicle
python3 -m pip install -e ./
```

<p align="right">[<a href="#readme-top">back to top</a>]</p>


## Usage
Please see the [example geometry generation](docs/source/example.md) 
to generate a mockup of the X-43A, shown above.

<p align="right">[<a href="#readme-top">back to top</a>]</p>


## Documentation
HYPERVEHICLE is well documented in-code. To access more information on a 
specific method, use the `help()` method of Python. To view documentation 
of the entire codebase in html format, run the commands below.

```
cd docs
make html
xdg-open _build/html/index.html
```

<p align="right">[<a href="#readme-top">back to top</a>]</p>


