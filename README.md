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
    <li><a href="#contributing">Contributing</a></li>
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
Please see the [example geometry generation](docs/source/examples/x43.md) 
to generate a mockup of the X-43A, shown above.

<p align="right">[<a href="#readme-top">back to top</a>]</p>



## Contributing 

<!-- start contribution guidelines -->

To contribute to `hypervehicle`, please read the instructions below 
to maintain the styling of the code.

### Setting up for Development

1. Create a new Python virtual environment to isolate the package. You 
can do so using [`venv`](https://docs.python.org/3/library/venv.html) or
[anaconda](https://www.anaconda.com/).

2. Install the code in editable mode using the command below (run from
inside the `hypervehicle` root directory).

```
pip install -e .[all]
```

3. Install the [pre-commit](https://pre-commit.com/) hooks.

```
pre-commit install
```

4. Start developing! After following the steps above, you are ready
to start developing the code. Make sure to follow the guidelines 
below.


### Developing *hypervehicle*

- Before making any changes, create a new branch to develop on using 
`git checkout -b new-branch-name`.

- Run [black](https://black.readthedocs.io/en/stable/index.html) on any
code you modify. This formats it according to 
[PEP8](https://peps.python.org/pep-0008/) standards.

- Document as you go: use 
[numpy style](https://numpydoc.readthedocs.io/en/latest/format.html) 
docstrings, and add to the docs where relevant.

- Write unit tests for the code you add, and include them in `tests/`. 
This project uses [pytest](https://docs.pytest.org/en/7.2.x/).

- Commit code regularly to avoid large commits with many changes. 

- Write meaningful commit messages, following the 
[Conventional Commits standard](https://www.conventionalcommits.org/en/v1.0.0/).
The python package [commitizen](https://commitizen-tools.github.io/commitizen/)
is a great tool to help with this, and is already configured for this
repo. Simply stage changed code, then use the `cz c` command to make a 
commit.

- Open a [Pull Request](https://github.com/kieran-mackle/hypervehicle/pulls) 
when your code is complete and ready to be merged.


### Building the Docs
To build the documentation, run the commands below. 

```
cd docs/
make html
xdg-open build/html/index.html
```

If you are actively developing the docs, consider using
[sphinx-autobuild](https://pypi.org/project/sphinx-autobuild/).
This will continuosly update the docs for you to see any changes
live, rather than re-building repeatadly. From the `docs/` 
directory, run the following command:

```
sphinx-autobuild source/ build/html --open-browser
```

<!-- end contribution guidelines -->

<p align="right">[<a href="#readme-top">back to top</a>]</p>


