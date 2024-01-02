# Contribution Guidelines

To contribute to `HyperVehicle`, please read the instructions below 
to maintain the styling of the code.


## Seek Feedback Early
Please open an [issue](https://github.com/kieran-mackle/hypervehicle/issues) before a 
[pull request](https://github.com/kieran-mackle/hypervehicle/pulls) to discuss any 
changes you wish to make.


## Setting up for Development

1. Create a new Python virtual environment to isolate the package. You 
can do so using [`venv`](https://docs.python.org/3/library/venv.html) or
[anaconda](https://www.anaconda.com/).

2. Install the code in editable mode using the command below (run from
inside the `hypervehicle` root directory).

```
pip install -e .[all]
```

3. Install the [pre-commit](https://pre-commit.com/) hooks. This will check
that your changes are formatted correctly before you make any commits.

```
pre-commit install
```

4. Start developing! After following the steps above, you are ready
to start developing the code. Make sure to follow the guidelines 
below.


## Developing *HyperVehicle*

- Before making any changes, fork this repository and clone it to your machine.

- Run [black](https://black.readthedocs.io/en/stable/index.html) on any
code you modify. This formats it according to 
[PEP8](https://peps.python.org/pep-0008/) standards.

- Document as you go: use 
[numpy style](https://numpydoc.readthedocs.io/en/latest/format.html) 
docstrings, and add to the docs where relevant.

- Write unit tests for the code you add, and include them in `tests/`. 
This project uses [pytest](https://docs.pytest.org/en/7.2.x/).

- Commit code regularly to avoid large commits with many unrelated changes. 

- Write meaningful commit messages, following the 
[Conventional Commits standard](https://www.conventionalcommits.org/en/v1.0.0/).
This is used for the purpose of maintaining semantic versioning and automated 
[changelogs](../changelog).
The Python package [`commitizen`](https://commitizen-tools.github.io/commitizen/)
is a great tool to help with this, and is already configured for this
repo. Simply stage changed code, then use the `cz c` command to make a 
commit. If you do not wish to do this, your commits will be squashed before being
merged into `main`.

- Open a [Pull Request](https://github.com/kieran-mackle/hypervehicle/pulls) 
when your code is complete and ready to be merged.


## Building the Docs
To build the documentation, run the commands below. 

```
cd docs/
make html
xdg-open build/html/index.html
```

If you are actively developing the docs, consider using
[sphinx-autobuild](https://pypi.org/project/sphinx-autobuild/).
This will continuosly update the docs for you to see any changes
live, rather than re-building repeatadly. From the root 
directory, run the following command:

```
sphinx-autobuild docs/source/ docs/build/html --open-browser
```
