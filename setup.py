import codecs
import os.path
import setuptools


def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), "r") as fp:
        return fp.read()


def get_version(rel_path):
    for line in read(rel_path).splitlines():
        if line.startswith("__version__"):
            delim = '"' if '"' in line else "'"
            return line.split(delim)[1]
    else:
        raise RuntimeError("Unable to find version string.")


# Load README
with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

# Define dependencies groups
dev = [
    "furo >= 2022.12.7",
    "myst_parser >= 0.18.1",
    "sphinx_copybutton >= 0.5.1",
    "sphinx_inline_tabs >= 2022.1.2b11",
    "sphinx-autobuild >= 2021.3.14",
]
docs = [
    "pytest >= 7.2.1",
    "black >= 22.12.0",
    "commitizen >= 2.39.1",
    "pre-commit >= 2.21.0",
]
all_dep = dev + docs

setuptools.setup(
    name="hypervehicle",
    version=get_version("hypervehicle/__init__.py"),
    author="Ingo Jahn, Kieran Mackle",
    author_email="ingo.jahn@usq.edu.au, k.mackle@uq.net.au",
    packages=setuptools.find_packages(),
    url="https://github.com/kieran-mackle/hypervehicle",
    description="Parametric geometry generation tool for hypersonic vehicles",
    long_description=open("README.md").read(),
    install_requires=[
        "numpy-stl >= 3.0.0",
        "scipy >= 1.10.0",
        "pandas >= 1.5.2",
        "art >= 5.8",
        "tqdm >= 4.64.1",
        "multiprocess >= 0.70.14",
        "pymeshfix >= 0.16.2",
    ],
    extras_require={
        "dev": dev,
        "docs": docs,
        "all": all_dep,
    },
)
