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
        "pytest",
        "openpyxl",
        "numpy-stl",
        "matplotlib",
        "scipy",
        "pandas",
        "furo",
        "myst_parser",
        "sphinx_copybutton",
        "sphinx_inline_tabs",
        "black",
        "commitizen",
        "pre-commit",
    ],
)
