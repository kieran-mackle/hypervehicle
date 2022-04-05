from setuptools import setup, find_packages
setup(
      name='hypervehicle',
      version='0.1.0',
      author='Ingo Jahn, Kieran Mackle',
      author_email='ingo.jahn@usq.edu.au, k.mackle@uq.net.au',
      packages=find_packages(),
      url='https://github.com/kieran-mackle/hypervehicle',
      description='Parametric geometry generation tool for hypersonic vehicles',
      long_description=open('README.md').read(),
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
                        ],
      )
