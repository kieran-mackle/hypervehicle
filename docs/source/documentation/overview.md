(overview)=
# Overview of HyperVehicle

The *hypervehicle* package employs a component build up approach. 
Using the core component types, a wide range of geometries can 
be constructed. While there is an upfront cost associated with 
constructing a good parametric model, the generalised nature of 
*hypervehicle* allows you to programmatically define relationships 
between geometric components.


## The *HyperVehicle* Workflow
Before diving into the component definitions, this section provides
an overview of the *hypervehicle* workflow, and how geometries are
created from arbitrary definitions.

A vehicle is constructed by stacking a collection of *components* 
together. Each component type defined below inherits from the 
{py:class}`.Component` class.

Below is a summary of other terminology:
- Patch: a *patch* is a surface represented by a continuous function
- Surface: a *surface* is a surface represented by a discrete function
- Mesh: a *mesh* is a [Numpy STL](https://numpy-stl.readthedocs.io/en/latest/stl.html#stl-mesh) object

