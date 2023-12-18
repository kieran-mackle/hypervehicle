(overview)=
# Overview of HyperVehicle

The <tt>HyperVehicle</tt> package employs a component build up approach. 
Using the core component types, a wide range of geometries can 
be constructed. While there is an upfront cost associated with 
constructing a good parametric model, the generalised nature of 
<tt>HyperVehicle</tt> allows you to programmatically define relationships 
between geometric components.


## The HyperVehicle Workflow
Before diving into the component definitions, this section provides
an overview of the <tt>HyperVehicle</tt> workflow, and how vehicle 
geometries are created.

A geometry is constructed by combining a collection of *components* together. 
Each component type of `hypervehicle` inherits from the {py:class}`.Component` 
class.

Once the components of a geometry are defined, they can be added to an instance
of the {py:class}`.Vehicle` class, which acts as a container for multiple 
{py:class}`.Component`s. Components can be added to a `Vehicle` object using the
{py:meth}`.Vehicle.add_component` method. The {py:class}`.Vehicle` object offers
many useful features and methods, allowing you to name a vehicle, name individual 
components, apply transformations, estimate the vehicle's inertial properties, and
more.
