# HYPERVEHICLE Components

The *hypervehicle* package employs a component build up approach. 
Using the core component types, a wide range of geometries can 
be constructed. While there is an upfront cost associated with 
constructing a good parametric model, the generalised nature of 
*hypervehicle* allows you to programmatically define relationships 
between geometric components.


## The *hypervehicle* Workflow
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



## General Component Types
The following defines the general component types of 
*hypervehicle*. These are not vehicle-specific.

### Revolved Component
Revolved geometries can be constructed using the 
{py:class}`.RevolvedComponent`. This component is defined
primarily by the line to be revolved.


### Swept Component
Swept geometries can be constructed using a 
{py:class}`.SweptComponent` type. This component
requires a series of cross-sectional patches to be
defined.


### Cube Component
A cube can be constructed via the {py:class}`.Cube`
class. It only requires the cube side length to be
provided.

### Sphere Component
A sphere can be constructed using the {py:class}`.Sphere`
class. Like the cube, it only requires a radius to 
be defined.


### Composite Component
In some advanced instances, you may wish to combine the 
patches of various components into a single component. Or,
you may have defined your own patch types. For this purpose,
the {py:class}`.CompositeComponent` class is available. This
class allows a user to stack individual {py:class}`.Component`
objects to form a composite, which can then be added to
the {py:class}`.Vehicle`, for example.





## Vehicle-Specific Component Types
The following section describes the component types defined for
helping with vehicle-specific geometries.

### Wing Components
For constructing wings, the {py:class}`.Wing` component
function is available. A wing component is constructed 
by first defining the planform according to the points 
defined in the schematic below.

![Wing planform](../images/components/wing_planform.png "Wing planform")

Next, thickness is added to the wing using user-defined 
thickness functions. These function can be as simple as 
providing a constant thickness, or as complex as 
providing 3-dimensionally varying thickness.

![Wing thickness functions](../images/components/wing_thickness.png "Wing thickness functions")


A trailing-edge flap can easily be added to a wing component 
by defining the flap length and flap type. The flap angle can 
also be provided to deflect the flap.

![Wing flap](../images/components/flap.png "Wing flap")



### Fin Components
Another helper component is the {py:class}`.Fin`.
Fin components are very similar to wing components, but 
offer a few convenient options to assist in positioning. 
As with a wing, a fin is first defined by its planform 
according to the points shown below.

![Fin planform definition](../images/components/fin_planform.png "Fin planform definition")

A rudder can also be added to the trailing-edge of a fin, 
producing something like that shown below.

![Rudder length](../images/components/rudder_length.png "Rudder length")


To assist in fin positioning, the fin angle argument can 
be used to specify the angle of the fin, as rotated about 
the vehicles longitudinal (x) axis.

![Fin angle](../images/components/fin_angle.png "Fin angle definition")


The pivot angle of the fin can also be controlled using 
the pivot angle argument. The pivot point can also be 
specified.

![Fin pivot angle](../images/components/fin_pivot.png "Fin pivot angle")

