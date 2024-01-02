# Vehicle Specific Component Types

The following section describes the component types defined for helping specifically
with vehicle geometries.

## Wing Components
For constructing wings, the {py:class}`.Wing` component
function is available. A wing component is constructed 
by first defining the planform according to the points 
defined in the schematic below.

![Wing planform](../../images/components/wing_planform.png "Wing planform")

Next, thickness is added to the wing using user-defined 
thickness functions. These function can be as simple as 
providing a constant thickness, or as complex as 
providing 3-dimensionally varying thickness.

![Wing thickness functions](../../images/components/wing_thickness.png "Wing thickness functions")


A trailing-edge flap can easily be added to a wing component 
by defining the flap length and flap type. The flap angle can 
also be provided to deflect the flap.

![Wing flap](../../images/components/flap.png "Wing flap")



## Fin Components
Another helper component is the {py:class}`.Fin`.
Fin components are very similar to wing components, but 
offer a few convenient options to assist in positioning. 
As with a wing, a fin is first defined by its planform 
according to the points shown below.

![Fin planform definition](../../images/components/fin_planform.png "Fin planform definition")

A rudder can also be added to the trailing-edge of a fin, 
producing something like that shown below.

![Rudder length](../../images/components/rudder_length.png "Rudder length")


To assist in fin positioning, the fin angle argument can 
be used to specify the angle of the fin, as rotated about 
the vehicles longitudinal (x) axis.

![Fin angle](../../images/components/fin_angle.png "Fin angle definition")


The pivot angle of the fin can also be controlled using 
the pivot angle argument. The pivot point can also be 
specified.

![Fin pivot angle](../../images/components/fin_pivot.png "Fin pivot angle")

