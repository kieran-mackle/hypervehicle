---
title: 'HyperVehicle: A Python package for parametric geometry generation'
tags:
  - Python
  - engineering
  - cfd
  - hypersonics
  - geometry
authors:
  - name: Kieran Mackle
    orcid: 0000-0001-8245-250X
    affiliation: 1
  - name: Ingo Jahn
    orcid: 0000-0001-6962-8187
    affiliation: 2
affiliations:
 - name: The University of Queensland, Australia
   index: 1
 - name: The University of Southern Queensland, Australia
   index: 2
date: 01 January 2024
bibliography: paper.bib
---

# Summary

HyperVehicle is a Python package for the rapid generation of component-based parametric geometries.
With a focus on hypersonic vehicle generation, a selection of vehicle-specific component constructors are provided for convenience.
More generic component types are also available, providing common CAD functions such as revolves and sweeps.
There are many more features available, such as geometry rotation and transformations, such as curvatures and scaling.


# Statement of need

An automated geometry generation tool lies at the heart of any design optimisation framework.
Without such a tool, design spaces cannot be efficiently explored to locate opimal designs.
Commonly, geometry design and generation will take place using a computer-aided design (CAD) software.
Although commercial CAD packages offer a comprehensive toolset for design, they are not usually intended for large-scale, automated design optimisation.
As such, they do not have the functionality desired for such a purpose.
There are also existing Python-native geometry packages [@numpy_stl, @PyMesh], which provide a good foundation for a more flexible application, as they offer a low-level API for the general purpose of generating meshes from vertex coordinate and face IDs.
HyperVehicle differs from and extends these packages in many ways.


As the name implies, HyperVehicle has a specific focus on the generation of vehicle geometries.
This focus is highlighted by the vehicle-specific component types provided, which act as templates for easily constructing a wide variety of vehicle geometries.
This does not mean that the package cannot be used for more general geometry generation purposes.
It also features all of the generic tools which are commonly found in a CAD software, but with the convenience and flexibility that Python affords, making it easy to integrate with.
Importantly, HyperVehicle also offers the capability to differentiate geometry with respect to design parameters, making it an extremely useful component of a gradient-based design optimisation framework.
This is exemplified in the hypersonic aerodynamics design and optimisation 
papers relying on HyperVehicle [@MackleAFMC, @MackleShapeOpt, @MackleCoDesign].


Similar frameworks appear abundantly throughout literature [@Bowcutt1992, @Bowcutt2001], yet there does not appear to be a publicly available library offering the required capabilities.
HyperVehicle aims to fill this gap, by providing an open-source, user-friendly and production-ready automated CAD tool as a Python package.


# Example vehicle geometries
Below are some sample vehicle configurations generated using HyperVehicle.
Note that these geometries are made avaiable in the hypervehicle hangar.
More examples can be seen on the documentation website.

![Generic hypersonic waverider](images/waverider.png){ width=70% }
![DLR ReFEX](images/refex.png){ width=70% }
![NASA X43-A demonstrator](images/x43a.png){ width=70% }


# References
