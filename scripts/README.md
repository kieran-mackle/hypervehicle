# HYPERVEHICLE Scripts

This directory contains a collection of useful scripts related to running
simulations with Hypervehicle and Batchjob.

## Cart3D Preparation Script

`cartprep.py`

This script converts all `*.stl` files in the current working directory
to `*.tri` files, combines and intersects them, producing a `Components.i.tri`
file ready to run in Cart3D.

It will also read a `task_parameters.txt` file, and update the `input.cntl` file
with the appropriate flow conditions.
