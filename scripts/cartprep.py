#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil
from random import random


def run_stl2tri(stl_files: list):
    tri_files = []
    for file in stl_files:
        prefix = file.split('.')[0]
        tri_file = prefix + '.tri'
        os.system(f"stl2tri.pl {file} {tri_file}")
        tri_files.append(tri_file)
        
    os.system("rm *.comp.tri *.off")
    return tri_files    
    

def jitter_tri_files(tri_files):
    for file in tri_files:
        prefix = file.split('.')[0]
        x_pert = random()/100 # Max of 0.001, min of 0
        y_pert = random()/100 # Max of 0.001, min of 0
        z_pert = random()/100 # Max of 0.001, min of 0
        os.system(f"trix -x {x_pert} -y {y_pert} -z {z_pert} -o {prefix} {file}")


def shift_all(tri_files, x_shift, y_shift, z_shift, component: str = None,
              reverse: bool = False):
    if component is None:
        transform_files = tri_files
    else:
        transform_files = [component]
    
    for file in transform_files:
        prefix = file.split('.')[0]
        if reverse:
            os.system(f"trix -x {-x_shift} -y {-y_shift} -z {-z_shift} -o {prefix} {file}")
        else:
            os.system(f"trix -x {x_shift} -y {y_shift} -z {z_shift} -o {prefix} {file}")
    

def rotate_all(tri_files, x_rot, y_rot, z_rot, component: str = None,
               reverse: bool = False):
    if component is None:
        transform_files = tri_files
    else:
        transform_files = [component]
    
    for file in transform_files:
        prefix = file.split('.')[0]
        if reverse:
            order = ['z', 'y', 'x']
        else:
            order = ['x', 'y', 'z']
        
        for axis in order:
            rotation = vars()[f'{axis}_rot']
            os.system(f"trix -r{axis} {rotation} -o {prefix} {file}")
        

def run_comp2tri(tri_files):
    tri_files_str = ' '.join(tri_files)
    os.system(f"comp2tri -inflate -makeGMPtags {tri_files_str} -config")


def run_intersect():
    os.system("intersect")


def get_stl_files():
    path = os.getcwd()
    all_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
    stl_files = []
    for file in all_files:
        if file.split('.')[-1] == 'stl':
            stl_files.append(file)
    return stl_files


def check_for_success():
    success = False
    if os.path.isfile('Components.i.tri'):
        success = True
        print("Success!")
    return success


def create_intersected_components():
    stl_files = get_stl_files()
    tri_files = run_stl2tri(stl_files)
    
    # First try intersect original files
    run_comp2tri(tri_files)
    run_intersect()
    successful = check_for_success()
    if successful:
        return True
    
    # First attempt failed, try jittering components
    jitter_tri_files(tri_files)
    run_comp2tri(tri_files)
    run_intersect()
    successful = check_for_success()
    if successful:
        return True
    
    # That also failed, try arbitrary shift away
    for attempt in range(3):
        # Define shifts
        x_shift = random()*10 # Max of 10, min of 0
        y_shift = random()*10 # Max of 10, min of 0
        z_shift = random()*10 # Max of 10, min of 0

        # Define rotations
        x_rot = random()*10 # Max of 10, min of 0
        y_rot = random()*10 # Max of 10, min of 0
        z_rot = random()*10 # Max of 10, min of 0
        
        # Apply transformations
        shift_all(tri_files, x_shift, y_shift, z_shift)
        rotate_all(tri_files, x_rot, y_rot, z_rot)
        
        # Make attempt
        run_comp2tri(tri_files)
        run_intersect()
        successful = check_for_success()
        
        if successful:
            # Move configuration back to original location
            shift_all(tri_files, x_shift, y_shift, z_shift, 'Components.i.tri', True)
            rotate_all(tri_files, x_rot, y_rot, z_rot, 'Components.i.tri', True)
            if successful:
                return True
        else:
            # Need to reset tri files
            tri_files = run_stl2tri(stl_files)
            
    print("\n\nAll attempts to intersect have failed :(")
    return False


if __name__ == '__main__':
    intersected = create_intersected_components()
    
    if intersected:
        # Intersect was successful, proceed
        # os.system("autoInputs -r 1")
        
        # Read parameter file to load geometry parameters
        params = {}
        with open('task_parameters.txt') as f:
            lines = f.readlines()
        
        for line in lines:
            split_line = line.split(' = ')
            if len(split_line) != len([line]):
                # Line included a variable assignment
                variable_name = split_line[0]
                variable_value = float(split_line[1].split('#')[0])
                params[variable_name] = variable_value
        
        # Modify Cart3D input.cntl file with sim params
        with open('temp', 'w+') as new_file:
            with open('input.cntl', 'r') as old_file:
                for line in old_file:
                    
                    if line.startswith('Mach'):
                        line = f'Mach  {params["MACH"]}\n'
                    elif line.startswith('alpha'):
                        line = f'alpha  {params["AOA"]}\n'
                    elif line.startswith('beta'):
                        line = f'beta  {params["BETA"]}\n'
                        
                    new_file.write(line)
        
            # Overwrite input.cntl file
            os.remove('input.cntl')
            shutil.move(os.path.join('temp'),
                        os.path.join('input.cntl'))