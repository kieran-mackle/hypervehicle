"""
Wing Geometry Generator for gliding hypersonic vehicle.

Authors: Ingo Jahn, Kieran Mackle
Created on: 02/07/2021
Last Modified: 02/07/2021
"""

from eilmer.geom.vector3 import Vector3
from eilmer.geom.path import Line, Path, ArcLengthParameterizedPath
from eilmer.geom.surface import CoonsPatch, ParametricSurface

import numpy as np
from stl import mesh  # part of 'numpy-stl' package

class SubRangedPath(Path):
    """
    A Path reparameterized to a subset of the original between t0 and t1.
    If t1 < t0 the path will be traveresed in the reverse direction.
    """
    __slots__ = ['underlying_path', 't0', 't1']

    def __init__(self, underlying_path, t0, t1):
        if isinstance(underlying_path, Path):
            self.underlying_path = underlying_path
            self.t0 = t0
            self.t1 = t1
        else:
            raise NotImplementedError("underlying_path should be a type of Path")
        return

    def __repr__(self):
        return "ArcLengthParameterizedPath(underlying_path={}, t0={}, t1={})".format(
            self.underlying_path, self.t0, self.t1)

    def __call__(self, t):
        t_dash = self.t0 + t * (self.t1 - self.t0)
        return self.underlying_path(t_dash)

    def length(self):
        return self.underlying_path.length()

class ElipsePath(Path):
    """
    A path following a quarter elipse from a -> b, around c

    b - _
          -_
            \
    c       a

    """
    __slots__ = ['centre', 'thickness', 'LE_width', 'side']

    def __init__(self, centre, thickness, LE_width, side):
        self.centre = centre
        self.thickness = thickness
        self.LE_width = LE_width
        self.side = side

    def __repr__(self):
        return "ElipsePath"

    def __call__(self, r):
        # establish local elipse shapes
        a = self.LE_width
        b = abs(self.thickness)
        #a = b*self.LE_ratio(t).y

        # construct elipse
        if self.side == 'bot':
            theta = 0 + r * np.pi/2
        elif self.side == 'top':
            theta = -np.pi / 2 + r * np.pi/2
        else:
            raise Exception("Value of 'side' not supported")

        x_elipse = a*b / np.sqrt(b*b + (a*np.tan(theta))**2)
        y_elipse = x_elipse * np.tan(theta)
        # set elipse angle
        angle = np.pi/2
        # add elipse to overall shape
        x = x_elipse * np.cos(angle)
        y = x_elipse * np.sin(angle)
        z = y_elipse
        elipse_base = self.centre
        return elipse_base + Vector3(x=x, y=y, z=z)

class OffsetPathFunction(Path):
    """
    Creates a line with an offset applied
    """
    __slots__ = ['underlying_line', 'offset_function']

    def __init__(self, underlying_line, offset_function):
        self.underlying_line = underlying_line
        self.offset_function = offset_function

    def __repr__(self):
        return "Offset path function"

    def __call__(self, t):
        # calculate postion
        pos = self.underlying_line(t)

        # calculate local thickness
        offset = self.offset_function(x=pos.x, y=pos.y)

        return pos+offset

class GeometricMeanPathFunction(Path):
    """
    Creates a line which is the geometric mean of two underlying lines.
    """
    __slots__ = ['underlying_line_1', 'underlying_line_2']

    def __init__(self, underlying_line_1, underlying_line_2):
        self.underlying_line_1 = underlying_line_1
        self.underlying_line_2 = underlying_line_2

    def __repr__(self):
        return "Mean path function"

    def __call__(self, t):
        # calculate postion
        pos_1 = self.underlying_line_1(t)
        pos_2 = self.underlying_line_2(t)

        # calculate local thickness
        mean_point = 0.5 * (pos_1 + pos_2)

        return mean_point

class OffsetPatchFunction(ParametricSurface):
    """
    Creates a patch with an offset applied.
    """
    __slots__ = ['underlying_surf', 'function']

    def __init__(self, underlying_surf, function):
        self.underlying_surf = underlying_surf
        self.function = function

    def __repr__(self):
        str = " + offset function"
        str = self.underlying_surf.__repr__() + str
        return str

    def __call__(self, r, s):
        pos = self.underlying_surf(r, s)
        offset = self.function(pos.x, pos.y, pos.z)
        return self.underlying_surf(r, s) + offset


class LeadingEdgePatchFunction(ParametricSurface):
    """
    Creates Leading Edge by pair of guiding lines and LE_width function.
    """
    __slots__ = ['centralLine', 'thickness_function', 'LE_width_function',
                 't0', 't1', 'side']

    def __init__(self, centralLine, thickness_function, LE_width_function,
                 t0, t1, side='top'):
        self.centralLine = centralLine
        self.thickness_function = thickness_function
        self.t0 = t0
        self.t1 = t1
        self.LE_width_function = LE_width_function
        self.side = side

    def __repr__(self):
        return "Leading edge surface"

    def __call__(self, t_raw, r):
        # convert to global t
        #t = self.t0 + t_raw * (self.t1 - self.t0)
        t = self.t1 - t_raw * (self.t1 - self.t0)
        # calculate local thickness
        pos = self.centralLine(t)
        thickness = self.thickness_function(x=pos.x, y=pos.y).z  # we only need to get z-value
        LE_width = self.LE_width_function(t)

        # establish local elipse shapes
        elipse = ElipsePath(centre=Vector3(x=0., y=0., z=0),
                            thickness=thickness,
                            LE_width=LE_width,
                            side=self.side)
        elipse_norm = ArcLengthParameterizedPath(underlying_path=elipse)
        pos_elipse = elipse_norm(r)
        x_elipse = pos_elipse.y
        # y_elipse = pos_elipse.x
        z_elipse = pos_elipse.z

        # set elipse angle
        if t == 0:
            angle = np.pi/2
        elif t == 1:
            angle = 0
        else:
            dt = min([0.001, abs(1.-t), abs(t-0.)])
            plus = self.centralLine(t+dt)
            minus = self.centralLine(t-dt)
            angle = np.arctan2((plus.y-minus.y), (plus.x-minus.x)) + np.pi/2

        #print(angle, x_elipse)

        # add elipse to overall shape
        x = x_elipse * np.cos(angle)
        y = x_elipse * np.sin(angle)
        z = z_elipse
        elipse_base = self.centralLine(t)
        return elipse_base + Vector3(x=x, y=y, z=z)

class MeanLeadingEdgePatchFunction(ParametricSurface):
    """
    Creates Leading Edge by mean line and guiding line, and LE_width function.
    """
    __slots__ = ['mean_line', 'guide_line', 'LE_width_function',
                 't0', 't1', 'side']

    def __init__(self, mean_line, guide_line, LE_width_function,
                 t0, t1, side='top'):
        self.mean_line = mean_line
        self.guide_line = guide_line
        self.t0 = t0
        self.t1 = t1
        self.LE_width_function = LE_width_function
        self.side = side

    def __repr__(self):
        return "Leading edge surface patch"

    def __call__(self, t_raw, r):
        # Convert to global t
        t = self.t1 - t_raw * (self.t1 - self.t0)

        # Calculate local thickness
        mean_point = self.mean_line(t)
        guide_point = self.guide_line(t)
        thickness = mean_point.z - guide_point.z
        LE_width = self.LE_width_function(t)

        # Establish local elipse shapes
        elipse = ElipsePath(centre=Vector3(x=0., y=0., z=0),
                            thickness=thickness,
                            LE_width=LE_width,
                            side=self.side)
        elipse_norm = ArcLengthParameterizedPath(underlying_path=elipse)
        pos_elipse = elipse_norm(r)
        x_elipse = pos_elipse.y
        z_elipse = pos_elipse.z

        # set elipse angle
        if t == 0:
            angle = np.pi/2
        elif t == 1:
            angle = 0
        else:
            dt = min([0.001, abs(1.-t), abs(t-0.)])
            plus = self.mean_line(t+dt)
            minus = self.mean_line(t-dt)
            angle = np.arctan2((plus.y-minus.y), (plus.x-minus.x)) + np.pi/2

        # Add elipse to overall shape
        x = x_elipse * np.cos(angle)
        y = x_elipse * np.sin(angle)
        z = z_elipse

        return mean_point + Vector3(x=x, y=y, z=z)

class FlatLeadingEdgePatchFunction(ParametricSurface):
    """
    Creates a flat leading edge between two paths.

    TODO - adapt this to allow sharp LE
    """
    __slots__ = ['path1', 'path2', 't0', 't1']

    def __init__(self, path1, path2, t0, t1):
        self.path1 = path1
        self.path2 = path2

        # Parametric section of mean and guide line
        self.t0 = t0
        self.t1 = t1

    def __repr__(self):
        return "Flat leading edge surface patch"

    def __call__(self, t_raw, r):
        # Convert to global t
        t = self.t1 - t_raw * (self.t1 - self.t0)

        # Calculate local thickness
        point1 = self.path1(t)
        point2 = self.path2(t)

        return Vector3(x=point1.x, y=point1.y, z=(1-r)*point1.z + r*point2.z)

class TrailingEdgePath(Path):
    """
    A path following rear of wing
    """
    __slots__ = ['A0', 'B0', 'thickness_function']

    def __init__(self, A0, B0, thickness_function):
        self.A0 = A0
        self.B0 = B0
        self.thickness_function = thickness_function
        self.Line = Line(p0=A0, p1=B0)

    def __repr__(self):
        return "Trailing Egde Path"

    def __call__(self, r):
        # calculate postion
        pos = self.Line(r)
        # calculate local thickness
        offset = self.thickness_function(x=pos.x, y=pos.y)
        return pos+offset


class TrailingEdgePatch(ParametricSurface):
    """
    Fucntion to create a trailing edge patch
    """
    __slots__=['A0', 'B0', 'TE_path', 'flap_length', 'flap_angle', 'side']

    def __init__(self, A0, B0, TE_path,
                       flap_length, flap_angle=0., side='top'):
        self.A0 = A0
        self.B0 = B0
        self.TE_path = TE_path
        self.flap_length= flap_length
        self.flap_angle = flap_angle
        self.side = side
        self.preparation()

    def __repr__(self):
        return "Trailing Edge for '{0}' with flap_angle={1} deg".format(self.side, np.rad2deg(self.flap_angle))

    def __call__(self, r, s):
        return self.patch(r, s)

    def preparation(self):
        if self.side == 'top':
            north = self.TE_path
            south = Line(p0=Vector3(x=self.A0.x-self.flap_length, y=self.A0.y,
                                    z=0.+self.flap_length*np.sin(self.flap_angle)),
                         p1=Vector3(x=self.B0.x-self.flap_length, y=self.B0.y,
                                    z=0.+self.flap_length*np.sin(self.flap_angle)))
            west = Line(p0=south(0.), p1=north(0.))
            east = Line(p0=south(1.), p1=north(1.))
        elif self.side == 'bot':
            south = self.TE_path
            north = Line(p0=Vector3(x=self.A0.x-self.flap_length, y=self.A0.y,
                                    z=0.+self.flap_length*np.sin(self.flap_angle)),
                         p1=Vector3(x=self.B0.x-self.flap_length, y=self.B0.y,
                                    z=0.+self.flap_length*np.sin(self.flap_angle)))
            west = Line(p0=south(0.), p1=north(0.))
            east = Line(p0=south(1.), p1=north(1.))
        else:
            raise Exception("Value of 'side' not supported")
        self.patch = CoonsPatch(south=south, north=north, west=west, east=east)

class MeanTrailingEdgePatch(ParametricSurface):
    """
    Fucntion to create a trailing edge patch, with flap angle defined
    from the geometric mean of upper and lower surfaces.
    """
    __slots__=['mean_line', 'TE_path', 'flap_length', 'flap_angle', 'side']

    def __init__(self, mean_line, TE_path, flap_length, flap_angle=0, side='top'):
        self.mean_line = mean_line
        self.TE_path = TE_path
        self.flap_length= flap_length
        self.flap_angle = flap_angle
        self.side = side
        self.preparation()

    def __repr__(self):
        return f"Trailing Edge for '{self.side}' with flap_angle={np.rad2deg(self.flap_angle)} deg"

    def __call__(self, r, s):
        return self.patch(r, s)

    def preparation(self):
        if self.side == 'top':
            north = self.TE_path
            south = Line(p0=Vector3(x=self.mean_line(0).x-self.flap_length*np.cos(self.flap_angle),
                                    y=self.mean_line(0).y,
                                    z=self.mean_line(0).z+self.flap_length*np.sin(self.flap_angle)),
                         p1=Vector3(x=self.mean_line(1).x-self.flap_length*np.cos(self.flap_angle),
                                    y=self.mean_line(1).y,
                                    z=self.mean_line(1).z+self.flap_length*np.sin(self.flap_angle)))


            west = Line(p0=south(0.), p1=north(0.))
            east = Line(p0=south(1.), p1=north(1.))

        elif self.side == 'bot':
            south = self.TE_path
            north = Line(p0=Vector3(x=self.mean_line(0).x-self.flap_length*np.cos(self.flap_angle),
                                    y=self.mean_line(0).y,
                                    z=self.mean_line(0).z+self.flap_length*np.sin(self.flap_angle)),
                         p1=Vector3(x=self.mean_line(1).x-self.flap_length*np.cos(self.flap_angle),
                                    y=self.mean_line(1).y,
                                    z=self.mean_line(1).z+self.flap_length*np.sin(self.flap_angle)))
            west = Line(p0=south(0.), p1=north(0.))
            east = Line(p0=south(1.), p1=north(1.))

        else:
            raise Exception("Value of 'side' not supported")

        self.patch = CoonsPatch(south=south, north=north, west=west, east=east)

class CurvedPatch(ParametricSurface):
    """
    Adds curvature in x or y direction to an existing patch
    """
    __slots__=['underlying_surf', 'direction', 'fun', 'fun_dash']

    def __init__(self, underlying_surf, direction=None,
                 fun=None, fun_dash=None):
        self.underlying_surf = underlying_surf
        self.direction = direction
        self.fun = fun
        self.fun_dash = fun_dash

    def __repr__(self):
        return self.underlying_surf.__repr__() + " with added curvature"

    def __call__(self, r, s):
        if self.fun == None or self.fun_dash == None:
            raise Exception("Both 'fun' and 'fun_dash' need to be specified.")
        pos = self.underlying_surf(r, s)
        offset = self.fun(pos.x, pos.y)
        slope = self.fun_dash(pos.x, pos.y)
        angle = np.arctan2(slope, 1)
        if self.direction == 'x':
            pos_new = Vector3(x=pos.x-pos.z*np.sin(angle),
                              y=pos.y,
                              z=pos.z*np.cos(angle)+offset)
            return pos_new
        if self.direction == 'y':
            pos_new = Vector3(x=pos.x,
                              y=pos.y-pos.z*np.sin(angle),
                              z=offset+pos.z*np.cos(angle))
            return pos_new


def parametricSurfce2stl(parametric_surface, triangles_per_edge, mirror_y=False, re_evaluate_centroid=False):
    """
    Function to convert parametric_surface generated using the Eilmer Geometry Package into a stl mesh objecy.
    Inputs:
        parametric_surface - surface object
        triangles_per_edge - resolution for stl object.
        mirror_y - create mirror image about x-z plane
    Outputs:
        stl_mesh - triangulated mesh object suitable for numpy-stl
    """
    # create list of vertices
    r_list = np.linspace(0., 1., triangles_per_edge+1)
    s_list = np.linspace(0., 1., triangles_per_edge+1)
    # create vertices for corner points of each quad cell
    vertices = np.zeros(((triangles_per_edge+1)**2 + triangles_per_edge**2, 3))
    for i, r in enumerate(r_list):
        for j, s in enumerate(s_list):
            pos = parametric_surface(r, s)
            if mirror_y == False:
                vertices[j*(triangles_per_edge+1)+i] = [pos.x, pos.y, pos.z]
            else:
                vertices[j*(triangles_per_edge+1)+i] = [pos.x, -pos.y, pos.z]
    # create vertices for centre point of each quad cell, which is used to split
    # each cell into 4x triangles
    index = (triangles_per_edge+1)**2
    for i in range(triangles_per_edge):
        for j in range(triangles_per_edge):
            if re_evaluate_centroid is True:
                r = 0.5 * (r_list[i] + r_list[i+1])
                r = 0.5 * (s_list[i] + s_list[i+1])
                pos = parametric_surface(r, s)
                if mirror_y == False:
                    vertices[index+(j*triangles_per_edge+i)] = [pos.x, pos.y, pos.z]
                else:
                    vertices[index+(j*triangles_per_edge+i)] = [pos.x, -pos.y, pos.z]
            else:
                r0 = r_list[i]
                r1 = r_list[i+1]
                s0 = s_list[i]
                s1 = s_list[i+1]
                pos00 = parametric_surface(r0, s0)
                pos10 = parametric_surface(r1, s0)
                pos01 = parametric_surface(r0, s1)
                pos11 = parametric_surface(r1, s1)
                pos_x = 0.25 * (pos00.x + pos10.x + pos01.x + pos11.x)
                pos_y = 0.25 * (pos00.y + pos10.y + pos01.y + pos11.y)
                pos_z = 0.25 * (pos00.z + pos10.z + pos01.z + pos11.z)
                if mirror_y == False:
                    vertices[index+(j*triangles_per_edge+i)] = [pos_x, pos_y, pos_z]
                else:
                    vertices[index+(j*triangles_per_edge+i)] = [pos_x, -pos_y, pos_z]
    # create list of faces
    faces = []  #np.zeros((triangles_per_edge**2*4, 3))
    for i in range(triangles_per_edge):
        for j in range(triangles_per_edge):
            p00 = j*(triangles_per_edge+1)+i  # bottom left
            p10 = j*(triangles_per_edge+1)+i+1  # bottom right
            p01 = (j+1)*(triangles_per_edge+1)+i  # top left
            p11 = (j+1)*(triangles_per_edge+1)+i+1   # top right
            pzz = index + (j*triangles_per_edge+i)  # vertex at centre of cell
            if mirror_y == False:
                faces.append([p00, p10, pzz])
                faces.append([p10, p11, pzz])
                faces.append([p11, p01, pzz])
                faces.append([p01, p00, pzz])
            else:
                faces.append([p00, pzz, p10])
                faces.append([p10, pzz, p11])
                faces.append([p11, pzz, p01])
                faces.append([p01, pzz, p00])
    faces = np.array(faces)
    # create the mesh object
    stl_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            stl_mesh.vectors[i][j] = vertices[f[j],:]
    return stl_mesh

class ConePatch(ParametricSurface):
    """
    Creates a patch describing a cone (or cylinder) between two rings.
    """
    __slots__ = ['x0', 'x1', 'r0', 'r1', 'angle0', 'angle1']

    def __init__(self, x0, x1, r0, r1, angle0, angle1):
        self.x0 = x0
        self.x1 = x1
        self.r0 = r0
        self.r1 = r1
        self.angle0 = angle0
        self.angle1 = angle1

    def __repr__(self):
        str = "Cone Patch"
        return str

    def __call__(self, r, s):
        angle = self.angle0 * (1-s) + self.angle1 * s
        x0 = self.x0
        y0 = self.r0 * np.cos(angle)
        z0 = self.r0 * np.sin(angle)
        x1 = self.x1
        y1 = self.r1 * np.cos(angle)
        z1 = self.r1 * np.sin(angle)
        x = x0 * (1-r) + x1 * r
        y = y0 * (1-r) + y1 * r
        z = z0 * (1-r) + z1 * r
        return Vector3(x=x, y=y, z=z)

class BluntConePatch(ParametricSurface):
    """
    Creates a patch describing a blunt cone.

    Parameters:
        x0: x coordinate of base of cone
        y0: y coordinate of base of cone
        rn: spherical nose radius
        rb: radius of cone at base of cone
        L: length of nominal cone (base to tip before spherical blunting)
        angle0: start revolve angle
        angle1: end revolve angle

    """
    __slots__ = ['x0', 'y0', 'rn', 'rb', 'L', 'angle0', 'angle1']

    def __init__(self, x0, y0, rn, rb, L, angle0, angle1):
        self.L = L
        self.x0 = x0
        self.y0 = y0
        self.rn = rn
        self.rb = rb
        self.angle0 = angle0
        self.angle1 = angle1

    def __repr__(self):
        str = "Blunt Cone Patch"
        return str

    def __call__(self, r, s):
        angle = self.angle0 * (1-s) + self.angle1 * s
        x0 = self.x0
        y0 = self.r0 * np.cos(angle)
        z0 = self.r0 * np.sin(angle)
        x1 = self.x1
        y1 = self.r1 * np.cos(angle)
        z1 = self.r1 * np.sin(angle)
        x = x0 * (1-r) + x1 * r
        y = y0 * (1-r) + y1 * r
        z = z0 * (1-r) + z1 * r
        return Vector3(x=x, y=y, z=z)


class RotatedPatch(ParametricSurface):
    """
    Rotates a surface about a point in an axis-specified direction.
    """
    __slots__ = ['underlying_surf', 'angle', 'axis', 'point']

    def __init__(self, underlying_surf, angle, axis='x', point=Vector3(x=0,y=0,z=0)):
        self.underlying_surf = underlying_surf
        self.angle = angle
        self.axis = axis.lower()
        self.point = point

    def __repr__(self):
        str = "+ Rotation by {} deg".format(np.rad2deg(self.angle))
        return self.underlying_surf.__repr__() + str

    def __call__(self, r, s):
        pos = self.underlying_surf(r, s) - self.point

        if self.axis == 'x':
            x = pos.x
            y = pos.y * np.cos(self.angle) - pos.z * np.sin(self.angle)
            z = pos.y * np.sin(self.angle) + pos.z * np.cos(self.angle)
        elif self.axis == 'y':
            x = pos.x * np.cos(self.angle) + pos.z * np.sin(self.angle)
            y = pos.y
            z = -pos.x * np.sin(self.angle) + pos.z * np.cos(self.angle)

        elif self.axis == 'z':
            x = pos.x * np.cos(self.angle) - pos.y * np.sin(self.angle)
            y = pos.x * np.sin(self.angle) + pos.y * np.cos(self.angle)
            z = pos.z

        return Vector3(x=x, y=y, z=z) + self.point
