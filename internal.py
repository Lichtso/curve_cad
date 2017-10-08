#  ***** GPL LICENSE BLOCK *****
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#  All rights reserved.
#
#  ***** GPL LICENSE BLOCK *****

import bpy, math, cmath
from mathutils import Vector, Matrix
from collections import namedtuple

SplineBezierSegement = namedtuple('SplineBezierSegement', 'spline points beginIndex endIndex beginPoint endPoint params')
AABB = namedtuple('AxisAlignedBoundingBox', 'center dimensions')

def aabbOfPoints(points):
    min = Vector(points[0])
    max = Vector(points[0])
    for point in points:
        for i in range(0, 3):
            if min[i] > point[i]:
                min[i] = point[i]
            if max[i] < point[i]:
                max[i] = point[i]
    return AABB(center=(max+min)*0.5, dimensions=max-min)

def aabbIntersectionTest(a, b, tollerance=0.0):
    for i in range(0, 3):
        if abs(a.center[i]-b.center[i]) > (a.dimensions[i]+b.dimensions[i]+tollerance):
            return False
    return True

def bezierPointAt(points, t):
    s = 1-t
    return s*s*s*points[0] + 3*s*s*t*points[1] + 3*s*t*t*points[2] + t*t*t*points[3]

def bezierTangentAt(points, t):
    s = 1-t
    return s*s*(points[1]-points[0])+2*s*t*(points[2]-points[1])+t*t*(points[3]-points[2])
    # return s*s*points[0] + (s*s-2*s*t)*points[1] + (2*s*t-t*t)*points[2] + t*t*points[3]

def bezierSubivideAt(points, params):
    if len(params) == 0:
        return []
    newPoints = []
    newPoints.append(points[0]+(points[1]-points[0])*params[0])

    for index, param in enumerate(params):
        paramLeft = param
        if index > 0:
            paramLeft -= params[index-1]
        paramRight = -param
        if index == len(params)-1:
            paramRight += 1.0
        else:
            paramRight += params[index+1]

        point = bezierPointAt(points, param)
        tangent = bezierTangentAt(points, param)
        newPoints.append(point-tangent*paramLeft)
        newPoints.append(point)
        newPoints.append(point+tangent*paramRight)

    newPoints.append(points[3]-(points[3]-points[2])*(1.0-params[-1]))
    return newPoints

def subdivideBezierSegmentAtParams(segment):
    # Blender only allows uniform subdivision. Use this method to subdivide at arbitrary params.
    # NOTE: Segment.params must be sorted in ascending order

    if len(segment.params) == 0:
        return
    newPoints = bezierSubivideAt(segment.points, segment.params)
    begin = segment.spline.bezier_points[segment.beginIndex]
    end = segment.spline.bezier_points[segment.endIndex]

    bpy.ops.curve.select_all(action='DESELECT')
    begin.select_right_handle = True
    end.select_left_handle = True
    begin.handle_left_type = 'FREE'
    begin.handle_right_type = 'FREE'
    end.handle_left_type = 'FREE'
    end.handle_right_type = 'FREE'
    bpy.ops.curve.subdivide(number_cuts=len(segment.params))

    begin = segment.spline.bezier_points[segment.beginIndex]
    if segment.endIndex > 0:
        end = segment.spline.bezier_points[segment.endIndex+len(segment.params)]
    else:
        end = segment.spline.bezier_points[segment.endIndex]

    begin.select_right_handle = False
    end.select_left_handle = False
    begin.handle_right = newPoints[0]
    end.handle_left = newPoints[-1]
    for index in range(0, len(segment.params)):
        newPoint = segment.spline.bezier_points[segment.beginIndex+1+index]
        newPoint.handle_left_type = 'FREE'
        newPoint.handle_right_type = 'FREE'
        newPoint.select_left_handle = False
        newPoint.select_control_point = False
        newPoint.select_right_handle = False
        newPoint.handle_left = newPoints[index*3+1]
        newPoint.co = newPoints[index*3+2]
        newPoint.handle_right = newPoints[index*3+3]

def subdivideBezierSegmentsAtParams(segments):
    # Segements of the same spline have to be sorted with the higher indecies being subdivided first
    # to prevent the lower ones from shifting the indecies of the higher ones
    groups = {}
    for segment in segments:
        spline = segment.spline
        if (spline in groups) == False:
            groups[spline] = []
        group = groups[spline]
        group.append(segment)
    for spline in groups:
        group = groups[spline]
        group.sort(key=(lambda segment: segment.beginIndex), reverse=True)
        for segment in group:
            subdivideBezierSegmentAtParams(segment)

def bezierSegments(splines, selection_only):
    segments = []
    for spline in splines:
        if spline.type != 'BEZIER':
            continue
        for index, next in enumerate(spline.bezier_points):
            if index == 0 and not spline.use_cyclic_u:
                continue
            prev = spline.bezier_points[index-1]
            if not selection_only or prev.select_right_handle or next.select_left_handle:
                segments.append(SplineBezierSegement(
                                spline=spline,
                                beginIndex=index-1 if index > 0 else len(spline.bezier_points)-1,
                                endIndex=index,
                                beginPoint=prev,
                                endPoint=next,
                                points=[Vector(prev.co), Vector(prev.handle_right), Vector(next.handle_left), Vector(next.co)],
                                params=[]))
    return segments
