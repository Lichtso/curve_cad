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

units = [
    ('-', 'None', '1.0', 0),
    ('px', 'Pixel', '1.0', 1),
    ('m', 'Meter', '1.0', 2),
    ('dm', 'Decimeter', '0.1', 3),
    ('cm', 'Centimeter', '0.01', 4),
    ('mm', 'Millimeter', '0.001', 5),
    ('yd', 'Yard', '0.9144', 6),
    ('ft', 'Foot', '0.3048', 7),
    ('in', 'Inch', '0.0254', 8)
]

param_tollerance = 0.0001
AABB = namedtuple('AxisAlignedBoundingBox', 'center dimensions')
Plane = namedtuple('Plane', 'normal distance')
Circle = namedtuple('Circle', 'plane center radius')

def nearestPointOfLines(originA, dirA, originB, dirB, tollerance=0.0):
    # https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points
    normal = dirA.cross(dirB)
    normalA = dirA.cross(normal)
    normalB = dirB.cross(normal)
    divisorA = dirA@normalB
    divisorB = dirB@normalA
    if abs(divisorA) <= tollerance or abs(divisorB) <= tollerance:
        return (float('nan'), float('nan'), originA, originB)
    else:
        paramA = (originB-originA)@normalB/divisorA
        paramB = (originA-originB)@normalA/divisorB
        return (paramA, paramB, originA+dirA*paramA, originB+dirB*paramB)

def lineIntersection(beginA, endA, beginB, endB):
    dirA = endA-beginA
    dirB = endB-beginB
    intersection = nearestPointOfLines(beginA, dirA, beginB, dirB)
    if intersection[0] != intersection[0] or intersection[0] < 0 or intersection[0] > 1 or intersection[1] < 0 or intersection[1] > 1:
        return None
    return (intersection[2]+intersection[3])*0.5

def linePlaneIntersection(lineBegin, lineEnd, plane):
    det = (lineEnd-lineBegin)@plane.normal
    return float('nan') if det == 0 else (plane.distance-lineBegin@plane.normal)/det
    # return float('nan') if det == 0 else (plane.normal*plane.distance-lineBegin)@plane.normal/det

def areaOfPolygon(vertices):
    area = 0
    for index, current in enumerate(vertices):
        prev = vertices[index-1]
        area += (current[0]+prev[0])*(current[1]-prev[1])
    return area*0.5

def circleOfTriangle(a, b, c):
    # https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_from_cross-_and_dot-products
    dirBA = a-b
    dirCB = b-c
    dirAC = c-a
    normal = dirBA.cross(dirCB)
    lengthBA = dirBA.length
    lengthCB = dirCB.length
    lengthAC = dirAC.length
    lengthN = normal.length
    if lengthN == 0:
        return None
    factor = -1/(2*lengthN*lengthN)
    alpha = (dirBA@dirAC)*(lengthCB*lengthCB*factor)
    beta = (dirBA@dirCB)*(lengthAC*lengthAC*factor)
    gamma = (dirAC@dirCB)*(lengthBA*lengthBA*factor)
    center = a*alpha+b*beta+c*gamma
    radius = (lengthBA*lengthCB*lengthAC)/(2*lengthN)
    plane = Plane(normal=normal/lengthN, distance=center@normal)
    return Circle(plane=plane, center=center, radius=radius)

def circleOfBezier(points, tollerance=0.000001):
    circle = circleOfTriangle(points[0], bezierPointAt(points, 0.5), points[3])
    if circle == None:
        return None
    samples = 16
    variance = 0
    for t in range(0, samples):
        variance += ((circle.center-bezierPointAt(points, (t+1)/(samples-1))).length/circle.radius-1) ** 2
    variance /= samples
    if variance > tollerance:
        return None
    return circle

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

def isPointInAABB(point, aabb, tollerance=0.0):
    for i in range(0, 3):
        if point[i] < aabb.center[i]-aabb.dimensions[i]*0.5-tollerance or point[i] > aabb.center[i]+aabb.dimensions[i]*0.5+tollerance:
            return False
    return True

def lineAABBIntersection(lineBegin, lineEnd, aabb):
    intersections = []
    for i in range(0, 3):
        normal = [0, 0, 0]
        normal = Vector(normal[0:i] + [1] + normal[i+1:])
        for j in range(-1, 2, 2):
            plane = Plane(normal=normal, distance=aabb.center[i]+j*aabb.dimensions[i]*0.5)
            param = linePlaneIntersection(lineBegin, lineEnd, plane)
            point = lineBegin+param*(lineEnd-lineBegin)
            if param > 0 and param < 1 and isPointInAABB(point, aabb):
                intersections.append((param, point))
    return intersections

def bezierPointAt(points, t):
    s = 1-t
    return s*s*s*points[0] + 3*s*s*t*points[1] + 3*s*t*t*points[2] + t*t*t*points[3]

def bezierTangentAt(points, t):
    s = 1-t
    return s*s*(points[1]-points[0])+2*s*t*(points[2]-points[1])+t*t*(points[3]-points[2])
    # return s*s*points[0] + (s*s-2*s*t)*points[1] + (2*s*t-t*t)*points[2] + t*t*points[3]

def bezierLength(points, beginT=0, endT=1, samples=1024):
    # https://en.wikipedia.org/wiki/Arc_length#Finding_arc_lengths_by_integrating
    vec = [points[1]-points[0], points[2]-points[1], points[3]-points[2]]
    dot = [vec[0]@vec[0], vec[0]@vec[1], vec[0]@vec[2], vec[1]@vec[1], vec[1]@vec[2], vec[2]@vec[2]]
    factors = [
        dot[0],
        4*(dot[1]-dot[0]),
        6*dot[0]+4*dot[3]+2*dot[2]-12*dot[1],
        12*dot[1]+4*(dot[4]-dot[0]-dot[2])-8*dot[3],
        dot[0]+dot[5]+2*dot[2]+4*(dot[3]-dot[1]-dot[4])
    ]
    # https://en.wikipedia.org/wiki/Trapezoidal_rule
    length = 0
    prev_value = math.sqrt(factors[4]+factors[3]+factors[2]+factors[1]+factors[0])
    for index in range(0, samples+1):
        t = beginT+(endT-beginT)*index/samples
        # value = math.sqrt(factors[4]*(t**4)+factors[3]*(t**3)+factors[2]*(t**2)+factors[1]*t+factors[0])
        value = math.sqrt((((factors[4]*t+factors[3])*t+factors[2])*t+factors[1])*t+factors[0])
        length += (prev_value+value)*0.5
        prev_value = value
    return length*3/samples

# https://en.wikipedia.org/wiki/Root_of_unity
# cubic_roots_of_unity = [cmath.rect(1, i/3*2*math.pi) for i in range(0, 3)]
cubic_roots_of_unity = [complex(1, 0), complex(-1, math.sqrt(3))*0.5, complex(-1, -math.sqrt(3))*0.5]
def bezierRoots(dists, tollerance=0.0001):
    # https://en.wikipedia.org/wiki/Cubic_function
    # y(t) = a*t^3 +b*t^2 +c*t^1 +d*t^0
    a = 3*(dists[1]-dists[2])+dists[3]-dists[0]
    b = 3*(dists[0]-2*dists[1]+dists[2])
    c = 3*(dists[1]-dists[0])
    d = dists[0]
    if abs(a) > tollerance: # Cubic
        E2 = a*c
        E3 = a*a*d
        A = (2*b*b-9*E2)*b+27*E3
        B = b*b-3*E2
        C = ((A+cmath.sqrt(A*A-4*B*B*B))*0.5) ** (1/3)
        roots = []
        for root in cubic_roots_of_unity:
            root *= C
            root = -1/(3*a)*(b+root+B/root)
            if abs(root.imag) < tollerance and root.real > -param_tollerance and root.real < 1.0+param_tollerance:
                roots.append(max(0.0, min(root.real, 1.0)))
        # Remove doubles
        roots.sort()
        for index in range(len(roots)-1, 0, -1):
            if abs(roots[index-1]-roots[index]) < param_tollerance:
                roots.pop(index)
        return roots
    elif abs(b) > tollerance: # Quadratic
        disc = c*c -4*b*d
        if disc < 0:
            return []
        disc = sqrt(disc)
        return [(-c-disc)/(2*b), (-c+disc)/(2*b)]
    elif abs(c) > tollerance: # Linear
        root = -d/c
        return [root] if root >= 0.0 and root <= 1.0 else []
    else: # Constant / Parallel
        return [] if abs(d) > tollerance else float('inf')

def xRaySplineIntersectionTest(spline, origin):
    spline_points = spline.bezier_points if spline.type == 'BEZIER' else spline.points
    cyclic_parallel_fix_flag = False
    intersections = []

    def areIntersectionsAdjacent(index):
        if len(intersections) < 2:
            return
        prev = intersections[index-1]
        current = intersections[index]
        if prev[1] == current[0] and \
           prev[2] > 1.0-param_tollerance and current[2] < param_tollerance and \
           ((prev[3] < 0 and current[3] < 0) or (prev[3] > 0 and current[3] > 0)):
            intersections.pop(index)

    def appendIntersection(index, root, tangentY, intersectionX):
        beginPoint = spline_points[index-1]
        endPoint = spline_points[index]
        if root == float('inf'): # Segment is parallel to ray
            if index == 0 and spline.use_cyclic_u:
                cyclic_parallel_fix_flag = True
            if len(intersections) > 0 and intersections[-1][1] == beginPoint:
                intersections[-1][1] = endPoint # Skip in adjacency test
        elif intersectionX >= origin[0]:
            intersections.append([beginPoint, endPoint, root, tangentY, intersectionX])
            areIntersectionsAdjacent(len(intersections)-1)

    if spline.type == 'BEZIER':
        for index, endPoint in enumerate(spline.bezier_points):
            if index == 0 and not spline.use_cyclic_u:
                continue
            beginPoint = spline_points[index-1]
            points = (beginPoint.co, beginPoint.handle_right, endPoint.handle_left, endPoint.co)
            roots = bezierRoots((points[0][1]-origin[1], points[1][1]-origin[1], points[2][1]-origin[1], points[3][1]-origin[1]))
            if roots == float('inf'): # Intersection
                appendIntersection(index, float('inf'), None, None)
            else:
                for root in roots:
                    appendIntersection(index, root, bezierTangentAt(points, root)[1], bezierPointAt(points, root)[0])
    elif spline.type == 'POLY':
        for index, endPoint in enumerate(spline.points):
            if index == 0 and not spline.use_cyclic_u:
                continue
            beginPoint = spline_points[index-1]
            points = (beginPoint.co, endPoint.co)
            if (points[0][0] < origin[0] and points[1][0] < origin[0]) or \
               (points[0][1] < origin[1] and points[1][1] < origin[1]) or \
               (points[0][1] > origin[1] and points[1][1] > origin[1]):
                continue
            diff = points[1]-points[0]
            height = origin[1]-points[0][1]
            if diff[1] == 0: # Parallel
                if height == 0: # Intersection
                    appendIntersection(index, float('inf'), None, None)
            else: # Not parallel
                root = height/diff[1]
                appendIntersection(index, root, diff[1], points[0][0]+diff[0]*root)

    if cyclic_parallel_fix_flag:
        appendIntersection(0, float('inf'), None, None)
    areIntersectionsAdjacent(0)
    return intersections

def isPointInSpline(point, spline):
    return spline.use_cyclic_u and len(xRaySplineIntersectionTest(spline, point))%2 == 1

def isSegmentLinear(points, tollerance=0.0001):
    return 1.0-(points[1]-points[0]).normalized()@(points[3]-points[2]).normalized() < tollerance

def bezierSegmentPoints(begin, end):
    return [begin.co, begin.handle_right, end.handle_left, end.co]

def deleteFromArray(item, array):
    for index, current in enumerate(array):
        if current is item:
            array.pop(index)
            break

def bezierSliceFromTo(points, minParam, maxParam):
    fromP = bezierPointAt(points, minParam)
    fromT = bezierTangentAt(points, minParam)
    toP = bezierPointAt(points, maxParam)
    toT = bezierTangentAt(points, maxParam)
    paramDiff = maxParam-minParam
    return [fromP, fromP+fromT*paramDiff, toP-toT*paramDiff, toP]

def bezierIntersectionBroadPhase(solutions, pointsA, pointsB, aMin=0.0, aMax=1.0, bMin=0.0, bMax=1.0, depth=8, tollerance=0.001):
    if aabbIntersectionTest(aabbOfPoints(bezierSliceFromTo(pointsA, aMin, aMax)), aabbOfPoints(bezierSliceFromTo(pointsB, bMin, bMax)), tollerance) == False:
        return
    if depth == 0:
        solutions.append([aMin, aMax, bMin, bMax])
        return
    depth -= 1
    aMid = (aMin+aMax)*0.5
    bMid = (bMin+bMax)*0.5
    bezierIntersectionBroadPhase(solutions, pointsA, pointsB, aMin, aMid, bMin, bMid, depth, tollerance)
    bezierIntersectionBroadPhase(solutions, pointsA, pointsB, aMin, aMid, bMid, bMax, depth, tollerance)
    bezierIntersectionBroadPhase(solutions, pointsA, pointsB, aMid, aMax, bMin, bMid, depth, tollerance)
    bezierIntersectionBroadPhase(solutions, pointsA, pointsB, aMid, aMax, bMid, bMax, depth, tollerance)

def bezierIntersectionNarrowPhase(broadPhase, pointsA, pointsB, tollerance=0.000001):
    aMin = broadPhase[0]
    aMax = broadPhase[1]
    bMin = broadPhase[2]
    bMax = broadPhase[3]
    while (aMax-aMin > tollerance) or (bMax-bMin > tollerance):
        aMid = (aMin+aMax)*0.5
        bMid = (bMin+bMax)*0.5
        a1 = bezierPointAt(pointsA, (aMin+aMid)*0.5)
        a2 = bezierPointAt(pointsA, (aMid+aMax)*0.5)
        b1 = bezierPointAt(pointsB, (bMin+bMid)*0.5)
        b2 = bezierPointAt(pointsB, (bMid+bMax)*0.5)
        a1b1Dist = (a1-b1).length
        a2b1Dist = (a2-b1).length
        a1b2Dist = (a1-b2).length
        a2b2Dist = (a2-b2).length
        minDist = min(a1b1Dist, a2b1Dist, a1b2Dist, a2b2Dist)
        if a1b1Dist == minDist:
            aMax = aMid
            bMax = bMid
        elif a2b1Dist == minDist:
            aMin = aMid
            bMax = bMid
        elif a1b2Dist == minDist:
            aMax = aMid
            bMin = bMid
        else:
            aMin = aMid
            bMin = bMid
    return [aMin, bMin, minDist]

def bezierIntersection(segmentA, segmentB, tollerance=0.001):
    solutions = []
    pointsA = bezierSegmentPoints(segmentA['beginPoint'], segmentA['endPoint'])
    pointsB = bezierSegmentPoints(segmentB['beginPoint'], segmentB['endPoint'])
    bezierIntersectionBroadPhase(solutions, pointsA, pointsB)
    if len(solutions) == 0:
        return []
    for index in range(0, len(solutions)):
        solutions[index] = bezierIntersectionNarrowPhase(solutions[index], pointsA, pointsB)
    for index in range(0, len(solutions)):
        for otherIndex in range(0, len(solutions)):
            if solutions[index][2] == float('inf'):
                break
            if index == otherIndex or solutions[otherIndex][2] == float('inf'):
                continue
            diffA = solutions[index][0]-solutions[otherIndex][0]
            diffB = solutions[index][1]-solutions[otherIndex][1]
            if diffA*diffA+diffB*diffB < 0.01:
                if solutions[index][2] < solutions[otherIndex][2]:
                    solutions[otherIndex][2] = float('inf')
                else:
                    solutions[index][2] = float('inf')
    def areIntersectionsAdjacent(segmentA, segmentB, paramA, paramB):
        return segmentA['endIndex'] == segmentB['beginIndex'] and paramA > 1-param_tollerance and paramB < param_tollerance
    result = []
    for solution in solutions:
        if (solution[2] > tollerance) or \
          (segmentA['spline'] == segmentB['spline'] and \
          (areIntersectionsAdjacent(segmentA, segmentB, solution[0], solution[1]) or \
           areIntersectionsAdjacent(segmentB, segmentA, solution[1], solution[0]))):
            continue
        cutA = {'param': solution[0], 'segment': segmentA}
        cutB = {'param': solution[1], 'segment': segmentB}
        cutA['otherCut'] = cutB
        cutB['otherCut'] = cutA
        segmentA['cuts'].append(cutA)
        segmentB['cuts'].append(cutB)
        result.append([cutA, cutB])
    return result

def bezierMultiIntersection(segments):
    for index in range(0, len(segments)):
        for otherIndex in range(index+1, len(segments)):
            bezierIntersection(segments[index], segments[otherIndex])
    cleanupBezierIntersections(segments)
    subdivideBezierSegments(segments)

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

def subdivideBezierSegment(segment):
    # Blender only allows uniform subdivision. Use this method to subdivide at arbitrary params.
    # NOTE: segment['cuts'] must be sorted by param
    if len(segment['cuts']) == 0:
        return

    segment['beginPoint'] = segment['spline'].bezier_points[segment['beginIndex']]
    segment['endPoint'] = segment['spline'].bezier_points[segment['endIndex']]
    params = [cut['param'] for cut in segment['cuts']]
    newPoints = bezierSubivideAt(bezierSegmentPoints(segment['beginPoint'], segment['endPoint']), params)
    bpy.ops.curve.select_all(action='DESELECT')
    segment['beginPoint'] = segment['spline'].bezier_points[segment['beginIndex']]
    segment['beginPoint'].select_right_handle = True
    segment['beginPoint'].handle_left_type = 'FREE'
    segment['beginPoint'].handle_right_type = 'FREE'
    segment['endPoint'] = segment['spline'].bezier_points[segment['endIndex']]
    segment['endPoint'].select_left_handle = True
    segment['endPoint'].handle_left_type = 'FREE'
    segment['endPoint'].handle_right_type = 'FREE'

    bpy.ops.curve.subdivide(number_cuts=len(params))
    if segment['endIndex'] > 0:
        segment['endIndex'] += len(params)
    segment['beginPoint'] = segment['spline'].bezier_points[segment['beginIndex']]
    segment['endPoint'] = segment['spline'].bezier_points[segment['endIndex']]
    segment['beginPoint'].select_right_handle = False
    segment['beginPoint'].handle_right = newPoints[0]
    segment['endPoint'].select_left_handle = False
    segment['endPoint'].handle_left = newPoints[-1]

    for index, cut in enumerate(segment['cuts']):
        cut['index'] = segment['beginIndex']+1+index
        newPoint = segment['spline'].bezier_points[cut['index']]
        newPoint.handle_left_type = 'FREE'
        newPoint.handle_right_type = 'FREE'
        newPoint.select_left_handle = False
        newPoint.select_control_point = False
        newPoint.select_right_handle = False
        newPoint.handle_left = newPoints[index*3+1]
        newPoint.co = newPoints[index*3+2]
        newPoint.handle_right = newPoints[index*3+3]

def cleanupBezierIntersections(segments):
    def areCutsAdjacent(cutA, cutB):
        return cutA['segment']['beginIndex'] == cutB['segment']['endIndex'] and \
               cutA['param'] < param_tollerance and cutB['param'] > 1.0-param_tollerance
    for segment in segments:
        segment['cuts'].sort(key=(lambda cut: cut['param']))
        for index in range(len(segment['cuts'])-1, 0, -1):
            prev = segment['cuts'][index-1]
            current = segment['cuts'][index]
            if abs(prev['param']-current['param']) < param_tollerance and \
               prev['otherCut']['segment']['spline'] == current['otherCut']['segment']['spline'] and \
               (areCutsAdjacent(prev['otherCut'], current['otherCut']) or \
                areCutsAdjacent(current['otherCut'], prev['otherCut'])):
                deleteFromArray(prev['otherCut'], prev['otherCut']['segment']['cuts'])
                deleteFromArray(current['otherCut'], current['otherCut']['segment']['cuts'])
                segment['cuts'].pop(index-1 if current['otherCut']['param'] < param_tollerance else index)
                current = segment['cuts'][index-1]['otherCut']
                current['segment']['extraCut'] = current

def subdivideBezierSegmentsOfSameSpline(segments):
    # NOTE: segment['cuts'] must be sorted by param
    indexOffset = 0
    for segment in segments:
        segment['beginIndex'] += indexOffset
        if segment['endIndex'] > 0:
            segment['endIndex'] += indexOffset
        subdivideBezierSegment(segment)
        indexOffset += len(segment['cuts'])
    for segment in segments:
        segment['beginPoint'] = segment['spline'].bezier_points[segment['beginIndex']]
        segment['endPoint'] = segment['spline'].bezier_points[segment['endIndex']]

def subdivideBezierSegments(segments):
    # NOTE: segment['cuts'] must be sorted by param
    groups = {}
    for segment in segments:
        spline = segment['spline']
        if (spline in groups) == False:
            groups[spline] = []
        group = groups[spline]
        group.append(segment)
    for spline in groups:
        subdivideBezierSegmentsOfSameSpline(groups[spline])

def curveObject():
    obj = bpy.context.object
    return obj if obj != None and obj.type == 'CURVE' and obj.mode == 'EDIT' else None

def bezierSegments(splines, selection_only):
    segments = []
    for spline in splines:
        if spline.type != 'BEZIER':
            continue
        for index, current in enumerate(spline.bezier_points):
            next = spline.bezier_points[(index+1) % len(spline.bezier_points)]
            if next == spline.bezier_points[0] and not spline.use_cyclic_u:
                continue
            if not selection_only or (current.select_right_handle and next.select_left_handle):
                segments.append({
                    'spline': spline,
                    'beginIndex': index,
                    'endIndex': index+1 if index < len(spline.bezier_points)-1 else 0,
                    'beginPoint': current,
                    'endPoint': next,
                    'cuts': []
                })
    return segments

def bezierSelectedSplines(includeBezier, includePolygon):
    result = []
    for spline in bpy.context.object.data.splines:
        selected = False
        if spline.type == 'BEZIER' and includeBezier:
            selected = True
            for index, point in enumerate(spline.bezier_points):
                if not point.select_left_handle or not point.select_control_point or not point.select_right_handle:
                    selected = False
                    break
        if spline.type == 'POLY' and includePolygon:
            selected = True
            for index, point in enumerate(spline.points):
                if not point.select:
                    selected = False
                    break
        if selected:
            result.append(spline)
    return result

def addCurveObject(name):
    bpy.ops.object.select_all(action='DESELECT')
    curve = bpy.data.curves.new(name=name, type='CURVE')
    curve.dimensions = '3D'
    obj = bpy.data.objects.new(name, curve)
    bpy.context.scene.collection.objects.link(obj)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    return obj

def addPolygonSpline(obj, cyclic, vertices, weights=None, select=False):
    spline = obj.data.splines.new(type='POLY')
    spline.use_cyclic_u = cyclic
    spline.points.add(len(vertices)-1)
    for index, point in enumerate(spline.points):
        point.co.xyz = vertices[index]
        point.select = select
        if weights:
            point.weight_softbody = weights[index]
    return spline

def addBezierSpline(obj, cyclic, vertices, weights=None, select=False):
    spline = obj.data.splines.new(type='BEZIER')
    spline.use_cyclic_u = cyclic
    spline.bezier_points.add(len(vertices)-1)
    for index, point in enumerate(spline.bezier_points):
        point.handle_left = vertices[index][0]
        point.co = vertices[index][1]
        point.handle_right = vertices[index][2]
        if weights:
            point.weight_softbody = weights[index]
        point.select_left_handle = select
        point.select_control_point = select
        point.select_right_handle = select
        if isSegmentLinear([vertices[index-1][1], vertices[index-1][2], vertices[index][0], vertices[index][1]]):
            spline.bezier_points[index-1].handle_right_type = 'VECTOR'
            point.handle_left_type = 'VECTOR'
    return spline

def polygonArcAt(center, radius, begin_angle, angle, step_angle, include_ends):
    vertices = []
    circle_samples = math.ceil(abs(angle)/step_angle)
    for t in (range(0, circle_samples+1) if include_ends else range(1, circle_samples)):
        t = begin_angle+angle*t/circle_samples
        normal = Vector((math.cos(t), math.sin(t), 0))
        vertices.append(center+normal*radius)
    return vertices

def offsetVertex(position, tangent, distance):
    normal = Vector((-tangent[1], tangent[0], 0))
    return (normal, position+normal*distance)

def offsetPolygonOfSpline(spline, offset, step_angle, bezier_samples=128):
    vertices = []
    spline_points = spline.bezier_points if spline.type == 'BEZIER' else spline.points
    for index, spline_point in enumerate(spline_points):
        if index == 0 and not spline.use_cyclic_u:
            continue

        prev = spline_points[index-2]
        current = spline_points[index-1]
        next = spline_points[index]
        if spline.type == 'BEZIER':
            segment_points = bezierSegmentPoints(current, next)
            prev_tangent = (current.co-current.handle_left).normalized()
            current_tangent = (current.handle_right-current.co).normalized()
            next_tangent = (next.co-next.handle_left).normalized()
        else:
            segment_points = [current.co.xyz, None, None, next.co.xyz]
            prev_tangent = (segment_points[0]-prev.co.xyz).normalized()
            current_tangent = next_tangent = (segment_points[3]-segment_points[0]).normalized()
        sign = math.copysign(1, prev_tangent.cross(current_tangent)[2])
        angle = prev_tangent@current_tangent
        angle = 0 if abs(angle-1.0) < 0.0001 else math.acos(angle)

        if angle != 0 and (spline.use_cyclic_u or index != 1) and sign != math.copysign(1, offset): # Convex Round Cap
            begin_angle = math.atan2(prev_tangent[1], prev_tangent[0])+math.pi*0.5
            vertices += polygonArcAt(segment_points[0], offset, begin_angle, math.copysign(angle, sign), step_angle, False)
        if angle != 0 or (not spline.use_cyclic_u and index == 1):
            vertices.append(offsetVertex(segment_points[0], current_tangent, offset)[1])
        if spline.type != 'BEZIER' or (current.handle_right_type == 'VECTOR' and next.handle_left_type == 'VECTOR'):
            vertices.append(offsetVertex(segment_points[3], next_tangent, offset)[1])
        else: # Trace Bezier Segment
            prev_tangent = bezierTangentAt(segment_points, 0).normalized()
            for t in range(1, bezier_samples+1):
                t /= bezier_samples
                tangent = bezierTangentAt(segment_points, t).normalized()
                if t == 1 or math.acos(min(max(-1, prev_tangent@tangent), 1)) >= step_angle:
                    vertices.append(offsetVertex(bezierPointAt(segment_points, t), tangent, offset)[1])
                    prev_tangent = tangent

    # Solve Self Intersections
    original_area = areaOfPolygon([point.co for point in spline_points])
    sign = -1 if offset < 0 else 1
    i = (0 if spline.use_cyclic_u else 1)
    while i < len(vertices):
        j = i+2
        while j < len(vertices) - (0 if i > 0 else 1):
            intersection = lineIntersection(vertices[i-1], vertices[i], vertices[j-1], vertices[j])
            if intersection == None:
                j += 1
                continue
            areaInner = sign*areaOfPolygon([intersection, vertices[i], vertices[j-1]])
            areaOuter = sign*areaOfPolygon([intersection, vertices[j], vertices[i-1]])
            if areaInner > areaOuter:
                vertices = vertices[i:j]+[intersection]
                i = (0 if spline.use_cyclic_u else 1)
            else:
                vertices = vertices[:i]+[intersection]+vertices[j:]
            j = i+2
        i += 1

    new_area = areaOfPolygon(vertices)
    return vertices if original_area*new_area >= 0 else []

def bezierBooleanGeometry(splineA, splineB, operation):
    if not splineA.use_cyclic_u or not splineB.use_cyclic_u:
        return False
    segmentsA = bezierSegments([splineA], False)
    segmentsB = bezierSegments([splineB], False)

    deletionFlagA = isPointInSpline(splineA.bezier_points[0].co, splineB)
    deletionFlagB = isPointInSpline(splineB.bezier_points[0].co, splineA)
    if operation == 'DIFFERENCE':
        deletionFlagB = not deletionFlagB
    elif operation == 'INTERSECTION':
        deletionFlagA = not deletionFlagA
        deletionFlagB = not deletionFlagB
    elif operation != 'UNION':
        return False

    intersections = []
    for segmentA in segmentsA:
        for segmentB in segmentsB:
            intersections.extend(bezierIntersection(segmentA, segmentB))
    if len(intersections) == 0:
        if deletionFlagA:
            bpy.context.object.data.splines.remove(splineA)
        if deletionFlagB:
            bpy.context.object.data.splines.remove(splineB)
        return True

    cleanupBezierIntersections(segmentsA)
    cleanupBezierIntersections(segmentsB)
    subdivideBezierSegmentsOfSameSpline(segmentsA)
    subdivideBezierSegmentsOfSameSpline(segmentsB)

    def collectCuts(cuts, segments, deletionFlag):
        for segmentIndex, segment in enumerate(segments):
            if 'extraCut' in segment:
                deletionFlag = not deletionFlag
                segment['extraCut']['index'] = segment['beginIndex']
                segment['extraCut']['deletionFlag'] = deletionFlag
                cuts.append(segment['extraCut'])
            else:
                cuts.append(None)
            cuts.extend(segments[segmentIndex]['cuts'])
            segment['deletionFlag'] = deletionFlag
            for cutIndex, cut in enumerate(segment['cuts']):
                deletionFlag = not deletionFlag
                cut['deletionFlag'] = deletionFlag
    cutsA = []
    cutsB = []
    collectCuts(cutsA, segmentsA, deletionFlagA)
    collectCuts(cutsB, segmentsB, deletionFlagB)

    beginIndex = 0
    for segment in segmentsA:
        if segment['deletionFlag'] == False:
            beginIndex = segment['beginIndex']
            break
        for cut in segment['cuts']:
            if cut['deletionFlag'] == False:
                beginIndex = cut['index']
                break

    cuts = cutsA
    spline = splineA
    index = beginIndex
    backward = False
    vertices = []
    while True:
        current = spline.bezier_points[index]
        vertices.append([current.handle_left, current.co, current.handle_right])
        if backward:
            current.handle_left, current.handle_right = current.handle_right.copy(), current.handle_left.copy()
        index += len(spline.bezier_points)-1 if backward else 1
        index %= len(spline.bezier_points)
        if spline == splineA and index == beginIndex:
            break

        cut = cuts[index]
        if cut != None:
            current = spline.bezier_points[index]
            current_handle = current.handle_right if backward else current.handle_left
            spline = splineA if spline == splineB else splineB
            cuts = cutsA if spline == splineA else cutsB
            index = cut['otherCut']['index']
            backward = cut['otherCut']['deletionFlag']
            next = spline.bezier_points[index]
            if backward:
                next.handle_right = current_handle
            else:
                next.handle_left = current_handle
            if spline == splineA and index == beginIndex:
                break

    spline = addBezierSpline(bpy.context.object, True, vertices)
    bpy.context.object.data.splines.remove(splineA)
    bpy.context.object.data.splines.remove(splineB)
    bpy.context.object.data.splines.active = spline
    return True
