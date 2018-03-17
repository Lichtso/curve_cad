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

AABB = namedtuple('AxisAlignedBoundingBox', 'center dimensions')
Plane = namedtuple('Plane', 'normal distance')
Circle = namedtuple('Circle', 'plane center radius')

def nearestPointOfLines(originA, dirA, originB, dirB, tollerance=0.0):
    # https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points
    normal = dirA.cross(dirB)
    normalA = dirA.cross(normal)
    normalB = dirB.cross(normal)
    divisorA = dirA*normalB
    divisorB = dirB*normalA
    if abs(divisorA) <= tollerance or abs(divisorB) <= tollerance:
        return (float('nan'), float('nan'), originA, originB)
    else:
        paramA = (originB-originA)*normalB/divisorA
        paramB = (originA-originB)*normalA/divisorB
        return (paramA, paramB, originA+dirA*paramA, originB+dirB*paramB)

def lineIntersection(beginA, endA, beginB, endB):
    dirA = endA-beginA
    dirB = endB-beginB
    intersection = nearestPointOfLines(beginA, dirA, beginB, dirB)
    if intersection[0] != intersection[0] or intersection[0] < 0 or intersection[0] > 1 or intersection[1] < 0 or intersection[1] > 1:
        return None
    return (intersection[2]+intersection[3])*0.5

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
    alpha = (dirBA*dirAC)*(lengthCB*lengthCB*factor)
    beta = (dirBA*dirCB)*(lengthAC*lengthAC*factor)
    gamma = (dirAC*dirCB)*(lengthBA*lengthBA*factor)
    center = a*alpha+b*beta+c*gamma
    radius = (lengthBA*lengthCB*lengthAC)/(2*lengthN)
    plane = Plane(normal=normal/lengthN, distance=center*normal)
    return Circle(plane=plane, center=center, radius=radius)

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

def bezierLength(points, beginT=0, endT=1, samples=1024):
    # https://en.wikipedia.org/wiki/Arc_length#Finding_arc_lengths_by_integrating
    vec = [points[1]-points[0], points[2]-points[1], points[3]-points[2]]
    dot = [vec[0]*vec[0], vec[0]*vec[1], vec[0]*vec[2], vec[1]*vec[1], vec[1]*vec[2], vec[2]*vec[2]]
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
    if abs(a) != 0: # Cubic
        E2 = a*c
        E3 = a*a*d
        A = (2*b*b-9*E2)*b+27*E3
        B = b*b-3*E2
        C = ((A+cmath.sqrt(A*A-4*B*B*B))*0.5) ** (1/3)
        roots = []
        for root in cubic_roots_of_unity:
            root *= C
            root = -1/(3*a)*(b+root+B/root)
            if abs(root.imag) < tollerance and root.real > -tollerance and root.real < 1.0+tollerance:
                roots.append(max(0.0, min(root.real, 1.0)))
        # Remove doubles
        roots.sort()
        for index in range(len(roots)-1, 0, -1):
            if abs(roots[index-1]-roots[index]) < tollerance:
                roots.pop(index)
        return roots
    elif abs(b) != 0: # Quadratic
        disc = c*c -4*b*d
        if disc < 0:
            return []
        disc = sqrt(disc)
        return [(-c-disc)/(2*b), (-c+disc)/(2*b)]
    elif abs(c) != 0: # Linear
        root = -d/c
        return [root] if root >= 0.0 and root <= 1.0 else []
    else: # Constant / Parallel
        return []

def xRaySplineIntersectionTest(spline, origin):
    intersections = []
    if not spline.use_cyclic_u:
        return intersections

    def areIntersectionsAdjacent(index, tollerance=0.0001):
        if len(intersections) < 2:
            return
        prev = intersections[index-1]
        current = intersections[index]
        if prev[1] == current[0] and \
           prev[2] > 1.0-tollerance and current[2] < tollerance and \
           ((prev[3] < 0 and current[3] < 0) or (prev[3] > 0 and current[3] > 0)):
            intersections.pop(index)

    def appendIntersection(prev, current, root, tangentY, intersectionX):
        if intersectionX >= origin[0]:
            intersections.append([prev, current, root, tangentY, intersectionX])
            areIntersectionsAdjacent(len(intersections)-1)

    if spline.type == 'BEZIER':
        for index, next in enumerate(spline.bezier_points):
            prev = spline.bezier_points[index-1]
            points = (prev.co, prev.handle_right, next.handle_left, next.co)
            roots = bezierRoots((points[0][1]-origin[1], points[1][1]-origin[1], points[2][1]-origin[1], points[3][1]-origin[1]))
            for root in roots:
                appendIntersection(spline.bezier_points[index-2], spline.bezier_points[index-1], root, bezierTangentAt(points, root)[1], bezierPointAt(points, root)[0])
    elif spline.type == 'POLY':
        for index, next in enumerate(spline.points):
            points = (spline.points[index-1].co, next.co)
            if (points[0][0] < origin[0] and points[1][0] < origin[0]) or \
               (points[0][1] < origin[1] and points[1][1] < origin[1]) or \
               (points[0][1] > origin[1] and points[1][1] > origin[1]):
                continue
            diff = points[1]-points[0]
            if diff[1] != 0: # Not parallel
                root = (origin[1]-points[0][1])/diff[1]
                appendIntersection(spline.points[index-2].co, spline.points[index-1], root, diff[1], points[0][0]+diff[0]*root)

    areIntersectionsAdjacent(0)
    return intersections

def isPointInSpline(spline, point):
    return len(xRaySplineIntersectionTest(spline, point))%2 == 1


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
    bezierIntersectionBroadPhase(solutions, segmentA['points'], segmentB['points'])
    if len(solutions) == 0:
        return []
    for index in range(0, len(solutions)):
        solutions[index] = bezierIntersectionNarrowPhase(solutions[index], segmentA['points'], segmentB['points'])
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
    def areIntersectionsAdjacent(segmentA, segmentB, paramA, paramB, tollerance=0.0001):
        return segmentA['endIndex'] == segmentB['beginIndex'] and paramA > 1-tollerance and paramB < tollerance
    result = []
    for solution in solutions:
        if (solution[2] > tollerance) or \
          (segmentA['spline'] == segmentB['spline'] and \
          (areIntersectionsAdjacent(segmentA, segmentB, solution[0], solution[1]) or \
           areIntersectionsAdjacent(segmentB, segmentA, solution[1], solution[0]))):
            continue
        cutA = {'param': solution[0], 'segment': segmentA, 'otherSegment': segmentB}
        cutB = {'param': solution[1], 'segment': segmentB, 'otherSegment': segmentA}
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
    subdivideBezierSegmentsAtParams(segments)

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
    if len(segment['cuts']) == 0:
        return
    segment['cuts'].sort(key=(lambda cut: cut['param']))

    newPoints = bezierSubivideAt(segment['points'], list(map(lambda cut: cut['param'], segment['cuts'])))
    bpy.ops.curve.select_all(action='DESELECT')
    segment['beginPoint'] = segment['spline'].bezier_points[segment['beginIndex']]
    segment['beginPoint'].select_right_handle = True
    segment['beginPoint'].handle_left_type = 'FREE'
    segment['beginPoint'].handle_right_type = 'FREE'
    segment['endPoint'] = segment['spline'].bezier_points[segment['endIndex']]
    segment['endPoint'].select_left_handle = True
    segment['endPoint'].handle_left_type = 'FREE'
    segment['endPoint'].handle_right_type = 'FREE'

    bpy.ops.curve.subdivide(number_cuts=len(segment['cuts']))
    if segment['endIndex'] > 0:
        segment['endIndex'] += len(segment['cuts'])
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

def subdivideBezierSegmentsAtParams(segments):
    groups = {}
    for segment in segments:
        spline = segment['spline']
        if (spline in groups) == False:
            groups[spline] = []
        group = groups[spline]
        group.append(segment)
    for spline in groups:
        group = groups[spline]
        group.sort(key=(lambda segment: segment['beginIndex']))
        indexOffset = 0
        for segment in group:
            segment['beginIndex'] += indexOffset
            if segment['endIndex'] > 0:
                segment['endIndex'] += indexOffset
            indexOffset += len(segment['cuts'])
            subdivideBezierSegmentAtParams(segment)
        for segment in group:
            segment['beginPoint'] = segment['spline'].bezier_points[segment['beginIndex']]
            segment['endPoint'] = segment['spline'].bezier_points[segment['endIndex']]

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
                    'points': [Vector(current.co), Vector(current.handle_right), Vector(next.handle_left), Vector(next.co)],
                    'cuts': []
                })
    return segments

def bezierSelectedSplines(splines):
    result = []
    for spline in splines:
        selected = (spline.type == 'BEZIER')
        for index, point in enumerate(spline.bezier_points):
            if not point.select_left_handle or not point.select_control_point or not point.select_right_handle:
                selected = False
                break
        if selected:
            result.append(spline)
    return result

def offsetPolygonOfSpline(spline, distance, max_angle, bezier_samples=128):
    corners = []
    vertices = []
    cyclic = spline.use_cyclic_u
    segments = bezierSegments([spline], False)
    if len(segments) == 0:
        return None

    def add_vertex_for(points, param):
        position = bezierPointAt(points, param)
        tangent = bezierTangentAt(points, param)
        tangent.normalize()
        normal = Vector((-tangent.y, tangent.x, 0))
        vertices.append(position+normal*distance)

    for index, segment in enumerate(segments):
        corner = segment['points'][0]
        begin_tangent = bezierTangentAt(segments[index-1]['points'], 1).normalized()
        end_tangent = bezierTangentAt(segment['points'], 0).normalized()
        angle = begin_tangent*end_tangent
        angle = 0 if abs(angle-1.0) < 0.0001 else math.acos(angle)
        place_vertex = angle != 0 or (not cyclic and index == 0)

        if angle != 0 and (cyclic or index > 0):
            begin_normal = Vector((-begin_tangent.y, begin_tangent.x, 0))
            end_normal = Vector((-end_tangent.y, end_tangent.x, 0))
            begin_point = corner+begin_normal*distance
            end_point = corner+end_normal*distance
            normal = begin_normal.cross(end_normal)
            circle_samples = math.ceil(angle/max_angle)
            angle = math.copysign(angle, normal[2])
            beginAngle = math.atan2(begin_normal[1], begin_normal[0])
            intersection = nearestPointOfLines(begin_point, begin_tangent, end_point, end_tangent)
            if intersection[0] > 0: # and intersection[1] < 0:
                for t in range(1, circle_samples):
                    t = beginAngle+angle*t/circle_samples
                    normal = Vector((math.cos(t), math.sin(t), 0))
                    vertices.append(corner+normal*distance)

        if place_vertex:
            add_vertex_for(segment['points'], 0)
        if segment['beginPoint'].handle_right_type == 'VECTOR' and segment['endPoint'].handle_left_type == 'VECTOR':
            add_vertex_for(segment['points'], 1)
        else:
            prev_tangent = bezierTangentAt(segment['points'], 0).normalized()
            for t in range(1, bezier_samples+1):
                t /= bezier_samples
                tangent = bezierTangentAt(segment['points'], t).normalized()
                if t == 1 or math.acos(min(max(-1, prev_tangent*tangent), 1)) >= max_angle:
                    add_vertex_for(segment['points'], t)
                    prev_tangent = tangent

    i = (0 if cyclic else 1)
    while i < len(vertices):
        j = i+2
        while j < (len(vertices) - (0 if i > 0 else 1)):
            intersection = lineIntersection(vertices[i-1], vertices[i], vertices[j-1], vertices[j])
            if intersection == None:
                j += 1
                continue
            if j-i < len(vertices)/2:
                vertices = vertices[:i] + [intersection] + vertices[j:]
                j = i+2
            else:
                vertices = vertices[i:j] + [intersection]
                j = (0 if cyclic else 1)
        i += 1

    spline = bpy.context.object.data.splines.new(type='POLY')
    spline.use_cyclic_u = cyclic
    spline.points.add(count=len(vertices)-1)
    for index, vertex in enumerate(vertices):
        point = spline.points[index]
        point.select = True
        point.co.xyz = vertex
    return spline

def mergeBezierEndPoints(splines):
    points = []
    selected_splines = []
    is_last_point = []
    for spline in splines:
        if spline.type != 'BEZIER' or spline.use_cyclic_u:
            continue
        if spline.bezier_points[0].select_control_point:
            points.append(spline.bezier_points[0])
            selected_splines.append(spline)
            is_last_point.append(False)
        if spline.bezier_points[-1].select_control_point:
            points.append(spline.bezier_points[-1])
            selected_splines.append(spline)
            is_last_point.append(True)

    if len(points) != 2:
        return False

    points[0].handle_left_type = 'FREE'
    points[0].handle_right_type = 'FREE'
    new_co = (points[0].co+points[1].co)*0.5

    handle = (points[1].handle_left if is_last_point[1] else points[1].handle_right)+new_co-points[1].co
    if is_last_point[0]:
        points[0].handle_left += new_co-points[0].co
        points[0].handle_right = handle
    else:
        points[0].handle_right += new_co-points[0].co
        points[0].handle_left = handle
    points[0].co = new_co

    bpy.ops.curve.select_all(action='DESELECT')
    points[1].select_control_point = True
    bpy.ops.curve.delete()
    selected_splines[0].bezier_points[-1 if is_last_point[0] else 0].select_control_point = True
    selected_splines[1].bezier_points[-1 if is_last_point[1] else 0].select_control_point = True
    bpy.ops.curve.make_segment()
    bpy.ops.curve.select_all(action='DESELECT')
    return True
