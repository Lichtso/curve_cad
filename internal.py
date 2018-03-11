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
    # https://en.wikipedia.org/wiki/Cubic_function#Lagrange.27s_method
    a  = 3*(dists[1]-dists[2])+dists[3]-dists[0]
    E1 = 3*(dists[2]-2*dists[1]+dists[0])
    E2 = a*3*(dists[1]-dists[0])
    E3 = a*a*dists[0]
    A  = (2*E1*E1-9*E2)*E1+27*E3
    B  = E1*E1-3*E2
    b  = ((A+cmath.sqrt(A*A-4*B*B*B))*0.5) ** (1/3)
    if abs(a) > tollerance and abs(b) > tollerance:
        roots = []
        for root in cubic_roots_of_unity:
            root *= b
            root = -1/(3*a)*(E1+root+B/root)
            if abs(root.imag) < tollerance and root.real >= 0.0 and root.real <= 1.0:
                roots.append(root.real)
        return roots
    elif abs(dists[3]-dists[0]) > tollerance: # Not parallel
        root = -dists[0]/(dists[3]-dists[0])
        return [root] if root >= 0.0 and root <= 1.0 else []
    else: # Parallel
        return [0.5] if abs(dists[0]) < tollerance else []

def isPointInSpline(spline, point):
    if not spline.use_cyclic_u:
        return False
    count = 0

    if spline.type == 'BEZIER':
        for index, next in enumerate(spline.bezier_points):
            prev = spline.bezier_points[index-1]
            points = (prev.co, prev.handle_right, next.handle_left, next.co)
            roots = bezierRoots((points[0][1]-point[1], points[1][1]-point[1], points[2][1]-point[1], points[3][1]-point[1]))
            for root in roots:
                if point[0] <= bezierPointAt(points, root)[0]:
                    count += 1

    elif spline.type == 'POLY':
        for index, next in enumerate(spline.points):
            points = (spline.points[index-1].co, next.co)
            if (points[0][0] < point[0] and points[1][0] < point[0]) or \
               (points[0][1] < point[1] and points[1][1] < point[1]) or \
               (points[0][1] > point[1] and points[1][1] > point[1]):
                continue
            diff = points[1]-points[0]
            if diff[1] == 0:
                count += 1 if points[0][0] == point[0] else 0
            else:
                count += 1 if (point[0] <= points[0][0]+diff[0]/diff[1]*(point[1]-points[0][1])) else 0

    return count%2 == 1

def bezierSliceFromTo(points, minParam, maxParam):
    fromP = bezierPointAt(points, minParam)
    fromT = bezierTangentAt(points, minParam)
    toP = bezierPointAt(points, maxParam)
    toT = bezierTangentAt(points, maxParam)
    paramDiff = maxParam-minParam
    return [fromP, fromP+fromT*paramDiff, toP-toT*paramDiff, toP]

def bezierIntersectionBroadPhase(solutions, depth, pointsA, pointsB, aMin, aMax, bMin, bMax, tollerance=0.001):
    if aabbIntersectionTest(aabbOfPoints(bezierSliceFromTo(pointsA, aMin, aMax)), aabbOfPoints(bezierSliceFromTo(pointsB, bMin, bMax)), tollerance) == False:
        return
    if depth == 0:
        solutions.append([aMin, aMax, bMin, bMax])
        return
    depth -= 1
    aMid = (aMin+aMax)*0.5
    bMid = (bMin+bMax)*0.5
    bezierIntersectionBroadPhase(solutions, depth, pointsA, pointsB, aMin, aMid, bMin, bMid)
    bezierIntersectionBroadPhase(solutions, depth, pointsA, pointsB, aMin, aMid, bMid, bMax)
    bezierIntersectionBroadPhase(solutions, depth, pointsA, pointsB, aMid, aMax, bMin, bMid)
    bezierIntersectionBroadPhase(solutions, depth, pointsA, pointsB, aMid, aMax, bMid, bMax)

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

def bezierIntersection(pointsA, pointsB, paramsA, paramsB, tollerance=0.001):
    solutions = []
    bezierIntersectionBroadPhase(solutions, 8, pointsA, pointsB, 0.0, 1.0, 0.0, 1.0)
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
    for solution in solutions:
        if solution[2] < tollerance:
            paramsA.append(solution[0])
            paramsB.append(solution[1])
    paramsA.sort()
    paramsB.sort()

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
            if not selection_only or (prev.select_right_handle and next.select_left_handle):
                segments.append(SplineBezierSegement(
                                spline=spline,
                                beginIndex=index-1 if index > 0 else len(spline.bezier_points)-1,
                                endIndex=index,
                                beginPoint=prev,
                                endPoint=next,
                                points=[Vector(prev.co), Vector(prev.handle_right), Vector(next.handle_left), Vector(next.co)],
                                params=[]))
    return segments

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
        corner = segment.points[0]
        begin_tangent = bezierTangentAt(segments[index-1].points, 1).normalized()
        end_tangent = bezierTangentAt(segment.points, 0).normalized()
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
            add_vertex_for(segment.points, 0)
        if segment.beginPoint.handle_right_type == 'VECTOR' and segment.endPoint.handle_left_type == 'VECTOR':
            add_vertex_for(segment.points, 1)
        else:
            prev_tangent = bezierTangentAt(segment.points, 0).normalized()
            for t in range(1, bezier_samples+1):
                t /= bezier_samples
                tangent = bezierTangentAt(segment.points, t).normalized()
                if t == 1 or math.acos(min(max(-1, prev_tangent*tangent), 1)) >= max_angle:
                    add_vertex_for(segment.points, t)
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
    points[0].select_control_point = True
    points[1] = selected_splines[1].bezier_points[-1 if is_last_point[1] else 0]
    points[1].select_control_point = True
    bpy.ops.curve.make_segment()
    bpy.ops.curve.select_all(action='DESELECT')

    return True
