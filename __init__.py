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

import os, bpy, importlib
if 'internal' in locals():
    importlib.reload(internal)
from . import internal

bl_info = {
    'name': 'Curve CAD Tools',
    'author': 'Alexander Meißner',
    'version': (1, 0, 0),
    'blender': (2, 79),
    'category': 'Curve',
    'location': 'View3D > EditMode > (w) Specials',
    'wiki_url': 'http://lichtso.github.io/curve_cad/',
    'tracker_url': 'https://github.com/lichtso/curve_cad/issues'
}

class BezierSubdivide(bpy.types.Operator):
    bl_idname = 'curve.bezier_subdivide'
    bl_description = bl_label = 'Bezier Subdivide'
    bl_options = {'REGISTER', 'UNDO'}

    params = bpy.props.StringProperty(name='Params', default='0.25 0.5 0.75')

    def execute(self, context):
        segments = internal.bezierSegments(bpy.context.object.data.splines, True)
        if len(segments) == 0:
            self.report({'WARNING'}, 'Nothing selected')
            return {'CANCELLED'}

        params = []
        for param in self.params.split(' '):
            params.append(max(0.0, min(float(param), 1.0)))
        params.sort()
        for segment in segments:
            segment.params.extend(params)
        internal.subdivideBezierSegmentsAtParams(segments)
        return {'FINISHED'}

class BezierIntersection(bpy.types.Operator):
    bl_idname = 'curve.bezier_intersection'
    bl_description = bl_label = 'Bezier Intersection'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        segments = internal.bezierSegments(bpy.context.object.data.splines, True)
        if len(segments) != 2:
            self.report({'WARNING'}, 'Invalid selection')
            return {'CANCELLED'}

        def removeAdjacentIntersections(segmentA, segmentB, threshold=0.0001):
            if segmentA.endIndex == segmentB.beginIndex:
                segmentA.params[:] = [param for param in segmentA.params if param < 1-threshold]
                segmentB.params[:] = [param for param in segmentB.params if param > threshold]

        internal.bezierIntersection(segments[0].points, segments[1].points, segments[0].params, segments[1].params)
        removeAdjacentIntersections(segments[0], segments[1])
        removeAdjacentIntersections(segments[1], segments[0])
        internal.subdivideBezierSegmentsAtParams(segments)
        return {'FINISHED'}

class BezierCircle(bpy.types.Operator):
    bl_idname = 'curve.bezier_circle'
    bl_description = bl_label = 'Bezier Circle'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        segments = internal.bezierSegments(bpy.context.object.data.splines, True)
        if len(segments) != 1:
            self.report({'WARNING'}, 'Invalid selection')
            return {'CANCELLED'}

        points = segments[0].points
        circle = internal.circleOfTriangle(points[0], internal.bezierPointAt(points, 0.5), points[3])
        if circle == None:
            return {'CANCELLED'}

        bpy.ops.curve.primitive_bezier_circle_add(radius=circle.radius, location=circle.center, rotation=circle.plane.normal.to_track_quat('Z', 'X').to_euler())
        return {'FINISHED'}

operators = [BezierSubdivide, BezierIntersection, BezierCircle]

class VIEW3D_MT_edit_curve_cad(bpy.types.Menu):
    bl_label = 'CurveCAD'

    @classmethod
    def poll(cls, context):
        obj = bpy.context.object
        return obj != None and obj.type == 'CURVE' and obj.mode == 'EDIT'

    def draw(self, context):
        for operator in operators:
            self.layout.operator(operator.bl_idname, text=operator.bl_description)

def menu_func(self, context):
    self.layout.menu('VIEW3D_MT_edit_curve_cad')
    self.layout.separator()

def register():
    bpy.utils.register_module(__name__)
    bpy.types.VIEW3D_MT_edit_curve_specials.prepend(menu_func)

def unregister():
    bpy.utils.unregister_module(__name__)
    bpy.types.VIEW3D_MT_edit_curve_specials.remove(menu_func)
