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

import os, bpy, importlib, math
if 'internal' in locals():
    importlib.reload(internal)
    importlib.reload(svg_export)
from . import internal, svg_export

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

        cuts = []
        for param in self.params.split(' '):
            cuts.append({'param': max(0.0, min(float(param), 1.0))})
        for segment in segments:
            segment['cuts'].extend(cuts)
        internal.subdivideBezierSegmentsAtParams(segments)
        return {'FINISHED'}

class BezierIntersection(bpy.types.Operator):
    bl_idname = 'curve.bezier_intersection'
    bl_description = bl_label = 'Bezier Intersection'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        segments = internal.bezierSegments(bpy.context.object.data.splines, True)
        if len(segments) < 2:
            self.report({'WARNING'}, 'Invalid selection')
            return {'CANCELLED'}

        internal.bezierMultiIntersection(segments)
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

        circle = internal.circleOfBezier(segments[0]['points'])
        if circle == None:
            self.report({'WARNING'}, 'Not a circle')
            return {'CANCELLED'}

        bpy.ops.curve.primitive_bezier_circle_add(radius=circle.radius, location=circle.center, rotation=circle.plane.normal.to_track_quat('Z', 'X').to_euler())
        return {'FINISHED'}

class BezierLength(bpy.types.Operator):
    bl_idname = 'curve.bezier_length'
    bl_description = bl_label = 'Bezier Length'

    def execute(self, context):
        segments = internal.bezierSegments(bpy.context.object.data.splines, True)
        if len(segments) == 0:
            self.report({'WARNING'}, 'Nothing selected')
            return {'CANCELLED'}

        length = 0
        for segment in segments:
            length += internal.bezierLength(segment['points'])
        self.report({'INFO'}, bpy.utils.units.to_string(bpy.context.scene.unit_settings.system, 'LENGTH', length))
        return {'FINISHED'}

class BezierOffset(bpy.types.Operator):
    bl_idname = 'curve.bezier_offset'
    bl_description = bl_label = 'Bezier Offset'
    bl_options = {'REGISTER', 'UNDO'}

    offset = bpy.props.FloatProperty(name='Offset', unit='LENGTH', default=0.1)
    pitch = bpy.props.FloatProperty(name='Pitch', unit='LENGTH', default=0.1)
    max_angle = bpy.props.FloatProperty(name='Resolution', unit='ROTATION', min=math.pi/128, default=math.pi/16)
    count = bpy.props.IntProperty(name='Count', min=1, default=1)

    def execute(self, context):
        splines = internal.bezierSelectedSplines(bpy.context.object.data.splines)
        if len(splines) == 0:
            self.report({'WARNING'}, 'Nothing selected')
            return {'CANCELLED'}
        for spline in splines:
            for index in range(0, self.count):
                internal.offsetPolygonOfSpline(spline, self.offset+self.pitch*index, self.max_angle)
        return {'FINISHED'}

class BezierMergeEnds(bpy.types.Operator):
    bl_idname = 'curve.bezier_merge_ends'
    bl_description = bl_label = 'Bezier Merge Ends'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if not internal.mergeBezierEndPoints(bpy.context.object.data.splines):
            self.report({'WARNING'}, 'Invalid selection')
            return {'CANCELLED'}
        return {'FINISHED'}

operators = [BezierIntersection, BezierMergeEnds, BezierSubdivide, BezierOffset, BezierCircle, BezierLength]

class VIEW3D_MT_edit_curve_cad(bpy.types.Menu):
    bl_label = bl_info['name']

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

def menu_func_export(self, context):
    self.layout.operator(svg_export.SVG_Export.bl_idname, text='Curves (.svg)')

def register():
    bpy.utils.register_module(__name__)
    bpy.types.VIEW3D_MT_edit_curve_specials.prepend(menu_func)
    bpy.types.INFO_MT_file_export.append(menu_func_export)

def unregister():
    bpy.utils.unregister_module(__name__)
    bpy.types.VIEW3D_MT_edit_curve_specials.remove(menu_func)
    bpy.types.INFO_MT_file_export.remove(menu_func_export)
