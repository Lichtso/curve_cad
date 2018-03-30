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

import bpy, math
from mathutils import Vector, Matrix
from . import internal

class OffsetCurve(bpy.types.Operator):
    bl_idname = 'curve.add_toolpath_offset_curve'
    bl_description = bl_label = 'Offset Curve'
    bl_options = {'REGISTER', 'UNDO'}

    offset = bpy.props.FloatProperty(name='Offset', unit='LENGTH', default=0.1)
    pitch = bpy.props.FloatProperty(name='Pitch', unit='LENGTH', default=0.1)
    max_angle = bpy.props.FloatProperty(name='Resolution', unit='ROTATION', min=math.pi/128, default=math.pi/16)
    count = bpy.props.IntProperty(name='Count', min=1, default=1)

    @classmethod
    def poll(cls, context):
        return bpy.context.object != None and bpy.context.object.type == 'CURVE'

    def execute(self, context):
        if bpy.context.object.mode == 'EDIT':
            splines = internal.bezierSelectedSplines(True, True)
        else:
            splines = bpy.context.object.data.splines

        if len(splines) == 0:
            self.report({'WARNING'}, 'Nothing selected')
            return {'CANCELLED'}

        if bpy.context.object.mode != 'EDIT':
            internal.addCurveObject('Toolpath')

        for spline in splines:
            for index in range(0, self.count):
                internal.offsetPolygonOfSpline(spline, self.offset+self.pitch*index, self.max_angle)
        return {'FINISHED'}

operators = [OffsetCurve]
