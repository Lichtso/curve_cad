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

    offset: bpy.props.FloatProperty(name='Offset', description='Distace between the original and the first trace', unit='LENGTH', default=0.1)
    pitch: bpy.props.FloatProperty(name='Pitch', description='Distace between two paralell traces', unit='LENGTH', default=0.1)
    step_angle: bpy.props.FloatProperty(name='Resolution', description='Smaller values make curves smoother by adding more vertices', unit='ROTATION', min=math.pi/128, default=math.pi/16)
    count: bpy.props.IntProperty(name='Count', description='Number of paralell traces', min=1, default=1)
    connect: bpy.props.BoolProperty(name='Connect', description='Connects all traces if count > 1')

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
            vertices = []
            for index in range(0, self.count):
                trace = internal.offsetPolygonOfSpline(spline, self.offset+self.pitch*index, self.step_angle)
                if len(trace) == 0:
                    continue
                if self.connect and self.count > 1:
                    if spline.use_cyclic_u:
                        trace.append(trace[0])
                        vertices = trace+vertices
                    else:
                        if index%2 == 1:
                            trace = list(reversed(trace))
                        vertices = trace+vertices
                else:
                    internal.addPolygonSpline(bpy.context.object, spline.use_cyclic_u, trace)
            if self.connect and self.count > 1:
                internal.addPolygonSpline(bpy.context.object, False, vertices)
        return {'FINISHED'}

class RectMacro(bpy.types.Operator):
    bl_idname = 'curve.add_toolpath_rect_macro'
    bl_description = bl_label = 'Rect Macro'
    bl_options = {'REGISTER', 'UNDO'}

    track_count: bpy.props.IntProperty(name='Number Tracks', description='How many tracks', min=1, default=10)
    stride: bpy.props.FloatProperty(name='Stride', unit='LENGTH', description='Distance to previous track on the way back', min=0.0, default=0.5)
    pitch: bpy.props.FloatProperty(name='Pitch', unit='LENGTH', description='Distance between two tracks', default=-1.0)
    length: bpy.props.FloatProperty(name='Length', unit='LENGTH', description='Length of one track', default=10.0)
    speed: bpy.props.FloatProperty(name='Speed', description='Stored in softbody goal weight', min=0.0, max=1.0, default=0.1)

    def execute(self, context):
        if not internal.curveObject():
            internal.addCurveObject('Toolpath').location = bpy.context.scene.cursor_location
            origin = Vector((0.0, 0.0, 0.0))
        else:
            origin = bpy.context.scene.cursor_location
        stride = math.copysign(self.stride, self.pitch)
        length = self.length*0.5
        vertices = []
        weights = []
        for i in range(0, self.track_count):
            shift = i*self.pitch
            flipped = -1 if (stride == 0 and i%2 == 1) else 1
            vertices.append(origin+Vector((shift, -length*flipped, 0.0)))
            weights.append(self.speed)
            vertices.append(origin+Vector((shift, length*flipped, 0.0)))
            weights.append(self.speed)
            if stride != 0:
                vertices.append(origin+Vector((shift-stride, length, 0.0)))
                weights.append(self.speed)
                vertices.append(origin+Vector((shift-stride, -length, 0.0)))
                weights.append(1)
        internal.addPolygonSpline(bpy.context.object, False, vertices, weights)
        return {'FINISHED'}

class DrillMacro(bpy.types.Operator):
    bl_idname = 'curve.add_toolpath_drill_macro'
    bl_description = bl_label = 'Drill Macro'
    bl_options = {'REGISTER', 'UNDO'}

    screw_count: bpy.props.FloatProperty(name='Screw Turns', description='How many screw truns', min=1.0, default=10.0)
    spiral_count: bpy.props.FloatProperty(name='Spiral Turns', description='How many spiral turns', min=0.0, default=0.0)
    vertex_count: bpy.props.IntProperty(name='Number Vertices', description = 'How many vertices per screw turn', min=3, default=32)
    radius: bpy.props.FloatProperty(name='Radius', unit='LENGTH', description='Radius at tool center', min=0.0, default=5.0)
    pitch: bpy.props.FloatProperty(name='Pitch', unit='LENGTH', description='Distance between two screw turns', min=0.0, default=1.0)
    speed: bpy.props.FloatProperty(name='Speed', description='Stored in softbody goal weight', min=0.0, max=1.0, default=0.1)

    def execute(self, context):
        if not internal.curveObject():
            internal.addCurveObject('Toolpath').location = bpy.context.scene.cursor_location
            origin = Vector((0.0, 0.0, 0.0))
        else:
            origin = bpy.context.scene.cursor_location
        count = int(self.vertex_count*self.screw_count)
        height = -count/self.vertex_count*self.pitch
        vertices = []
        weights = []
        def addRadialVertex(param, radius, height):
            angle = param*math.pi*2
            vertices.append(origin+Vector((math.sin(angle)*radius, math.cos(angle)*radius, height)))
            weights.append(self.speed)
        if self.radius > 0:
            if self.spiral_count > 0.0:
                sCount = math.ceil(self.spiral_count*self.vertex_count)
                for j in range(1, int(self.screw_count)+1):
                    if j > 1:
                        vertices.append(origin+Vector((0.0, 0.0, sHeight)))
                        weights.append(self.speed)
                    sHeight = max(-j*self.pitch, height)
                    for i in range(0, sCount+1):
                        sParam = i/self.vertex_count
                        addRadialVertex(sParam, i/sCount*self.radius, sHeight)
                    for i in range(0, self.vertex_count+1):
                        addRadialVertex(sParam+(count+i)/self.vertex_count, self.radius, sHeight)
            else:
                for i in range(0, count):
                    param = i/self.vertex_count
                    addRadialVertex(param, self.radius, -param*self.pitch)
                for i in range(0, self.vertex_count+1):
                    addRadialVertex((count+i)/self.vertex_count, self.radius, height)
            weights += [1, 1]
        else:
            weights += [self.speed, 1]
        vertices += [origin+Vector((0.0, 0.0, height)), origin]
        internal.addPolygonSpline(bpy.context.object, False, vertices, weights)
        return {'FINISHED'}

operators = [OffsetCurve, RectMacro, DrillMacro]
