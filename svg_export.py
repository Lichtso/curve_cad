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

import bpy
from mathutils import Vector, Matrix
from bpy_extras.io_utils import ExportHelper
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

class SVG_Export(bpy.types.Operator, ExportHelper):
    bl_idname = 'export_svg_format.svg'
    bl_label = 'SVG Export'
    filename_ext = '.svg'

    selection_only = bpy.props.BoolProperty(name='Selection only', description='instead of exporting all visible curves')
    absolute_coordinates = bpy.props.BoolProperty(name='Absolute coordinates', description='instead of relative coordinates')
    viewport_projection = bpy.props.BoolProperty(name='Viewport projection', description='WYSIWYG instead of an local orthographic projection')
    unit_name = bpy.props.EnumProperty(name='Unit', items=units, default='mm')

    def serialize_point(self, position, update_ref_position=True):
        if self.transform:
            position = self.transform*Vector((position[0], position[1], position[2], 1.0))
            position *= 0.5/position.w
        ref_position = self.origin if self.absolute_coordinates else self.ref_position
        command = '{},{}'.format((position[0]-ref_position[0])*self.scale[0], (position[1]-ref_position[1])*self.scale[1])
        if update_ref_position:
            self.ref_position = position
        return command

    def serialize_point_command(self, point, drawing):
        if self.absolute_coordinates:
            return ('L' if drawing else 'M')+self.serialize_point(point.co)
        else:
            return ('l' if drawing else 'm')+self.serialize_point(point.co)

    def serialize_curve_command(self, prev, next):
        return ('C' if self.absolute_coordinates else 'c')+self.serialize_point(prev.handle_right, False)+' '+self.serialize_point(next.handle_left, False)+' '+self.serialize_point(next.co)

    def serialize_spline(self, spline):
        path = ''
        points = spline.bezier_points if spline.type == 'BEZIER' else spline.points

        for index, next in enumerate(points):
            if index == 0:
                path += self.serialize_point_command(next, False)
            elif spline.type == 'BEZIER' and (points[index-1].handle_right_type != 'VECTOR' or next.handle_left_type != 'VECTOR'):
                path += self.serialize_curve_command(points[index-1], next)
            else:
                path += self.serialize_point_command(next, True)

        if spline.use_cyclic_u:
            if spline.type == 'BEZIER' and (points[-1].handle_right_type != 'VECTOR' or points[0].handle_left_type != 'VECTOR'):
                path += self.serialize_curve_command(points[-1], points[0])
            else:
                self.serialize_point(points[0].co)
            path += 'Z' if self.absolute_coordinates else 'z'

        return path

    def serialize_object(self, obj):
        if self.area:
            self.transform = self.area.spaces.active.region_3d.perspective_matrix*obj.matrix_world
            self.origin = Vector((-0.5, 0.5, 0, 0))
        else:
            self.transform = None
            self.origin = Vector((obj.bound_box[0][0], obj.bound_box[7][1], obj.bound_box[0][2], 0))

        xml = '\t<g id="'+obj.name+'">\n'
        styles = {}
        for spline in obj.data.splines:
            style = 'none'
            if obj.data.dimensions == '2D' and spline.use_cyclic_u:
                if spline.material_index < len(obj.data.materials) and obj.data.materials[spline.material_index] != None:
                    style = Vector(obj.data.materials[spline.material_index].diffuse_color)*255
                else:
                    style = Vector((0.8, 0.8, 0.8))*255
                style = 'rgb({0},{1},{2})'.format(round(style[0]), round(style[1]), round(style[2]))
            if style in styles:
                styles[style].append(spline)
            else:
                styles[style] = [spline]

        for style, splines in styles.items():
            style = 'fill:'+style+';'
            if style == 'fill:none;':
                style += 'stroke:black;'
            xml += '\t\t<path style="'+style+'" d="'
            self.ref_position = self.origin
            for spline in splines:
                xml += self.serialize_spline(spline)
            xml += '"/>\n'

        return xml+'\t</g>\n'

    def execute(self, context):
        objects = bpy.context.selected_objects if self.selection_only else bpy.context.visible_objects
        curves = []
        for obj in objects:
            if obj.type == 'CURVE':
                curves.append(obj)
        if len(curves) == 0:
            self.report({'WARNING'}, 'Nothing to export')
            return {'CANCELLED'}

        self.area = None
        if self.viewport_projection:
            for area in bpy.context.screen.areas:
                if area.type == 'VIEW_3D':
                    self.region = None
                    for region in area.regions:
                        if region.type == 'WINDOW':
                            self.region = region
                    if self.region == None:
                        continue
                    self.area = area
                    self.bounds = Vector((self.region.width, self.region.height, 0))
                    self.scale = Vector(self.bounds)
                    if self.unit_name != 'px':
                        self.unit_name = '-'

        if self.area == None:
            self.bounds = Vector((0, 0, 0))
            for obj in curves:
                self.bounds[0] = max(self.bounds[0], obj.bound_box[7][0]-obj.bound_box[0][0])
                self.bounds[1] = max(self.bounds[1], obj.bound_box[7][1]-obj.bound_box[0][1])
            self.scale = Vector((1, 1, 0))
            for unit in units:
                if self.unit_name == unit[0]:
                    self.scale *= 1.0/float(unit[2])
                    break
            self.scale *= context.scene.unit_settings.scale_length
            self.bounds = Vector(a*b for a,b in zip(self.bounds, self.scale))

        self.scale[1] *= -1
        with open(self.filepath, 'w') as f:
            svg_view = ('' if self.unit_name == '-' else 'width="{0}{2}" height="{1}{2}" ')+'viewBox="0 0 {0} {1}">\n'
            f.write('''<?xml version="1.0" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg xmlns="http://www.w3.org/2000/svg" '''+svg_view.format(self.bounds[0], self.bounds[1], self.unit_name))
            for obj in curves:
                f.write(self.serialize_object(obj))
            f.write('</svg>')

        return {'FINISHED'}
