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
    importlib.reload(cad)
    importlib.reload(toolpath)
    importlib.reload(exports)
from . import internal, cad, toolpath, exports

bl_info = {
    'name': 'Curve CAD Tools',
    'author': 'Alexander Meißner',
    'version': (1, 0, 0),
    'blender': (2, 79),
    'category': 'Curve',
    'wiki_url': 'https://github.com/Lichtso/curve_cad',
    'tracker_url': 'https://github.com/lichtso/curve_cad/issues'
}

class VIEW3D_MT_edit_curve_cad(bpy.types.Menu):
    bl_label = 'Bezier CAD'

    def draw(self, context):
        for operator in cad.operators:
            self.layout.operator(operator.bl_idname)

class INFO_MT_curve_add_toolpath(bpy.types.Menu):
    bl_label = 'Toolpath'

    def draw(self, context):
        for operator in toolpath.operators:
            self.layout.operator(operator.bl_idname)

def menu_edit_curve_specials(self, context):
    self.layout.menu('VIEW3D_MT_edit_curve_cad')
    self.layout.separator()

def menu_curve_add(self, context):
    self.layout.separator()
    self.layout.menu('INFO_MT_curve_add_toolpath')

def menu_file_export(self, context):
    for operator in exports.operators:
        self.layout.operator(operator.bl_idname)

def register():
    bpy.utils.register_module(__name__)
    bpy.types.VIEW3D_MT_edit_curve_specials.prepend(menu_edit_curve_specials)
    bpy.types.INFO_MT_curve_add.append(menu_curve_add)
    bpy.types.INFO_MT_file_export.append(menu_file_export)

def unregister():
    bpy.utils.unregister_module(__name__)
    bpy.types.VIEW3D_MT_edit_curve_specials.remove(menu_edit_curve_specials)
    bpy.types.INFO_MT_curve_add.remove(menu_curve_add)
    bpy.types.INFO_MT_file_export.remove(menu_file_export)
