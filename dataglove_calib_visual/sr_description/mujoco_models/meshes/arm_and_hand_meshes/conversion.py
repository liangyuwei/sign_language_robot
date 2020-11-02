# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# This is a python script, written for blender for batch conversion of dexterous hand collada (.dae) mesh files to .stl
# format, for use in the Mujoco simulator.

import bpy
import os

sr_description_path = '/home/user/projects/shadow_robot/base_deps/src/sr_common/sr_description'

file_names = ['forearm', 'forearm_muscle', 'forearm_muscle_disk', 'forearm_lite', 'wrist', 'palm', 'knuckle',
              'lfmetacarpal', 'F1', 'F2', 'F3', 'TH1_z', 'TH2_z', 'TH3_z']

for file_name in file_names:
    source_file_name = '{0}/meshes/hand/{1}.dae'.format(sr_description_path, file_name)
    dest_file_name = '{0}/mujoco_models/meshes/arm_and_hand_meshes/{1}.stl'.format(sr_description_path, file_name)
    print('Converting {0} to {1}...'.format(source_file_name, dest_file_name))

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    bpy.ops.wm.collada_import(filepath=source_file_name)  # change this line

    bpy.ops.object.select_all(action='SELECT')

    bpy.ops.transform.rotate(value=4.71238898038, axis=(1.0, 0, 0))

    bpy.ops.export_mesh.stl(filepath=dest_file_name)
