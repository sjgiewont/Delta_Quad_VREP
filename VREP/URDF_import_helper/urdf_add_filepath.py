import fileinput
import sys
import os

current_filepath = os.path.dirname(os.path.abspath(__file__))

stl_filepath = []

stl_filepath.append('filename=\"')
stl_filepath.append(current_filepath)
stl_filepath.append('\\')
stl_filepath = ''.join(stl_filepath)

urdf_filename = []
urdf_filename.append(sys.argv[1])
urdf_filename.append('.urdf')
urdf_filename = ''.join(urdf_filename)

mod_urdf_filename = []
mod_urdf_filename.append(sys.argv[1])
mod_urdf_filename.append('_mod.urdf')
mod_urdf_filename = ''.join(mod_urdf_filename)

f1 = open(urdf_filename, 'r')
f2 = open(mod_urdf_filename, 'w')
for line in f1:
    f2.write(line.replace('filename="', stl_filepath))
f1.close()
f2.close()