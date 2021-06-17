import pyvista as pv
import numpy as np
import os, argparse

parser = argparse.ArgumentParser(description='Generate toolpath by slicing mesh')
parser.add_argument('mesh_file', metavar='IMPLANT_MESH_FILE', help='Input mesh file path')
parser.add_argument('transformation_file', nargs='+', metavar='TRANSFORMATION_FILE', help='Saved transformation')
args = parser.parse_args()

if not args.mesh_file:
    print("Missing input mesh file")
    exit(1)

input_mesh = args.mesh_file
data_dir = os.path.dirname(input_mesh)
input_mesh_name = os.path.basename(input_mesh)
mesh = pv.read(input_mesh)

transformation_mat_0 = np.loadtxt(args.transformation_file[0]).reshape((4,4))
transformation_mat_1 = np.loadtxt(args.transformation_file[1]).reshape((4,4))
mesh.transform(np.linalg.inv(transformation_mat_1), inplace=True)
mesh.transform(np.linalg.inv(transformation_mat_0), inplace=True)
mesh.save('mesh_transformed.ply')

mesh_sliced_x_0 = mesh.slice_along_axis(n=50, axis='x')
mesh.translate([0,0,-1])
mesh_sliced_y_0 = mesh.slice_along_axis(n=50, axis='y')
mesh.translate([0,0,-1])
mesh_sliced_x_1 = mesh.slice_along_axis(n=50, axis='x')
mesh.translate([0,0,-1])
mesh_sliced_y_1 = mesh.slice_along_axis(n=50, axis='y')

plotter = pv.Plotter()
# plotter.add_mesh(mesh, rgb=True, name='mesh')
plotter.add_mesh(mesh_sliced_x_0, color='r', name='slice_line_x_0')
plotter.add_mesh(mesh_sliced_y_0, color='w', name='slice_line_y_0')
plotter.add_mesh(mesh_sliced_x_1, color='b', name='slice_line_x_1')
plotter.add_mesh(mesh_sliced_y_1, color='m', name='slice_line_y_1')
plotter.show()