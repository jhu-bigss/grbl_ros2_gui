import pyvista as pv
import trimesh
import numpy as np
from scipy.spatial import cKDTree

from pytransform3d.rotations import axis_angle_from_two_directions, matrix_from_axis_angle, active_matrix_from_angle

import os
import argparse

parser = argparse.ArgumentParser(description='Extract spherical fiducials')
parser.add_argument('mesh_file', metavar='INPUT_FILE', help='Input mesh file path')
parser.add_argument('dxf_file', nargs='+', metavar='INPUT_FILE', help='Input dxf file path')
args = parser.parse_args()

if not args.mesh_file or not args.dxf_file:
    print("Missing input mesh file or dxf file")
    exit(1)

# params
extrude_height = 10
neighbor_search_radius = 10 #mm this param can be tuned, the user picks a point on the top surface of the implant
z_axis_rotate_angle = 0

input_mesh = args.mesh_file
data_dir = os.path.dirname(input_mesh)
input_mesh_name = os.path.basename(input_mesh)

input_path2d_list = []
output_file_names = []
for input_dxf in args.dxf_file:
    input_path2d_list.append(trimesh.exchange.load.load(input_dxf))
    input_dxf_name = os.path.basename(input_dxf)
    output_file_names.append(os.path.splitext(input_dxf_name)[0] + "_extrusion.ply")
    
input_mesh = pv.read(input_mesh)
tree = cKDTree(input_mesh.points)

plotter = pv.Plotter()
plotter.add_mesh(input_mesh, rgb=True, name="input_mesh")

def point_picking_callback(mesh, point_id):
    picked_point = mesh.points[point_id]

    # get neighboring points of the picked point (within given radius)
    picked_point_neighbors = mesh.points[tree.query_ball_point(picked_point, neighbor_search_radius)]
    # plotter.add_points(picked_point_neighbors, color='b', point_size=3, name='pocket_ref_pt')

    # fit plane
    plane_center, plane_normal = trimesh.points.plane_fit(picked_point_neighbors)
    center_sphere = pv.Sphere(radius=0.5, center=picked_point, direction=plane_normal)
    plotter.add_mesh(center_sphere, color='r', name='pocket_ref_point')
    plane_normal_arrow = pv.Arrow(start=picked_point, direction=plane_normal, shaft_radius=0.02, scale=20)
    plotter.add_mesh(plane_normal_arrow, color='b', name='pocket_ref_plane_normal')

    # compute the transformation
    z_axis = np.array([0,0,1])
    R_axis_angle = axis_angle_from_two_directions(z_axis, plane_normal)
    R_mat = matrix_from_axis_angle(R_axis_angle)
    global H
    H = np.identity(4)
    H[:3,:3] = R_mat
    H[:3,3] = picked_point
    
    # extrude 2D path into a primitive shape, then apply the transformation
    global path2d_extruded_list
    path2d_extruded_list = []
    for input_path2d in input_path2d_list:
        path2d_extruded = input_path2d.extrude(extrude_height)
        path2d_extruded.apply_transform(H)
        # path2d_extruded.apply_translation(picked_point)
        path2d_extruded.slide(-extrude_height/2)
        path2d_extruded_list.append(path2d_extruded)
    for i, j, k in zip(range(len(path2d_extruded_list)), ['k', 'r', 'r'], [0.3, 0.5, 0.5]):
        plotter.add_mesh(pv.wrap(path2d_extruded_list[i].to_mesh()), color=j, opacity=k, name="dxf_extrusion_"+str(i))

def slider_callback(value):
    global z_axis_rotate_angle
    rotate_angle = z_axis_rotate_angle - value*np.pi/180
    z_axis_rotate_angle = value*np.pi/180
    for path2d_extrusion in path2d_extruded_list:
        path2d_extrusion.apply_transform(np.linalg.inv(H))
        R_mat = active_matrix_from_angle(2, rotate_angle)
        M = np.identity(4)
        M[:3,:3] = R_mat
        path2d_extrusion.apply_transform(M)
        path2d_extrusion.apply_transform(H)
    for i, j, k in zip(range(len(path2d_extruded_list)), ['k', 'r', 'r'], [0.3, 0.5, 0.5]):
        plotter.add_mesh(pv.wrap(path2d_extruded_list[i].to_mesh()), color=j, opacity=k, name="dxf_extrusion_"+str(i))

# define key callback to save extrusions: key "m"
def m_key_callback():
    for extruded_primitive, file_name in zip(path2d_extruded_list, output_file_names):
        extruded_primitive.export(file_name)
    np.savetxt('transformation.txt', H.flatten(), fmt='%.5f', newline=' ')

plotter.enable_point_picking(callback=point_picking_callback, show_message=False, point_size=3, color='black', use_mesh=True, show_point=True)
plotter.add_slider_widget(callback=slider_callback, rng=(-180,180), value=0, title="Rotate Angle")
plotter.add_key_event("m", m_key_callback)
plotter.show()