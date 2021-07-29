import pyvista as pv
import trimesh
import numpy as np
from scipy.spatial import cKDTree

from pytransform3d.rotations import axis_angle_from_two_directions, matrix_from_axis_angle, active_matrix_from_angle

import os
import argparse

parser = argparse.ArgumentParser(description='Extract spherical fiducials')
parser.add_argument('mesh_file', metavar='IMPLANT_MESH_FILE', help='Input mesh file path')
parser.add_argument('dxf_file', nargs='+', metavar='INPUT_DXF_FILE', help='Input dxf file path')
parser.add_argument('-p', '--param', dest='parameters', metavar='PARAM_FILE', help='Load saved parameters')
args = parser.parse_args()

if not args.mesh_file or not args.dxf_file:
    print("Missing input mesh file or dxf file")
    exit(1)

# params
if args.parameters:
     saved_params = np.loadtxt(args.parameters)
     picked_point = saved_params[:3]
     extrude_height = saved_params[3]
     z_axis_rotate_angle = saved_params[4]*180/np.pi
else:
    picked_point = None
    extrude_height = 10
    z_axis_rotate_angle = 0
    
neighbor_search_radius = 10 #mm this param can be tuned, the user picks a point on the top surface of the implant

input_mesh = args.mesh_file
data_dir = os.path.dirname(input_mesh)
input_mesh_name = os.path.basename(input_mesh)

input_path2d_list = []
output_file_names = []
for input_dxf in args.dxf_file:
    input_path2d_list.append(trimesh.exchange.load.load(input_dxf))
    input_dxf_name = os.path.basename(input_dxf)
    output_file_names.append(os.path.splitext(input_dxf_name)[0] + "_extrusion.ply")
    
mesh = pv.read(input_mesh)
tree = cKDTree(mesh.points)

plotter = pv.Plotter()
plotter.add_mesh(mesh, rgb=True, name="mesh")

def point_picking_callback(point):
    if not args.parameters:
        global picked_point
        picked_point = point

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
    # First, rotate along absolute z-axis (2)
    R_mat = active_matrix_from_angle(2, z_axis_rotate_angle)
    M_1 = np.identity(4)
    M_1[:3,:3] = R_mat
    
    # Second, rotate z-axis to align with fit plane normal, then translate to picked point
    z_axis = np.array([0,0,1])
    R_axis_angle = axis_angle_from_two_directions(z_axis, plane_normal)
    R_mat = matrix_from_axis_angle(R_axis_angle)
    M_2 = np.identity(4)
    M_2[:3,:3] = R_mat
    M_2[:3,3] = picked_point
    
    # save transformations to file
    np.savetxt('pocket_transformation_1.csv', M_1)
    np.savetxt('pocket_transformation_2.csv', M_2)
    
    # extrude 2D path into a primitive shape, then apply the transformation
    global path2d_extruded_list
    path2d_extruded_list = []
    for input_path2d in input_path2d_list:
        path2d_extruded = input_path2d.extrude(extrude_height)
        path2d_extruded.apply_transform(M_1)
        path2d_extruded.apply_transform(M_2)
        path2d_extruded.slide(-extrude_height/2)
        path2d_extruded_list.append(path2d_extruded)
    for i, j, k in zip(range(len(path2d_extruded_list)), ['k', 'r', 'r'], [0.3, 0.5, 0.5]):
        plotter.add_mesh(pv.wrap(path2d_extruded_list[i].to_mesh()), color=j, opacity=k, name="dxf_extrusion_"+str(i))

def extrude_slider_callback(value):
    global extrude_height
    extrude_height = value
    if picked_point is not None:
        point_picking_callback(picked_point)

def rotate_angle_slider_callback(value):
    global z_axis_rotate_angle
    z_axis_rotate_angle = value*np.pi/180
    if picked_point is not None:
        point_picking_callback(picked_point)

# define key callback to save extrusions: key "m"
def m_key_callback():
    for extruded_primitive, file_name in zip(path2d_extruded_list, output_file_names):
        extruded_primitive.export(file_name)
    f = open('pocket_position_extruheight_rotangle.txt', 'a+')
    f.truncate(0)
    np.savetxt(f, picked_point, fmt='%.3f', newline=' ')
    f.write('%.2f'%extrude_height)
    f.write(' ')
    f.write('%.5f'%z_axis_rotate_angle)
    # np.savetxt(f, extrude_height, fmt='%.2f', newline=' ')
    # np.savetxt(f, z_axis_rotate_angle, fmt='%.2f', newline=' ')
    f.close()

plotter.enable_point_picking(callback=point_picking_callback, show_message=False, point_size=3, color='black', show_point=True)
plotter.add_slider_widget(callback=extrude_slider_callback, rng=(0,100), value=extrude_height, title="Extrude", pointa=(.6, .94), pointb=(.95, .94))
plotter.add_slider_widget(callback=rotate_angle_slider_callback, rng=(-180,180), value=z_axis_rotate_angle, title="Rotate", pointa=(.6, .81), pointb=(.95, .81))
plotter.add_key_event("m", m_key_callback)
plotter.show()