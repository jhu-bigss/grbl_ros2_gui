import pyvista as pv
import trimesh
import numpy as np
from scipy.spatial import cKDTree

from pytransform3d.rotations import matrix_from_two_vectors

import os
import argparse

parser = argparse.ArgumentParser(description='Extract spherical fiducials')
parser.add_argument('mesh_file', metavar='INPUT_FILE', help='Input mesh file path')
parser.add_argument('dxf_file', metavar='INPUT_FILE', help='Input dxf file path')
args = parser.parse_args()

if not args.mesh_file or not args.dxf_file:
    print("Missing input mesh file or dxf file")
    exit(1)

# params
input_mesh = args.mesh_file
data_dir = os.path.dirname(input_mesh)
input_mesh_name = os.path.basename(input_mesh)
output_file_name = os.path.splitext(input_mesh_name)[0] + "_topcap.ply"
neighbor_search_radius = 10 #mm this param can be tuned, the user picks a point on the top surface of the implant

input_mesh = pv.read(input_mesh)
tree = cKDTree(input_mesh.points)

plotter = pv.Plotter()
plotter.add_mesh(input_mesh, rgb=True, name="implant")

input_dxf = args.dxf_file
input_path2d = trimesh.exchange.load.load(input_dxf)
input_path2d_extruded = input_path2d.extrude(10)
result = pv.wrap(input_path2d_extruded)
print(result)

# define point picking callback
def point_picking_callback(mesh, point_id):
    picked_point = mesh.points[point_id]

    # get neighboring points within a radius
    picked_point_neighbors = mesh.points[tree.query_ball_point(picked_point, neighbor_search_radius)]
    # plotter.add_points(picked_point_neighbors, color='b', point_size=3, name='pocket_ref_pt')

    # fit sphere
    plane_center, plane_normal = trimesh.points.plane_fit(picked_point_neighbors)
    center_sphere = pv.Sphere(radius=0.5, center=picked_point, direction=plane_normal)
    plotter.add_mesh(center_sphere, color='r', name='pocket_ref_point')
    plane_normal_arrow = pv.Arrow(start=picked_point, direction=plane_normal, shaft_radius=0.03, scale=10)
    plotter.add_mesh(plane_normal_arrow, color='b', name='pocket_ref_plane_normal')



# define key callback to save centers: key "s"
def key_callback():
    if poly.n_points > 0:
        # save center of fiducials
        np.savetxt(output_file, poly.points, fmt='%.5f', delimiter=",")
        print("csv file saved")
    else:
        print("No points to save")

plotter.enable_point_picking(callback=point_picking_callback, show_message=False, point_size=3, color='black', use_mesh=True, show_point=True)
plotter.add_key_event("s", key_callback)
plotter.show()