
import pyvista as pv
import numpy as np
from skspatial.objects import Line, Points, Plane

import os, argparse

# Mode
SAVE_CLDATA = False
SAVE_GCODE = True

parser = argparse.ArgumentParser(description='5-Axis Toolpath Generation')
parser.add_argument('mesh_cci', metavar='INPUT_MESH_FILE', help='Input CCI mesh file path')
parser.add_argument('mesh_defect', metavar='INPUT_MESH_FILE', help='Input defect edge mesh file path')
parser.add_argument('transform_reg', metavar='INPUT_CSV_FILE', help='Input transformation file path')
args = parser.parse_args()

# Read data from input arguments
mesh_cci = args.mesh_cci
mesh_defect = args.mesh_defect
tansform_input = args.transform_reg

output_file_cldata = os.path.splitext(args.mesh_cci)[0] + ".csv"
output_file_gcode = os.path.splitext(args.mesh_cci)[0] + ".gcode"

mesh_implant = pv.read(mesh_cci)
mesh_defect_wall = pv.read(mesh_defect)
transform_mat = np.genfromtxt(tansform_input)

# Transform the CCI and defect edge to the laser space
mesh_implant.transform(transform_mat, inplace=True)
mesh_defect_wall.transform(transform_mat, inplace=True)

# Plotting Themes
pv.global_theme.show_scalar_bar = False

DEBUG = False

if DEBUG:
    from pyvistaqt import BackgroundPlotter
    plotter = BackgroundPlotter()
else:
    plotter = pv.Plotter()

plotter.add_axes()
plotter.add_axes_at_origin(labels_off=True)
plotter.add_mesh(mesh_implant, color='gray')
plotter.add_mesh(mesh_defect_wall)

# project vertices to plane (deprecated)
# plane, center, normal = pv.fit_plane_to_points(mesh_defect_wall.points, return_meta=True)
# plane_pv = pv.Plane(center=center, direction=normal, i_size=100, j_size=100)

z_axis = np.array([0., 0., 1.])
center = np.array([0., 0., 0.])

# generate cutting contour by the intersection of implant and fitted lines on defect wall
num = 360
angles_total = np.linspace(np.pi/2, 5/2*np.pi, num, endpoint=False) # Start from Y+
distance_threshold = 2
radius = 1
line_length = 10
line_fitting = False

tcp_points = []
tcp_vectors = []

for angle in angles_total:

    # 1. construct a plane (plane_2) by normal and the vector (from polar angle)
    vec_a_in_plane_2 = np.array([radius * np.cos(angle), radius * np.sin(angle), 0])
    normal_2 = np.cross(z_axis, vec_a_in_plane_2)
    plane_2 = pv.Plane(mesh_defect_wall.center, normal_2, 200, 200, 1, 1) # NOTE: the plane has to be large enough for correct implicit distance
    mesh_defect_wall.compute_implicit_distance(plane_2, inplace=True) # Signed distance
    distances_vertex_to_plane = np.absolute(mesh_defect_wall['implicit_distance'])
    vertices_candidate = mesh_defect_wall.points[np.where(distances_vertex_to_plane < distance_threshold)]

    # 2. filter out incorrect candidates
    vectors_center_to_candidate = vertices_candidate - mesh_defect_wall.center
    dot_product = np.matmul(vectors_center_to_candidate, vec_a_in_plane_2.reshape((3,1)))
    vertices_candidate = vertices_candidate[np.where(dot_product > 0)[0]]

    # 3. fit a line to vertice candidate
    pts_sk = Points(vertices_candidate)

    if line_fitting:
        line_k = Line.best_fit(pts_sk)
        point_k = line_k.point

        # 3.5.1 project the best_fit line to the plane_2, get its unit direction vector
        proj_k_to_normal_2 = line_k.direction.dot(normal_2) * normal_2
        vec_u_in_plane_2 = line_k.direction - proj_k_to_normal_2
        vec_u_in_plane_2 = vec_u_in_plane_2 / np.linalg.norm(vec_u_in_plane_2)
        vec_u = vec_u_in_plane_2
    else:
        plane_X = Plane.best_fit(pts_sk)
        point_k = plane_X.point

        # 3.5.2 the normal vec of the best_fit plane cross product the normal vec of the plane_2
        vec_n_of_plane_x = plane_X.normal.round(3)
        vec_u = np.cross(vec_n_of_plane_x, normal_2)
        # Make sure all the direction are consistant
        if vec_u.dot(z_axis) <0:
            vec_u = -vec_u

    # 4. ray-tracing for intersection
    intersection_pt, ind = mesh_implant.ray_trace(point_k, point_k + vec_u * line_length)
    if intersection_pt.any():
        tcp_points.append(intersection_pt[0])
        tcp_vectors.append(vec_u)
        # Plotting
        plotter.add_points(intersection_pt, color='red')
        plotter.add_lines(np.array([intersection_pt[0], intersection_pt[0] + vec_u * line_length]), color="green")

toolpath = np.column_stack((tcp_points,tcp_vectors))

if SAVE_CLDATA:
    np.savetxt(output_file_cldata, toolpath)

if SAVE_GCODE:
    # B axis rotate 360 degree
    angles_total = np.linspace(0, 360, num, endpoint=False)

    file = open(output_file_gcode, 'w')
    file.write("G21         ; Set units to mm\n")
    file.write("G90         ; Absolute positioning\n")
    file.write("M4 S0       ; Enable Laser (0 power)\n")
    file.write("\n")

    # Parameters
    A_axis_offset = 92  # Machine home position to vertical z_axis offset
    feed_rate = 500 # What is the a good feedrate for laser cutting?
    laser_power = 100 # Maxmium laser pwr

    rotary_center_to_rotary_top = 125  # mm
    rotary_center_to_machine_x = 103.5
    rotary_center_to_machine_y = 212.5
    rotary_center_to_machine_z = 211.5
    laser_focal_len = 23.0  # TODO: Measure the focal len

    def inv_kins(th1, th2, p):
        """
        Inverse kinematics of the 5-axis laser systems
        
        Parameters
        ----------
        th1: float
            Rotary A-axis
        th2: float
            Rotary B-axis
        p: numpy array
            point in {W} frame

        Returns
        -------
        : tuple
            Joint values X, Y, Z
        """
        p = np.append(p, 1) # Convert to homogeneous coord
        th1 = np.radians(th1)
        th2 = np.radians(th2)
        c1, s1 = np.cos(th1), np.sin(th1)
        c2, s2 = np.cos(th2), np.sin(th2)
        tf_O_W = np.matmul( np.array([[1, 0, 0, 0],
                                [0, c1, -s1, -s1 * rotary_center_to_rotary_top],
                                [0, s1, c1, c1 * rotary_center_to_rotary_top],
                                [0, 0, 0, 1]]).astype(np.float64),
                            np.array([[c2, -s2, 0, 0],
                                [s2, c2, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]]).astype(np.float64) )

        tf_L_O = np.empty((4,4))
        tf_L_O[:3,:3] = np.array([[1, 0, 0],
                                [0, -1, 0],
                                [0, 0, -1]])  # Rotate about x-axis by 180 degree
        tf_L_O[:3,3] = np.array([-rotary_center_to_machine_x,
                                -rotary_center_to_machine_y,
                                rotary_center_to_machine_z-laser_focal_len])
        tf_L_O[3,:] = [0, 0, 0, 1]

        # Transform the point from {W} to {L}
        tf_L_W = np.matmul(tf_L_O, tf_O_W)
        p_in_L = tf_L_W.dot(p)

        return p_in_L[:3]
        
    XYZAB_start = None

    for n, th_2 in enumerate(angles_total):
        point_in_W = toolpath[n,:3]
        vec_in_W = toolpath[n,3:]

        # 1. Compute the A-axis value
        vec_unit = vec_in_W / np.linalg.norm(vec_in_W)
        th_1 = np.arccos(np.dot(vec_unit,z_axis))
        th_1 = 90 - np.degrees(np.pi/2 - th_1)

        # 2. Rotating along {W}_Z_axis clockwise
        th_2 = -th_2

        # 3. Get the X, Y, Z -axes joint values, need to invert Z
        XYZ = inv_kins(th_1, th_2, point_in_W)
        XYZ[-1] = -XYZ[-1]

        # 4. Append the A, B -axes values
        XYZAB = np.append(XYZ, [th_1 - A_axis_offset, th_2])

        if n == 0:
            # Save the starting point to add at the end to close the loop
            XYZAB_start = XYZAB
            line = "G0 X%.3f  Y%.3f  Z%.3f A%.3f B%.3f\n" %(XYZAB[0], XYZAB[1], XYZAB[2], XYZAB[3], XYZAB[4])
            file.write(line)
            line = "G1 F" + str(feed_rate) + "    ; Feed rate\n"
            file.write(line)
            line = "S" + str(laser_power) + "\n"
            file.write(line)
        else:
            line = "X%.3f  Y%.3f  Z%.3f A%.3f B%.3f\n" %(XYZAB[0], XYZAB[1], XYZAB[2], XYZAB[3], XYZAB[4])
            file.write(line)

            if n == num-1:
                line = "X%.3f  Y%.3f  Z%.3f A%.3f B%.3f\n" %(XYZAB_start[0], XYZAB_start[1], XYZAB_start[2], XYZAB_start[3], -360)
                file.write(line)

    file.write("M5          ; Disable Laser\n")
    file.write("G0 X-280 Y-280 Z0 A-90")

    # Close the file after writing
    file.close()

plotter.show()