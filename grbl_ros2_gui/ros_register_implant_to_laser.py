import argparse

from numpy.core.numeric import ones

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import TransformStamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from visualization_msgs.msg import Marker

import numpy as np
from scipy.spatial.transform import Rotation as R

class RegistrationNode(Node):

    def __init__(self, name, mesh, points_ct):
        super().__init__(name)

        # create a TransformListener object. Once the listener is created, it starts receiving tf2 transformations over the wire, and buffers them for up to 10 seconds.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, qos=QoSProfile(depth=10))

        # A static tansform broadcaster to publish the registration result
        self.tf_static_publisher = StaticTransformBroadcaster(self)

        self.pub_vis_marker = self.create_publisher(Marker, 'implant_mesh_rviz_marker', QoSProfile(depth=10))

        # Reference points in {y_box1}
        ring_top_left = [-0.220556, 0.321296, -0.069628]
        ring_top_right = [-0.078448, 0.325427, -0.069864]
        ring_bottom_left = [-0.215818, 0.179873, -0.072110]
        ring_bottom_right = [-0.074046, 0.184153, -0.072792]

        self.points_laser = np.array([ring_top_left,
                                      ring_top_right,
                                      ring_bottom_right,
                                      ring_bottom_left]) * 1000
        self.points_ct = points_ct

        # Timers for excuting the callback functions
        self.timer_one_shot = self.create_timer(timer_period_sec=0.5, callback=self.register)
        self.timer = self.create_timer(timer_period_sec=0.5, callback=self.publish_mesh_marker)

        self.mesh = mesh
        self.flag_registered = False

    def register(self):
        try:
            trans = self.tf_buffer.lookup_transform('W', 'y_box1', time=rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1))
        except:
            print('lookup_transform(): No transformation found')
        else:
            # One time only, cancel the timer
            self.timer_one_shot.cancel()

            # Homogeneous transformation from W to y_box1 (Machine)
            H_trans_W__M = np.eye(4)
            H_trans_W__M[:3,:3] = R.from_quat([
                                        trans.transform.rotation.x,
                                        trans.transform.rotation.y,
                                        trans.transform.rotation.z,
                                        trans.transform.rotation.w
                                        ]).as_matrix()
            H_trans_W__M[0,3] = trans.transform.translation.x * 1000
            H_trans_W__M[1,3] = trans.transform.translation.y * 1000
            H_trans_W__M[2,3] = trans.transform.translation.z * 1000

            # Transform the measured point to {W}
            points_laser = np.hstack((self.points_laser, np.ones((4,1)))) # Convert to homogeneous coordinates
            self.points_laser = H_trans_W__M.dot(points_laser.T).T[:,:3]

            # Register the two correspondence point set
            rot, t = RegistrationNode.rigid_transform_3D(self.points_ct.T, self.points_laser.T)

            # Export the registration result to file
            H = np.eye(4)
            H[:3,:3] = rot
            H[:3,3] = t.flatten()
            np.savetxt('transformation.csv', H, fmt='%1.6f')

            # Publish a static transformation for visulizing the implant in Rviz2
            rot = R.from_matrix(rot)
            rot_quat = rot.as_quat()
            static_transformStamped = TransformStamped()
            static_transformStamped.header.stamp = self.get_clock().now().to_msg()
            static_transformStamped.header.frame_id = 'W'
            static_transformStamped.child_frame_id = 'implant'
            static_transformStamped.transform.rotation.x = rot_quat[0]
            static_transformStamped.transform.rotation.y = rot_quat[1]
            static_transformStamped.transform.rotation.z = rot_quat[2]
            static_transformStamped.transform.rotation.w = rot_quat[3]
            static_transformStamped.transform.translation.x = H[0,3] * 0.001
            static_transformStamped.transform.translation.y = H[1,3] * 0.001
            static_transformStamped.transform.translation.z = H[2,3] * 0.001
            self.tf_static_publisher.sendTransform(static_transformStamped)

            self.flag_registered = True

    def publish_mesh_marker(self):
        if self.flag_registered:
            marker = Marker()
            marker.header.frame_id = "implant"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "implant"
            marker.id = 0
            marker.action = marker.ADD
            marker.type = marker.MESH_RESOURCE
            marker.mesh_resource = "file://" + self.mesh
            marker.scale.x = 0.001
            marker.scale.y = 0.001
            marker.scale.z = 0.001
            # marker.color.r = 1.
            # marker.color.g = 1.
            # marker.color.b = 1.
            # marker.color.a = 0.8
            marker.frame_locked = True
            marker.mesh_use_embedded_materials = True
            self.pub_vis_marker.publish(marker)

    @staticmethod
    def rigid_transform_3D(A, B):
        '''
        Input: expects 3xN matrix of points
        Returns R,t
        R = 3x3 rotation matrix
        t = 3x1 column vector
        This function is borrowed from https://github.com/nghiaho12/rigid_transform_3D
        '''
        assert A.shape == B.shape

        num_rows, num_cols = A.shape
        if num_rows != 3:
            raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

        num_rows, num_cols = B.shape
        if num_rows != 3:
            raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

        # find mean column wise
        centroid_A = np.mean(A, axis=1)
        centroid_B = np.mean(B, axis=1)

        # ensure centroids are 3x1
        centroid_A = centroid_A.reshape(-1, 1)
        centroid_B = centroid_B.reshape(-1, 1)

        # subtract mean
        Am = A - centroid_A
        Bm = B - centroid_B

        H = Am @ np.transpose(Bm)

        # sanity check
        #if linalg.matrix_rank(H) < 3:
        #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

        # find rotation
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # special reflection case
        if np.linalg.det(R) < 0:
            print("det(R) < R, reflection detected!, correcting for it ...")
            Vt[2,:] *= -1
            R = Vt.T @ U.T

        t = -R @ centroid_A + centroid_B

        return R, t

def main(args=None):

    parser = argparse.ArgumentParser()
    parser.add_argument('mesh', metavar='MESH_FILE', help="Input mesh file")
    parser.add_argument('points', metavar='POINTS_FILE', help="Input correspondent points")
    my_args = parser.parse_args()
    points_ct = np.genfromtxt(my_args.points)

    rclpy.init(args=args)
    laser_registration_node = RegistrationNode('registration', my_args.mesh, points_ct)
    rclpy.spin(laser_registration_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laser_registration_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
