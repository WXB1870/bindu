"""ROS message-level tests; real numerical LIO/ICP tests live separately."""
import math
import unittest
try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS 2 unavailable')
class LioProjectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args',
            '-p', 'imu_from_lidar:=[0.5,0.,0.,0.,0.,0.,1.]',
            '-p', 'imu_from_base:=[0.,0.,0.,0.,0.,0.,1.]',
            '-p', 'min_height:=0.0', '-p', 'max_height:=1.0'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_projection_uses_lidar_origin_and_preserves_timestamp(self):
        from bindu_lio.lio_scan_node import LioScan
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Header
        from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
        node = LioScan()
        try:
            outputs = []
            # Intercept only the publication; the real ROS node, parameters,
            # PointCloud2 parser, pairing and projection are exercised.
            class Capture:
                def publish(self, message):
                    outputs.append(message)
            node.publisher = Capture()
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header = Header(stamp=stamp, frame_id='odom')
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = 1.
            odom.pose.pose.orientation.w = 1.
            node.receive_odom(odom)
            cloud = create_cloud_xyz32(odom.header, [[3.5, 0., .5], [4.5, 0., .5], [2., 0., 2.]])
            node.receive_cloud(cloud)
            node.publish_pending()
            self.assertEqual(len(outputs), 1)
            scan = outputs[0]
            self.assertEqual(scan.header.stamp, stamp)
            self.assertEqual(scan.header.frame_id, 'lidar')
            valid = [value for value in scan.ranges if math.isfinite(value)]
            self.assertEqual(valid, [2.])
            self.assertTrue(any(math.isnan(value) for value in scan.ranges))
        finally:
            node.destroy_node()

    def test_wrong_frame_cloud_is_rejected(self):
        from bindu_lio.lio_scan_node import LioScan
        from std_msgs.msg import Header
        from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
        node = LioScan()
        try:
            cloud = create_cloud_xyz32(Header(stamp=node.get_clock().now().to_msg(), frame_id='map'), [[1., 0., .5]])
            node.receive_cloud(cloud)
            self.assertIsNone(node.pending)
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
