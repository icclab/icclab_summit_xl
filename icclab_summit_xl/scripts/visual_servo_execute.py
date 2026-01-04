#!/usr/bin/env python3
"""
Visual Servoing Execute Node
Performs visual servoing to approach and grasp taught objects
Uses feature tracking to maintain desired relative pose
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
from pathlib import Path

from camera_utils import CameraUtils
from feature_tracker import FeatureTracker


class VisualServoExecute(Node):
    """
    Node for executing visual servoing to taught grasp poses
    Tracks object features and servos gripper to maintain taught pose
    """

    def __init__(self):
        super().__init__('visual_servo_execute')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Camera parameters
                ('camera.topic_rgb', '/camera/color/image_raw'),
                ('camera.topic_depth', '/camera/depth/image_rect_raw'),
                ('camera.topic_camera_info', '/camera/color/camera_info'),
                ('camera.depth_scale', 0.001),
                ('camera.min_depth', 0.1),
                ('camera.max_depth', 2.0),

                # Feature extraction
                ('feature_extractor.type', 'orb'),
                ('feature_extractor.max_keypoints', 500),
                ('matching.match_threshold', 0.75),
                ('matching.min_matches', 10),
                ('matching.ransac_threshold', 0.01),

                # Control
                ('control.rate', 30.0),
                ('control.k_position', 0.5),
                ('control.k_orientation', 0.3),
                ('control.max_linear_velocity', 0.1),
                ('control.max_angular_velocity', 0.3),
                ('control.position_tolerance', 0.005),
                ('control.orientation_tolerance', 0.087),
                ('control.servo_topic', '/servo_node/delta_twist_cmds'),
                ('control.servo_frame', 'arm_tool0'),

                # Tracking
                ('tracking.lost_track_threshold', 5),
                ('tracking.motion_smoothing', True),
                ('tracking.smoothing_alpha', 0.3),
                ('detection.redetect_interval', 10),

                # Storage
                ('storage.taught_grasps_directory', '~/taught_grasps'),

                # Visualization
                ('visualization.enabled', True),
                ('visualization.debug_image_topic', '/visual_servo/debug_image'),
            ]
        )

        # Get parameters
        self.rgb_topic = self.get_parameter('camera.topic_rgb').value
        self.depth_topic = self.get_parameter('camera.topic_depth').value
        self.info_topic = self.get_parameter('camera.topic_camera_info').value
        self.depth_scale = self.get_parameter('camera.depth_scale').value
        self.min_depth = self.get_parameter('camera.min_depth').value
        self.max_depth = self.get_parameter('camera.max_depth').value

        self.extractor_type = self.get_parameter('feature_extractor.type').value
        self.max_keypoints = self.get_parameter('feature_extractor.max_keypoints').value
        self.match_threshold = self.get_parameter('matching.match_threshold').value
        self.min_matches = self.get_parameter('matching.min_matches').value
        self.ransac_threshold = self.get_parameter('matching.ransac_threshold').value

        self.control_rate = self.get_parameter('control.rate').value
        self.k_position = self.get_parameter('control.k_position').value
        self.k_orientation = self.get_parameter('control.k_orientation').value
        self.max_linear_vel = self.get_parameter('control.max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('control.max_angular_velocity').value
        self.position_tol = self.get_parameter('control.position_tolerance').value
        self.orientation_tol = self.get_parameter('control.orientation_tolerance').value
        self.servo_topic = self.get_parameter('control.servo_topic').value
        self.servo_frame = self.get_parameter('control.servo_frame').value

        self.lost_track_threshold = self.get_parameter('tracking.lost_track_threshold').value
        self.motion_smoothing = self.get_parameter('tracking.motion_smoothing').value
        self.smoothing_alpha = self.get_parameter('tracking.smoothing_alpha').value
        self.redetect_interval = self.get_parameter('detection.redetect_interval').value

        self.storage_dir = os.path.expanduser(self.get_parameter('storage.taught_grasps_directory').value)
        self.viz_enabled = self.get_parameter('visualization.enabled').value
        self.debug_topic = self.get_parameter('visualization.debug_image_topic').value

        # Initialize components
        self.bridge = CvBridge()
        self.camera_utils = CameraUtils()
        self.feature_tracker = FeatureTracker(
            self.extractor_type,
            max_keypoints=self.max_keypoints,
            match_threshold=self.match_threshold
        )

        # State variables
        self.current_rgb = None
        self.current_depth = None
        self.camera_info = None
        self.camera_info_received = False

        self.servoing_active = False
        self.taught_data = None
        self.target_object = None
        self.frame_count = 0
        self.lost_track_count = 0
        self.last_velocity = np.zeros(6)

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.camera_info_callback,
            10
        )

        # Publishers
        self.twist_pub = self.create_publisher(
            TwistStamped,
            self.servo_topic,
            10
        )

        if self.viz_enabled:
            self.debug_pub = self.create_publisher(
                Image,
                self.debug_topic,
                10
            )

        # Services
        self.start_srv = self.create_service(
            Trigger,
            '/visual_servo/start',
            self.start_servoing_callback
        )
        self.stop_srv = self.create_service(
            Trigger,
            '/visual_servo/stop',
            self.stop_servoing_callback
        )

        # Control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        self.get_logger().info('Visual Servo Execute Node initialized')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info('Call /visual_servo/start to begin servoing')

    def camera_info_callback(self, msg):
        """Callback for camera info"""
        if not self.camera_info_received:
            self.camera_info = msg
            self.camera_utils = CameraUtils(msg)
            self.camera_info_received = True
            self.get_logger().info('Camera intrinsics received')

    def rgb_callback(self, msg):
        """Callback for RGB image"""
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')

    def depth_callback(self, msg):
        """Callback for depth image"""
        try:
            if msg.encoding == '16UC1':
                self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            elif msg.encoding == '32FC1':
                depth_float = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                self.current_depth = (depth_float * 1000.0).astype(np.uint16)
            else:
                self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def start_servoing_callback(self, request, response):
        """Service callback to start visual servoing"""
        if self.servoing_active:
            response.success = False
            response.message = 'Servoing already active'
            return response

        # Find most recent taught grasp
        taught_files = list(Path(self.storage_dir).glob('*.yaml'))
        if not taught_files:
            response.success = False
            response.message = f'No taught grasps found in {self.storage_dir}'
            return response

        # Load most recent file
        latest_file = max(taught_files, key=lambda p: p.stat().st_mtime)
        self.get_logger().info(f'Loading taught grasp: {latest_file}')

        try:
            with open(latest_file, 'r') as f:
                self.taught_data = yaml.safe_load(f)

            # Convert lists to numpy arrays
            self.taught_data['keypoints'] = np.array(self.taught_data['keypoints'])
            self.taught_data['descriptors'] = np.array(self.taught_data['descriptors'])
            self.taught_data['points_3d'] = np.array(self.taught_data['points_3d'])
            self.taught_data['object_centroid'] = np.array(self.taught_data['object_centroid'])
            self.taught_data['gripper_pose'] = np.array(self.taught_data['gripper_pose'])

            self.servoing_active = True
            self.frame_count = 0
            self.lost_track_count = 0

            response.success = True
            response.message = f'Started servoing to {self.taught_data["object_name"]}'
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Error loading taught grasp: {e}'
            self.get_logger().error(response.message)

        return response

    def stop_servoing_callback(self, request, response):
        """Service callback to stop visual servoing"""
        if not self.servoing_active:
            response.success = False
            response.message = 'Servoing not active'
            return response

        self.servoing_active = False
        self.send_zero_velocity()

        response.success = True
        response.message = 'Stopped servoing'
        self.get_logger().info(response.message)

        return response

    def control_loop(self):
        """Main control loop - runs at specified rate"""
        if not self.servoing_active:
            return

        if self.current_rgb is None or self.current_depth is None:
            return

        if not self.camera_info_received or self.taught_data is None:
            return

        try:
            # Extract current features
            keypoints_current, descriptors_current = self.feature_tracker.extract_features(
                self.current_rgb
            )

            if len(keypoints_current) < self.min_matches:
                self.handle_lost_tracking()
                return

            # Match with taught features
            matches = self.feature_tracker.match_features(
                self.taught_data['descriptors'],
                descriptors_current
            )

            if len(matches) < self.min_matches:
                self.handle_lost_tracking()
                return

            # Reset lost track counter
            self.lost_track_count = 0

            # Get matched keypoints
            kp_taught = self.taught_data['keypoints'][matches[:, 0]]
            kp_current = keypoints_current[matches[:, 1]]

            # Backproject current keypoints to 3D
            points_3d_current, valid_mask = self.camera_utils.backproject_points(
                kp_current,
                self.current_depth,
                depth_scale=self.depth_scale,
                min_depth=self.min_depth,
                max_depth=self.max_depth
            )

            # Filter matches by valid depth
            points_3d_taught = self.taught_data['points_3d'][matches[:, 0]][valid_mask]
            points_3d_current_valid = points_3d_current[valid_mask]

            if len(points_3d_current_valid) < self.min_matches:
                self.handle_lost_tracking()
                return

            # Estimate rigid transform from taught to current
            R, t, inliers = self.camera_utils.estimate_rigid_transform(
                points_3d_taught,
                points_3d_current_valid,
                method='ransac',
                ransac_threshold=self.ransac_threshold
            )

            num_inliers = np.sum(inliers)
            if num_inliers < self.min_matches:
                self.handle_lost_tracking()
                return

            # Compute current object pose in camera frame
            current_object_pose = self.camera_utils.pose_to_transform_matrix(R, t)

            # Compute desired gripper pose
            # desired_gripper = current_object @ taught_relative_pose
            # For now, we'll use a simple approach: move to maintain same relative position
            taught_gripper_pose = self.taught_data['gripper_pose']

            # Compute velocity command
            # Since we're in camera frame, we want to move the gripper to match taught pose
            # This is simplified - in reality, you'd use TF transforms
            velocity_cmd = self.camera_utils.compute_cartesian_velocity(
                np.eye(4),  # Current pose (camera frame as reference)
                current_object_pose,  # Desired pose
                k_p=self.k_position,
                k_r=self.k_orientation,
                max_linear_vel=self.max_linear_vel,
                max_angular_vel=self.max_angular_vel
            )

            # Apply motion smoothing
            if self.motion_smoothing:
                velocity_cmd = (self.smoothing_alpha * velocity_cmd +
                              (1 - self.smoothing_alpha) * self.last_velocity)
                self.last_velocity = velocity_cmd

            # Send velocity command
            self.send_velocity_command(velocity_cmd)

            # Visualize
            if self.viz_enabled:
                self.visualize_tracking(
                    self.current_rgb,
                    kp_current[valid_mask][inliers],
                    len(matches),
                    num_inliers
                )

            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
            self.handle_lost_tracking()

    def handle_lost_tracking(self):
        """Handle lost tracking condition"""
        self.lost_track_count += 1
        self.get_logger().warn(f'Lost tracking: {self.lost_track_count}/{self.lost_track_threshold}')

        if self.lost_track_count >= self.lost_track_threshold:
            self.get_logger().error('Tracking lost, stopping servoing')
            self.servoing_active = False
            self.send_zero_velocity()

    def send_velocity_command(self, velocity: np.ndarray):
        """Send velocity command to servo controller"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.servo_frame

        msg.twist.linear.x = float(velocity[0])
        msg.twist.linear.y = float(velocity[1])
        msg.twist.linear.z = float(velocity[2])
        msg.twist.angular.x = float(velocity[3])
        msg.twist.angular.y = float(velocity[4])
        msg.twist.angular.z = float(velocity[5])

        self.twist_pub.publish(msg)

    def send_zero_velocity(self):
        """Send zero velocity command"""
        self.send_velocity_command(np.zeros(6))

    def visualize_tracking(self, image, keypoints, num_matches, num_inliers):
        """Visualize tracking status"""
        viz_image = image.copy()

        # Draw keypoints
        for kp in keypoints:
            pt = (int(kp[0]), int(kp[1]))
            cv2.circle(viz_image, pt, 3, (0, 255, 0), -1)

        # Add status text
        cv2.putText(viz_image, f'Matches: {num_matches}',
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(viz_image, f'Inliers: {num_inliers}',
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(viz_image, f'Frame: {self.frame_count}',
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoExecute()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
