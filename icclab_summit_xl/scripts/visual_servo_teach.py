#!/usr/bin/env python3
"""
Visual Servoing Teach Node
Allows user to teach grasp poses by demonstration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
from datetime import datetime
from pathlib import Path

from camera_utils import CameraUtils
from feature_tracker import FeatureTracker


class VisualServoTeach(Node):
    """
    Node for teaching grasp poses by demonstration
    Captures object features and gripper pose for later servoing
    """

    def __init__(self):
        super().__init__('visual_servo_teach')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera.topic_rgb', '/camera/color/image_raw'),
                ('camera.topic_depth', '/camera/depth/image_rect_raw'),
                ('camera.topic_camera_info', '/camera/color/camera_info'),
                ('camera.depth_scale', 0.001),
                ('camera.min_depth', 0.1),
                ('camera.max_depth', 2.0),
                ('feature_extractor.type', 'orb'),
                ('feature_extractor.max_keypoints', 500),
                ('feature_extractor.match_threshold', 0.75),
                ('storage.taught_grasps_directory', '~/taught_grasps'),
                ('storage.auto_save', True),
                ('visualization.enabled', True),
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
        self.match_threshold = self.get_parameter('feature_extractor.match_threshold').value
        self.storage_dir = os.path.expanduser(self.get_parameter('storage.taught_grasps_directory').value)
        self.auto_save = self.get_parameter('storage.auto_save').value
        self.viz_enabled = self.get_parameter('visualization.enabled').value

        # Create storage directory
        Path(self.storage_dir).mkdir(parents=True, exist_ok=True)

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
        self.taught_objects = {}

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

        # Services
        self.teach_srv = self.create_service(
            Trigger,
            '/visual_servo/teach_grasp',
            self.teach_grasp_callback
        )

        # Visualization publisher
        if self.viz_enabled:
            self.viz_pub = self.create_publisher(
                Image,
                '/visual_servo/teach_viz',
                10
            )

        self.get_logger().info('Visual Servo Teach Node initialized')
        self.get_logger().info(f'Storage directory: {self.storage_dir}')
        self.get_logger().info('Waiting for camera data...')
        self.get_logger().info('Call service /visual_servo/teach_grasp to capture a grasp')

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
            # Depth is typically uint16 or float32
            if msg.encoding == '16UC1':
                self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            elif msg.encoding == '32FC1':
                depth_float = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                # Convert to uint16 (millimeters)
                self.current_depth = (depth_float * 1000.0).astype(np.uint16)
            else:
                self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def teach_grasp_callback(self, request, response):
        """
        Service callback to teach a grasp pose
        Captures current image, extracts features, and saves grasp data
        """
        if self.current_rgb is None or self.current_depth is None:
            response.success = False
            response.message = 'No camera data available'
            return response

        if not self.camera_info_received:
            response.success = False
            response.message = 'Camera info not received yet'
            return response

        try:
            # Get object name from user (for now, use timestamp)
            object_name = f"object_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

            self.get_logger().info(f'Teaching grasp for: {object_name}')

            # Let user select region of interest (ROI)
            roi_result = self.select_roi(self.current_rgb)
            if roi_result is None:
                response.success = False
                response.message = 'ROI selection cancelled'
                return response

            x, y, w, h = roi_result

            # Create mask from ROI
            mask = np.zeros(self.current_rgb.shape[:2], dtype=np.uint8)
            mask[y:y+h, x:x+w] = 255

            # Extract features in ROI
            keypoints, descriptors = self.feature_tracker.extract_features(
                self.current_rgb,
                mask=mask
            )

            if len(keypoints) < 10:
                response.success = False
                response.message = f'Too few features detected: {len(keypoints)}. Need at least 10.'
                return response

            self.get_logger().info(f'Extracted {len(keypoints)} features')

            # Backproject keypoints to 3D
            points_3d, valid_mask = self.camera_utils.backproject_points(
                keypoints,
                self.current_depth,
                depth_scale=self.depth_scale,
                min_depth=self.min_depth,
                max_depth=self.max_depth
            )

            # Filter valid points
            keypoints_3d = keypoints[valid_mask]
            descriptors_3d = descriptors[valid_mask]
            points_3d_valid = points_3d[valid_mask]

            if len(keypoints_3d) < 10:
                response.success = False
                response.message = f'Too few valid 3D points: {len(keypoints_3d)}. Need at least 10.'
                return response

            self.get_logger().info(f'Valid 3D points: {len(points_3d_valid)}')

            # Get current gripper pose (for now, we'll use identity)
            # In real implementation, this should come from TF2
            gripper_pose = np.eye(4)
            gripper_pose[:3, 3] = [0.0, 0.0, 0.0]  # Placeholder

            # Compute object centroid
            object_centroid = np.mean(points_3d_valid, axis=0)

            # Store taught data
            taught_data = {
                'object_name': object_name,
                'timestamp': datetime.now().isoformat(),
                'keypoints': keypoints_3d.tolist(),
                'descriptors': descriptors_3d.tolist(),
                'points_3d': points_3d_valid.tolist(),
                'object_centroid': object_centroid.tolist(),
                'gripper_pose': gripper_pose.tolist(),
                'roi': [int(x), int(y), int(w), int(h)],
                'extractor_type': self.extractor_type,
                'num_features': len(keypoints_3d)
            }

            self.taught_objects[object_name] = taught_data

            # Save to file
            if self.auto_save:
                save_path = os.path.join(self.storage_dir, f'{object_name}.yaml')
                with open(save_path, 'w') as f:
                    yaml.dump(taught_data, f)
                self.get_logger().info(f'Saved taught grasp to: {save_path}')

            # Save snapshot images
            rgb_path = os.path.join(self.storage_dir, f'{object_name}_rgb.png')
            depth_path = os.path.join(self.storage_dir, f'{object_name}_depth.png')
            cv2.imwrite(rgb_path, self.current_rgb)

            # Normalize depth for visualization
            depth_viz = self.current_depth.astype(np.float32)
            depth_viz = cv2.normalize(depth_viz, None, 0, 255, cv2.NORM_MINMAX)
            cv2.imwrite(depth_path, depth_viz.astype(np.uint8))

            # Visualize
            if self.viz_enabled:
                viz_image = self.visualize_taught_grasp(
                    self.current_rgb,
                    keypoints_3d,
                    roi_result
                )
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                self.viz_pub.publish(viz_msg)

            response.success = True
            response.message = f'Successfully taught grasp for {object_name} with {len(keypoints_3d)} features'

        except Exception as e:
            self.get_logger().error(f'Error teaching grasp: {e}')
            response.success = False
            response.message = f'Error: {str(e)}'

        return response

    def select_roi(self, image):
        """
        Interactive ROI selection using OpenCV
        Returns (x, y, w, h) or None if cancelled
        """
        # Display image
        display_image = image.copy()
        cv2.putText(display_image, 'Select ROI around object, then press SPACE or ENTER',
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_image, 'Press C to cancel',
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        roi = cv2.selectROI('Select Object ROI', display_image, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow('Select Object ROI')

        if roi[2] == 0 or roi[3] == 0:  # Width or height is 0
            return None

        return roi

    def visualize_taught_grasp(self, image, keypoints, roi):
        """
        Visualize taught grasp with features and ROI
        """
        viz_image = image.copy()

        # Draw ROI
        x, y, w, h = roi
        cv2.rectangle(viz_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Draw keypoints
        for kp in keypoints:
            pt = (int(kp[0]), int(kp[1]))
            cv2.circle(viz_image, pt, 3, (0, 0, 255), -1)

        # Add text
        cv2.putText(viz_image, f'Taught: {len(keypoints)} features',
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return viz_image

    def list_taught_objects(self):
        """List all taught objects"""
        return list(self.taught_objects.keys())

    def load_taught_grasp(self, object_name):
        """Load a taught grasp from file"""
        file_path = os.path.join(self.storage_dir, f'{object_name}.yaml')
        if not os.path.exists(file_path):
            self.get_logger().error(f'Taught grasp file not found: {file_path}')
            return None

        with open(file_path, 'r') as f:
            taught_data = yaml.safe_load(f)

        # Convert lists back to numpy arrays
        taught_data['keypoints'] = np.array(taught_data['keypoints'])
        taught_data['descriptors'] = np.array(taught_data['descriptors'])
        taught_data['points_3d'] = np.array(taught_data['points_3d'])
        taught_data['object_centroid'] = np.array(taught_data['object_centroid'])
        taught_data['gripper_pose'] = np.array(taught_data['gripper_pose'])

        self.taught_objects[object_name] = taught_data
        return taught_data


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoTeach()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
