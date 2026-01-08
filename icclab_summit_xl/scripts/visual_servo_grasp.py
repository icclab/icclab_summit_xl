#!/usr/bin/env python3
"""
Visual Servoing Grasp Node

This node implements a visual servoing approach for grasping flat objects from planar surfaces:
1. Uses Lang-SAM for initial object detection and segmentation
2. Fits a plane to the table surface using RANSAC
3. Estimates grasp pose using PCA on the object point cloud
4. Moves to pre-grasp pose above the object with feedback monitoring
5. Descends vertically while tracking the object with a lightweight tracker
6. Uses PID control to maintain XY alignment during descent
7. Monitors arm motion and servo status for feedback control
8. Closes gripper when reaching the target height

Assumptions:
- Top-down grasps for flat objects on planar surfaces
- 2-finger gripper
- Gripper-mounted RGB-D camera
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, JointState, CameraInfo
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose, Point, TransformStamped
from std_msgs.msg import String, ColorRGBA
from moveit_msgs.srv import ServoCommandType
from moveit_msgs.msg import ServoStatus
from control_msgs.action import GripperCommand
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
# open3d imported lazily in fit_plane_ransac() to speed up node startup
from scipy.spatial.transform import Rotation
import sensor_msgs_py.point_cloud2 as pc2
import time
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Servo command type constants
SERVO_JOINT_JOG = 0
SERVO_TWIST = 1
SERVO_POSE = 2

# Servo status codes
SERVO_NO_WARNING = 0
SERVO_DECEL_SINGULARITY = 1
SERVO_HALT_SINGULARITY = 2
SERVO_DECEL_LEAVING_SINGULARITY = 3
SERVO_DECEL_COLLISION = 4
SERVO_HALT_COLLISION = 5
SERVO_JOINT_BOUND = 6


class MotionMonitor:
    """
    Monitors arm motion to detect if velocity commands are being followed.
    Uses joint state feedback to verify actual motion matches commanded motion.
    """

    def __init__(self, stall_threshold=0.001, stall_time=0.5):
        """
        Args:
            stall_threshold: Minimum joint velocity to consider arm moving (rad/s)
            stall_time: Time without motion before declaring stall (seconds)
        """
        self.stall_threshold = stall_threshold
        self.stall_time = stall_time

        # Joint state tracking
        self.current_positions = None
        self.current_velocities = None
        self.previous_positions = None
        self.joint_names = []

        # Motion tracking
        self.last_motion_time = None
        self.is_moving = False
        self.commanded_velocity = np.zeros(6)

        # Statistics for debugging
        self.total_commands_sent = 0
        self.motion_detected_count = 0

    def update_joint_state(self, joint_state_msg, arm_joint_names):
        """
        Update with new joint state from /joint_states topic.

        Args:
            joint_state_msg: JointState message
            arm_joint_names: List of arm joint names to track
        """
        self.previous_positions = self.current_positions

        # Extract arm joint positions and velocities
        positions = []
        velocities = []

        for name in arm_joint_names:
            if name in joint_state_msg.name:
                idx = joint_state_msg.name.index(name)
                positions.append(joint_state_msg.position[idx])
                if len(joint_state_msg.velocity) > idx:
                    velocities.append(joint_state_msg.velocity[idx])
                else:
                    velocities.append(0.0)

        if len(positions) == len(arm_joint_names):
            self.current_positions = np.array(positions)
            self.current_velocities = np.array(velocities)
            self.joint_names = arm_joint_names

            # Detect motion
            self._check_motion()

    def _check_motion(self):
        """Check if arm is actually moving based on joint velocities."""
        if self.current_velocities is None:
            return

        # Check if any joint is moving above threshold
        max_velocity = np.max(np.abs(self.current_velocities))

        if max_velocity > self.stall_threshold:
            self.is_moving = True
            self.last_motion_time = time.time()
            self.motion_detected_count += 1
        else:
            # Also check position change as backup
            if self.previous_positions is not None:
                position_change = np.max(np.abs(self.current_positions - self.previous_positions))
                if position_change > self.stall_threshold * 0.033:  # ~30Hz dt
                    self.is_moving = True
                    self.last_motion_time = time.time()
                    self.motion_detected_count += 1
                    return

            self.is_moving = False

    def set_commanded_velocity(self, velocity):
        """Record the commanded velocity for comparison."""
        self.commanded_velocity = np.array(velocity)
        self.total_commands_sent += 1

    def is_stalled(self):
        """
        Check if arm is stalled (commanded to move but not moving).

        Returns:
            True if arm should be moving but isn't
        """
        # Not stalled if no commands sent
        if self.last_motion_time is None:
            return False

        # Not stalled if command is zero
        if np.max(np.abs(self.commanded_velocity)) < 0.001:
            return False

        # Check if arm hasn't moved for stall_time
        if not self.is_moving:
            time_since_motion = time.time() - self.last_motion_time
            return time_since_motion > self.stall_time

        return False

    def get_motion_quality(self):
        """
        Get a quality metric for how well commands are being followed.

        Returns:
            Float in [0, 1] where 1 means perfect command following
        """
        if self.total_commands_sent == 0:
            return 1.0

        return min(1.0, self.motion_detected_count / max(1, self.total_commands_sent))

    def reset(self):
        """Reset motion tracking state."""
        self.current_positions = None
        self.current_velocities = None
        self.previous_positions = None
        self.last_motion_time = None
        self.is_moving = False
        self.commanded_velocity = np.zeros(6)
        self.total_commands_sent = 0
        self.motion_detected_count = 0


class PlanarPIDController:
    """Simple PID controller for XY alignment during vertical descent."""

    def __init__(self, kp_xy=0.5, kp_yaw=0.3, kp_z=0.2):
        self.kp_xy = kp_xy
        self.kp_yaw = kp_yaw
        self.kp_z = kp_z

        # Derivative terms
        self.kd_xy = 0.1
        self.kd_yaw = 0.05

        self.prev_error_xy = np.zeros(2)
        self.prev_error_yaw = 0.0
        self.dt = 0.033  # ~30Hz

    def update(self, error_xy, error_yaw, error_z=None):
        """
        Compute velocity command from position errors.

        Args:
            error_xy: [dx, dy] in meters
            error_yaw: rotation error in radians
            error_z: optional vertical error (if None, use constant descent)

        Returns:
            [vx, vy, vz, omega_x, omega_y, omega_z] velocity command
        """
        # Derivative terms
        d_error_xy = (error_xy - self.prev_error_xy) / self.dt
        d_error_yaw = (error_yaw - self.prev_error_yaw) / self.dt

        # PD control for XY
        v_xy = self.kp_xy * error_xy + self.kd_xy * d_error_xy

        # PD control for yaw
        omega_z = self.kp_yaw * error_yaw + self.kd_yaw * d_error_yaw

        # Z control (simple proportional or constant descent)
        if error_z is not None:
            v_z = self.kp_z * error_z
        else:
            v_z = -0.01  # Constant 1cm/s descent

        # Clamp velocities
        v_xy = np.clip(v_xy, -0.05, 0.05)  # Max 5cm/s in XY
        v_z = np.clip(v_z, -0.02, 0.02)     # Max 2cm/s in Z
        omega_z = np.clip(omega_z, -0.2, 0.2)  # Max 0.2 rad/s yaw

        # Update previous errors
        self.prev_error_xy = error_xy.copy()
        self.prev_error_yaw = error_yaw

        return np.array([v_xy[0], v_xy[1], v_z, 0.0, 0.0, omega_z])


class SimpleMaskTracker:
    """
    Scale-invariant tracker using OpenCV CSRT.
    Handles scale changes, rotation, and partial occlusion.
    """

    def __init__(self):
        self.tracker = None  # OpenCV CSRT tracker instance
        self.template_color = None  # Store initial template for visualization
        self.template_bbox = None  # Current bounding box (x, y, w, h) - updated each frame
        self.prev_center = None
        self.prev_angle = 0.0
        self.initialized = False
        self.init_bbox = None  # Initial bbox for reference

    def initialize(self, image, mask, debug_logger=None):
        """Initialize CSRT tracker with bounding box from mask."""
        # Get bounding box from mask
        y_coords, x_coords = np.where(mask > 0)
        if len(x_coords) == 0:
            if debug_logger:
                debug_logger.warn('Tracker init failed: empty mask')
            return False

        x_min, x_max = x_coords.min(), x_coords.max()
        y_min, y_max = y_coords.min(), y_coords.max()

        # Add small margin
        margin = 10
        x_min = max(0, x_min - margin)
        x_max = min(image.shape[1], x_max + margin)
        y_min = max(0, y_min - margin)
        y_max = min(image.shape[0], y_max + margin)

        # Validate dimensions
        bbox_w = x_max - x_min
        bbox_h = y_max - y_min
        if bbox_w < 10 or bbox_h < 10:
            if debug_logger:
                debug_logger.warn(f'Tracker init failed: bbox too small {bbox_w}x{bbox_h}')
            return False

        # Initialize OpenCV CSRT tracker
        self.tracker = cv2.TrackerCSRT_create()
        bbox = (x_min, y_min, bbox_w, bbox_h)

        # CSRT needs BGR image
        if len(image.shape) == 2:
            image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            image_bgr = image

        try:
            self.tracker.init(image_bgr, bbox)
        except Exception as e:
            if debug_logger:
                debug_logger.error(f'Tracker init failed: {e}')
            return False

        # Store initial bbox and template for visualization
        self.init_bbox = bbox
        self.template_bbox = bbox
        template_crop = image_bgr[y_min:y_max, x_min:x_max].copy()
        self.template_color = template_crop

        self.prev_center = np.array([x_min + bbox_w/2, y_min + bbox_h/2])

        # Estimate initial angle from mask
        mask_crop = mask[y_min:y_max, x_min:x_max]
        _, self.prev_angle = self._get_pose_from_mask(mask_crop)

        if debug_logger:
            debug_logger.info(f'CSRT tracker initialized: bbox={bbox}, center=({self.prev_center[0]:.1f},{self.prev_center[1]:.1f})')

        self.initialized = True
        return True

    def _get_pose_from_mask(self, mask):
        """Extract 2D center and orientation from mask."""
        if mask.sum() == 0:
            return None, 0.0

        # Compute centroid
        moments = cv2.moments(mask.astype(np.uint8))
        if moments['m00'] == 0:
            return None, 0.0

        cx = moments['m10'] / moments['m00']
        cy = moments['m01'] / moments['m00']

        # Estimate orientation using minimum area rectangle
        contours, _ = cv2.findContours(
            mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            return np.array([cx, cy]), 0.0

        # Get largest contour
        contour = max(contours, key=cv2.contourArea)

        if len(contour) < 5:  # Need at least 5 points for fitEllipse
            return np.array([cx, cy]), 0.0

        # Fit oriented bounding box
        rect = cv2.minAreaRect(contour)
        angle = np.deg2rad(rect[2])

        return np.array([cx, cy]), angle

    def update(self, image):
        """
        Track object in new frame using CSRT tracker.

        Returns:
            center: [x, y] pixel coordinates
            angle: orientation in radians
            confidence: tracking confidence [0, 1]
        """
        if not self.initialized or self.tracker is None:
            return None, 0.0, 0.0

        # Store the image used for tracking (for visualization consistency)
        self._last_tracked_image = image

        # Skip tracker update every N frames for speed (CSRT is slow)
        # Can be disabled by setting skip_frames=False
        if getattr(self, 'skip_frames', True):
            if not hasattr(self, '_frame_skip_counter'):
                self._frame_skip_counter = 0
            self._frame_skip_counter += 1

            if self._frame_skip_counter % 2 != 0 and self.prev_center is not None:
                # Skip this frame, return previous result
                return self.prev_center, self.prev_angle, 0.85

        # CSRT needs BGR image
        if len(image.shape) == 2:
            image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            image_bgr = image

        # Update CSRT tracker
        try:
            success, bbox = self.tracker.update(image_bgr)
        except Exception as e:
            # Tracker update failed
            return self.prev_center, self.prev_angle, 0.0

        if not success:
            # Tracking failed - return previous position with zero confidence
            return self.prev_center, self.prev_angle, 0.0

        # Extract bbox components
        x, y, w, h = bbox
        x, y, w, h = int(x), int(y), int(w), int(h)

        # Update stored bbox for visualization
        self.template_bbox = (x, y, w, h)

        # Calculate center from bbox
        center = np.array([x + w/2, y + h/2])

        # Estimate confidence based on bbox size consistency
        # CSRT doesn't provide confidence directly, so we estimate it
        if self.init_bbox is not None:
            init_area = self.init_bbox[2] * self.init_bbox[3]
            current_area = w * h

            # Confidence is high when area is reasonably close to initial
            # Allow 3x growth (object getting closer) or 0.3x shrink (object further)
            area_ratio = current_area / max(1, init_area)
            if 0.3 <= area_ratio <= 3.0:
                confidence = 0.9  # Good tracking
            elif 0.1 <= area_ratio <= 5.0:
                confidence = 0.6  # Marginal tracking
            else:
                confidence = 0.3  # Poor tracking (extreme scale change)
        else:
            confidence = 0.8  # Default confidence

        # Estimate angle from bbox - the long axis indicates object orientation
        # CSRT bbox is axis-aligned, so we need to analyze the image region
        # For now, use bbox aspect ratio to determine if object is more horizontal or vertical
        # and refine using edge detection within the bbox
        angle = self._estimate_angle_from_bbox(image_bgr, x, y, w, h)

        # Update tracking state
        self.prev_center = center
        self.prev_angle = angle

        return center, angle, confidence

    def _estimate_angle_from_bbox(self, image, x, y, w, h):
        """
        Estimate object orientation angle from the tracked bounding box region.
        Uses edge detection and line fitting to find the dominant orientation.

        Returns angle in radians where 0 = horizontal, pi/2 = vertical.
        We want the gripper to grasp across the long axis, so we return
        the angle that needs to be corrected (target is 0 for horizontal grip).
        """
        # Extract region of interest
        img_h, img_w = image.shape[:2]
        x1 = max(0, x)
        y1 = max(0, y)
        x2 = min(img_w, x + w)
        y2 = min(img_h, y + h)

        if x2 <= x1 or y2 <= y1:
            return self.prev_angle

        roi = image[y1:y2, x1:x2]

        # Convert to grayscale if needed
        if len(roi.shape) == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi

        # Use Canny edge detection
        edges = cv2.Canny(gray, 50, 150)

        # Find contours in the edge image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return self.prev_angle

        # Combine all contour points
        all_points = np.vstack(contours)

        if len(all_points) < 5:
            return self.prev_angle

        # Fit minimum area rectangle to get orientation
        rect = cv2.minAreaRect(all_points)
        rect_angle = rect[2]  # Angle in degrees from -90 to 0
        rect_w, rect_h = rect[1]

        # minAreaRect returns angle of the shorter side relative to horizontal
        # We want angle of the LONG axis
        # If width > height, long axis is at rect_angle
        # If height > width, long axis is at rect_angle + 90
        if rect_h > rect_w:
            long_axis_angle = rect_angle + 90
        else:
            long_axis_angle = rect_angle

        # Normalize to [-90, 90] range
        while long_axis_angle > 90:
            long_axis_angle -= 180
        while long_axis_angle < -90:
            long_axis_angle += 180

        # Convert to radians
        angle_rad = np.deg2rad(long_axis_angle)

        # Apply low-pass filter to reduce noise
        alpha = 0.3
        filtered_angle = alpha * angle_rad + (1 - alpha) * self.prev_angle

        return filtered_angle

    def get_visualization(self, confidence=None):
        """
        Get visualization image showing tracker state.
        Uses the same image that was passed to update() for consistency.

        Args:
            confidence: Optional tracking confidence to display

        Returns:
            debug_image: BGR image with tracker visualization
        """
        if not self.initialized or not hasattr(self, '_last_tracked_image'):
            return None

        debug_img = self._last_tracked_image.copy()

        # Draw template bounding box with current match location
        if hasattr(self, 'template_bbox'):
            x, y, w, h = self.template_bbox

            # Green for good confidence, yellow for medium, red for poor
            if confidence is not None:
                if confidence > 0.7:
                    bbox_color = (0, 255, 0)  # Green
                elif confidence > 0.4:
                    bbox_color = (0, 255, 255)  # Yellow
                else:
                    bbox_color = (0, 0, 255)  # Red
            else:
                bbox_color = (0, 255, 0)

            cv2.rectangle(debug_img, (int(x), int(y)), (int(x+w), int(y+h)), bbox_color, 2)

            # Draw label
            label = f"Match: {w}x{h}"
            cv2.putText(debug_img, label, (int(x), int(y) - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, bbox_color, 2)

        # Draw center point
        if self.prev_center is not None:
            cx, cy = int(self.prev_center[0]), int(self.prev_center[1])
            cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)
            cv2.circle(debug_img, (cx, cy), 10, (0, 0, 255), 2)

            # Draw orientation line
            length = 50
            end_x = int(cx + length * np.cos(self.prev_angle))
            end_y = int(cy + length * np.sin(self.prev_angle))
            cv2.line(debug_img, (cx, cy), (end_x, end_y), (255, 0, 0), 2)

        # Draw image center crosshair
        h, w = debug_img.shape[:2]
        center_x, center_y = w // 2, h // 2
        cv2.drawMarker(debug_img, (center_x, center_y), (255, 255, 0),
                      cv2.MARKER_CROSS, 20, 2)

        # Draw error vector from object to center
        if self.prev_center is not None:
            cv2.line(debug_img, (int(self.prev_center[0]), int(self.prev_center[1])),
                    (center_x, center_y), (255, 0, 255), 2)

            # Calculate pixel error
            error_x = self.prev_center[0] - center_x
            error_y = self.prev_center[1] - center_y
            error_norm = np.sqrt(error_x**2 + error_y**2)

            # Add text overlay with tracking info
            info_y = 30
            cv2.putText(debug_img, f"Center: ({int(self.prev_center[0])}, {int(self.prev_center[1])})",
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            info_y += 25

            cv2.putText(debug_img, f"Error: ({int(error_x)}, {int(error_y)}) = {int(error_norm)}px",
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            info_y += 25

            if confidence is not None:
                # Color based on confidence
                if confidence > 0.7:
                    color = (0, 255, 0)  # Green
                elif confidence > 0.4:
                    color = (0, 255, 255)  # Yellow
                else:
                    color = (0, 0, 255)  # Red

                cv2.putText(debug_img, f"Confidence: {confidence:.3f}",
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                info_y += 25

        # Show template as inset in corner
        if hasattr(self, 'template_color') and self.template_color is not None:
            # Resize template to reasonable size for display (max 150x150)
            th, tw = self.template_color.shape[:2]
            scale = min(150.0 / tw, 150.0 / th)
            new_w, new_h = int(tw * scale), int(th * scale)

            if new_w > 0 and new_h > 0:
                template_display = cv2.resize(self.template_color, (new_w, new_h))

                # Place in bottom-right corner with margin
                margin = 10
                y_start = h - new_h - margin
                x_start = w - new_w - margin

                # Add border
                cv2.rectangle(debug_img, (x_start - 2, y_start - 2),
                            (x_start + new_w + 2, y_start + new_h + 2), (255, 255, 255), 2)

                # Overlay template
                debug_img[y_start:y_start+new_h, x_start:x_start+new_w] = template_display

                # Label
                cv2.putText(debug_img, "Template", (x_start, y_start - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Add image shape info for debugging
        cv2.putText(debug_img, f"Image: {debug_img.shape}", (10, h - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        return debug_img


class VisualServoGrasp(Node):
    """Visual servoing grasp node."""

    # Arm joint names for Summit XL with UR5
    ARM_JOINT_NAMES = [
        'arm_shoulder_pan_joint'
        'arm_shoulder_lift_joint'
        'arm_elbow_joint'
        'arm_wrist_1_joint'
        'arm_wrist_2_joint'
        'arm_wrist_3_joint'
    ]

    def __init__(self):
        super().__init__('visual_servo_grasp')

        # Parameters - distances from fingertips to the table plane
        self.declare_parameter('pre_grasp_height', 0.25)  # Fingertip distance to table plane at pre-grasp
        self.declare_parameter('grasp_clearance', 0.20)  # Fingertip distance to table plane when grasping
        self.declare_parameter('descent_speed', 0.01)     # Vertical descent speed (m/s)
        self.declare_parameter('servo_rate', 30.0)        # Control loop rate (Hz)
        self.declare_parameter('xy_tolerance', 0.005)     # Position tolerance (m)
        self.declare_parameter('approach_speed', 0.03)    # Approach speed (m/s)
        self.declare_parameter('state_timeout', 30.0)     # Timeout for each state (seconds)
        self.declare_parameter('stall_threshold', 0.001)  # Joint velocity threshold for stall detection
        self.declare_parameter('stall_time', 1.0)         # Time without motion before stall (seconds)
        self.declare_parameter('max_stall_retries', 3)    # Max retries on stall before aborting
        self.declare_parameter('orientation_tolerance', 0.15)  # Radians (~8.6 deg) tolerance for gripper alignment
        # Camera-to-fingertip offset in camera optical frame (meters)
        self.declare_parameter('fingertip_offset_x', 0.128)  # Fingertips ahead of camera
        self.declare_parameter('fingertip_offset_y', -0.031) # Fingertips slightly right of camera
        self.declare_parameter('fingertip_offset_z', 0.080)  # Fingertips below camera (closer to table)

        self.pre_grasp_height = self.get_parameter('pre_grasp_height').value
        self.grasp_clearance = self.get_parameter('grasp_clearance').value
        self.descent_speed = self.get_parameter('descent_speed').value
        self.servo_rate = self.get_parameter('servo_rate').value
        self.xy_tolerance = self.get_parameter('xy_tolerance').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.state_timeout = self.get_parameter('state_timeout').value
        self.stall_threshold = self.get_parameter('stall_threshold').value
        self.stall_time = self.get_parameter('stall_time').value
        self.max_stall_retries = self.get_parameter('max_stall_retries').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.max_orientation_time = self.get_parameter('state_timeout').value
        # Camera-to-fingertip offset vector in camera optical frame
        self.fingertip_offset = np.array([
            self.get_parameter('fingertip_offset_x').value,
            self.get_parameter('fingertip_offset_y').value,
            self.get_parameter('fingertip_offset_z').value
        ])
        self.get_logger().info(
            f'Fingertip offset (camera frame): [{self.fingertip_offset[0]:.3f}, '
            f'{self.fingertip_offset[1]:.3f}, {self.fingertip_offset[2]:.3f}]m'
        )

        # CV Bridge
        self.bridge = CvBridge()

        # State
        self.current_rgb = None
        self.current_depth = None
        self.current_scene_pointcloud = None  # Full scene pointcloud from arm camera
        self.segmented_pointcloud = None  # Segmented object pointcloud
        self.segmentation_mask = None
        self.segmentation_rgb = None  # RGB image synchronized with segmentation mask
        self.intrinsic_matrix = None
        self.fx = None  # Focal length x (will be set from camera_info)
        self.fy = None  # Focal length y
        self.cx = None  # Principal point x
        self.cy = None  # Principal point y

        self.table_plane = None  # [a, b, c, d] plane equation in camera frame (for visualization)
        self.table_plane_world = None  # [a, b, c, d] plane equation in world frame (base_footprint)
        self.table_normal_world = None  # Table normal in world/base frame
        self.grasp_pose = None  # Grasp pose in camera frame (for visualization)
        self.grasp_position_world = None  # Target grasp position in world frame [x, y, z]
        self.object_thickness = 0.005  # Default 5mm
        self.target_z = None  # Target Z height for grasp
        self.initial_object_z = None  # Initial Z position of object in camera frame
        self.use_orientation_fallback = False  # Flag to use fallback grasp without perfect orientation

        # Controllers and trackers
        self.pid = PlanarPIDController()
        self.tracker = SimpleMaskTracker()
        self.motion_monitor = MotionMonitor(
            stall_threshold=self.stall_threshold,
            stall_time=self.stall_time
        )

        # Tracker re-initialization tracking
        self.last_segmentation_request_time = 0.0
        self.segmentation_request_interval = 2.0  # Request new segmentation every 2 seconds
        self.tracker_reinit_confidence_threshold = 0.4  # Re-init if confidence drops below this
        self.object_description = None  # Store for re-segmentation requests
        self.waiting_for_reinit_segmentation = False  # Flag to track if we're waiting for new mask

        # State machine
        # IDLE -> DETECTING -> APPROACH (XYZ, no rotation) -> ORIENT (rotation + XY) -> DESCENDING -> GRASPING -> LIFTING -> DONE
        self.state = 'IDLE'
        self.state_start_time = None
        self.stall_retry_count = 0

        # Servo status tracking
        self.servo_status_code = SERVO_NO_WARNING
        self.servo_status_msg = ""

        # TF2 for tracking gripper orientation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for sensor data - depth 1 and reliable
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/arm_camera/color/image_raw',
            self.rgb_callback,
            sensor_qos
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/arm_camera/depth/image_raw',
            self.depth_callback,
            sensor_qos,
        )

        # Subscribe to full scene pointcloud from arm camera for plane fitting
        self.scene_pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/arm_camera/points',
            self.scene_pointcloud_callback,
            sensor_qos,
        )

        # Subscribe to segmented object pointcloud
        self.segmented_pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/segmented_pointcloud',
            self.segmented_pointcloud_callback,
            sensor_qos,
        )

        # Subscribe to segmentation mask and RGB
        # Segmentation node publishes both the mask AND the RGB it used
        # This guarantees they match perfectly (no sync needed)
        # Synchronized subscription for segmentation mask and RGB
        self.mask_sub = Subscriber(self, Image, '/segmentation_mask')
        self.segmentation_rgb_sub = Subscriber(self, Image, '/segmentation_rgb')

        self.segmentation_sync = ApproximateTimeSynchronizer(
            [self.mask_sub, self.segmentation_rgb_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.segmentation_sync.registerCallback(self.segmentation_sync_callback)

        # Trigger subscriber to start grasp
        self.grasp_trigger_sub = self.create_subscription(
            String,
            '/start_grasp',
            self.start_grasp_callback,
            10
        )

        # Camera intrinsics subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/arm_camera/color/camera_info',
            self.camera_info_callback,
            sensor_qos
        )

        # Joint state subscriber for motion feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Servo status subscriber for collision/singularity detection
        servo_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.servo_status_sub = self.create_subscription(
            ServoStatus,
            '/servo_node/status',
            self.servo_status_callback,
            servo_status_qos
        )

        # Publishers
        # MoveIt Servo expects BEST_EFFORT reliability
        servo_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            servo_qos
        )

        self.status_pub = self.create_publisher(
            String,
            '/grasp_status',
            10
        )

        # Gripper action client
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd'
        )

        # Gripper positions (from summit_xl.srdf)
        self.gripper_open_position = 0.01    # finger_joint position for open
        self.gripper_closed_position = 0.69  # finger_joint position for closed

        # Segment request publisher (to trigger segmentation)
        self.segment_request_pub = self.create_publisher(
            String,
            '/segment_text',
            10
        )

        # Visualization publishers
        self.plane_marker_pub = self.create_publisher(
            Marker,
            '/plane_marker',
            10
        )

        self.gripper_orientation_marker_pub = self.create_publisher(
            Marker,
            '/gripper_orientation_marker',
            10
        )

        self.table_normal_marker_pub = self.create_publisher(
            Marker,
            '/table_normal_marker',
            10
        )

        # Tracker debug visualization publisher
        self.tracker_debug_pub = self.create_publisher(
            Image,
            '/tracker_debug_image',
            10
        )

        # Servo mode switch service client
        self.servo_switch_client = self.create_client(
            ServoCommandType,
            '/servo_node/switch_command_type',
        )

        # Timer for control loop (use callback group for parallel execution)
        self.control_timer = self.create_timer(
            1.0 / self.servo_rate,
            self.control_loop
        )

        self.get_logger().info('Visual Servo Grasp Node initialized')
        self.get_logger().info('Waiting for grasp trigger on /start_grasp')
        self.get_logger().info('Send object description as String message to start')

    def rgb_callback(self, msg):
        """Store current RGB image."""
        if not hasattr(self, '_rgb_callback_count'):
            self._rgb_callback_count = 0
            self._rgb_skipped_count = 0
        self._rgb_callback_count += 1

        # Skip conversion in IDLE/DONE/ERROR states to save CPU
        if self.state in ['IDLE', 'DONE', 'ERROR']:
            self._rgb_skipped_count += 1
            if self._rgb_callback_count % 100 == 0:
                self.get_logger().info(f'RGB callback: received={self._rgb_callback_count}, skipped={self._rgb_skipped_count}, state={self.state}')
            return
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.current_rgb_frame_id = msg.header.frame_id
            if self._rgb_callback_count % 100 == 0:
                self.get_logger().info(f'RGB callback: received={self._rgb_callback_count}, processed, state={self.state}')
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')

    def depth_callback(self, msg):
        """Store current depth image."""
        # Skip conversion in IDLE/DONE/ERROR states to save CPU
        if self.state in ['IDLE', 'DONE', 'ERROR']:
            return

        # Gazebo RGBD camera publishes depth in float32 meters
        if msg.encoding == '32FC1':
            depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            # Replace NaN/inf with 0, keep in meters
            self.current_depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        elif msg.encoding == '16UC1':
            # Real cameras may use uint16 in millimeters
            depth_mm = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.current_depth = depth_mm.astype(np.float32) / 1000.0  # Convert to meters
        else:
            self.get_logger().warn(f'Unknown depth encoding: {msg.encoding}')

    def camera_info_callback(self, msg):
        """Extract and store camera intrinsics from CameraInfo message."""
        # K is a 3x3 intrinsic matrix stored as a flat array [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        K = np.array(msg.k).reshape(3, 3)
        self.intrinsic_matrix = K
        self.fx = K[0, 0]
        self.fy = K[1, 1]
        self.cx = K[0, 2]
        self.cy = K[1, 2]
        self.get_logger().info(
            f'Camera intrinsics received: fx={self.fx:.1f}, fy={self.fy:.1f}, '
            f'cx={self.cx:.1f}, cy={self.cy:.1f}'
        )
        # Unsubscribe - intrinsics don't change
        self.destroy_subscription(self.camera_info_sub)

    def scene_pointcloud_callback(self, msg):
        """Store current full scene point cloud from arm camera."""
        # Only needed in DETECTING state (for plane fitting)
        # Skip in all other states to save CPU - we use depth image instead
        if self.state != 'DETECTING':
            return
        self.current_scene_pointcloud = msg
        self.get_logger().info(f'Received scene pointcloud: {msg.width}x{msg.height} points')

    def segmented_pointcloud_callback(self, msg):
        """Store current segmented object point cloud."""
        # Only needed in DETECTING state (for object bounds calculation)
        if self.state != 'DETECTING':
            return
        self.segmented_pointcloud = msg
        self.get_logger().info(f'Received segmented pointcloud: {msg.width}x{msg.height} points')

    def segmentation_sync_callback(self, mask_msg, rgb_msg):
        """
        Synchronized callback for segmentation mask and RGB.
        Both come from the same segmentation, guaranteed to match.
        """
        mask = self.bridge.imgmsg_to_cv2(mask_msg, 'mono8')
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')

        self.segmentation_mask = (mask > 0).astype(np.uint8)
        self.segmentation_rgb = rgb

        if self.state == 'DETECTING':
            self.get_logger().info(
                f'Received SYNCHRONIZED segmentation: mask {mask.shape} (non-zero: {np.sum(mask > 0)}), '
                f'RGB {rgb.shape}'
            )

        # Handle tracker re-initialization during active servoing states
        # Always reinitialize when fresh segmentation arrives (not just when explicitly waiting)
        if self.state in ['APPROACH', 'ORIENT', 'DESCENDING']:
            mask_area = np.sum(self.segmentation_mask)
            if mask_area > 50 and self.segmentation_rgb is not None:
                # Trust the segmentation model - no heuristic rejection
                y_coords, x_coords = np.where(self.segmentation_mask > 0)
                mask_width = x_coords.max() - x_coords.min() if len(x_coords) > 0 else 0
                mask_height = y_coords.max() - y_coords.min() if len(y_coords) > 0 else 0

                self.get_logger().info(
                    f'Re-initializing tracker with SYNCHRONIZED mask+RGB: {mask_width}x{mask_height}, '
                    f'area={mask_area}'
                )

                # In ORIENT state with yaw segmentation requested: compute target yaw from mask
                # This is more reliable than tracker bbox for orientation
                if self.state == 'ORIENT' and getattr(self, '_yaw_segmentation_requested', False):
                    target_yaw = self.compute_yaw_from_mask(self.segmentation_mask)
                    if target_yaw is not None:
                        self._target_yaw_from_mask = target_yaw
                        self.get_logger().info(
                            f'Target yaw computed from mask: {np.rad2deg(target_yaw):.1f}deg'
                        )
                    else:
                        self.get_logger().warn('Failed to compute yaw from mask')

                # Use synchronized RGB image that matches the mask timestamp
                if self.tracker.initialize(self.segmentation_rgb, self.segmentation_mask, self.get_logger()):
                    self.get_logger().info('Tracker successfully re-initialized with synchronized images')
                    self.waiting_for_reinit_segmentation = False
                    # Track when tracker was re-initialized for stabilization period
                    self._tracker_reinit_time = time.time()
                else:
                    self.get_logger().warn('Tracker re-initialization failed')
                    self.waiting_for_reinit_segmentation = False

    def joint_state_callback(self, msg):
        """Update motion monitor with current joint states."""
        self.motion_monitor.update_joint_state(msg, self.ARM_JOINT_NAMES)

    def servo_status_callback(self, msg):
        """Track servo status for collision/singularity detection."""
        self.servo_status_code = msg.code
        self.servo_status_msg = msg.message

        # Rate-limit servo warnings to once every 2 seconds
        current_time = time.time()
        if not hasattr(self, '_last_servo_warn_time'):
            self._last_servo_warn_time = 0.0

        if current_time - self._last_servo_warn_time < 2.0:
            return  # Skip logging if within rate limit

        # Log warnings and errors
        if msg.code == SERVO_HALT_SINGULARITY:
            self.get_logger().error(f'Servo HALTED: singularity - {msg.message}')
            self._last_servo_warn_time = current_time
        elif msg.code == SERVO_HALT_COLLISION:
            self.get_logger().error(f'Servo HALTED: collision - {msg.message}')
            self._last_servo_warn_time = current_time
        elif msg.code == SERVO_JOINT_BOUND:
            self.get_logger().warn(f'Servo: joint bound reached - {msg.message}')
            self._last_servo_warn_time = current_time
        elif msg.code in [SERVO_DECEL_SINGULARITY, SERVO_DECEL_COLLISION]:
            self.get_logger().warn(f'Servo decelerating: {msg.message}')
            self._last_servo_warn_time = current_time

    def is_servo_halted(self):
        """Check if servo is halted due to collision or singularity."""
        return self.servo_status_code in [SERVO_HALT_SINGULARITY, SERVO_HALT_COLLISION, SERVO_JOINT_BOUND]

    def compute_yaw_from_mask(self, mask):
        """
        Compute object orientation (yaw) directly from segmentation mask using PCA.

        PCA finds the principal axes of the mask pixel distribution, which is more
        robust than minAreaRect because it considers all pixels (mass distribution)
        rather than just the contour outline.

        Args:
            mask: Binary segmentation mask (numpy array)

        Returns:
            angle: Orientation in radians (angle of long axis), or None if mask invalid
        """
        if mask is None or mask.sum() == 0:
            return None

        # Get all mask pixel coordinates
        y_coords, x_coords = np.where(mask > 0)

        if len(x_coords) < 10:
            return None

        # Stack into points array (N x 2)
        points = np.column_stack((x_coords, y_coords)).astype(np.float64)

        # Compute mean (centroid)
        mean = np.mean(points, axis=0)

        # Center the points
        centered = points - mean

        # Compute covariance matrix
        cov = np.cov(centered.T)

        # Compute eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eig(cov)

        # First principal component (largest eigenvalue) = long axis direction
        long_axis_idx = np.argmax(eigenvalues)
        long_axis = eigenvectors[:, long_axis_idx]

        # Compute angle of long axis (in image coordinates: x=right, y=down)
        angle_rad = np.arctan2(long_axis[1], long_axis[0])

        # Normalize to [-pi/2, pi/2] - we don't care about 180 degree ambiguity
        while angle_rad > np.pi / 2:
            angle_rad -= np.pi
        while angle_rad < -np.pi / 2:
            angle_rad += np.pi

        # Compute eigenvalue ratio for logging (indicates how elongated the object is)
        eigenvalue_ratio = max(eigenvalues) / (min(eigenvalues) + 1e-6)

        self.get_logger().info(
            f'Yaw from mask (PCA): angle={np.rad2deg(angle_rad):.1f}deg, '
            f'eigenvalue_ratio={eigenvalue_ratio:.1f} (higher=more elongated)'
        )

        return angle_rad

    def get_gripper_orientation(self, target_frame='base_footprint'):
        """
        Get current gripper orientation in target frame.

        Args:
            target_frame: Frame to transform to (default: base_footprint)

        Returns:
            Rotation object or None if transform unavailable
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                'arm_tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            quat = transform.transform.rotation
            return Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def get_gripper_yaw(self):
        """
        Get the gripper's yaw angle (rotation around its Z-axis) relative to base.

        This extracts the yaw component of the gripper rotation, which corresponds
        to rotation in the image plane when the gripper is pointing down.

        Returns:
            float: Yaw angle in radians, or None if TF unavailable
        """
        gripper_rot = self.get_gripper_orientation()
        if gripper_rot is None:
            return None

        # Extract Euler angles (ZYX convention: yaw, pitch, roll)
        # The gripper's Z-axis yaw is what matters for image-plane rotation
        euler = gripper_rot.as_euler('ZYX')
        return euler[0]  # First component is Z rotation (yaw)

    def get_gripper_position(self, target_frame='base_footprint'):
        """
        Get current gripper position in target frame.

        Args:
            target_frame: Frame to transform to (default: base_footprint)

        Returns:
            numpy array [x, y, z], or None if transform fails
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                'arm_tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            pos = transform.transform.translation
            return np.array([pos.x, pos.y, pos.z])

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def get_gripper_distance_to_plane(self):
        """
        Compute the distance from arm_tool0 to the detected table plane.

        The plane is stored in world frame (base_footprint), so we get gripper
        position in world frame and compute distance to plane.

        Returns:
            float: Distance in meters (positive = above plane), or None if unavailable
        """
        if self.table_plane_world is None:
            return None

        # Get gripper position in world frame
        gripper_pos = self.get_gripper_position(target_frame='base_footprint')
        if gripper_pos is None:
            return None

        # Compute signed distance from gripper to plane
        # Plane equation: ax + by + cz + d = 0
        # Signed distance = (ax + by + cz + d) / sqrt(a² + b² + c²)
        a, b, c, d = self.table_plane_world
        normal_magnitude = np.sqrt(a*a + b*b + c*c)

        signed_distance = (a * gripper_pos[0] +
                          b * gripper_pos[1] +
                          c * gripper_pos[2] + d) / normal_magnitude

        # Plane normal points DOWN (negative Z in world frame)
        # Positive signed_distance means gripper is above the plane
        return abs(signed_distance)

    def get_fingertip_distance_to_plane(self):
        """
        Compute the distance from fingertips to the detected table plane.

        This accounts for the camera-to-fingertip offset, projecting it onto
        the table normal to determine how much closer the fingertips are to
        the table than the camera.

        Returns:
            float: Distance in meters (positive = above plane), or None if unavailable
        """
        if self.table_plane_world is None:
            return None

        # Get camera (gripper) position and orientation in world frame
        gripper_pos = self.get_gripper_position(target_frame='base_footprint')
        gripper_rot = self.get_gripper_orientation()
        if gripper_pos is None or gripper_rot is None:
            return None

        # Transform fingertip offset from camera frame to world frame
        # The offset is defined in camera optical frame
        rot_matrix = gripper_rot.as_matrix()
        fingertip_offset_world = rot_matrix @ self.fingertip_offset

        # Compute fingertip position in world frame
        fingertip_pos = gripper_pos + fingertip_offset_world

        # Compute signed distance from fingertip to plane
        a, b, c, d = self.table_plane_world
        normal_magnitude = np.sqrt(a*a + b*b + c*c)

        signed_distance = (a * fingertip_pos[0] +
                          b * fingertip_pos[1] +
                          c * fingertip_pos[2] + d) / normal_magnitude

        return abs(signed_distance)

    def get_fingertip_offset_toward_plane(self):
        """
        Compute how much closer the fingertips are to the table than the camera.

        Projects the camera-to-fingertip offset vector onto the table normal.
        This gives the vertical component of the offset that matters for collision.

        Returns:
            float: Offset in meters (positive = fingertips closer to table), or None if unavailable
        """
        if self.table_plane_world is None:
            return None

        gripper_rot = self.get_gripper_orientation()
        if gripper_rot is None:
            return None

        # Transform fingertip offset from camera frame to world frame
        rot_matrix = gripper_rot.as_matrix()
        fingertip_offset_world = rot_matrix @ self.fingertip_offset

        # Table normal (a, b, c) points DOWN toward table
        a, b, c, d = self.table_plane_world
        table_normal = np.array([a, b, c])
        table_normal = table_normal / np.linalg.norm(table_normal)

        # Project offset onto table normal
        # Positive result means fingertips are closer to table
        offset_toward_table = np.dot(fingertip_offset_world, table_normal)

        return offset_toward_table

    def compute_orientation_error(self, gripper_rotation, target_normal_world):
        """
        Compute angular error between gripper Z-axis and target normal.

        Args:
            gripper_rotation: Rotation object for gripper in world frame
            target_normal_world: Table normal vector in world frame (pointing DOWN towards table)

        Returns:
            angular_error: Angle in radians between gripper Z and target direction
            axis: Rotation axis to correct the error (unit vector in world frame)
        """
        if gripper_rotation is None:
            return None, None

        # Get gripper Z-axis in world frame (this is the direction gripper points)
        gripper_z_world = gripper_rotation.as_matrix()[:, 2]

        # Target direction: gripper should align with table_normal_world (which points DOWN)
        target_direction = target_normal_world

        # Compute alignment
        dot_product = np.dot(gripper_z_world, target_direction)
        dot_product = np.clip(dot_product, -1.0, 1.0)
        angular_error = np.arccos(dot_product)

        # Compute rotation axis (cross product)
        axis = np.cross(gripper_z_world, target_direction)
        axis_norm = np.linalg.norm(axis)

        if axis_norm < 1e-6:
            # Already aligned or opposite
            return angular_error, np.array([0.0, 0.0, 0.0])

        axis = axis / axis_norm

        return angular_error, axis

    def _update_plane_and_grasp_pose(self):
        """
        Continuously update table plane estimation.
        Called during APPROACH and ORIENT states to handle humanoid base movement.
        Uses rate limiting (1 second) to avoid blocking the servo control loop.

        IMPORTANT: Plane estimation with RANSAC is computationally expensive.
        Running it too frequently will block servo commands and cause the arm to stop.
        MoveIt Servo requires consistent high-frequency commands to maintain motion.
        """
        # Rate limit: update at most every 5.0 seconds to avoid blocking servo commands
        current_time = time.time()
        if not hasattr(self, '_last_plane_update_time'):
            self._last_plane_update_time = 0.0

        if current_time - self._last_plane_update_time < 5.0:
            return

        # Only need scene pointcloud for plane fitting
        if self.current_scene_pointcloud is None:
            return

        # Mark update time BEFORE computation to prevent re-entry
        self._last_plane_update_time = current_time

        try:
            scene_points = self.pointcloud2_to_array(self.current_scene_pointcloud)
            if len(scene_points) < 500:
                return

            # Subsample points to reduce computation time
            # Use at most 5000 points for plane fitting
            max_points = 5000
            if len(scene_points) > max_points:
                indices = np.random.choice(len(scene_points), max_points, replace=False)
                scene_points = scene_points[indices]

            # Fit table plane on scene (largest plane should be table)
            # Reduced iterations for faster computation
            new_table_plane, _ = self.fit_plane_ransac(
                scene_points, distance_threshold=0.01, num_iterations=200
            )

            # Validate and orient plane normal
            a, b, c, d = new_table_plane
            normal = np.array([a, b, c])
            normal = normal / np.linalg.norm(normal)

            # Ensure normal points toward camera (negative Z in camera frame)
            if normal[2] > 0:
                new_table_plane = [-a, -b, -c, -d]
                normal = -normal
                a, b, c, d = new_table_plane

            self.table_plane = new_table_plane

            # Transform plane to world frame using pointcloud timestamp
            # CRITICAL: Use message timestamp, not now(), because arm is in motion
            camera_frame = self.current_scene_pointcloud.header.frame_id
            pointcloud_time = rclpy.time.Time.from_msg(self.current_scene_pointcloud.header.stamp)
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_footprint',
                    camera_frame,
                    pointcloud_time,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                # Get rotation matrix from transform
                q = transform.transform.rotation
                rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
                R = rot.as_matrix()

                # Get translation
                t = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])

                # Transform normal to world frame
                normal_world = R @ normal
                normal_world = normal_world / np.linalg.norm(normal_world)

                # Find a point on the plane in camera frame: p = -d * normal (for normalized normal)
                point_on_plane_cam = -d * normal

                # Transform point to world frame
                point_on_plane_world = R @ point_on_plane_cam + t

                # Compute d for world frame plane: d = -normal_world . point_on_plane_world
                d_world = -np.dot(normal_world, point_on_plane_world)

                # Ensure normal points DOWN in world frame (negative Z) for consistent gripper alignment
                # Gripper approaches downward, so normal should point down (negative Z)
                if normal_world[2] > 0:
                    normal_world = -normal_world
                    d_world = -d_world

                self.table_plane_world = [normal_world[0], normal_world[1], normal_world[2], d_world]
                self.table_normal_world = normal_world

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'Failed to transform plane to world frame: {e}')

        except Exception as e:
            self.get_logger().warn(f'Plane update failed: {e}')

    def transform_vector_camera_to_world(self, vector_camera, camera_frame='arm_camera_color_optical_frame', world_frame='base_footprint'):
        """
        Transform a vector from camera frame to world frame.

        Args:
            vector_camera: Vector in camera frame (3,)
            camera_frame: Source frame
            world_frame: Target frame

        Returns:
            Vector in world frame (3,) or None
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                world_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Extract rotation
            quat = transform.transform.rotation
            rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])

            # Rotate vector
            vector_world = rotation.apply(vector_camera)

            return vector_world

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def switch_servo_mode(self, mode):
        """Switch MoveIt Servo to specified command mode (fire-and-forget)."""
        if not self.servo_switch_client.service_is_ready():
            self.get_logger().warn('Servo switch service not ready, waiting...')
            if not self.servo_switch_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Servo switch service not available')
                return False

        request = ServoCommandType.Request()
        request.command_type = mode

        # Fire-and-forget with callback
        future = self.servo_switch_client.call_async(request)
        future.add_done_callback(self._servo_switch_callback)

        mode_names = {SERVO_JOINT_JOG: 'JOINT_JOG', SERVO_TWIST: 'TWIST', SERVO_POSE: 'POSE'}
        self.get_logger().info(f'Requested servo switch to {mode_names.get(mode, mode)} mode')
        return True

    def _servo_switch_callback(self, future):
        """Callback for servo mode switch result."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Servo mode switch confirmed')
            else:
                self.get_logger().error('Servo mode switch failed (service returned false)')
        except Exception as e:
            self.get_logger().error(f'Servo mode switch error: {e}')

    def request_segmentation_update(self):
        """Request a fresh segmentation for tracker re-initialization."""
        self._try_request_segmentation()

    def _try_request_segmentation(self):
        """Request a fresh segmentation for tracker re-initialization.

        Returns:
            True if request was actually sent, False if rate-limited or skipped.
        """
        if self.object_description is None:
            return False

        current_time = time.time()
        if current_time - self.last_segmentation_request_time < self.segmentation_request_interval:
            return False  # Rate limit requests

        self.get_logger().info('Requesting fresh segmentation for tracker update')
        self.segment_request_pub.publish(String(data=self.object_description))
        self.last_segmentation_request_time = current_time
        self.waiting_for_reinit_segmentation = True
        return True

    def start_grasp_callback(self, msg):
        """Start grasp sequence with object description."""
        if self.state not in ['IDLE', 'DONE', 'ERROR']:
            self.get_logger().warn(f'Grasp already in progress (state: {self.state})')
            return

        object_description = msg.data
        self.object_description = object_description  # Store for re-segmentation
        self.get_logger().info(f'Starting grasp for: {object_description}')

        # Reset state variables
        self.current_scene_pointcloud = None
        self.segmented_pointcloud = None
        self.segmentation_mask = None
        self.table_plane = None
        self.table_plane_world = None
        self.table_normal_world = None
        self.grasp_pose = None
        self.grasp_position_world = None
        self.target_z = None
        self.initial_object_z = None
        self.use_orientation_fallback = False
        self.tracker = SimpleMaskTracker()  # Fresh tracker
        self.motion_monitor.reset()
        self.servo_status_code = SERVO_NO_WARNING

        # Switch servo to TWIST mode for velocity commands
        if not self.switch_servo_mode(SERVO_TWIST):
            self.get_logger().error('Cannot start grasp: servo mode switch failed')
            return

        # Request segmentation
        self.segment_request_pub.publish(msg)

        # Transition to detection state with timeout tracking
        self.transition_to_state('DETECTING')

    def fit_plane_ransac(self, points, distance_threshold=0.01, num_iterations=1000):
        """
        Fit plane using RANSAC.

        Args:
            points: Nx3 numpy array of 3D points
            distance_threshold: Max distance for inliers (meters)
            num_iterations: RANSAC iterations

        Returns:
            plane_model: [a, b, c, d] where ax + by + cz + d = 0
            inliers: Boolean mask of inlier points
        """
        # Lazy import to speed up node startup (open3d takes ~5s to import)
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=num_iterations
        )

        return plane_model, inliers

    def estimate_grasp_pose_pca(self, points):
        """
        Estimate grasp pose using PCA for orientation.

        For flat objects, this gives us:
        - Position: centroid of the object
        - Orientation: principal axes from PCA

        Args:
            points: Nx3 numpy array of object points

        Returns:
            pose: PoseStamped with position and orientation
        """
        if len(points) < 10:
            self.get_logger().error('Not enough points for PCA')
            return None

        # Compute centroid
        centroid = np.mean(points, axis=0)

        # Center points
        centered = points - centroid

        # PCA
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov)

        # Sort by eigenvalues (largest first)
        idx = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]

        # For top-down grasp, we want Z-axis pointing down
        # Eigenvector with smallest eigenvalue is normal to the plane (Z-axis)
        z_axis = eigenvectors[:, 2]

        # Ensure Z points down (negative)
        if z_axis[2] > 0:
            z_axis = -z_axis

        # X-axis is the longest axis (direction along object)
        x_axis = eigenvectors[:, 0]

        # Y-axis from cross product
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Recompute X to ensure orthogonality
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        # Build rotation matrix
        R = np.column_stack([x_axis, y_axis, z_axis])

        # Convert to quaternion
        rot = Rotation.from_matrix(R)
        quat = rot.as_quat()  # [x, y, z, w]

        # Create pose
        pose = PoseStamped()
        pose.header.frame_id = 'arm_camera_color_optical_frame'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(centroid[0])
        pose.pose.position.y = float(centroid[1])
        pose.pose.position.z = float(centroid[2])

        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        return pose

    def extract_points_above_plane(self, points, plane_model, min_height=0.002):
        """
        Extract points above the plane (object points, not table).

        Args:
            points: Nx3 array
            plane_model: [a, b, c, d]
            min_height: Minimum height above plane (meters)

        Returns:
            object_points: Mx3 array of points above plane
        """
        a, b, c, d = plane_model
        normal = np.array([a, b, c])
        normal = normal / np.linalg.norm(normal)

        # Compute signed distance from plane
        distances = np.abs(points @ normal + d)

        # Keep points above threshold
        mask = distances > min_height

        return points[mask]

    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array."""
        points_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
            points_list.append([point[0], point[1], point[2]])

        return np.array(points_list)

    def publish_plane_marker(self, plane_model, frame_id, reference_point=None):
        """
        Publish a visualization marker for the detected plane.

        Args:
            plane_model: [a, b, c, d] plane equation ax + by + cz + d = 0
            frame_id: Frame ID for the marker (from pointcloud)
            reference_point: Optional reference point (e.g., object center) to position marker
        """
        a, b, c, d = plane_model
        normal = np.array([a, b, c])
        normal = normal / np.linalg.norm(normal)

        # Position the marker
        if reference_point is not None:
            # Project reference point onto the plane
            # Point on plane closest to reference_point
            t = -(a * reference_point[0] + b * reference_point[1] + c * reference_point[2] + d)
            center_point = reference_point + t * normal
            self.get_logger().info(f'Plane marker at projected point: {center_point} (from ref: {reference_point})')
        else:
            # Create a square plane marker centered at a point on the plane
            # Find a point on the plane (use z=0 plane intersection if possible)
            if abs(c) > 0.01:  # Plane is not vertical
                # Set x=0, y=0, solve for z: cz + d = 0 => z = -d/c
                center_point = np.array([0.0, 0.0, -d/c])
            elif abs(b) > 0.01:  # Plane is vertical, use y
                # Set x=0, z=0, solve for y: by + d = 0 => y = -d/b
                center_point = np.array([0.0, -d/b, 0.0])
            else:  # Use x
                # Set y=0, z=0, solve for x: ax + d = 0 => x = -d/a
                center_point = np.array([-d/a, 0.0, 0.0])
            self.get_logger().info(f'Plane marker at origin intersection: {center_point}')

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "table_plane"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position at center point
        marker.pose.position.x = float(center_point[0])
        marker.pose.position.y = float(center_point[1])
        marker.pose.position.z = float(center_point[2])

        # Orientation: align marker's Z-axis with plane normal
        # Default cube has Z pointing up, we want it aligned with plane normal
        z_axis = normal
        # Choose an arbitrary perpendicular vector for X
        if abs(normal[2]) < 0.9:
            x_axis = np.cross(normal, np.array([0, 0, 1]))
        else:
            x_axis = np.cross(normal, np.array([1, 0, 0]))
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        R = np.column_stack([x_axis, y_axis, z_axis])
        rot = Rotation.from_matrix(R)
        quat = rot.as_quat()  # [x, y, z, w]

        marker.pose.orientation.x = float(quat[0])
        marker.pose.orientation.y = float(quat[1])
        marker.pose.orientation.z = float(quat[2])
        marker.pose.orientation.w = float(quat[3])

        # Size: large flat plane (50cm x 50cm x 1mm)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.001

        # Color: semi-transparent blue
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.5

        marker.lifetime.sec = 0  # Persistent

        self.plane_marker_pub.publish(marker)
        self.get_logger().info(f'Published plane marker at {center_point}')

    def publish_orientation_markers(self):
        """
        Publish visualization markers for gripper orientation and table normal.
        Shows arrows indicating directions for debugging orientation alignment.
        """
        # Get gripper pose
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_footprint',
                'arm_tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            return

        # Gripper orientation marker (red arrow showing gripper Z-axis direction)
        gripper_marker = Marker()
        gripper_marker.header.frame_id = 'base_footprint'
        gripper_marker.header.stamp = self.get_clock().now().to_msg()
        gripper_marker.ns = "gripper_orientation"
        gripper_marker.id = 0
        gripper_marker.type = Marker.ARROW
        gripper_marker.action = Marker.ADD

        # Start at gripper position
        gripper_marker.points.append(Point(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            z=transform.transform.translation.z
        ))

        # End along gripper Z-axis (20cm arrow)
        quat = transform.transform.rotation
        gripper_rot = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        gripper_z = gripper_rot.as_matrix()[:, 2]

        end_point = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ]) + 0.2 * gripper_z

        gripper_marker.points.append(Point(
            x=float(end_point[0]),
            y=float(end_point[1]),
            z=float(end_point[2])
        ))

        gripper_marker.scale.x = 0.01  # Shaft diameter
        gripper_marker.scale.y = 0.02  # Head diameter
        gripper_marker.scale.z = 0.03  # Head length

        gripper_marker.color.r = 1.0
        gripper_marker.color.g = 0.0
        gripper_marker.color.b = 0.0
        gripper_marker.color.a = 1.0

        self.gripper_orientation_marker_pub.publish(gripper_marker)

        # Table normal marker (blue arrow showing target direction - points DOWN)
        if self.table_normal_world is not None:
            table_marker = Marker()
            table_marker.header.frame_id = 'base_footprint'
            table_marker.header.stamp = self.get_clock().now().to_msg()
            table_marker.ns = "table_normal"
            table_marker.id = 0
            table_marker.type = Marker.ARROW
            table_marker.action = Marker.ADD

            # Start at gripper position
            table_marker.points.append(Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z
            ))

            # table_normal_world points DOWN (towards table), which is the target direction
            target_direction = self.table_normal_world

            end_point = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]) + 0.2 * target_direction

            table_marker.points.append(Point(
                x=float(end_point[0]),
                y=float(end_point[1]),
                z=float(end_point[2])
            ))

            table_marker.scale.x = 0.01  # Shaft diameter
            table_marker.scale.y = 0.02  # Head diameter
            table_marker.scale.z = 0.03  # Head length

            table_marker.color.r = 0.0
            table_marker.color.g = 0.5
            table_marker.color.b = 1.0
            table_marker.color.a = 1.0

            self.table_normal_marker_pub.publish(table_marker)

    def transition_to_state(self, new_state):
        """Transition to a new state with proper initialization."""
        old_state = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self.stall_retry_count = 0
        self.motion_monitor.reset()

        # Reset idle flag when leaving IDLE state
        if old_state == 'IDLE' and new_state != 'IDLE':
            self._idle_stopped = False

        # Reset yaw segmentation flag and target when entering ORIENT
        if new_state == 'ORIENT':
            self._yaw_segmentation_requested = False
            self._target_yaw_from_mask = None  # Will be set when segmentation arrives
            self._tracker_angle_at_mask = None  # Reference tracker angle when mask was taken
            self._gripper_yaw_at_mask = None  # Gripper yaw from TF when mask was taken (for rotation tracking)

        self.get_logger().info(f'State transition: {old_state} -> {new_state}')
        self.status_pub.publish(String(data=f'{new_state}'))

    def check_timeout(self):
        """Check if current state has timed out."""
        if self.state_start_time is None:
            return False
        elapsed = time.time() - self.state_start_time
        return elapsed > self.state_timeout

    def handle_stall(self):
        """Handle arm stall condition with retry logic."""
        self.stall_retry_count += 1
        self.get_logger().warn(
            f'Stall detected in {self.state} (retry {self.stall_retry_count}/{self.max_stall_retries})'
        )

        if self.stall_retry_count >= self.max_stall_retries:
            self.get_logger().error('Max stall retries exceeded, aborting grasp')
            self.transition_to_state('ERROR')
            return False

        # Reset motion monitor for retry
        self.motion_monitor.reset()
        return True

    def handle_servo_halt(self):
        """Handle servo halt due to collision/singularity."""
        self.get_logger().error(
            f'Servo halted (code={self.servo_status_code}): {self.servo_status_msg}'
        )
        self.send_zero_velocity()
        self.transition_to_state('ERROR')

    def control_loop(self):
        """Main control loop for visual servoing with feedback monitoring."""

        # Timing diagnostics
        loop_start = time.time()
        if not hasattr(self, '_last_loop_time'):
            self._last_loop_time = loop_start
            self._loop_count = 0

        self._loop_count += 1
        if self._loop_count % 30 == 0:
            actual_rate = 30.0 / (loop_start - self._last_loop_time)
            self.get_logger().info(f'Control loop actual rate: {actual_rate:.1f} Hz (target: {self.servo_rate} Hz)')
            self._last_loop_time = loop_start

        # Global checks for all active states
        if self.state not in ['IDLE', 'DONE', 'ERROR']:
            # Check for servo halt
            if self.is_servo_halted():
                self.handle_servo_halt()
                return

            # Check for timeout
            if self.check_timeout():
                self.get_logger().error(f'Timeout in state {self.state}')
                self.send_zero_velocity()
                self.transition_to_state('ERROR')
                return

        if self.state == 'IDLE':
            # Stop motion (only send once, not every loop)
            if not hasattr(self, '_idle_stopped') or not self._idle_stopped:
                self.send_zero_velocity()
                self._idle_stopped = True

        elif self.state == 'DETECTING':
            # Wait for segmentation, full scene pointcloud, and segmented object pointcloud
            # Debug: Log what data we're waiting for (only once per second)
            if not hasattr(self, '_detecting_debug_count'):
                self._detecting_debug_count = 0
            self._detecting_debug_count += 1

            if self._detecting_debug_count % 30 == 1:
                self.get_logger().info(
                    f'DETECTING: scene_pc={self.current_scene_pointcloud is not None}, '
                    f'seg_pc={self.segmented_pointcloud is not None}, '
                    f'mask={self.segmentation_mask is not None}, '
                    f'rgb={self.current_rgb is not None}'
                )

            if (self.current_scene_pointcloud is not None and
                self.segmented_pointcloud is not None and
                self.segmentation_mask is not None and
                self.current_rgb is not None):

                # Convert segmented object pointcloud to numpy first
                object_points = self.pointcloud2_to_array(self.segmented_pointcloud)

                self.get_logger().info(f'Segmented object has {len(object_points)} points')

                if len(object_points) < 50:
                    self.get_logger().error('Not enough object points in segmented cloud')
                    self.transition_to_state('ERROR')
                    return

                # Get bounding box of object to filter scene points
                obj_min = np.min(object_points, axis=0)
                obj_max = np.max(object_points, axis=0)
                obj_center = np.mean(object_points, axis=0)

                self.get_logger().info(f'Object bounds: min={obj_min}, max={obj_max}, center={obj_center}')

                # Convert full scene pointcloud to numpy for plane fitting
                scene_points = self.pointcloud2_to_array(self.current_scene_pointcloud)

                if len(scene_points) < 1000:
                    self.get_logger().warn(f'Not enough points in scene cloud: {len(scene_points)}')
                    return

                self.get_logger().info(f'Full scene has {len(scene_points)} points')

                # Filter scene points to region near the object (within expanded bounding box)
                # Expand horizontally (XY) to capture table surface, but limit vertically (Z)
                margin_xy = 0.15  # 15cm margin in X and Y
                margin_z_below = 0.05  # 5cm below object (where table should be)
                margin_z_above = 0.02  # 2cm above object (to exclude overhead surfaces)

                # Filter points in XY plane around object
                xy_mask = (
                    (scene_points[:, 0] > obj_min[0] - margin_xy) &
                    (scene_points[:, 0] < obj_max[0] + margin_xy) &
                    (scene_points[:, 1] > obj_min[1] - margin_xy) &
                    (scene_points[:, 1] < obj_max[1] + margin_xy)
                )

                # Filter points BELOW object in Z (table should be below object)
                # In camera optical frame, Z is forward (into scene), so "below" depends on orientation
                # Typically for top-down grasping, we want points with Z close to object Z
                z_mask = (
                    (scene_points[:, 2] > obj_min[2] - margin_z_below) &
                    (scene_points[:, 2] < obj_max[2] + margin_z_above)
                )

                filtered_scene = scene_points[xy_mask & z_mask]

                self.get_logger().info(f'Filtered scene near object: {len(filtered_scene)} points')

                if len(filtered_scene) < 100:
                    self.get_logger().warn('Not enough filtered scene points for plane fitting, using full scene')
                    filtered_scene = scene_points

                # Fit table plane using filtered scene pointcloud
                self.table_plane, inliers = self.fit_plane_ransac(filtered_scene,
                                                                   distance_threshold=0.01,
                                                                   num_iterations=1000)

                # Validate plane: normal should point generally upward (negative Z in camera frame)
                # Camera optical frame: Z+ is forward/down, so table normal should have negative Z
                a, b, c, d = self.table_plane
                normal = np.array([a, b, c])
                normal = normal / np.linalg.norm(normal)

                # If normal points away from camera (positive Z), flip it
                if normal[2] > 0:
                    self.table_plane = [-a, -b, -c, -d]
                    normal = -normal
                    self.get_logger().info('Flipped plane normal to point toward camera')

                self.get_logger().info(
                    f'Table plane: {self.table_plane}, normal={normal}, '
                    f'inliers: {np.sum(inliers)}/{len(filtered_scene)}'
                )

                # Transform table plane to world frame for distance computation
                camera_frame = self.current_scene_pointcloud.header.frame_id
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_footprint',
                        camera_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )

                    # Get rotation matrix from transform
                    q = transform.transform.rotation
                    rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
                    R = rot.as_matrix()

                    # Get translation
                    t = np.array([
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z
                    ])

                    # Transform normal to world frame
                    normal_world = R @ normal
                    normal_world = normal_world / np.linalg.norm(normal_world)

                    # Find a point on the plane in camera frame: p = -d * normal (for normalized normal)
                    a, b, c, d = self.table_plane
                    point_on_plane_cam = -d * normal

                    # Transform point to world frame
                    point_on_plane_world = R @ point_on_plane_cam + t

                    # Compute d for world frame plane: d = -normal_world . point_on_plane_world
                    d_world = -np.dot(normal_world, point_on_plane_world)

                    # Ensure normal points DOWN in world frame (negative Z)
                    if normal_world[2] > 0:
                        normal_world = -normal_world
                        d_world = -d_world

                    self.table_plane_world = [normal_world[0], normal_world[1], normal_world[2], d_world]
                    self.table_normal_world = normal_world

                    self.get_logger().info(f'Table plane in world frame: {self.table_plane_world}')
                    self.get_logger().info(f'Table normal in world frame (pointing down): {self.table_normal_world}')

                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    self.get_logger().error(f'Failed to transform table plane to world frame: {e}')
                    self.transition_to_state('ERROR')
                    return

                # Check current gripper orientation
                gripper_rot = self.get_gripper_orientation()
                if gripper_rot is not None:
                    angular_error, _ = self.compute_orientation_error(gripper_rot, self.table_normal_world)
                    if angular_error is not None:
                        self.get_logger().info(
                            f'Initial gripper orientation error: {np.rad2deg(angular_error):.1f} degrees'
                        )

                # Publish plane marker in world frame for visualization
                # Transform obj_center to world frame for reference point
                obj_center_world = R @ obj_center + t
                self.publish_plane_marker(self.table_plane_world, 'base_footprint', obj_center_world)

                # Estimate object thickness (distance from table plane)
                a, b, c, d = self.table_plane
                normal = np.array([a, b, c])
                distances = np.abs(object_points @ normal + d)
                self.object_thickness = np.percentile(distances, 95)
                self.get_logger().info(f'Object thickness: {self.object_thickness*1000:.1f}mm')

                # Estimate grasp pose using PCA on segmented object points
                self.grasp_pose = self.estimate_grasp_pose_pca(object_points)

                if self.grasp_pose is None:
                    self.get_logger().error('Failed to estimate grasp pose')
                    self.transition_to_state('ERROR')
                    return

                # Transform grasp position to world frame for 3D position tracking
                try:
                    # Use tf2 to transform the pose from camera frame to world frame
                    transform = self.tf_buffer.lookup_transform(
                        'base_footprint',  # target frame
                        self.grasp_pose.header.frame_id,  # source frame (camera optical frame)
                        rclpy.time.Time(),  # latest
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )

                    # Transform the pose using tf2_geometry_msgs
                    import tf2_geometry_msgs
                    grasp_pose_world = tf2_geometry_msgs.do_transform_pose_stamped(
                        self.grasp_pose, transform
                    )

                    # Store the position in world frame
                    self.grasp_position_world = np.array([
                        grasp_pose_world.pose.position.x,
                        grasp_pose_world.pose.position.y,
                        grasp_pose_world.pose.position.z
                    ])

                    self.get_logger().info(
                        f'Target grasp position in world frame: '
                        f'[{self.grasp_position_world[0]:.3f}, '
                        f'{self.grasp_position_world[1]:.3f}, '
                        f'{self.grasp_position_world[2]:.3f}]'
                    )

                except Exception as e:
                    self.get_logger().error(f'Failed to transform grasp pose to world frame: {e}')
                    self.transition_to_state('ERROR')
                    return

                # Store initial object position for height tracking
                self.initial_object_z = self.grasp_pose.pose.position.z
                # Target Z is at object center minus clearance (approaching in camera frame)
                self.target_z = self.initial_object_z + self.grasp_clearance

                self.get_logger().info(
                    f'Grasp pose (camera frame): pos=[{self.grasp_pose.pose.position.x:.3f}, '
                    f'{self.grasp_pose.pose.position.y:.3f}, {self.grasp_pose.pose.position.z:.3f}]'
                )
                self.get_logger().info(f'Target Z (camera frame): {self.target_z:.3f}m')

                # Initialize tracker with synchronized RGB image that matches mask
                if self.segmentation_rgb is None:
                    self.get_logger().error('No synchronized RGB image available for tracker initialization')
                    self.transition_to_state('ERROR')
                    return

                if not self.tracker.initialize(self.segmentation_rgb, self.segmentation_mask, self.get_logger()):
                    self.get_logger().error('Failed to initialize tracker')
                    self.transition_to_state('ERROR')
                    return

                self.send_gripper_command(False)  # False = open

                # Transition to approach (XYZ only, no rotation)
                self.transition_to_state('APPROACH')
                self.get_logger().info('Starting approach phase (XYZ only)')

        elif self.state == 'APPROACH':
            # Visual servoing approach to pre_grasp_height above the table plane
            self.visual_servo_to_plane(
                target_distance=self.pre_grasp_height,
                z_speed=self.approach_speed,
                min_confidence=0.2,
                state_name='APPROACH',
                next_state='ORIENT'
            )

        elif self.state == 'ORIENT':
            # Orientation correction with XY compensation - limited Z motion
            # Rotate gripper to align with table normal while keeping object centered
            if self.current_rgb is None:
                return

            # Disable tracker frame skipping in ORIENT for stability
            self.tracker.skip_frames = False

            # Track object
            center, angle, confidence = self.tracker.update(self.current_rgb)

            # Check FOV boundaries using bbox CENTER (not corners)
            # Large objects may have corners outside FOV but center still trackable
            # - Warning zone: center approaching edge (within 20% of image dimension)
            # - Danger zone: center very close to edge (within 10% of image dimension)
            if center is not None and hasattr(self.tracker, 'template_bbox'):
                img_height, img_width = self.current_rgb.shape[:2]
                x, y, w, h = self.tracker.template_bbox

                # Use bbox center for FOV check, not corners
                bbox_center_x = x + w / 2
                bbox_center_y = y + h / 2

                # Distance from center to nearest edge (as fraction of image size)
                dist_to_left = bbox_center_x / img_width
                dist_to_right = (img_width - bbox_center_x) / img_width
                dist_to_top = bbox_center_y / img_height
                dist_to_bottom = (img_height - bbox_center_y) / img_height

                min_dist_fraction = min(dist_to_left, dist_to_right, dist_to_top, dist_to_bottom)

                # Danger if center is within 10% of edge, warning if within 20%
                in_danger_zone = min_dist_fraction < 0.10
                in_warning_zone = 0.10 <= min_dist_fraction < 0.20
                min_distance_to_edge = min_dist_fraction * min(img_width, img_height)  # For logging

                if in_danger_zone:
                    # Object at/beyond FOV edge - actively re-center it, no orientation
                    # Rate-limit this warning to once every 2 seconds
                    if not hasattr(self, '_last_orient_danger_warn') or time.time() - self._last_orient_danger_warn > 2.0:
                        self.get_logger().warn(
                            f'ORIENT: DANGER zone - bbox corner only {min_distance_to_edge:.0f}px from edge - '
                            f'pausing orientation, actively re-centering object'
                        )
                        self._last_orient_danger_warn = time.time()
                    # Skip to re-centering logic below (no return, let XY correction run)
                    # Don't request re-segmentation (would get partial object)

                elif in_warning_zone:
                    # Object approaching edge - pause orientation and re-center
                    # Rate-limit this warning
                    if not hasattr(self, '_last_orient_warning_warn') or time.time() - self._last_orient_warning_warn > 2.0:
                        self.get_logger().warn(
                            f'ORIENT: WARNING zone - bbox corner {min_distance_to_edge:.0f}px from edge - '
                            f'pausing orientation to re-center'
                        )
                        self._last_orient_warning_warn = time.time()
                    # Continue to re-centering logic (don't return)

                elif confidence < self.tracker_reinit_confidence_threshold and not self.waiting_for_reinit_segmentation:
                    # In safe zone but low confidence - re-segment without stopping
                    # Only log if we actually send a request (rate-limited internally)
                    if self._try_request_segmentation():
                        self.get_logger().warn(
                            f'[ORIENT] Tracker conf={confidence:.2f} below threshold, requested re-segmentation'
                        )

            # Publish tracker debug visualization
            if self.tracker.initialized:
                self.publish_tracker_debug(confidence)

            if center is None or confidence < 0.2:
                self.get_logger().warn(f'Tracking lost during orient (confidence: {confidence:.2f})')
                self.send_zero_velocity()
                return

            # Compute XY error for compensation
            image_center = np.array([self.current_rgb.shape[1] / 2,
                                    self.current_rgb.shape[0] / 2])
            error_pixels = center - image_center

            # FOV safety check
            img_height, img_width = self.current_rgb.shape[:2]
            fov_margin = 100
            fov_edge_margin = 50

            near_boundary = (center[0] < fov_margin or center[0] > img_width - fov_margin or
                           center[1] < fov_margin or center[1] > img_height - fov_margin)
            at_boundary = (center[0] < fov_edge_margin or center[0] > img_width - fov_edge_margin or
                         center[1] < fov_edge_margin or center[1] > img_height - fov_edge_margin)

            # Get depth for pixel-to-meter conversion
            current_depth = self._get_depth_at_point(center)
            if current_depth is None or current_depth < 0.1:
                current_depth = 0.3

            # Use camera intrinsics if available, fallback to estimate
            focal_length = self.fx if self.fx is not None else 500.0
            pixel_to_meter = current_depth / focal_length

            error_cam_x = error_pixels[0] * pixel_to_meter
            error_cam_y = error_pixels[1] * pixel_to_meter
            xy_error_norm = np.sqrt(error_cam_x**2 + error_cam_y**2)

            # Check gripper orientation
            gripper_rot = self.get_gripper_orientation()
            angular_error, rotation_axis_world = None, None

            if gripper_rot is not None and self.table_normal_world is not None:
                angular_error, rotation_axis_world = self.compute_orientation_error(
                    gripper_rot, self.table_normal_world
                )

            # XY compensation with PD control to reduce oscillation at low control rates (~6Hz)
            # Lower gains to prevent overshoots at low control rate
            if in_danger_zone if (center is not None and hasattr(self.tracker, 'template_bbox')) else at_boundary:
                kp_xy = 0.8  # Re-centering in danger zone
                kd_xy = 0.3
            elif in_warning_zone if (center is not None and hasattr(self.tracker, 'template_bbox')) else near_boundary:
                kp_xy = 0.6  # Re-centering in warning zone
                kd_xy = 0.25
            else:
                kp_xy = 0.5  # Normal gain when safe
                kd_xy = 0.2

            # Compute derivative term
            error_xy = np.array([error_cam_x, error_cam_y])
            if not hasattr(self, '_orient_prev_error_xy'):
                self._orient_prev_error_xy = error_xy
                self._orient_prev_time = time.time()

            dt = time.time() - self._orient_prev_time
            if dt > 0.001:  # Avoid division by zero
                d_error_xy = (error_xy - self._orient_prev_error_xy) / dt
            else:
                d_error_xy = np.zeros(2)

            self._orient_prev_error_xy = error_xy.copy()
            self._orient_prev_time = time.time()

            # PD control
            v_tool_x = kp_xy * error_cam_x + kd_xy * d_error_xy[0]
            v_tool_y = kp_xy * error_cam_y + kd_xy * d_error_xy[1]

            # Limit max velocity
            max_xy_vel = 0.04
            v_tool_x = np.clip(v_tool_x, -max_xy_vel, max_xy_vel)
            v_tool_y = np.clip(v_tool_y, -max_xy_vel, max_xy_vel)

            # Z control: maintain pre_grasp_height during orientation
            # Use fingertip distance for safety (same as APPROACH/DESCENDING)
            fingertip_distance = self.get_fingertip_distance_to_plane()
            if fingertip_distance is None:
                # Fallback if plane/TF not available
                fingertip_offset = self.get_fingertip_offset_toward_plane()
                if fingertip_offset is not None:
                    fingertip_distance = current_depth - fingertip_offset
                else:
                    fingertip_distance = current_depth - self.fingertip_offset[2]

            z_error = fingertip_distance - self.pre_grasp_height
            # Small proportional control to maintain height
            kp_z = 0.3
            v_tool_z = np.clip(kp_z * z_error, -0.02, 0.02)

            # Orientation correction - only skip in DANGER zone (bbox at edge)
            # WARNING zone is OK - bbox will naturally move during yaw rotation
            omega_tool = np.zeros(3)

            # Only skip orientation in danger zone (not warning zone)
            skip_orientation = in_danger_zone if (center is not None and hasattr(self.tracker, 'template_bbox')) else False

            # Yaw control: rotate gripper so object's long axis is VERTICAL in camera
            # Gripper fingers close horizontally, so we want to grasp across the long axis
            #
            # Strategy: Use mask-derived angle to compute initial error, then track rotation
            # using GRIPPER TF (not tracker angle, which is noisy and causes oscillation).
            # yaw_error = initial_mask_error - gripper_rotation_since_mask
            #
            # The gripper yaw from TF is stable and accurate, unlike tracker angle which
            # suffers from 180-degree ambiguity issues at near-vertical orientations.
            current_gripper_yaw = self.get_gripper_yaw()

            if getattr(self, '_target_yaw_from_mask', None) is not None and current_gripper_yaw is not None:
                # Store gripper yaw at mask time for reference (first time only)
                if getattr(self, '_gripper_yaw_at_mask', None) is None:
                    self._gripper_yaw_at_mask = current_gripper_yaw
                    self._tracker_angle_at_mask = angle  # Still store for logging
                    self.get_logger().info(
                        f'Yaw reference set: mask={np.rad2deg(self._target_yaw_from_mask):.1f}deg, '
                        f'gripper_yaw={np.rad2deg(current_gripper_yaw):.1f}deg, '
                        f'tracker={np.rad2deg(angle):.1f}deg'
                    )

                # Compute how much gripper has actually rotated since mask (from TF, accurate!)
                gripper_rotation_since_mask = current_gripper_yaw - self._gripper_yaw_at_mask

                # Initial error from mask (target is pi/2 = vertical)
                initial_error = self._target_yaw_from_mask - np.pi / 2

                # Current error = initial error minus rotation applied
                # As gripper rotates, the object appears to rotate opposite direction in image
                yaw_error = initial_error + gripper_rotation_since_mask
            else:
                # Fallback to tracker angle directly (less reliable, may oscillate)
                yaw_error = angle - np.pi / 2

            # Normalize yaw error to [-pi/2, pi/2] - we don't care about 180 degree ambiguity
            while yaw_error > np.pi / 2:
                yaw_error -= np.pi
            while yaw_error < -np.pi / 2:
                yaw_error += np.pi

            # Check alignment status (always check, even when not controlling)
            yaw_aligned = abs(yaw_error) < 0.1  # ~5.7 degrees tolerance
            pitch_roll_aligned = angular_error is None or angular_error <= self.orientation_tolerance

            if not skip_orientation:
                # Check timeout
                elapsed_time = time.time() - self.state_start_time
                if elapsed_time > self.max_orientation_time:
                    self.get_logger().warn(
                        f'Orientation timeout ({elapsed_time:.1f}s), using fallback'
                    )
                    self.use_orientation_fallback = True
                    pitch_roll_aligned = True
                    yaw_aligned = True
                else:
                    # 1. Pitch/Roll control: align gripper Z with table normal (come from above)
                    if not pitch_roll_aligned and angular_error is not None and rotation_axis_world is not None:
                        # Higher gains - pitch/roll was way too slow
                        kp_rot = 0.15
                        min_omega = 0.04  # Minimum speed to avoid stalling
                        max_omega = 0.15

                        omega_magnitude = kp_rot * angular_error
                        # Apply minimum speed when not aligned (avoid stalling near target)
                        if omega_magnitude > 0.001:
                            omega_magnitude = max(omega_magnitude, min_omega)
                        omega_magnitude = np.clip(omega_magnitude, 0.0, max_omega)
                        omega_world = omega_magnitude * rotation_axis_world

                        # Transform to tool frame
                        R_world_to_tool = gripper_rot.as_matrix().T
                        omega_pitch_roll = R_world_to_tool @ omega_world
                        omega_tool[0] = omega_pitch_roll[0]
                        omega_tool[1] = omega_pitch_roll[1]

                    # 2. Yaw control: align gripper fingers with object (only if pitch/roll mostly done)
                    if pitch_roll_aligned and not yaw_aligned:
                        # Request fresh segmentation once when starting yaw alignment
                        # Now that we're looking straight down, the object proportions are correct
                        # This helps the tracker get a proper bbox for yaw estimation
                        if not getattr(self, '_yaw_segmentation_requested', False):
                            self.get_logger().info(
                                'ORIENT: Pitch/roll aligned, requesting fresh segmentation for yaw alignment'
                            )
                            self._try_request_segmentation()
                            self._yaw_segmentation_requested = True
                            self.waiting_for_reinit_segmentation = True
                            # Wait for new segmentation before starting yaw control
                            self.send_zero_velocity()
                            return

                        # Wait for new segmentation to arrive and reinit tracker
                        if self.waiting_for_reinit_segmentation:
                            self.send_zero_velocity()
                            return

                        # Conservative yaw gains for low control rate
                        kp_yaw = 0.4
                        kd_yaw = 0.1
                        min_omega_yaw = 0.03  # Minimum speed to avoid stalling
                        max_omega_yaw = 0.12

                        # Derivative term for yaw
                        if not hasattr(self, '_orient_prev_yaw_error'):
                            self._orient_prev_yaw_error = yaw_error
                            self._orient_prev_yaw_time = time.time()

                        dt_yaw = time.time() - self._orient_prev_yaw_time
                        if dt_yaw > 0.001:
                            d_yaw_error = (yaw_error - self._orient_prev_yaw_error) / dt_yaw
                        else:
                            d_yaw_error = 0.0

                        self._orient_prev_yaw_error = yaw_error
                        self._orient_prev_yaw_time = time.time()

                        # PD control for yaw - rotate around tool Z axis
                        omega_z = kp_yaw * yaw_error + kd_yaw * d_yaw_error
                        # Apply minimum speed when not aligned (avoid stalling near target)
                        if abs(omega_z) > 0.001:
                            omega_z = np.sign(omega_z) * max(abs(omega_z), min_omega_yaw)
                        omega_z = np.clip(omega_z, -max_omega_yaw, max_omega_yaw)
                        omega_tool[2] = omega_z

            orientation_aligned = pitch_roll_aligned and yaw_aligned

            # Check if orientation is complete
            if orientation_aligned and xy_error_norm < self.xy_tolerance * 2:
                if self.use_orientation_fallback:
                    self.get_logger().info(
                        f'ORIENT complete with fallback (xy_err={xy_error_norm:.4f}m, '
                        f'orient_err={np.rad2deg(angular_error) if angular_error else 0:.1f}deg)'
                    )
                else:
                    self.get_logger().info(
                        f'ORIENT complete (xy_err={xy_error_norm:.4f}m, '
                        f'orient_err={np.rad2deg(angular_error) if angular_error else 0:.1f}deg)'
                    )
                self.transition_to_state('DESCENDING')
                self._descent_count = 0
                return

            # Debug logging
            if not hasattr(self, '_orient_count'):
                self._orient_count = 0
            self._orient_count += 1

            if self._orient_count % 30 == 1:
                pitch_roll_str = f'{np.rad2deg(angular_error):.1f}deg' if angular_error else 'N/A'
                gripper_yaw_str = f'{np.rad2deg(current_gripper_yaw):.1f}deg' if current_gripper_yaw else 'N/A'
                self.get_logger().info(
                    f'ORIENT: xy_err={xy_error_norm:.4f}m, pitch_roll={pitch_roll_str}, '
                    f'yaw_err={np.rad2deg(yaw_error):.1f}deg, gripper_yaw={gripper_yaw_str}, conf={confidence:.2f}'
                )
                self.publish_orientation_markers()

            # Check for stall
            if self.motion_monitor.is_stalled():
                if not self.handle_stall():
                    return

            # Send velocity command
            velocity = np.array([v_tool_x, v_tool_y, v_tool_z,
                               omega_tool[0], omega_tool[1], omega_tool[2]])
            self.send_velocity_command(velocity)
            self.motion_monitor.set_commanded_velocity(velocity)

        elif self.state == 'DESCENDING':
            # Visual servoing descent to grasp_clearance above the table plane
            self.visual_servo_to_plane(
                target_distance=self.grasp_clearance,
                z_speed=self.descent_speed,
                min_confidence=0.3,
                state_name='DESCENDING',
                next_state='GRASPING'
            )

        elif self.state == 'GRASPING':
            # Close gripper - only send command once
            self.send_zero_velocity()

            if not hasattr(self, '_grasp_start_time'):
                self._grasp_start_time = time.time()
                self.send_gripper_command(True)  # True = close
                self.get_logger().info('Closing gripper...')
                return

            # Wait for gripper to close, then lift
            if time.time() - self._grasp_start_time > 1.0:  # 1 second to close
                del self._grasp_start_time
                self.transition_to_state('LIFTING')
                self.get_logger().info('Gripper closed, lifting object')

        elif self.state == 'LIFTING':
            # Lift object straight up
            if not hasattr(self, '_lift_count'):
                self._lift_count = 0
                self._lift_start_time = time.time()

            self._lift_count += 1

            # Lift for 2 seconds at lift speed
            lift_duration = 2.0
            lift_speed = 0.03  # 3 cm/s upward

            elapsed = time.time() - self._lift_start_time

            if elapsed < lift_duration:
                # Check for stall during lift
                if self.motion_monitor.is_stalled():
                    self.get_logger().warn('Stall during lift - object may be stuck')
                    # Continue anyway, might just be at joint limit

                # Lift command (negative Z in camera frame = up)
                velocity = np.array([0.0, 0.0, -lift_speed, 0.0, 0.0, 0.0])
                self.send_velocity_command(velocity)
                self.motion_monitor.set_commanded_velocity(velocity)

                if self._lift_count % 30 == 1:
                    self.get_logger().info(f'Lifting: {elapsed:.1f}s / {lift_duration:.1f}s')
            else:
                # Done lifting
                del self._lift_count
                del self._lift_start_time
                self.send_zero_velocity()
                self.transition_to_state('DONE')
                self.get_logger().info('Grasp complete!')

        elif self.state == 'DONE':
            self.send_zero_velocity()
            # Stay in DONE state until reset

        elif self.state == 'ERROR':
            self.send_zero_velocity()
            # Log error state periodically
            if not hasattr(self, '_error_log_time') or time.time() - self._error_log_time > 5.0:
                self._error_log_time = time.time()
                self.get_logger().error(
                    f'Grasp failed. Send new /start_grasp message to retry.'
                )

    def _get_depth_at_point(self, pixel_coords):
        """Get depth value at a pixel location with neighborhood averaging."""
        if self.current_depth is None:
            return None

        x, y = int(pixel_coords[0]), int(pixel_coords[1])
        h, w = self.current_depth.shape

        # Clamp to image bounds
        x = max(0, min(w - 1, x))
        y = max(0, min(h - 1, y))

        # Sample neighborhood for robustness
        neighborhood = 5
        x_min = max(0, x - neighborhood)
        x_max = min(w, x + neighborhood + 1)
        y_min = max(0, y - neighborhood)
        y_max = min(h, y + neighborhood + 1)

        depth_patch = self.current_depth[y_min:y_max, x_min:x_max]

        # Filter out invalid depths
        valid_depths = depth_patch[(depth_patch > 0.05) & (depth_patch < 3.0)]

        if len(valid_depths) == 0:
            return None

        # Return median for robustness
        return float(np.median(valid_depths))

    def _get_min_depth_from_camera(self):
        """
        Get minimum depth from entire depth image for collision avoidance.
        This measures the closest obstacle in front of the camera.
        Optimized for speed - uses min instead of percentile.

        Returns:
            float: Minimum depth in meters, or None if unavailable
        """
        if self.current_depth is None:
            return None

        # Filter out invalid depths (too close/far or zero)
        valid_depths = self.current_depth[(self.current_depth > 0.05) & (self.current_depth < 3.0)]

        if len(valid_depths) == 0:
            return None

        # Use min for speed (percentile is slow on large arrays)
        # Apply small safety margin by adding 1cm to account for noise
        return float(np.min(valid_depths)) + 0.01

    def visual_servo_to_plane(self, target_distance, z_speed, min_confidence, state_name, next_state):
        """
        Visual servoing to maintain XY centering while descending to target distance from plane.

        Uses fingertip distance to table plane (not camera distance) for collision safety.
        The camera-to-fingertip offset is projected onto the table normal based on
        current gripper orientation.

        Args:
            target_distance: Target distance from fingertips to table plane (meters)
            z_speed: Maximum Z approach speed (m/s)
            min_confidence: Minimum tracking confidence to continue
            state_name: Current state name for logging
            next_state: State to transition to when target reached

        Returns:
            True if should continue, False if target reached or error
        """
        if self.current_rgb is None:
            return True

        # Update plane estimation (rate-limited internally)
        # self._update_plane_and_grasp_pose()

        # Track object for XY centering
        center, angle, confidence = self.tracker.update(self.current_rgb)

        # Check FOV boundaries and tracker confidence
        # Check if any bbox corner is too close to or outside image edges
        # - Safe zone: all corners >50px from edges (normal operation)
        # - Warning zone: any corner 20-50px from edges (proactive re-segment + STOP)
        # - Danger zone: any corner <20px from edges or outside (STOP, no re-segment)
        if center is not None and hasattr(self.tracker, 'template_bbox'):
            img_height, img_width = self.current_rgb.shape[:2]
            x, y, w, h = self.tracker.template_bbox

            warning_margin = 50
            danger_margin = 20

            self.get_logger().debug(
                f'{state_name}: Checking FOV for bbox=({x},{y})+({w}x{h}), img=({img_width}x{img_height})'
            )

            # Check all 4 corners of bbox
            corners = [
                (x, y),              # top-left
                (x + w, y),          # top-right
                (x, y + h),          # bottom-left
                (x + w, y + h)       # bottom-right
            ]

            # Calculate minimum distance from any corner to image edges
            # If any corner is outside image bounds, immediately DANGER zone
            min_distance_to_edge = float('inf')
            any_corner_outside = False
            for i, (cx, cy) in enumerate(corners):
                # Check if corner is outside image
                if cx < 0 or cy < 0 or cx >= img_width or cy >= img_height:
                    any_corner_outside = True
                    self.get_logger().debug(
                        f'Corner {i} at ({cx},{cy}) is outside image bounds ({img_width}x{img_height})'
                    )
                    min_distance_to_edge = -1  # Negative indicates outside
                    continue

                dist_to_left = cx
                dist_to_top = cy
                dist_to_right = img_width - cx
                dist_to_bottom = img_height - cy

                corner_min_dist = min(dist_to_left, dist_to_top, dist_to_right, dist_to_bottom)
                self.get_logger().debug(
                    f'  Corner {i} ({cx},{cy}): L={dist_to_left:.0f} T={dist_to_top:.0f} '
                    f'R={dist_to_right:.0f} B={dist_to_bottom:.0f} -> min={corner_min_dist:.0f}'
                )
                min_distance_to_edge = min(min_distance_to_edge, corner_min_dist)

            in_danger_zone = any_corner_outside or min_distance_to_edge < danger_margin
            in_warning_zone = (not any_corner_outside) and (danger_margin <= min_distance_to_edge < warning_margin)

            # Store FOV state for Z speed control later (don't stop XY - we need to center the object)
            self._fov_in_danger_zone = in_danger_zone
            self._fov_in_warning_zone = in_warning_zone
            self._fov_min_distance = min_distance_to_edge

            # Don't request re-segmentation if:
            # - bbox is too large (>60% of image area) - segmentation won't help
            # - we're in DESCENDING state - object filling FOV is expected
            bbox_area = w * h
            image_area = img_width * img_height
            bbox_too_large = bbox_area > 0.6 * image_area
            skip_resegmentation = bbox_too_large or state_name == 'DESCENDING'

            if in_warning_zone and not self.waiting_for_reinit_segmentation and not skip_resegmentation:
                # Object approaching edge - request re-segment but don't stop XY motion
                # Rate-limit warnings
                if not hasattr(self, '_last_servo_warning_warn') or time.time() - self._last_servo_warning_warn > 2.0:
                    self.get_logger().warn(
                        f'{state_name}: Object in WARNING zone - bbox corner {min_distance_to_edge:.0f}px from edge - '
                        f'requesting fresh segmentation (XY motion continues)'
                    )
                    self._last_servo_warning_warn = time.time()
                self.request_segmentation_update()

            elif confidence < self.tracker_reinit_confidence_threshold and not self.waiting_for_reinit_segmentation and not skip_resegmentation:
                # In safe zone but low confidence - re-segment without stopping
                # Only log if we actually send a request (rate-limited internally)
                if self._try_request_segmentation():
                    self.get_logger().warn(
                        f'{state_name}: Low tracking confidence ({confidence:.2f}) in safe zone - '
                        f'requesting fresh segmentation (motion continues)'
                    )

        # Publish tracker debug visualization
        self.publish_tracker_debug(confidence)

        if center is None or confidence < min_confidence:
            self.get_logger().warn(f'Tracking lost in {state_name} (confidence: {confidence:.2f})')
            self.send_zero_velocity()
            if confidence < 0.1:
                self.tracker.initialized = False
            return True

        # Get depth to tracked object for XY visual servo calculations
        object_depth_raw = self._get_depth_at_point(center)
        if object_depth_raw is None or object_depth_raw < 0.05:
            self.get_logger().warn(f'Cannot get depth measurement at tracked object in {state_name}')
            self.send_zero_velocity()
            return True

        # Low-pass filter depth to reduce noise (alpha = 0.2 for moderate filtering)
        if not hasattr(self, '_prev_object_depth_filtered'):
            self._prev_object_depth_filtered = object_depth_raw
        object_depth = 0.8 * self._prev_object_depth_filtered + 0.2 * object_depth_raw
        self._prev_object_depth_filtered = object_depth

        # Get fingertip distance to table plane for Z control (collision safety)
        # This accounts for camera-to-fingertip offset based on current gripper orientation
        fingertip_distance = self.get_fingertip_distance_to_plane()
        if fingertip_distance is None:
            # Fallback to camera-based estimate if plane/TF not available
            fingertip_offset = self.get_fingertip_offset_toward_plane()
            if fingertip_offset is not None:
                fingertip_distance = object_depth - fingertip_offset
            else:
                # Last resort: assume fingertips are fingertip_offset_z closer
                fingertip_distance = object_depth - self.fingertip_offset[2]
            self.get_logger().debug(
                f'Using estimated fingertip distance: {fingertip_distance:.3f}m'
            )

        # Compute image-space error for XY control
        image_center = np.array([self.current_rgb.shape[1] / 2,
                                self.current_rgb.shape[0] / 2])
        error_pixels = center - image_center

        img_height, img_width = self.current_rgb.shape[:2]
        fov_margin = 100
        fov_edge_margin = 50

        near_boundary = (center[0] < fov_margin or center[0] > img_width - fov_margin or
                        center[1] < fov_margin or center[1] > img_height - fov_margin)
        at_boundary = (center[0] < fov_edge_margin or center[0] > img_width - fov_edge_margin or
                      center[1] < fov_edge_margin or center[1] > img_height - fov_edge_margin)

        if at_boundary:
            self.get_logger().warn(
                f'Object at FOV edge in {state_name}! pos=({center[0]:.0f},{center[1]:.0f})'
            )

        # Convert pixel error to meters using depth estimate
        current_depth = self._get_depth_at_point(center)
        if current_depth is None or current_depth < 0.05:
            current_depth = 0.3
        # Use camera intrinsics if available, fallback to estimate
        focal_length = self.fx if self.fx is not None else 500.0
        pixel_to_meter = current_depth / focal_length

        error_cam_x = error_pixels[0] * pixel_to_meter
        error_cam_y = error_pixels[1] * pixel_to_meter
        xy_error_norm = np.sqrt(error_cam_x**2 + error_cam_y**2)

        # XY velocity in tool frame (visual servoing) with PD control
        if at_boundary:
            kp_xy = 0.8
            kd_xy = 0.3
        elif near_boundary:
            kp_xy = 0.5
            kd_xy = 0.2
        else:
            kp_xy = 0.3
            kd_xy = 0.15

        # Compute derivative term
        error_xy = np.array([error_cam_x, error_cam_y])
        if not hasattr(self, '_servo_prev_error_xy'):
            self._servo_prev_error_xy = error_xy
            self._servo_prev_time = time.time()

        dt = time.time() - self._servo_prev_time
        if dt > 0.001:  # Avoid division by zero
            d_error_xy = (error_xy - self._servo_prev_error_xy) / dt
        else:
            d_error_xy = np.zeros(2)

        self._servo_prev_error_xy = error_xy.copy()
        self._servo_prev_time = time.time()

        # PD control
        v_tool_x = kp_xy * error_cam_x + kd_xy * d_error_xy[0]
        v_tool_y = kp_xy * error_cam_y + kd_xy * d_error_xy[1]

        max_xy_vel = 0.08 if at_boundary else 0.05
        v_tool_x = np.clip(v_tool_x, -max_xy_vel, max_xy_vel)
        v_tool_y = np.clip(v_tool_y, -max_xy_vel, max_xy_vel)

        # Z control: descend until fingertip_distance == target_distance
        # Using fingertip distance (not camera depth) for collision safety
        z_error = fingertip_distance - target_distance

        # Use FOV state from bbox corner check (more accurate than center-only check)
        # In danger zone: stop Z motion but allow XY to center the object
        # In warning zone: slow Z motion
        # Exception: During DESCENDING, object may fill FOV - ignore FOV warnings
        fov_in_danger = getattr(self, '_fov_in_danger_zone', False)
        fov_in_warning = getattr(self, '_fov_in_warning_zone', False)

        if state_name == 'DESCENDING':
            # During descent we're very close - object filling FOV is expected
            fov_penalty = 1.0
        elif fov_in_danger:
            fov_penalty = 0.0
            # Rate-limit this warning
            if not hasattr(self, '_last_fov_danger_warn') or time.time() - self._last_fov_danger_warn > 2.0:
                self.get_logger().warn(f'{state_name}: Object bbox at FOV edge - pausing Z, XY centering continues')
                self._last_fov_danger_warn = time.time()
        elif fov_in_warning or at_boundary:
            fov_penalty = 0.3
        elif near_boundary:
            fov_penalty = 0.5
        else:
            fov_penalty = 1.0

        # Check if at target height (within 1cm)
        at_target = abs(z_error) < 0.01

        if at_target:
            if xy_error_norm < self.xy_tolerance * 3:
                self.get_logger().info(
                    f'{state_name} complete (xy_err={xy_error_norm:.4f}m, fingertip_dist={fingertip_distance:.3f}m), '
                    f'transitioning to {next_state}'
                )
                self.transition_to_state(next_state)
                return False
            else:
                v_tool_z_raw = 0.0
                v_tool_z = 0.0
        else:
            # Scale Z speed based on XY alignment with smooth transition
            # Use proportional control for smooth speed adjustment
            if xy_error_norm > 0.06:
                # Poor XY alignment - slow Z approach
                actual_z_speed = 0.02
            elif xy_error_norm < 0.03:
                # Good XY alignment - dynamic Z speed based on distance
                actual_z_speed = min(z_speed, abs(z_error) * 0.5)
                actual_z_speed = max(0.01, actual_z_speed)
            else:
                # Transition zone (0.03 to 0.06) - blend between speeds
                blend_factor = (0.06 - xy_error_norm) / 0.03  # 0 at 0.06, 1 at 0.03
                fast_speed = min(z_speed, abs(z_error) * 0.5)
                fast_speed = max(0.01, fast_speed)
                actual_z_speed = 0.02 * (1 - blend_factor) + fast_speed * blend_factor

            # Positive z_error means descend (positive v_tool_z in tool frame = forward)
            v_tool_z_raw = actual_z_speed * fov_penalty if z_error > 0 else -actual_z_speed * fov_penalty

            # Low-pass filter to smooth velocity commands (alpha = 0.3 for smoothing)
            if not hasattr(self, '_prev_v_tool_z_filtered'):
                self._prev_v_tool_z_filtered = v_tool_z_raw
            v_tool_z = 0.7 * self._prev_v_tool_z_filtered + 0.3 * v_tool_z_raw
            self._prev_v_tool_z_filtered = v_tool_z

        # Check for stall
        if self.motion_monitor.is_stalled():
            if not self.handle_stall():
                return True

        # Send velocity command - NO rotation
        velocity = np.array([v_tool_x, v_tool_y, v_tool_z, 0.0, 0.0, 0.0])
        self.send_velocity_command(velocity)
        self.motion_monitor.set_commanded_velocity(velocity)
        return True

    def publish_tracker_debug(self, confidence):
        """Publish tracker debug visualization image."""
        debug_img = self.tracker.get_visualization(confidence)
        if debug_img is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = getattr(self, 'current_rgb_frame_id', '')
            self.tracker_debug_pub.publish(debug_msg)

    def send_velocity_command(self, velocity):
        """Send velocity command to servo."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arm_tool0'

        msg.twist.linear.x = float(velocity[0])
        msg.twist.linear.y = float(velocity[1])
        msg.twist.linear.z = float(velocity[2])
        msg.twist.angular.x = float(velocity[3])
        msg.twist.angular.y = float(velocity[4])
        msg.twist.angular.z = float(velocity[5])

        # # Debug: log occasionally to verify values
        # if not hasattr(self, '_velocity_log_count'):
        #     self._velocity_log_count = 0
        # self._velocity_log_count += 1
        # if self._velocity_log_count % 30 == 1:
        #     self.get_logger().info(
        #         f'Sending velocity: linear=[{msg.twist.linear.x:.4f}, {msg.twist.linear.y:.4f}, {msg.twist.linear.z:.4f}], '
        #         f'angular=[{msg.twist.angular.x:.4f}, {msg.twist.angular.y:.4f}, {msg.twist.angular.z:.4f}]'
        #     )

        self.twist_pub.publish(msg)

    def send_zero_velocity(self):
        """Stop all motion."""
        self.send_velocity_command(np.zeros(6))

    def send_gripper_command(self, close):
        """
        Send gripper command using GripperCommand action.

        Args:
            close: True to close gripper, False to open
        """
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Gripper action server not available')
            return

        goal = GripperCommand.Goal()
        if close:
            goal.command.position = self.gripper_closed_position
            goal.command.max_effort = 100.0
            self.get_logger().info(f'Sending gripper CLOSE command (position={goal.command.position})')
        else:
            goal.command.position = self.gripper_open_position
            goal.command.max_effort = 100.0
            self.get_logger().info(f'Sending gripper OPEN command (position={goal.command.position})')

        # Send goal asynchronously with callback
        future = self.gripper_action_client.send_goal_async(goal)
        future.add_done_callback(self._gripper_goal_response_callback)

    def _gripper_goal_response_callback(self, future):
        """Callback when gripper goal is accepted/rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return

        self.get_logger().info('Gripper goal accepted')
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result_callback)

    def _gripper_result_callback(self, future):
        """Callback when gripper action completes."""
        result = future.result().result
        if result.reached_goal:
            self.get_logger().info(f'Gripper reached goal position: {result.position}')
        elif result.stalled:
            self.get_logger().warn(f'Gripper stalled at position: {result.position}')
        else:
            self.get_logger().info(f'Gripper action completed, position: {result.position}')


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoGrasp()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Note: To restore joint control after shutdown, run:
        # ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 0}"


if __name__ == '__main__':
    main()
