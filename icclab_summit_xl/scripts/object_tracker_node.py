#!/usr/bin/env python3
"""
Object Tracker Node

A generic, reusable tracker node that estimates the 2D and 3D position of a
segmented object.  It does NOT compute any motion commands.

Pipeline
--------
1. Receive a segmentation mask + the paired RGB image (via ApproximateTimeSynchronizer).
2. Initialise (or re-initialise) an OpenCV CSRT tracker from the mask.
3. On every RGB frame, run the tracker and back-project the tracked 2D centre
   through the depth image to obtain a 3D position in the camera frame.
4. Publish 2D position, 3D position, tracking confidence, and a debug image.

Topics (all relative to the node namespace unless remapped)
-----------------------------------------------------------
Subscribed:
  ~/rgb              sensor_msgs/Image          Live RGB feed
  ~/depth            sensor_msgs/Image          Aligned depth (32FC1 or 16UC1)
  ~/camera_info      sensor_msgs/CameraInfo     Camera intrinsics
  /segmentation_mask sensor_msgs/Image          Binary mask (mono8)
  /segmentation_rgb  sensor_msgs/Image          RGB image paired with the mask

Published:
  ~/object_position_2d   geometry_msgs/PointStamped
      x = column (pixels), y = row (pixels), z = orientation angle (radians)
  ~/object_position_3d   geometry_msgs/PointStamped
      3-D position in the camera optical frame (metres)
  ~/tracking_confidence  std_msgs/Float32
      Confidence in [0, 1]; 0 means tracking lost
  ~/tracker_debug_image  sensor_msgs/Image
      BGR debug visualisation from the tracker

Parameters
----------
  update_rate          float  (default 30.0)   Tracker update rate in Hz
  depth_min            float  (default 0.05)   Min valid depth in metres
  depth_max            float  (default 5.0)    Max valid depth in metres
  depth_neighborhood   int    (default 5)      Half-size of depth sampling patch
  min_confidence       float  (default 0.3)    Minimum confidence to publish 3-D position
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

import numpy as np
import cv2
import time


# ---------------------------------------------------------------------------
# Tracker
# ---------------------------------------------------------------------------

class SimpleMaskTracker:
    """
    Scale-invariant tracker using OpenCV CSRT.
    Handles scale changes, rotation, and partial occlusion.
    """

    def __init__(self):
        self.tracker = None
        self.template_color = None
        self.template_bbox = None
        self.prev_center = None
        self.prev_angle = 0.0
        self.initialized = False
        self.init_bbox = None

    def initialize(self, image, mask, debug_logger=None):
        """Initialize CSRT tracker with bounding box derived from the mask."""
        y_coords, x_coords = np.where(mask > 0)
        if len(x_coords) == 0:
            if debug_logger:
                debug_logger.warn('Tracker init failed: empty mask')
            return False

        x_min, x_max = int(x_coords.min()), int(x_coords.max())
        y_min, y_max = int(y_coords.min()), int(y_coords.max())

        # Add a small margin
        margin = 10
        x_min = max(0, x_min - margin)
        x_max = min(image.shape[1], x_max + margin)
        y_min = max(0, y_min - margin)
        y_max = min(image.shape[0], y_max + margin)

        bbox_w = x_max - x_min
        bbox_h = y_max - y_min
        if bbox_w < 10 or bbox_h < 10:
            if debug_logger:
                debug_logger.warn(f'Tracker init failed: bbox too small {bbox_w}x{bbox_h}')
            return False

        self.tracker = cv2.TrackerCSRT_create()
        bbox = (x_min, y_min, bbox_w, bbox_h)

        image_bgr = image if len(image.shape) == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        try:
            self.tracker.init(image_bgr, bbox)
        except Exception as e:
            if debug_logger:
                debug_logger.error(f'Tracker init failed: {e}')
            return False

        self.init_bbox = bbox
        self.template_bbox = bbox
        self.template_color = image_bgr[y_min:y_max, x_min:x_max].copy()
        self.prev_center = np.array([x_min + bbox_w / 2.0, y_min + bbox_h / 2.0])

        mask_crop = mask[y_min:y_max, x_min:x_max]
        _, self.prev_angle = self._get_pose_from_mask(mask_crop)

        if debug_logger:
            debug_logger.info(
                f'CSRT tracker initialised: bbox={bbox}, '
                f'center=({self.prev_center[0]:.1f},{self.prev_center[1]:.1f})'
            )

        self.initialized = True
        return True

    # ------------------------------------------------------------------
    def _get_pose_from_mask(self, mask):
        """Extract 2D centroid and orientation from a mask patch."""
        if mask.sum() == 0:
            return None, 0.0

        moments = cv2.moments(mask.astype(np.uint8))
        if moments['m00'] == 0:
            return None, 0.0

        cx = moments['m10'] / moments['m00']
        cy = moments['m01'] / moments['m00']

        contours, _ = cv2.findContours(
            mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return np.array([cx, cy]), 0.0

        contour = max(contours, key=cv2.contourArea)
        if len(contour) < 5:
            return np.array([cx, cy]), 0.0

        rect = cv2.minAreaRect(contour)
        angle = np.deg2rad(rect[2])

        return np.array([cx, cy]), angle

    # ------------------------------------------------------------------
    def update(self, image):
        """
        Track object in a new frame.

        Returns
        -------
        center : ndarray [u, v] or None
        angle  : float – orientation in radians
        confidence : float in [0, 1]
        """
        if not self.initialized or self.tracker is None:
            return None, 0.0, 0.0

        self._last_tracked_image = image

        # Skip every other frame (CSRT is slow)
        if getattr(self, 'skip_frames', True):
            if not hasattr(self, '_frame_skip_counter'):
                self._frame_skip_counter = 0
            self._frame_skip_counter += 1
            if self._frame_skip_counter % 2 != 0 and self.prev_center is not None:
                return self.prev_center, self.prev_angle, 0.85

        image_bgr = image if len(image.shape) == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        try:
            success, bbox = self.tracker.update(image_bgr)
        except Exception:
            return self.prev_center, self.prev_angle, 0.0

        if not success:
            return self.prev_center, self.prev_angle, 0.0

        x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        self.template_bbox = (x, y, w, h)
        center = np.array([x + w / 2.0, y + h / 2.0])

        if self.init_bbox is not None:
            init_area = self.init_bbox[2] * self.init_bbox[3]
            area_ratio = (w * h) / max(1, init_area)
            if 0.3 <= area_ratio <= 3.0:
                confidence = 0.9
            elif 0.1 <= area_ratio <= 5.0:
                confidence = 0.6
            else:
                confidence = 0.3
        else:
            confidence = 0.8

        angle = self._estimate_angle_from_bbox(image_bgr, x, y, w, h)

        self.prev_center = center
        self.prev_angle = angle

        return center, angle, confidence

    # ------------------------------------------------------------------
    def _estimate_angle_from_bbox(self, image, x, y, w, h):
        """Estimate object orientation from edge content inside the tracked bbox."""
        img_h, img_w = image.shape[:2]
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(img_w, x + w), min(img_h, y + h)

        if x2 <= x1 or y2 <= y1:
            return self.prev_angle

        roi = image[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi
        edges = cv2.Canny(gray, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return self.prev_angle

        all_points = np.vstack(contours)
        if len(all_points) < 5:
            return self.prev_angle

        rect = cv2.minAreaRect(all_points)
        rect_angle = rect[2]
        rect_w, rect_h = rect[1]

        long_axis_angle = (rect_angle + 90) if rect_h > rect_w else rect_angle

        while long_axis_angle > 90:
            long_axis_angle -= 180
        while long_axis_angle < -90:
            long_axis_angle += 180

        angle_rad = np.deg2rad(long_axis_angle)
        return 0.3 * angle_rad + 0.7 * self.prev_angle  # low-pass filter

    # ------------------------------------------------------------------
    def get_visualization(self, confidence=None):
        """Return a debug BGR image annotated with tracker state."""
        if not self.initialized or not hasattr(self, '_last_tracked_image'):
            return None

        debug_img = self._last_tracked_image.copy()

        if hasattr(self, 'template_bbox') and self.template_bbox is not None:
            x, y, w, h = self.template_bbox
            if confidence is not None:
                color = (0, 255, 0) if confidence > 0.7 else (0, 255, 255) if confidence > 0.4 else (0, 0, 255)
            else:
                color = (0, 255, 0)
            cv2.rectangle(debug_img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(debug_img, f'Match: {w}x{h}', (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        if self.prev_center is not None:
            cx, cy = int(self.prev_center[0]), int(self.prev_center[1])
            cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)
            cv2.circle(debug_img, (cx, cy), 10, (0, 0, 255), 2)
            length = 50
            ex = int(cx + length * np.cos(self.prev_angle))
            ey = int(cy + length * np.sin(self.prev_angle))
            cv2.line(debug_img, (cx, cy), (ex, ey), (255, 0, 0), 2)

        img_h, img_w = debug_img.shape[:2]
        cv2.drawMarker(debug_img, (img_w // 2, img_h // 2), (255, 255, 0), cv2.MARKER_CROSS, 20, 2)

        if self.prev_center is not None:
            err_x = self.prev_center[0] - img_w / 2
            err_y = self.prev_center[1] - img_h / 2
            err_norm = np.sqrt(err_x ** 2 + err_y ** 2)
            cv2.putText(debug_img, f'Center: ({int(self.prev_center[0])}, {int(self.prev_center[1])})',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(debug_img, f'Error: ({int(err_x)}, {int(err_y)}) = {int(err_norm)}px',
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if confidence is not None:
                color = (0, 255, 0) if confidence > 0.7 else (0, 255, 255) if confidence > 0.4 else (0, 0, 255)
                cv2.putText(debug_img, f'Confidence: {confidence:.3f}',
                            (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Template inset (bottom-right corner)
        if self.template_color is not None:
            th, tw = self.template_color.shape[:2]
            scale = min(150.0 / tw, 150.0 / th)
            nw, nh = int(tw * scale), int(th * scale)
            if nw > 0 and nh > 0:
                tmpl = cv2.resize(self.template_color, (nw, nh))
                margin = 10
                ys = img_h - nh - margin
                xs = img_w - nw - margin
                cv2.rectangle(debug_img, (xs - 2, ys - 2), (xs + nw + 2, ys + nh + 2), (255, 255, 255), 2)
                debug_img[ys:ys + nh, xs:xs + nw] = tmpl
                cv2.putText(debug_img, 'Template', (xs, ys - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        return debug_img


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class ObjectTrackerNode(Node):
    """
    Generic object tracker node.

    Initialises a CSRT tracker from incoming segmentation masks and tracks
    the object across live RGB frames.  The 3-D position is recovered by
    back-projecting the tracked 2-D centre through the aligned depth image.
    """

    def __init__(self):
        super().__init__('object_tracker')

        # --- Parameters ---
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('depth_min', 0.05)
        self.declare_parameter('depth_max', 5.0)
        self.declare_parameter('depth_neighborhood', 5)
        self.declare_parameter('min_confidence', 0.3)
        self.declare_parameter('rgb_topic', '/arm_camera/color/image_raw')
        self.declare_parameter('depth_topic', '/arm_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/arm_camera/color/camera_info')

        self._update_rate = self.get_parameter('update_rate').value
        self._depth_min = self.get_parameter('depth_min').value
        self._depth_max = self.get_parameter('depth_max').value
        self._depth_neighborhood = self.get_parameter('depth_neighborhood').value
        self._min_confidence = self.get_parameter('min_confidence').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        # --- State ---
        self._bridge = CvBridge()
        self._tracker = SimpleMaskTracker()

        self._current_rgb = None          # latest BGR image (numpy)
        self._current_rgb_frame_id = ''
        self._current_rgb_stamp = None

        self._current_depth = None        # depth in metres (float32 numpy)

        # Camera intrinsics (set once from CameraInfo)
        self._fx = self._fy = self._cx = self._cy = None

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        # --- Subscribers ---
        self._rgb_sub = self.create_subscription(
            Image, rgb_topic, self._rgb_callback, sensor_qos)

        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._depth_callback, sensor_qos)

        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, sensor_qos)

        # Synchronised mask + RGB from segmentation node
        self._mask_sub = Subscriber(self, Image, '/segmentation_mask')
        self._seg_rgb_sub = Subscriber(self, Image, '/segmentation_rgb')
        self._seg_sync = ApproximateTimeSynchronizer(
            [self._mask_sub, self._seg_rgb_sub], queue_size=10, slop=0.1)
        self._seg_sync.registerCallback(self._segmentation_callback)

        # --- Publishers ---
        self._pub_2d = self.create_publisher(PointStamped, '~/object_position_2d', 10)
        self._pub_3d = self.create_publisher(PointStamped, '~/object_position_3d', 10)
        self._pub_conf = self.create_publisher(Float32, '~/tracking_confidence', 10)
        self._pub_debug = self.create_publisher(Image, '~/tracker_debug_image', 10)

        # --- Timer ---
        self._timer = self.create_timer(1.0 / self._update_rate, self._update)

        self.get_logger().info(
            f'ObjectTrackerNode started at {self._update_rate} Hz.\n'
            f'  RGB:         {rgb_topic}\n'
            f'  Depth:       {depth_topic}\n'
            f'  CameraInfo:  {camera_info_topic}\n'
            f'  Mask/RGB:    /segmentation_mask + /segmentation_rgb\n'
            f'Waiting for segmentation mask to initialise tracker …'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _rgb_callback(self, msg: Image):
        try:
            self._current_rgb = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._current_rgb_frame_id = msg.header.frame_id
            self._current_rgb_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')

    def _depth_callback(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                depth = self._bridge.imgmsg_to_cv2(msg, '32FC1')
                self._current_depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            elif msg.encoding == '16UC1':
                depth_mm = self._bridge.imgmsg_to_cv2(msg, '16UC1')
                self._current_depth = depth_mm.astype(np.float32) / 1000.0
            else:
                self.get_logger().warn(f'Unknown depth encoding: {msg.encoding}')
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    def _camera_info_callback(self, msg: CameraInfo):
        K = np.array(msg.k).reshape(3, 3)
        self._fx = K[0, 0]
        self._fy = K[1, 1]
        self._cx = K[0, 2]
        self._cy = K[1, 2]
        self.get_logger().info(
            f'Camera intrinsics: fx={self._fx:.1f}, fy={self._fy:.1f}, '
            f'cx={self._cx:.1f}, cy={self._cy:.1f}'
        )
        self.destroy_subscription(self._camera_info_sub)

    def _segmentation_callback(self, mask_msg: Image, rgb_msg: Image):
        """Reinitialise the tracker whenever a new segmentation arrives."""
        try:
            mask = self._bridge.imgmsg_to_cv2(mask_msg, 'mono8')
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Segmentation callback error: {e}')
            return

        binary_mask = (mask > 0).astype(np.uint8)
        mask_area = binary_mask.sum()

        if mask_area < 50:
            self.get_logger().warn(f'Segmentation mask too small ({mask_area} px), ignoring.')
            return

        if self._tracker.initialize(rgb, binary_mask, self.get_logger()):
            self.get_logger().info(
                f'Tracker (re)initialised from segmentation mask (area={mask_area} px).'
            )
        else:
            self.get_logger().warn('Tracker initialisation from segmentation mask failed.')

    # ------------------------------------------------------------------
    # Update loop
    # ------------------------------------------------------------------

    def _update(self):
        if self._current_rgb is None or not self._tracker.initialized:
            return

        center, angle, confidence = self._tracker.update(self._current_rgb)

        stamp = self._current_rgb_stamp or self.get_clock().now().to_msg()
        frame_id = self._current_rgb_frame_id

        # Always publish confidence
        conf_msg = Float32()
        conf_msg.data = float(confidence)
        self._pub_conf.publish(conf_msg)

        # Publish debug image
        debug_img = self._tracker.get_visualization(confidence)
        if debug_img is not None:
            try:
                debug_ros = self._bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                debug_ros.header.stamp = stamp
                debug_ros.header.frame_id = frame_id
                self._pub_debug.publish(debug_ros)
            except Exception as e:
                self.get_logger().error(f'Debug image publish error: {e}')

        if center is None or confidence < self._min_confidence:
            if center is None:
                self.get_logger().warn('Tracking lost (no center).')
            return

        # --- 2D position ---
        pos2d = PointStamped()
        pos2d.header.stamp = stamp
        pos2d.header.frame_id = frame_id
        pos2d.point.x = float(center[0])   # column (pixels)
        pos2d.point.y = float(center[1])   # row    (pixels)
        pos2d.point.z = float(angle)        # orientation (radians)
        self._pub_2d.publish(pos2d)

        # --- 3D position (back-projection through depth) ---
        depth = self._get_depth_at_pixel(center)
        if depth is None:
            self.get_logger().debug('No valid depth at tracked object centre.')
            return

        if self._fx is None:
            self.get_logger().warn('Camera intrinsics not yet received; cannot compute 3D position.')
            return

        x3d = (center[0] - self._cx) * depth / self._fx
        y3d = (center[1] - self._cy) * depth / self._fy
        z3d = depth

        pos3d = PointStamped()
        pos3d.header.stamp = stamp
        pos3d.header.frame_id = frame_id
        pos3d.point.x = float(x3d)
        pos3d.point.y = float(y3d)
        pos3d.point.z = float(z3d)
        self._pub_3d.publish(pos3d)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_depth_at_pixel(self, pixel_coords):
        """
        Return a robust depth estimate at a pixel location.

        Samples a small neighbourhood and returns the median of valid depths.
        Returns None if no valid depth is found.
        """
        if self._current_depth is None:
            return None

        u, v = int(pixel_coords[0]), int(pixel_coords[1])
        h, w = self._current_depth.shape

        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))

        n = self._depth_neighborhood
        patch = self._current_depth[max(0, v - n):v + n + 1,
                                    max(0, u - n):u + n + 1]

        valid = patch[(patch > self._depth_min) & (patch < self._depth_max)]
        if len(valid) == 0:
            return None

        return float(np.median(valid))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
