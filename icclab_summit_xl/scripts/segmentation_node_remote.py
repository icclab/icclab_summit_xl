#!/usr/bin/env python3
"""
Remote Segmentation Node for Summit XL

Similar to segmentation_node.py but uses a remote LangSAM service instead of
instantiating the model locally. This allows running the heavy model on a
separate machine with a GPU while keeping the ROS node lightweight.

This node:
1. Subscribes to synchronized RGB and depth images from the arm camera
2. Accepts segmentation prompts (text prompts only for LangSAM)
3. Sends requests to remote LangSAM server
4. Receives segmentation masks from the server
5. Publishes binary masks for perception
6. Generates and publishes segmented point clouds with:
   - Mask erosion to remove edge noise
   - Voxel downsampling (2mm default)
   - Statistical outlier removal
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
import numpy as np
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import open3d as o3d
import struct
import requests
from io import BytesIO
import base64
import threading


class RemoteSegmentationNode(Node):
    """Remote segmentation node using LangSAM service."""

    def __init__(self):
        super().__init__('remote_segmentation_node')

        # Declare parameters
        self.declare_parameter('server_url', 'http://localhost:8001')
        self.declare_parameter('server_timeout', 30.0)  # seconds
        self.declare_parameter('sam_type', 'sam2.1_hiera_small')
        self.declare_parameter('box_threshold', 0.3)
        self.declare_parameter('text_threshold', 0.25)
        self.declare_parameter('rgb_topic', '/arm_camera/color/image_raw')
        self.declare_parameter('depth_topic', '/arm_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/arm_camera/color/camera_info')
        self.declare_parameter('voxel_size', 0.002)  # 2mm voxel size for downsampling
        self.declare_parameter('remove_outliers', True)
        # Prefix prepended to /segment_text, /segmentation_mask, /segmented_pointcloud,
        # /segmentation_status. Empty for arm camera, "/front" for the front camera
        # instance. Lets two nodes coexist without topic collisions.
        self.declare_parameter('topic_prefix', '')

        # Get parameters
        self.server_url = self.get_parameter('server_url').value
        self.server_timeout = self.get_parameter('server_timeout').value
        self.sam_type = self.get_parameter('sam_type').value
        self.box_threshold = self.get_parameter('box_threshold').value
        self.text_threshold = self.get_parameter('text_threshold').value
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.remove_outliers = self.get_parameter('remove_outliers').value
        self.topic_prefix = self.get_parameter('topic_prefix').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Current RGB and depth images (guarded by a lock to avoid partial reads)
        self._image_lock = threading.Lock()
        self.current_rgb = None
        self.current_depth = None
        self.current_rgb_frame_id = None
        self.intrinsic_matrix = None
        self._sync_count = 0

        # QoS for camera topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers with synchronization
        self.rgb_sub = Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=qos_profile
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile
        )

        # Approximate time synchronizer for RGB and depth images
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance for synchronization
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile
        )

        # Segmentation prompt subscribers (only text for LangSAM)
        self.text_prompt_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/segment_text',
            self.text_prompt_callback,
            10
        )

        # Publishers
        self.mask_pub = self.create_publisher(
            Image,
            f'{self.topic_prefix}/segmentation_mask',
            10
        )

        # Latched QoS so gripper_attach_node receives the pointcloud
        # even if it subscribes after the one-shot publish
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            f'{self.topic_prefix}/segmented_pointcloud',
            latched_qos,
        )

        self.status_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/segmentation_status',
            latched_qos,
        )

        self.get_logger().info('Remote Segmentation Node initialized')
        self.get_logger().info(f'Server URL: {self.server_url}')
        self.get_logger().info(f'SAM type: {self.sam_type}')
        self.get_logger().info(f'RGB topic: {self.rgb_topic}')
        self.get_logger().info(f'Depth topic: {self.depth_topic}')
        self.get_logger().info('Synchronized RGB-D image acquisition enabled')

        # Test server connectivity
        self.test_server_connection()

    def test_server_connection(self):
        """Test connection to the remote LangSAM server."""
        try:
            # Try a simple request to see if server is reachable
            response = requests.get(f"{self.server_url.rstrip('/')}/health", timeout=5.0)
            self.get_logger().info('Successfully connected to LangSAM server')
        except requests.exceptions.RequestException:
            # Health endpoint might not exist, that's okay
            self.get_logger().warn(
                f'Could not verify server connection at {self.server_url}. '
                'Make sure the LangSAM server is running before sending segmentation requests.'
            )

    def camera_info_callback(self, msg):
        """Callback for camera info to extract intrinsic matrix."""
        if self.intrinsic_matrix is None:
            # Extract intrinsic matrix from CameraInfo
            K = np.array(msg.k).reshape(3, 3)
            self.intrinsic_matrix = K
            self.get_logger().info(f'Camera intrinsic matrix received:\n{K}')

    def intrinsic_matrix_to_o3d(self, intrinsic_matrix, width, height):
        """Convert numpy intrinsic matrix to Open3D format with image dimensions."""
        intrinsic_matrix_o3d = o3d.camera.PinholeCameraIntrinsic()
        intrinsic_matrix_o3d.set_intrinsics(
            width=width,
            height=height,
            fx=intrinsic_matrix[0, 0],
            fy=intrinsic_matrix[1, 1],
            cx=intrinsic_matrix[0, 2],
            cy=intrinsic_matrix[1, 2]
        )
        return intrinsic_matrix_o3d

    def erode_mask(self, mask):
        """Erode segmentation mask to remove edge noise."""
        kernel = np.ones((3, 3), dtype=np.float32)
        kernel = kernel / kernel.sum()
        mask_float = mask.astype(np.float32)
        for _ in range(1):
            mask_float = cv2.filter2D(src=mask_float, ddepth=-1, kernel=kernel)
            mask_float = (mask_float >= 1)
        return mask_float.astype(bool)

    def create_pointcloud_from_rgbd(self, rgb, depth, mask, intrinsic_matrix):
        """
        Create Open3D point cloud from RGB-D images and segmentation mask.

        Args:
            rgb: RGB image (H, W, 3) uint8
            depth: Depth image (H, W) in millimeters, uint16 or float
            mask: Segmentation mask (H, W) bool
            intrinsic_matrix: Camera intrinsic matrix (3, 3)

        Returns:
            Open3D point cloud
        """
        # Gazebo RGBD camera publishes depth in float32 meters with NaN for invalid pixels
        # Keep it as float32 for Open3D processing
        if depth.dtype == np.float32 or depth.dtype == np.float64:
            # Replace NaN/inf with 0
            depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        else:
            # If it's uint16, assume it's in millimeters
            depth = depth.astype(np.float32) / 1000.0

        # Apply mask to depth and RGB
        # For Open3D, we need to preserve image dimensions but set non-masked pixels to 0 depth
        # Open3D will skip pixels with 0 or invalid depth when creating point cloud
        masked_depth = np.where(mask, depth, 0.0)
        masked_rgb = np.where(mask[..., None], rgb, 0)

        # Debug: Check depth values in masked region
        valid_depths = masked_depth[mask > 0]
        if len(valid_depths) > 0:
            self.get_logger().info(f'Depth values in mask: min={valid_depths.min()}, max={valid_depths.max()}, mean={valid_depths.mean():.3f}, nonzero={np.count_nonzero(valid_depths)}')
        else:
            self.get_logger().warn('No valid depth values in masked region!')

        # Convert to Open3D format with proper image dimensions
        height, width = rgb.shape[:2]
        intrinsic_o3d = self.intrinsic_matrix_to_o3d(intrinsic_matrix, width, height)

        # Convert depth to Open3D Image (needs to be float32 or uint16)
        depth_o3d = o3d.geometry.Image(masked_depth.astype(np.float32))
        rgb_o3d = o3d.geometry.Image(masked_rgb.astype(np.uint8))

        # Create RGBD image
        rgbd_o3d = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color=rgb_o3d,
            depth=depth_o3d,
            depth_scale=1.0,  # Depth is already in meters from Gazebo simulation
            depth_trunc=20.0,  # Match the camera far clipping plane
            convert_rgb_to_intensity=False
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            image=rgbd_o3d,
            intrinsic=intrinsic_o3d
        )

        self.get_logger().info(f'Pointcloud created: {len(pcd.points)} points before filtering')

        # Debug: Check point distribution
        if len(pcd.points) > 0:
            points = np.asarray(pcd.points)
            self.get_logger().info(f'Point cloud stats:')
            self.get_logger().info(f'  X: min={points[:, 0].min():.4f}, max={points[:, 0].max():.4f}, mean={points[:, 0].mean():.4f}')
            self.get_logger().info(f'  Y: min={points[:, 1].min():.4f}, max={points[:, 1].max():.4f}, mean={points[:, 1].mean():.4f}')
            self.get_logger().info(f'  Z: min={points[:, 2].min():.4f}, max={points[:, 2].max():.4f}, mean={points[:, 2].mean():.4f}')

        return pcd

    def downsample_and_filter_pointcloud(self, pcd):
        """Downsample and filter point cloud using voxel grid and statistical outlier removal."""
        initial_count = len(pcd.points)

        # Voxel downsampling
        pcd = pcd.voxel_down_sample(self.voxel_size)
        self.get_logger().info(f'After voxel downsampling: {len(pcd.points)} points (from {initial_count})')

        # Statistical outlier removal
        if self.remove_outliers and len(pcd.points) > 20:
            before_outlier = len(pcd.points)
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.8)
            self.get_logger().info(f'After outlier removal: {len(pcd.points)} points (from {before_outlier})')

        return pcd

    def o3d_to_ros_pointcloud2(self, pcd, frame_id=None):
        """Convert Open3D point cloud to ROS PointCloud2 message."""
        points = np.asarray(pcd.points)       # (N, 3) float64
        colors = np.asarray(pcd.colors)       # (N, 3) float64  or empty

        n = len(points)

        # Build the packed buffer entirely with numpy — no per-point Python loop.
        # Layout per point (24 bytes, matching arm_camera/points):
        #   offset  0: x      float32
        #   offset  4: y      float32
        #   offset  8: z      float32
        #   offset 12: pad    uint32  = 0
        #   offset 16: rgb    float32 (BGR packed as uint32 reinterpreted as float)
        #   offset 20: pad    uint32  = 0
        buf = np.zeros((n, 6), dtype=np.float32)
        buf[:, 0] = points[:, 0]
        buf[:, 1] = points[:, 1]
        buf[:, 2] = points[:, 2]
        # offset 12 (col 3) stays 0 (padding)

        if n > 0 and colors.size > 0:
            rgb_u8 = (colors * 255).clip(0, 255).astype(np.uint8)  # (N, 3) BGR-ready
            # Pack as 0x00RRGGBB stored in little-endian float32 slot
            packed = (rgb_u8[:, 0].astype(np.uint32) << 16 |
                      rgb_u8[:, 1].astype(np.uint32) << 8  |
                      rgb_u8[:, 2].astype(np.uint32))
            # Reinterpret the uint32 bits as float32 (same trick as the old struct code)
            buf[:, 4] = packed.view(np.float32)
        # offset 20 (col 5) stays 0 (padding)

        # Create header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id if frame_id is not None else 'camera_color_optical_frame'

        # Define fields with proper alignment (rgb at offset 16 like arm_camera/points)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = n
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 24
        pc2_msg.row_step = pc2_msg.point_step * n
        pc2_msg.is_dense = True
        pc2_msg.data = buf.tobytes()

        return pc2_msg

    def synchronized_callback(self, rgb_msg, depth_msg):
        """Callback for synchronized RGB and depth images."""
        try:
            new_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            new_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            frame_id = rgb_msg.header.frame_id

            with self._image_lock:
                self.current_rgb = new_rgb
                self.current_depth = new_depth
                self.current_rgb_frame_id = frame_id
                self._sync_count += 1
                count = self._sync_count

            if count % 100 == 1:
                self.get_logger().info(
                    f'Synchronized RGB-D pair received (RGB: {new_rgb.shape}, '
                    f'Depth: {new_depth.shape}, Frame: {frame_id})'
                )
        except Exception as e:
            self.get_logger().error(f'Error converting synchronized images: {e}')

    def text_prompt_callback(self, msg):
        """Callback for text-based segmentation prompt."""
        text_prompt = msg.data
        self.get_logger().info(f'Text segmentation request: "{text_prompt}"')

        with self._image_lock:
            current_rgb = self.current_rgb
            current_depth = self.current_depth
            current_frame_id = self.current_rgb_frame_id

        if current_rgb is None:
            self.get_logger().error('No RGB image available')
            self.publish_status('ERROR: No image')
            return

        # Send request to remote server
        self.publish_status('SEGMENTING')

        try:
            # Encode image as PNG
            success, image_encoded = cv2.imencode('.png', cv2.cvtColor(current_rgb, cv2.COLOR_RGB2BGR))
            if not success:
                raise ValueError("Failed to encode image")

            # Prepare multipart form data
            files = {
                'image': ('image.png', BytesIO(image_encoded.tobytes()), 'image/png'),
            }
            data = {
                'sam_type': self.sam_type,
                'box_threshold': str(self.box_threshold),
                'text_threshold': str(self.text_threshold),
                'text_prompt': text_prompt,
                'output_format': 'json',  # Request JSON output
            }

            # Send request
            self.get_logger().info(f'Sending request to {self.server_url}/predict')
            response = requests.post(
                f"{self.server_url.rstrip('/')}/predict",
                files=files,
                data=data,
                timeout=self.server_timeout
            )

            if response.status_code != 200:
                raise ValueError(f"Server returned status {response.status_code}: {response.text}")

            # Parse JSON response
            result = response.json()

            if not result.get('masks'):
                self.get_logger().warn(f'No objects found for prompt: "{text_prompt}"')
                self.publish_status('NO_OBJECTS_FOUND')
                return

            # Decode masks
            masks = []
            
            # for mask_data in result['masks']:
            #     mask_bytes = base64.b64decode(mask_data['data'])
            #     mask = np.frombuffer(mask_bytes, dtype=np.uint8).reshape(mask_data['shape'])
            #     masks.append(mask.astype(bool))

            for i, mask_data in enumerate(result["masks"]):
                mask = np.array(mask_data, dtype=bool)
                masks.append(mask.astype(bool))

            # Use the first (highest confidence) mask
            mask = masks[0]
            self.get_logger().info(f'Found {len(masks)} object(s), using highest confidence')
            if result.get('labels'):
                self.get_logger().info(f'Detected phrase: "{result["labels"][0]}"')

            # Publish mask (pass the locally captured snapshot to avoid TOCTOU races)
            self.publish_mask(mask, current_rgb, current_depth, current_frame_id)
            self.publish_status('SUCCESS')

        except requests.exceptions.Timeout:
            self.get_logger().error(f'Request to server timed out after {self.server_timeout}s')
            self.publish_status('ERROR: Server timeout')
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f'Could not connect to server at {self.server_url}')
            self.publish_status('ERROR: Connection failed')
        except Exception as e:
            self.get_logger().error(f'Error during segmentation: {e}')
            import traceback
            traceback.print_exc()
            self.publish_status(f'ERROR: {str(e)}')

    def publish_mask(self, mask, current_rgb, current_depth, current_frame_id):
        """Publish segmentation mask and segmented point cloud."""
        # Convert boolean mask to uint8 (0 or 255)
        mask_uint8 = (mask * 255).astype(np.uint8)

        # Convert to ROS Image message
        mask_msg = self.bridge.cv2_to_imgmsg(mask_uint8, encoding='mono8')
        mask_msg.header.stamp = self.get_clock().now().to_msg()
        mask_msg.header.frame_id = current_frame_id if current_frame_id else 'camera_color_optical_frame'

        self.mask_pub.publish(mask_msg)
        self.get_logger().info('Published segmentation mask')

        # Generate and publish point cloud if we have all required data
        if current_rgb is not None and current_depth is not None and self.intrinsic_matrix is not None:
            try:
                # Step 1: Erode mask to remove edge noise
                eroded_mask = self.erode_mask(mask)

                # Step 2: Create point cloud from RGB-D and mask
                pcd = self.create_pointcloud_from_rgbd(
                    rgb=current_rgb,
                    depth=current_depth,
                    mask=eroded_mask,
                    intrinsic_matrix=self.intrinsic_matrix
                )

                # Step 3: Downsample and filter
                pcd = self.downsample_and_filter_pointcloud(pcd)

                # Step 4: Convert to ROS message and publish
                if len(pcd.points) > 0:
                    pc2_msg = self.o3d_to_ros_pointcloud2(pcd, frame_id=current_frame_id)
                    self.pointcloud_pub.publish(pc2_msg)
                    self.get_logger().info(f'Published segmented point cloud with {len(pcd.points)} points in frame {current_frame_id}')
                else:
                    self.get_logger().warn('Point cloud is empty after filtering')

            except Exception as e:
                self.get_logger().error(f'Error generating point cloud: {e}')
                import traceback
                traceback.print_exc()
        else:
            missing = []
            if current_rgb is None:
                missing.append('RGB')
            if current_depth is None:
                missing.append('depth')
            if self.intrinsic_matrix is None:
                missing.append('camera intrinsics')
            self.get_logger().warn(f'Cannot generate point cloud, missing: {", ".join(missing)}')

    def publish_status(self, status):
        """Publish segmentation status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RemoteSegmentationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
