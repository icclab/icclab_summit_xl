#!/usr/bin/env python3
"""
Segmentation Node for Summit XL

Provides object segmentation using SAM (Segment Anything Model) or LangSAM
(Language + SAM) for the perception pipeline.

This node:
1. Subscribes to synchronized RGB and depth images from the arm camera
2. Accepts segmentation prompts (text, points, or bounding boxes)
3. Runs SAM/LangSAM to generate segmentation masks
4. Publishes binary masks for perception
5. Generates and publishes segmented point clouds with:
   - Mask erosion to remove edge noise
   - Voxel downsampling (2mm default)
   - Statistical outlier removal
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection2D, BoundingBox2D
from cv_bridge import CvBridge
import numpy as np
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import open3d as o3d
import struct


class SegmentationNode(Node):
    """Segmentation node using SAM/LangSAM."""

    def __init__(self):
        super().__init__('segmentation_node')

        # Declare parameters
        self.declare_parameter('model_type', 'langsam')  # 'sam', 'langsam', or 'mobile_sam'
        self.declare_parameter('sam_checkpoint', 'sam_vit_h_4b8939.pth')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('rgb_topic', '/arm_camera/color/image_raw')
        self.declare_parameter('depth_topic', '/arm_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/arm_camera/color/camera_info')
        self.declare_parameter('voxel_size', 0.002)  # 2mm voxel size for downsampling
        self.declare_parameter('remove_outliers', True)

        # Get parameters
        self.model_type = self.get_parameter('model_type').value
        self.sam_checkpoint = self.get_parameter('sam_checkpoint').value
        self.device = self.get_parameter('device').value
        self.use_gpu = self.get_parameter('use_gpu').value
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.remove_outliers = self.get_parameter('remove_outliers').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Current RGB and depth images
        self.current_rgb = None
        self.current_depth = None
        self.current_rgb_frame_id = None
        self.intrinsic_matrix = None

        # Segmentation model (lazy loading)
        self.sam_predictor = None
        self.lang_sam = None

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

        # Segmentation prompt subscribers (different input modalities)
        self.text_prompt_sub = self.create_subscription(
            String,
            '/segment_text',
            self.text_prompt_callback,
            10
        )

        self.point_prompt_sub = self.create_subscription(
            Point,
            '/segment_point',
            self.point_prompt_callback,
            10
        )

        self.bbox_prompt_sub = self.create_subscription(
            BoundingBox2D,
            '/segment_bbox',
            self.bbox_prompt_callback,
            10
        )

        # Publishers
        self.mask_pub = self.create_publisher(
            Image,
            '/segmentation_mask',
            10
        )

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/segmented_pointcloud',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/segmentation_status',
            10
        )

        self.get_logger().info('Segmentation Node initialized')
        self.get_logger().info(f'Model type: {self.model_type}')
        self.get_logger().info(f'RGB topic: {self.rgb_topic}')
        self.get_logger().info(f'Depth topic: {self.depth_topic}')
        self.get_logger().info('Synchronized RGB-D image acquisition enabled')
        self.get_logger().info('Waiting for segmentation model to load...')
        self.get_logger().info('NOTE: First segmentation will be slow due to model loading')

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

    def crop_to_bbox(self, rgb, depth, mask, margin=0.5):
        """Crop images to object bounding box with margin."""
        y, x = np.where(mask != 0)
        if len(x) == 0 or len(y) == 0:
            return rgb, depth, mask

        x_min, x_max = np.min(x), np.max(x)
        y_min, y_max = np.min(y), np.max(y)

        # Add margin
        width = x_max - x_min
        height = y_max - y_min
        x_margin = int(width * margin / 2)
        y_margin = int(height * margin / 2)

        x_min = max(0, x_min - x_margin)
        x_max = min(rgb.shape[1], x_max + x_margin)
        y_min = max(0, y_min - y_margin)
        y_max = min(rgb.shape[0], y_max + y_margin)

        return rgb[y_min:y_max, x_min:x_max], depth[y_min:y_max, x_min:x_max], mask[y_min:y_max, x_min:x_max]

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

        # Create point cloud
        # NOTE: Gazebo RGBD camera frame alignment requires specific rotation.
        # Working tf2 transform: roll=-π/2, yaw=-π/2
        # ROS applies rotations in X,Y,Z order, meaning: first Rx, then Ry, then Rz
        # So the extrinsic needs the INVERSE: Rx^-1 @ Ry^-1 @ Rz^-1

        roll = -np.pi / 2
        yaw = -np.pi / 2

        # Compute rotation matrices
        cos_r, sin_r = np.cos(roll), np.sin(roll)
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)

        # Rx(roll)
        Rx = np.array([
            [1, 0, 0],
            [0, cos_r, -sin_r],
            [0, sin_r, cos_r]
        ])

        # Rz(yaw)
        Rz = np.array([
            [cos_y, -sin_y, 0],
            [sin_y, cos_y, 0],
            [0, 0, 1]
        ])

        # For extrinsic, we need the inverse transformation
        # (Rz @ Rx)^-1 = Rx^T @ Rz^T
        R = Rx.T @ Rz.T

        extrinsic = np.eye(4)
        extrinsic[:3, :3] = R
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            image=rgbd_o3d,
            intrinsic=intrinsic_o3d,
            extrinsic=extrinsic
        )

        self.get_logger().info(f'Pointcloud created: {len(pcd.points)} points before filtering')

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
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)

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

        # Pack point cloud data with padding for alignment
        cloud_data = []
        for i in range(len(points)):
            x, y, z = points[i]
            if len(colors) > 0:
                r, g, b = (colors[i] * 255).astype(np.uint8)
                rgb = struct.unpack('f', struct.pack('I', struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]))[0]
            else:
                rgb = 0.0
            # Pack: x, y, z (12 bytes) + 4 bytes padding + rgb as float (4 bytes) + 4 bytes padding = 24 bytes total
            cloud_data.append(struct.pack('fffIfI', x, y, z, 0, rgb, 0))

        # Create PointCloud2 message
        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(points)
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 24  # Updated to match arm_camera/points
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width
        pc2_msg.is_dense = True
        pc2_msg.data = b''.join(cloud_data)

        return pc2_msg

    def initialize_model(self):
        """Lazy initialization of segmentation model."""
        if self.model_type == 'langsam' and self.lang_sam is None:
            self.initialize_langsam()
        elif self.model_type in ['sam', 'mobile_sam'] and self.sam_predictor is None:
            self.initialize_sam()

    def initialize_langsam(self):
        """Initialize LangSAM (Language + SAM)."""
        try:
            self.get_logger().info('Loading LangSAM model...')
            from lang_sam import LangSAM

            self.lang_sam = LangSAM()
            self.get_logger().info('LangSAM loaded successfully')

        except ImportError:
            self.get_logger().error('LangSAM not installed. Install with:')
            self.get_logger().error('  pip install git+https://github.com/luca-medeiros/lang-segment-anything.git')
            self.publish_status('ERROR: LangSAM not installed')
            raise

        except Exception as e:
            self.get_logger().error(f'Error loading LangSAM: {e}')
            self.publish_status(f'ERROR: {str(e)}')
            raise

    def initialize_sam(self):
        """Initialize SAM (Segment Anything Model)."""
        try:
            self.get_logger().info(f'Loading SAM model: {self.sam_checkpoint}')
            from segment_anything import sam_model_registry, SamPredictor

            if self.model_type == 'mobile_sam':
                model_type = 'vit_t'  # MobileSAM uses ViT-Tiny
            else:
                model_type = 'vit_h'  # Default SAM uses ViT-Huge

            sam = sam_model_registry[model_type](checkpoint=self.sam_checkpoint)

            if self.use_gpu:
                sam.to(device=self.device)

            self.sam_predictor = SamPredictor(sam)
            self.get_logger().info('SAM loaded successfully')

        except ImportError:
            self.get_logger().error('segment-anything not installed. Install with:')
            self.get_logger().error('  pip install segment-anything')
            self.publish_status('ERROR: segment-anything not installed')
            raise

        except Exception as e:
            self.get_logger().error(f'Error loading SAM: {e}')
            self.publish_status(f'ERROR: {str(e)}')
            raise

    def synchronized_callback(self, rgb_msg, depth_msg):
        """Callback for synchronized RGB and depth images."""
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            self.current_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            self.current_rgb_frame_id = rgb_msg.header.frame_id

            # Log once every 100 messages to avoid spam
            if not hasattr(self, '_sync_count'):
                self._sync_count = 0
            self._sync_count += 1
            if self._sync_count % 100 == 1:
                self.get_logger().info(
                    f'Synchronized RGB-D pair received (RGB: {self.current_rgb.shape}, '
                    f'Depth: {self.current_depth.shape}, Frame: {self.current_rgb_frame_id})'
                )
        except Exception as e:
            self.get_logger().error(f'Error converting synchronized images: {e}')

    def text_prompt_callback(self, msg):
        """Callback for text-based segmentation prompt (LangSAM)."""
        text_prompt = msg.data
        self.get_logger().info(f'Text segmentation request: "{text_prompt}"')

        if self.current_rgb is None:
            self.get_logger().error('No RGB image available')
            self.publish_status('ERROR: No image')
            return

        if self.model_type != 'langsam':
            self.get_logger().error('Text prompts require LangSAM model')
            self.publish_status('ERROR: Wrong model type')
            return

        # Initialize model if needed
        self.initialize_model()

        # Run LangSAM
        self.publish_status('SEGMENTING')

        try:
            # LangSAM expects PIL Image and lists
            from PIL import Image as PILImage
            pil_image = PILImage.fromarray(self.current_rgb)

            # Run prediction (LangSAM expects lists)
            results = self.lang_sam.predict([pil_image], [text_prompt])

            if len(results) == 0 or len(results[0].get('masks', [])) == 0:
                self.get_logger().warn(f'No objects found for prompt: "{text_prompt}"')
                self.publish_status('NO_OBJECTS_FOUND')
                return

            # Extract results from first image
            result = results[0]
            masks = result['masks']
            labels = result.get('text_labels', [])

            # Use the first (highest confidence) mask
            mask = masks[0]
            self.get_logger().info(f'Found {len(masks)} object(s), using highest confidence')
            if labels:
                self.get_logger().info(f'Detected phrase: "{labels[0]}"')

            # Publish mask
            self.publish_mask(mask)
            self.publish_status('SUCCESS')

        except Exception as e:
            self.get_logger().error(f'Error during segmentation: {e}')
            import traceback
            traceback.print_exc()
            self.publish_status(f'ERROR: {str(e)}')

    def point_prompt_callback(self, msg):
        """Callback for point-based segmentation prompt (SAM)."""
        self.get_logger().info(f'Point segmentation request: ({msg.x}, {msg.y})')

        if self.current_rgb is None:
            self.get_logger().error('No RGB image available')
            return

        if self.model_type not in ['sam', 'mobile_sam']:
            self.get_logger().error('Point prompts require SAM model')
            return

        # Initialize model if needed
        self.initialize_model()

        # Run SAM with point prompt
        self.publish_status('SEGMENTING')

        try:
            # Set image
            self.sam_predictor.set_image(self.current_rgb)

            # Point prompt (positive)
            point_coords = np.array([[msg.x, msg.y]])
            point_labels = np.array([1])  # 1 = foreground point

            # Predict
            masks, scores, logits = self.sam_predictor.predict(
                point_coords=point_coords,
                point_labels=point_labels,
                multimask_output=True
            )

            # Use highest scoring mask
            best_mask_idx = np.argmax(scores)
            mask = masks[best_mask_idx]

            self.get_logger().info(f'Generated mask with score: {scores[best_mask_idx]:.3f}')

            # Publish mask
            self.publish_mask(mask)
            self.publish_status('SUCCESS')

        except Exception as e:
            self.get_logger().error(f'Error during segmentation: {e}')
            self.publish_status(f'ERROR: {str(e)}')

    def bbox_prompt_callback(self, msg):
        """Callback for bounding box segmentation prompt (SAM)."""
        self.get_logger().info(f'Bbox segmentation request')

        if self.current_rgb is None:
            self.get_logger().error('No RGB image available')
            return

        if self.model_type not in ['sam', 'mobile_sam']:
            self.get_logger().error('Bbox prompts require SAM model')
            return

        # Initialize model if needed
        self.initialize_model()

        # Run SAM with bbox prompt
        self.publish_status('SEGMENTING')

        try:
            # Set image
            self.sam_predictor.set_image(self.current_rgb)

            # Convert bbox to SAM format [x_min, y_min, x_max, y_max]
            cx = msg.center.position.x
            cy = msg.center.position.y
            w = msg.size_x
            h = msg.size_y

            bbox = np.array([
                cx - w/2,  # x_min
                cy - h/2,  # y_min
                cx + w/2,  # x_max
                cy + h/2   # y_max
            ])

            # Predict
            masks, scores, logits = self.sam_predictor.predict(
                box=bbox[None, :],  # Add batch dimension
                multimask_output=False
            )

            mask = masks[0]

            self.get_logger().info(f'Generated mask from bbox')

            # Publish mask
            self.publish_mask(mask)
            self.publish_status('SUCCESS')

        except Exception as e:
            self.get_logger().error(f'Error during segmentation: {e}')
            self.publish_status(f'ERROR: {str(e)}')

    def publish_mask(self, mask):
        """Publish segmentation mask and segmented point cloud."""
        # Convert boolean mask to uint8 (0 or 255)
        mask_uint8 = (mask * 255).astype(np.uint8)

        # Convert to ROS Image message
        mask_msg = self.bridge.cv2_to_imgmsg(mask_uint8, encoding='mono8')
        mask_msg.header.stamp = self.get_clock().now().to_msg()
        mask_msg.header.frame_id = self.current_rgb_frame_id if self.current_rgb_frame_id else 'camera_color_optical_frame'

        self.mask_pub.publish(mask_msg)
        self.get_logger().info('Published segmentation mask')

        # Generate and publish point cloud if we have all required data
        if self.current_rgb is not None and self.current_depth is not None and self.intrinsic_matrix is not None:
            try:
                # Step 1: Erode mask to remove edge noise
                eroded_mask = self.erode_mask(mask)

                # Step 2: Create point cloud from RGB-D and mask
                pcd = self.create_pointcloud_from_rgbd(
                    rgb=self.current_rgb,
                    depth=self.current_depth,
                    mask=eroded_mask,
                    intrinsic_matrix=self.intrinsic_matrix
                )

                # Step 3: Downsample and filter
                pcd = self.downsample_and_filter_pointcloud(pcd)

                # Step 4: Convert to ROS message and publish
                if len(pcd.points) > 0:
                    pc2_msg = self.o3d_to_ros_pointcloud2(pcd, frame_id=self.current_rgb_frame_id)
                    self.pointcloud_pub.publish(pc2_msg)
                    self.get_logger().info(f'Published segmented point cloud with {len(pcd.points)} points in frame {self.current_rgb_frame_id}')
                else:
                    self.get_logger().warn('Point cloud is empty after filtering')

            except Exception as e:
                self.get_logger().error(f'Error generating point cloud: {e}')
                import traceback
                traceback.print_exc()
        else:
            missing = []
            if self.current_rgb is None:
                missing.append('RGB')
            if self.current_depth is None:
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
    node = SegmentationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
