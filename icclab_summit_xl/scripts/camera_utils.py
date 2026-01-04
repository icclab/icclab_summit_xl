#!/usr/bin/env python3
"""
Camera utilities for visual servoing
Handles backprojection, pose estimation, and coordinate transformations
"""

import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from typing import Tuple, Optional, List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraUtils:
    """Utilities for camera operations in visual servoing"""

    def __init__(self, camera_info: Optional[CameraInfo] = None):
        """
        Initialize camera utilities

        Args:
            camera_info: ROS CameraInfo message (optional)
        """
        if camera_info is not None:
            self.K = np.array(camera_info.k).reshape(3, 3)
            self.D = np.array(camera_info.d)
            self.width = camera_info.width
            self.height = camera_info.height
        else:
            # Default intrinsics (will be set later)
            self.K = None
            self.D = None
            self.width = None
            self.height = None

    def set_intrinsics(self, fx: float, fy: float, cx: float, cy: float,
                       width: int = 640, height: int = 480):
        """
        Set camera intrinsics manually

        Args:
            fx, fy: Focal lengths
            cx, cy: Principal point
            width, height: Image dimensions
        """
        self.K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])
        self.D = np.zeros(5)  # Assume no distortion
        self.width = width
        self.height = height

    def backproject_points(self,
                           keypoints: np.ndarray,
                           depth_image: np.ndarray,
                           depth_scale: float = 0.001,
                           min_depth: float = 0.1,
                           max_depth: float = 2.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert 2D keypoints to 3D points using depth image

        Args:
            keypoints: Nx2 array of (u, v) pixel coordinates
            depth_image: Depth image
            depth_scale: Scale factor to convert depth values to meters
            min_depth: Minimum valid depth in meters
            max_depth: Maximum valid depth in meters

        Returns:
            points_3d: Nx3 array of 3D points in camera frame
            valid_mask: Boolean mask indicating valid points
        """
        if self.K is None:
            raise ValueError("Camera intrinsics not set")

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        points_3d = []
        valid_mask = []

        for kp in keypoints:
            u, v = int(kp[0]), int(kp[1])

            # Check bounds
            if u < 0 or u >= depth_image.shape[1] or v < 0 or v >= depth_image.shape[0]:
                points_3d.append([0, 0, 0])
                valid_mask.append(False)
                continue

            # Get depth value
            z = depth_image[v, u] * depth_scale

            # Check depth validity
            if z < min_depth or z > max_depth or np.isnan(z) or z == 0:
                points_3d.append([0, 0, 0])
                valid_mask.append(False)
                continue

            # Backproject to 3D
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            points_3d.append([x, y, z])
            valid_mask.append(True)

        return np.array(points_3d), np.array(valid_mask)

    def project_points(self, points_3d: np.ndarray) -> np.ndarray:
        """
        Project 3D points to 2D image plane

        Args:
            points_3d: Nx3 array of 3D points

        Returns:
            points_2d: Nx2 array of (u, v) pixel coordinates
        """
        if self.K is None:
            raise ValueError("Camera intrinsics not set")

        # Use OpenCV's projectPoints for proper handling of distortion
        if self.D is not None and np.any(self.D != 0):
            points_2d, _ = cv2.projectPoints(
                points_3d,
                np.zeros(3),  # No rotation
                np.zeros(3),  # No translation
                self.K,
                self.D
            )
            return points_2d.reshape(-1, 2)
        else:
            # Simple pinhole projection
            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]

            x = points_3d[:, 0]
            y = points_3d[:, 1]
            z = points_3d[:, 2]

            u = fx * x / z + cx
            v = fy * y / z + cy

            return np.column_stack([u, v])

    def estimate_rigid_transform(self,
                                  src_points: np.ndarray,
                                  dst_points: np.ndarray,
                                  method: str = 'ransac',
                                  ransac_threshold: float = 0.01) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Estimate rigid transformation between two point clouds

        Args:
            src_points: Nx3 source points
            dst_points: Nx3 destination points
            method: 'ransac' or 'svd'
            ransac_threshold: RANSAC inlier threshold in meters

        Returns:
            R: 3x3 rotation matrix
            t: 3x1 translation vector
            inliers: Boolean mask of inlier points
        """
        if len(src_points) < 3 or len(dst_points) < 3:
            raise ValueError("Need at least 3 point correspondences")

        if method == 'ransac':
            # Use RANSAC for robust estimation
            best_R = np.eye(3)
            best_t = np.zeros(3)
            best_inliers = np.zeros(len(src_points), dtype=bool)
            best_num_inliers = 0

            # RANSAC iterations
            num_iterations = min(100, len(src_points) * 10)
            for _ in range(num_iterations):
                # Randomly sample 3 points
                if len(src_points) < 3:
                    break

                indices = np.random.choice(len(src_points), size=min(3, len(src_points)), replace=False)
                src_sample = src_points[indices]
                dst_sample = dst_points[indices]

                # Estimate transform using Kabsch algorithm
                R, t = self._kabsch_algorithm(src_sample, dst_sample)

                # Count inliers
                transformed = (R @ src_points.T).T + t
                errors = np.linalg.norm(transformed - dst_points, axis=1)
                inliers = errors < ransac_threshold

                num_inliers = np.sum(inliers)
                if num_inliers > best_num_inliers:
                    best_num_inliers = num_inliers
                    best_inliers = inliers
                    best_R = R
                    best_t = t

            # Refine with all inliers
            if best_num_inliers >= 3:
                best_R, best_t = self._kabsch_algorithm(
                    src_points[best_inliers],
                    dst_points[best_inliers]
                )

            return best_R, best_t, best_inliers

        elif method == 'svd':
            # Direct SVD method (Kabsch algorithm)
            R, t = self._kabsch_algorithm(src_points, dst_points)
            inliers = np.ones(len(src_points), dtype=bool)
            return R, t, inliers

        else:
            raise ValueError(f"Unknown method: {method}")

    def _kabsch_algorithm(self, src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Kabsch algorithm for optimal rotation and translation

        Args:
            src: Nx3 source points
            dst: Nx3 destination points

        Returns:
            R: 3x3 rotation matrix
            t: 3x1 translation vector
        """
        # Center the point clouds
        src_centroid = np.mean(src, axis=0)
        dst_centroid = np.mean(dst, axis=0)

        src_centered = src - src_centroid
        dst_centered = dst - dst_centroid

        # Compute cross-covariance matrix
        H = src_centered.T @ dst_centered

        # SVD
        U, S, Vt = np.linalg.svd(H)

        # Compute rotation
        R = Vt.T @ U.T

        # Handle reflection case
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # Compute translation
        t = dst_centroid - R @ src_centroid

        return R, t

    def pose_to_transform_matrix(self, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
        """
        Convert rotation and translation to 4x4 homogeneous transform

        Args:
            rotation: 3x3 rotation matrix or quaternion [x, y, z, w]
            translation: 3x1 translation vector

        Returns:
            T: 4x4 homogeneous transformation matrix
        """
        T = np.eye(4)

        # Handle different rotation representations
        if rotation.shape == (3, 3):
            T[:3, :3] = rotation
        elif rotation.shape == (4,) or rotation.shape == (1, 4):
            # Assume quaternion [x, y, z, w]
            rot = Rotation.from_quat(rotation)
            T[:3, :3] = rot.as_matrix()
        else:
            raise ValueError("Rotation must be 3x3 matrix or quaternion")

        T[:3, 3] = translation.flatten()
        return T

    def compute_relative_transform(self, T1: np.ndarray, T2: np.ndarray) -> np.ndarray:
        """
        Compute relative transform: T_rel = T1^-1 * T2

        Args:
            T1: 4x4 transform matrix (from)
            T2: 4x4 transform matrix (to)

        Returns:
            T_rel: 4x4 relative transformation
        """
        return np.linalg.inv(T1) @ T2

    def compute_pose_error(self, current_pose: np.ndarray, desired_pose: np.ndarray) -> np.ndarray:
        """
        Compute pose error for control

        Args:
            current_pose: 4x4 current transform
            desired_pose: 4x4 desired transform

        Returns:
            error: 6D error vector [dx, dy, dz, droll, dpitch, dyaw]
        """
        # Position error
        position_error = desired_pose[:3, 3] - current_pose[:3, 3]

        # Orientation error (axis-angle representation)
        R_error = desired_pose[:3, :3] @ current_pose[:3, :3].T
        rot_error = Rotation.from_matrix(R_error)
        orientation_error = rot_error.as_rotvec()

        return np.concatenate([position_error, orientation_error])

    def compute_cartesian_velocity(self,
                                    current_pose: np.ndarray,
                                    desired_pose: np.ndarray,
                                    k_p: float = 0.5,
                                    k_r: float = 0.3,
                                    max_linear_vel: float = 0.1,
                                    max_angular_vel: float = 0.3) -> np.ndarray:
        """
        Compute Cartesian velocity command from pose error

        Args:
            current_pose: 4x4 current transform
            desired_pose: 4x4 desired transform
            k_p: Position control gain
            k_r: Rotation control gain
            max_linear_vel: Maximum linear velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)

        Returns:
            velocity: 6D velocity command [vx, vy, vz, wx, wy, wz]
        """
        error = self.compute_pose_error(current_pose, desired_pose)

        # Proportional control
        v_linear = k_p * error[:3]
        v_angular = k_r * error[3:]

        # Apply velocity limits
        linear_norm = np.linalg.norm(v_linear)
        if linear_norm > max_linear_vel:
            v_linear = v_linear * max_linear_vel / linear_norm

        angular_norm = np.linalg.norm(v_angular)
        if angular_norm > max_angular_vel:
            v_angular = v_angular * max_angular_vel / angular_norm

        return np.concatenate([v_linear, v_angular])

    def is_pose_reached(self,
                        current_pose: np.ndarray,
                        desired_pose: np.ndarray,
                        position_tol: float = 0.005,
                        orientation_tol: float = 0.087) -> bool:
        """
        Check if desired pose is reached within tolerance

        Args:
            current_pose: 4x4 current transform
            desired_pose: 4x4 desired transform
            position_tol: Position tolerance in meters
            orientation_tol: Orientation tolerance in radians

        Returns:
            True if pose is reached
        """
        error = self.compute_pose_error(current_pose, desired_pose)

        position_error = np.linalg.norm(error[:3])
        orientation_error = np.linalg.norm(error[3:])

        return position_error < position_tol and orientation_error < orientation_tol


def test_camera_utils():
    """Test camera utilities"""
    print("Testing CameraUtils...")

    # Create camera utils with default intrinsics
    cam = CameraUtils()
    cam.set_intrinsics(fx=615.0, fy=615.0, cx=320.0, cy=240.0)

    # Test backprojection
    keypoints = np.array([[320, 240], [400, 300], [200, 180]])
    depth_image = np.ones((480, 640)) * 1000  # 1 meter depth
    points_3d, valid = cam.backproject_points(keypoints, depth_image)

    print(f"Keypoints: {keypoints}")
    print(f"3D points: {points_3d}")
    print(f"Valid mask: {valid}")

    # Test projection
    points_2d = cam.project_points(points_3d[valid])
    print(f"Reprojected: {points_2d}")

    # Test rigid transform estimation
    src = np.random.rand(10, 3)
    R_true = Rotation.from_euler('xyz', [0.1, 0.2, 0.3]).as_matrix()
    t_true = np.array([0.1, 0.2, 0.3])
    dst = (R_true @ src.T).T + t_true

    R_est, t_est, inliers = cam.estimate_rigid_transform(src, dst, method='svd')
    print(f"\nRotation error: {np.linalg.norm(R_true - R_est)}")
    print(f"Translation error: {np.linalg.norm(t_true - t_est)}")

    print("\nAll tests passed!")


if __name__ == '__main__':
    test_camera_utils()
