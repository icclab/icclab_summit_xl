#!/usr/bin/env python3
"""
Feature extraction and matching for visual servoing
Supports multiple feature extractors: ORB, SIFT, SuperPoint, XFeat
"""

import numpy as np
import cv2
from typing import Tuple, Optional, List, Dict
import warnings


class FeatureTracker:
    """
    Feature extraction and matching for visual servoing
    Supports multiple backends for robustness and flexibility
    """

    def __init__(self, extractor_type: str = 'orb', **kwargs):
        """
        Initialize feature tracker

        Args:
            extractor_type: Type of feature extractor ('orb', 'sift', 'superpoint', 'xfeat')
            **kwargs: Additional parameters for the extractor
        """
        self.extractor_type = extractor_type.lower()
        self.extractor = None
        self.matcher = None

        # Configuration
        self.config = {
            'nms_radius': kwargs.get('nms_radius', 4),
            'keypoint_threshold': kwargs.get('keypoint_threshold', 0.005),
            'max_keypoints': kwargs.get('max_keypoints', 500),
            'match_threshold': kwargs.get('match_threshold', 0.75),
        }

        # Initialize extractor
        self._init_extractor()
        self._init_matcher()

    def _init_extractor(self):
        """Initialize the feature extractor"""
        if self.extractor_type == 'orb':
            self.extractor = cv2.ORB_create(
                nfeatures=self.config['max_keypoints'],
                scaleFactor=1.2,
                nlevels=8,
                edgeThreshold=31,
                firstLevel=0,
                WTA_K=2,
                scoreType=cv2.ORB_HARRIS_SCORE,
                patchSize=31,
                fastThreshold=20
            )
            print("Initialized ORB feature extractor")

        elif self.extractor_type == 'sift':
            self.extractor = cv2.SIFT_create(
                nfeatures=self.config['max_keypoints'],
                nOctaveLayers=3,
                contrastThreshold=0.04,
                edgeThreshold=10,
                sigma=1.6
            )
            print("Initialized SIFT feature extractor")

        elif self.extractor_type == 'superpoint':
            try:
                # Try to import SuperPoint (requires additional installation)
                from .superpoint_extractor import SuperPointExtractor
                self.extractor = SuperPointExtractor(
                    max_keypoints=self.config['max_keypoints'],
                    keypoint_threshold=self.config['keypoint_threshold'],
                    nms_radius=self.config['nms_radius']
                )
                print("Initialized SuperPoint feature extractor")
            except ImportError:
                warnings.warn("SuperPoint not available, falling back to ORB")
                self.extractor_type = 'orb'
                self._init_extractor()

        elif self.extractor_type == 'xfeat':
            try:
                # Try to import XFeat (requires additional installation)
                from .xfeat_extractor import XFeatExtractor
                self.extractor = XFeatExtractor(
                    max_keypoints=self.config['max_keypoints']
                )
                print("Initialized XFeat feature extractor")
            except ImportError:
                warnings.warn("XFeat not available, falling back to ORB")
                self.extractor_type = 'orb'
                self._init_extractor()

        else:
            raise ValueError(f"Unknown extractor type: {self.extractor_type}")

    def _init_matcher(self):
        """Initialize the feature matcher"""
        if self.extractor_type in ['orb']:
            # Use Hamming distance for binary descriptors
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        else:
            # Use L2 distance for float descriptors
            self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)

    def extract_features(self,
                         image: np.ndarray,
                         mask: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Extract features from an image

        Args:
            image: Input image (grayscale or RGB)
            mask: Optional binary mask to restrict feature extraction

        Returns:
            keypoints: Nx2 array of (x, y) keypoint coordinates
            descriptors: NxD array of feature descriptors
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Apply mask if provided
        if mask is not None:
            # Ensure mask is binary
            if mask.dtype != np.uint8:
                mask = (mask > 0).astype(np.uint8) * 255

        # Extract features using OpenCV-style extractors
        if self.extractor_type in ['orb', 'sift']:
            kp, desc = self.extractor.detectAndCompute(gray, mask)

            # Convert keypoints to numpy array
            if len(kp) == 0:
                return np.array([]).reshape(0, 2), np.array([]).reshape(0, 128)

            keypoints = np.array([[k.pt[0], k.pt[1]] for k in kp])

        # Custom extractors (SuperPoint, XFeat)
        else:
            keypoints, desc = self.extractor.detect(gray, mask)

        return keypoints, desc

    def match_features(self,
                       desc1: np.ndarray,
                       desc2: np.ndarray,
                       ratio_threshold: Optional[float] = None) -> np.ndarray:
        """
        Match features between two descriptor sets

        Args:
            desc1: First set of descriptors
            desc2: Second set of descriptors
            ratio_threshold: Lowe's ratio test threshold (default: use config)

        Returns:
            matches: Mx2 array of matching indices (index in desc1, index in desc2)
        """
        if len(desc1) == 0 or len(desc2) == 0:
            return np.array([]).reshape(0, 2)

        if ratio_threshold is None:
            ratio_threshold = self.config['match_threshold']

        # Use k-NN matching with k=2 for ratio test
        try:
            matches_knn = self.matcher.knnMatch(desc1, desc2, k=2)
        except cv2.error:
            # Fallback to regular matching if kNN fails
            matches_raw = self.matcher.match(desc1, desc2)
            matches = np.array([[m.queryIdx, m.trainIdx] for m in matches_raw])
            return matches

        # Apply Lowe's ratio test
        good_matches = []
        for match_pair in matches_knn:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < ratio_threshold * n.distance:
                    good_matches.append([m.queryIdx, m.trainIdx])
            elif len(match_pair) == 1:
                # Only one match found, keep it
                m = match_pair[0]
                good_matches.append([m.queryIdx, m.trainIdx])

        return np.array(good_matches)

    def track_features(self,
                       image_current: np.ndarray,
                       image_taught: np.ndarray,
                       keypoints_taught: np.ndarray,
                       descriptors_taught: np.ndarray,
                       mask_current: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Track features from taught image to current image

        Args:
            image_current: Current image
            image_taught: Taught/reference image
            keypoints_taught: Keypoints from taught image
            descriptors_taught: Descriptors from taught image
            mask_current: Optional mask for current image

        Returns:
            keypoints_current: Matched keypoints in current image
            matches: Match indices (Mx2)
            match_confidence: Confidence score for each match
        """
        # Extract features from current image
        keypoints_current, descriptors_current = self.extract_features(
            image_current, mask_current
        )

        if len(keypoints_current) == 0 or len(descriptors_current) == 0:
            return np.array([]).reshape(0, 2), np.array([]).reshape(0, 2), np.array([])

        # Match features
        matches = self.match_features(descriptors_taught, descriptors_current)

        if len(matches) == 0:
            return np.array([]).reshape(0, 2), np.array([]).reshape(0, 2), np.array([])

        # Get matched keypoints
        matched_kp_current = keypoints_current[matches[:, 1]]

        # Compute match confidence (inverse of descriptor distance)
        match_confidence = np.ones(len(matches))  # Simplified confidence

        return matched_kp_current, matches, match_confidence

    def visualize_features(self,
                          image: np.ndarray,
                          keypoints: np.ndarray,
                          color: Tuple[int, int, int] = (0, 255, 0),
                          radius: int = 3) -> np.ndarray:
        """
        Visualize keypoints on image

        Args:
            image: Input image
            keypoints: Nx2 array of keypoint coordinates
            color: Color for keypoints (BGR)
            radius: Radius of keypoint circles

        Returns:
            vis_image: Image with keypoints drawn
        """
        vis_image = image.copy()

        for kp in keypoints:
            pt = (int(kp[0]), int(kp[1]))
            cv2.circle(vis_image, pt, radius, color, -1)

        return vis_image

    def visualize_matches(self,
                         image1: np.ndarray,
                         image2: np.ndarray,
                         keypoints1: np.ndarray,
                         keypoints2: np.ndarray,
                         matches: np.ndarray,
                         max_display: int = 50) -> np.ndarray:
        """
        Visualize feature matches between two images

        Args:
            image1: First image (taught)
            image2: Second image (current)
            keypoints1: Keypoints from first image
            keypoints2: Keypoints from second image
            matches: Match indices (Mx2)
            max_display: Maximum number of matches to display

        Returns:
            vis_image: Side-by-side image with matches drawn
        """
        # Limit number of displayed matches
        if len(matches) > max_display:
            indices = np.random.choice(len(matches), max_display, replace=False)
            matches_display = matches[indices]
        else:
            matches_display = matches

        # Create side-by-side image
        h1, w1 = image1.shape[:2]
        h2, w2 = image2.shape[:2]
        vis_image = np.zeros((max(h1, h2), w1 + w2, 3), dtype=np.uint8)

        # Convert to BGR if needed
        if len(image1.shape) == 2:
            image1 = cv2.cvtColor(image1, cv2.COLOR_GRAY2BGR)
        if len(image2.shape) == 2:
            image2 = cv2.cvtColor(image2, cv2.COLOR_GRAY2BGR)

        vis_image[:h1, :w1] = image1
        vis_image[:h2, w1:w1+w2] = image2

        # Draw matches
        for match in matches_display:
            pt1 = tuple(keypoints1[match[0]].astype(int))
            pt2 = tuple((keypoints2[match[1]] + np.array([w1, 0])).astype(int))

            # Random color for each match
            color = tuple(np.random.randint(0, 255, 3).tolist())

            cv2.circle(vis_image, pt1, 3, color, -1)
            cv2.circle(vis_image, pt2, 3, color, -1)
            cv2.line(vis_image, pt1, pt2, color, 1)

        return vis_image


def test_feature_tracker():
    """Test feature tracker"""
    print("Testing FeatureTracker...")

    # Create synthetic images
    img1 = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
    img2 = np.roll(img1, 50, axis=1)  # Shifted version

    # Test with ORB
    tracker = FeatureTracker('orb')

    # Extract features
    kp1, desc1 = tracker.extract_features(img1)
    kp2, desc2 = tracker.extract_features(img2)

    print(f"Extracted {len(kp1)} keypoints from image 1")
    print(f"Extracted {len(kp2)} keypoints from image 2")

    # Match features
    matches = tracker.match_features(desc1, desc2)
    print(f"Found {len(matches)} matches")

    # Track features
    kp_current, matches, confidence = tracker.track_features(
        img2, img1, kp1, desc1
    )
    print(f"Tracked {len(kp_current)} features")

    print("\nAll tests passed!")


if __name__ == '__main__':
    test_feature_tracker()
