import numpy as np
import cv2
from typing import Tuple, Optional

class LaneKeepingAssist:
    def __init__(self, warning_threshold: float = 0.3):
        """
        Initialize Lane Keeping Assist system.
        
        Args:
            warning_threshold: Threshold for lane departure warning (ratio of lane width)
        """
        self.warning_threshold = warning_threshold
        self.lane_width = None
        self.smoothing_factor = 0.3
        self.previous_distances = None
        
    def _find_lane_boundaries(self, ll_seg_mask: np.ndarray, row_height: int) -> Tuple[Optional[int], Optional[int]]:
        """
        Find left and right lane boundaries in a specific row of the image.
        
        Args:
            ll_seg_mask: Lane line segmentation mask
            row_height: Row to check for lane boundaries
            
        Returns:
            Tuple of (left_boundary, right_boundary) positions
        """
        try:
            # Get positions of lane markings in the specified row
            lane_positions = np.where(ll_seg_mask[row_height, :] > 0)[0]
            
            if len(lane_positions) < 2:
                return None, None
                
            # Get leftmost and rightmost lane markings
            left_boundary = lane_positions[0]
            right_boundary = lane_positions[-1]
            
            return left_boundary, right_boundary
            
        except Exception as e:
            print(f"Error finding lane boundaries: {e}")
            return None, None
            
    def calculate_lane_distances(self, ll_seg_mask: np.ndarray) -> Tuple[float, float, float]:
        """
        Calculate distances from vehicle center to lane boundaries.
        
        Args:
            ll_seg_mask: Lane line segmentation mask
            
        Returns:
            Tuple of (left_distance, right_distance, lane_width) in pixels
        """
        # Get image dimensions
        height, width = ll_seg_mask.shape
        
        # Calculate reference row (near bottom of image)
        ref_row = int(height * 0.8)  # 80% down from top
        
        # Find lane boundaries
        left_bound, right_bound = self._find_lane_boundaries(ll_seg_mask, ref_row)
        
        if left_bound is None or right_bound is None:
            return 0.0, 0.0, 0.0
            
        # Calculate center of image
        center = width // 2
        
        # Calculate distances
        left_distance = center - left_bound
        right_distance = right_bound - center
        current_lane_width = right_bound - left_bound
        
        # Apply smoothing if we have previous measurements
        if self.previous_distances is not None:
            left_distance = (self.smoothing_factor * left_distance + 
                           (1 - self.smoothing_factor) * self.previous_distances[0])
            right_distance = (self.smoothing_factor * right_distance + 
                            (1 - self.smoothing_factor) * self.previous_distances[1])
            current_lane_width = (self.smoothing_factor * current_lane_width + 
                                (1 - self.smoothing_factor) * self.previous_distances[2])
        
        self.previous_distances = (left_distance, right_distance, current_lane_width)
        return left_distance, right_distance, current_lane_width
        
    def check_lane_departure(self, left_distance: float, right_distance: float, 
                           lane_width: float) -> Tuple[bool, str]:
        """
        Check if vehicle is departing from lane.
        
        Returns:
            Tuple of (is_warning, warning_message)
        """
        if lane_width == 0:
            return False, ""
            
        # Calculate normalized distances (as ratio of lane width)
        left_ratio = left_distance / lane_width
        right_ratio = right_distance / lane_width
        
        # Check for lane departure
        if left_ratio < self.warning_threshold:
            return True, "WARNING: Lane Departure - Drifting Left"
        elif right_ratio < self.warning_threshold:
            return True, "WARNING: Lane Departure - Drifting Right"
            
        return False, ""
        
    def visualize_distances(self, image: np.ndarray, left_distance: float, 
                          right_distance: float, lane_width: float) -> np.ndarray:
        """
        Visualize lane distances and warnings on the image.
        """
        vis_img = image.copy()
        height, width = vis_img.shape[:2]
        ref_row = int(height * 0.8)
        
        if lane_width > 0:
            # Draw distance measurements
            cv2.putText(vis_img, f'Left: {left_distance:.1f}px', 
                       (10, height - 60), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 0), 2)
            cv2.putText(vis_img, f'Right: {right_distance:.1f}px', 
                       (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 0), 2)
            
            # Check and draw warnings
            is_warning, warning_msg = self.check_lane_departure(
                left_distance, right_distance, lane_width)
            
            if is_warning:
                # Draw warning message
                cv2.putText(vis_img, warning_msg, 
                           (width // 4, height // 4),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                
                # Draw warning indicators
                if "Left" in warning_msg:
                    cv2.rectangle(vis_img, (0, 0), (10, height), 
                                (0, 0, 255), -1)
                elif "Right" in warning_msg:
                    cv2.rectangle(vis_img, (width - 10, 0), (width, height), 
                                (0, 0, 255), -1)
        
        return vis_img