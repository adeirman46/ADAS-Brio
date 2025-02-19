import numpy as np
import cv2
from typing import List, Tuple, Optional
from dataclasses import dataclass
import pyzed.sl as sl

@dataclass
class Waypoint:
    x: float
    y: float
    distance: float
    heading: Optional[float] = None

class WaypointExtractor:
    def __init__(self, num_points: int = 6, min_distance: float = 50):
        """
        Initialize waypoint extractor.
        
        Args:
            num_points: Number of waypoints to extract
            min_distance: Minimum pixel distance between waypoints
        """
        self.num_points = num_points
        self.min_distance = min_distance
        self.previous_waypoints: List[Waypoint] = []
        self.smoothing_factor = 0.3  # For waypoint smoothing
        
    def _find_drivable_region_bounds(self, mask: np.ndarray, row: int) -> Tuple[int, int]:
        """Find left and right bounds of drivable area in a row."""
        where_drivable = np.where(mask[row] > 0)[0]
        if len(where_drivable) == 0:
            return -1, -1
        return where_drivable[0], where_drivable[-1]
    
    def _calculate_midpoint(self, left: int, right: int) -> int:
        """Calculate midpoint between left and right bounds."""
        return left + (right - left) // 2
    
    # def _get_depth_at_point(self, depth_map, x: int, y: int) -> float:
    #     """Get depth value at given point using ZED depth map."""
    #     try:
    #         if 0 <= y < depth_map.get_height() and 0 <= x < depth_map.get_width():
    #             depth = depth_map.get_value(x, y)[1]  # ZED depth value
    #             if np.isfinite(depth):
    #                 return depth
    #         return 0.0  # Return 0 if depth is invalid or out of bounds
    #     except Exception as e:
    #         print(f"Error getting depth at point ({x}, {y}): {e}")
    #         return 0.0

    def _get_depth_at_point(self, depth_map, x: int, y: int) -> float:
        """Get depth value at given point using ZED depth map."""
        try:
            # Convert coordinates to integers
            x, y = int(x), int(y)
            
            # Check if point is within bounds
            if 0 <= y < depth_map.get_height() and 0 <= x < depth_map.get_width():
                # Get depth value and confidence
                err, depth_value = depth_map.get_value(x, y)
                
                # Check if depth measurement is valid
                if err == sl.ERROR_CODE.SUCCESS and depth_value > 0 and depth_value < 40:  # 40m max range
                    return float(depth_value)
                    
            # Sample neighboring points if direct measurement fails
            sample_radius = 3
            valid_depths = []
            
            for dx in range(-sample_radius, sample_radius + 1):
                for dy in range(-sample_radius, sample_radius + 1):
                    sample_x, sample_y = x + dx, y + dy
                    if 0 <= sample_y < depth_map.get_height() and 0 <= sample_x < depth_map.get_width():
                        err, sample_depth = depth_map.get_value(sample_x, sample_y)
                        if err == sl.ERROR_CODE.SUCCESS and sample_depth > 0 and sample_depth < 40:
                            valid_depths.append(sample_depth)
            
            # Return median of valid neighboring depths if any exist
            if valid_depths:
                return float(np.median(valid_depths))
                
            return 0.0  # Return 0 if no valid depth measurements found
        except Exception as e:
            print(f"Error getting depth at point ({x}, {y}): {e}")
            return 0.0
    
    def extract_waypoints(self, 
                         da_seg_mask: np.ndarray, 
                         depth_map,
                         image_height: int,
                         image_width: int) -> List[Waypoint]:
        """
        Extract waypoints from drivable area mask with distances from ZED depth map.
        """
        waypoints = []
        
        # Define sampling rows (from bottom to top)
        row_positions = np.linspace(
            image_height - 50,  # Start 50 pixels from bottom
            image_height // 2,  # End at middle of image
            self.num_points
        ).astype(int)
        
        # Get depth for each waypoint
        for row in row_positions:
            left, right = self._find_drivable_region_bounds(da_seg_mask, row)
            
            if left != -1 and right != -1:
                mid_x = self._calculate_midpoint(left, right)
                
                # Sample multiple points around the midpoint for more robust depth estimation
                sample_radius = 5
                depths = []
                for dx in range(-sample_radius, sample_radius + 1):
                    for dy in range(-sample_radius, sample_radius + 1):
                        sample_x = mid_x + dx
                        sample_y = row + dy
                        depth = self._get_depth_at_point(depth_map, sample_x, sample_y)
                        if depth > 0:  # Only consider valid depths
                            depths.append(depth)
                
                # Use median depth if available, otherwise use single point measurement
                if depths:
                    final_depth = float(np.median(depths))
                else:
                    final_depth = self._get_depth_at_point(depth_map, mid_x, row)
                
                waypoints.append(Waypoint(
                    x=float(mid_x),
                    y=float(row),
                    distance=final_depth
                ))
        
        # Apply smoothing if we have previous waypoints
        if self.previous_waypoints and len(waypoints) == len(self.previous_waypoints):
            for i in range(len(waypoints)):
                waypoints[i].x = (self.smoothing_factor * waypoints[i].x + 
                                (1 - self.smoothing_factor) * self.previous_waypoints[i].x)
                waypoints[i].y = (self.smoothing_factor * waypoints[i].y + 
                                (1 - self.smoothing_factor) * self.previous_waypoints[i].y)
                waypoints[i].distance = (self.smoothing_factor * waypoints[i].distance + 
                                      (1 - self.smoothing_factor) * self.previous_waypoints[i].distance)
        
        # Calculate heading angles
        if len(waypoints) >= 2:
            for i in range(len(waypoints) - 1):
                dx = waypoints[i + 1].x - waypoints[i].x
                dy = waypoints[i + 1].y - waypoints[i].y
                waypoints[i].heading = np.arctan2(dy, dx)
            
            # Last waypoint keeps the same heading as the previous one
            if waypoints:
                waypoints[-1].heading = waypoints[-2].heading if len(waypoints) > 1 else 0
        
        self.previous_waypoints = waypoints
        return waypoints
    
    def visualize_waypoints(self, 
                          image: np.ndarray, 
                          waypoints: List[Waypoint],
                          color: Tuple[int, int, int] = (0, 255, 255)) -> np.ndarray:
        """
        Visualize waypoints with actual distances from ZED depth map.
        """
        vis_img = image.copy()
        
        # Draw waypoints
        for i, wp in enumerate(waypoints):
            # Draw point
            cv2.circle(vis_img, (int(wp.x), int(wp.y)), 5, color, -1)
            
            # Draw distance text with actual depth value
            cv2.putText(vis_img, f'{wp.distance:.2f}m', 
                       (int(wp.x) + 10, int(wp.y)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw heading arrow if available
            if wp.heading is not None:
                arrow_length = 30
                end_x = int(wp.x + arrow_length * np.cos(wp.heading))
                end_y = int(wp.y + arrow_length * np.sin(wp.heading))
                cv2.arrowedLine(vis_img, (int(wp.x), int(wp.y)), 
                              (end_x, end_y), color, 2)
        
        # Draw connecting lines between waypoints
        if len(waypoints) >= 2:
            for i in range(len(waypoints) - 1):
                pt1 = (int(waypoints[i].x), int(waypoints[i].y))
                pt2 = (int(waypoints[i + 1].x), int(waypoints[i + 1].y))
                cv2.line(vis_img, pt1, pt2, color, 2)
        
        return vis_img