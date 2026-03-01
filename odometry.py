"""
Odometry Bug Fixes & Documentation

Issues Identified:
1. Initial Pose Mismatch: The estimated `self.theta` was initialized to 0.523 (pi/6) instead of 0.0 (the ground truth), causing an immediate rotational offset.
2. Angular Scaling Error: `self.conversion_factor` was set to 1.3, which erroneously scaled the integrated angular velocity (omega), causing the estimated heading to drift faster than the actual rotation.
3. Integration Axis Swap and Offset: The `x` integration incorrectly used `math.sin` with an arbitrary phase offset of 0.1, and the `y` integration incorrectly used `math.cos`. For a standard differential drive in a 2D plane with theta=0 along the x-axis, `x` should be updated with `cos` and `y` with `sin`.
4. Missing Angle Normalization: The estimated `self.theta` was not normalized after integration, potentially causing unbounded angle growth (though not the primary drift cause, it's a structural bug).

Reasoning & Diagnosis:
By comparing the ground truth motion integration with the odometry motion integration, the discrepancies were clear. 
- The ground truth correctly updates `x` with `cos(theta)` and `y` with `sin(theta)`. The odometry swapped these and added a 0.1 offset to the angle.
- The ground truth updates `theta` directly with `omega * dt`. The odometry multiplied this by `self.conversion_factor` (1.3), corrupting the heading.
- The initialization for the odometry heading `self.theta` didn't match `gt_theta`.

Corrections Applied:
- Set `self.conversion_factor` to 1.0.
- Initialize `self.theta` to 0.0.
- Update `self.x` using `v * math.cos(self.theta) * dt`.
- Update `self.y` using `v * math.sin(self.theta) * dt`.
- Add `self.theta = normalize_angle(self.theta)` to maintain angle within bounds.
"""

import math
from utils.geometry import normalize_angle


class Odometry:
    def __init__(self):
        # Ground truth pose
        self.gt_x = 5.0
        self.gt_y = 2.5
        self.gt_theta = 0.0
        self.conversion_factor = 1.0  # Corrected from 1.3

        # Estimated pose (odometry)
        self.x = 5.0
        self.y = 2.5
        self.theta = 0.0  # Corrected from 0.523 (pi/6)

    def update(self, v, omega, dt):
        """
        Update ground truth and odometry states
        using commanded linear and angular velocity.
        """

        # --------------------------------
        # Ground truth motion integration
        # --------------------------------
        self.gt_x += v * math.cos(self.gt_theta) * dt
        self.gt_y += v * math.sin(self.gt_theta) * dt
        self.gt_theta += omega * dt
        self.gt_theta = normalize_angle(self.gt_theta)

        # --------------------------------
        # Odometry motion integration
        # --------------------------------
        # Corrected trigonometric functions and removed offsets
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        
        # Heading integration
        self.theta += omega * dt * self.conversion_factor
        self.theta = normalize_angle(self.theta)  # Added missing normalization
