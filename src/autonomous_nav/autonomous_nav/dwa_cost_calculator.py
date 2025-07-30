import math
from typing import List, Tuple

import numpy as np


class DWACostCalculations:
    """
    Detailed mathematical calculations for DWA trajectory costs
    """

    def __init__(self, robot_radius: float = 0.3):
        self.robot_radius = robot_radius

    # ==============================================================================
    # 1. HEADING COST - How well aligned is the trajectory with the goal?
    # ==============================================================================
    def heading_cost(
        self, trajectory_end: Tuple[float, float, float], goal: Tuple[float, float]
    ) -> float:
        """
        Calculate heading cost based on angular difference to goal

        Mathematical basis:
        - Calculate angle from trajectory end to goal
        - Compare with robot's final heading
        - Normalize to [0, 1] where 0 = perfectly aligned, 1 = opposite direction

        Args:
            trajectory_end: (x, y, theta) - final pose of trajectory
            goal: (x, y) - goal position

        Returns:
            cost in range [0, 1]
        """
        final_x, final_y, final_theta = trajectory_end
        goal_x, goal_y = goal

        # Step 1: Calculate angle to goal from final position
        dx = goal_x - final_x
        dy = goal_y - final_y

        # Handle case where we're at the goal
        if abs(dx) < 0.001 and abs(dy) < 0.001:
            return 0.0  # At goal, any heading is fine

        angle_to_goal = math.atan2(dy, dx)

        # Step 2: Calculate heading error (angular difference)
        heading_error = angle_to_goal - final_theta

        # Step 3: Normalize angle to [-π, π]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Step 4: Convert to cost [0, 1]
        # cos(heading_error) gives us: 1 when aligned, -1 when opposite
        # We transform this to: 0 when aligned, 1 when opposite
        cost = (1.0 - math.cos(heading_error)) / 2.0

        # Alternative linear scaling (simpler but less smooth):
        # cost = abs(heading_error) / math.pi

        return cost

    # ==============================================================================
    # 2. OBSTACLE/DISTANCE COST - How close does trajectory get to obstacles?
    # ==============================================================================
    def distance_cost(
        self,
        trajectory_points: List[Tuple[float, float, float]],
        obstacles: np.ndarray,
        safety_margin: float = 0.1,
    ) -> float:
        """
        Calculate cost based on proximity to obstacles

        Mathematical approaches:
        1. Inverse distance: cost = 1/distance
        2. Exponential decay: cost = exp(-k*distance)
        3. Threshold-based: high cost if too close, zero otherwise

        Args:
            trajectory_points: List of (x, y, theta) along trajectory
            obstacles: Nx3 array of obstacle points (x, y, z)
            safety_margin: Additional clearance beyond robot radius

        Returns:
            cost in range [0, 1] or higher for very close obstacles
        """
        if len(obstacles) == 0:
            return 0.0

        min_safe_distance = self.robot_radius + safety_margin
        min_distance_found = float("inf")

        # Find minimum distance across entire trajectory
        for point in trajectory_points:
            x, y, _ = point

            # Vectorized distance calculation to all obstacles
            distances = np.sqrt((obstacles[:, 0] - x) ** 2 + (obstacles[:, 1] - y) ** 2)

            closest_obstacle_dist = np.min(distances)
            min_distance_found = min(min_distance_found, closest_obstacle_dist)

        # Convert distance to cost using different methods:

        # Method 1: Exponential decay (smooth, differentiable)
        if min_distance_found < min_safe_distance:
            # Very high cost for collision
            k = 2.0  # Decay rate
            cost = math.exp(k * (min_safe_distance - min_distance_found))
        else:
            # Gradual decay for safe distances
            k = 0.5
            cost = math.exp(-k * (min_distance_found - min_safe_distance))

        # Method 2: Inverse distance with cutoff
        # if min_distance_found < min_safe_distance:
        #     cost = min_safe_distance / (min_distance_found + 0.01)  # Avoid division by zero
        # else:
        #     cost = 0.0

        # Method 3: Piecewise linear
        # if min_distance_found < self.robot_radius:
        #     cost = 10.0  # Collision
        # elif min_distance_found < min_safe_distance:
        #     cost = 1.0 - (min_distance_found - self.robot_radius) / safety_margin
        # else:
        #     cost = max(0.0, 1.0 - (min_distance_found - min_safe_distance) / 2.0)

        return min(cost, 10.0)  # Cap maximum cost

    # ==============================================================================
    # 3. VELOCITY COST - Prefer faster forward motion
    # ==============================================================================
    def velocity_cost(
        self, velocity: Tuple[float, float, float], max_vel_x: float, max_vel_y: float
    ) -> float:
        """
        Calculate cost based on forward velocity (prefer faster motion)

        Mathematical basis:
        - Higher forward speed = lower cost
        - Can penalize backward motion
        - Can penalize excessive rotation

        Args:
            velocity: (vx, vy, omega) - commanded velocity
            max_vel_x: Maximum forward velocity
            max_vel_y: Maximum lateral velocity (0 for diff drive)

        Returns:
            cost in range [0, 1]
        """
        vx, vy, omega = velocity

        # Method 1: Simple normalized forward velocity
        # Negative vx (backward) gets high cost
        if vx < 0:
            cost = 1.0 + abs(vx) / max_vel_x  # Cost > 1 for backward motion
        else:
            cost = 1.0 - (vx / max_vel_x)  # Cost decreases with forward speed

        # Method 2: Consider total linear velocity (for holonomic robots)
        # linear_speed = math.sqrt(vx**2 + vy**2)
        # max_speed = math.sqrt(max_vel_x**2 + max_vel_y**2)
        # cost = 1.0 - (linear_speed / max_speed)

        # Method 3: Penalize pure rotation
        # if abs(vx) < 0.01 and abs(omega) > 0.1:
        #     cost += 0.5  # Additional penalty for spinning in place

        return cost

    # ==============================================================================
    # 4. GOAL DISTANCE COST (Optional) - Distance to goal
    # ==============================================================================
    def goal_distance_cost(
        self, trajectory_end: Tuple[float, float, float], goal: Tuple[float, float]
    ) -> float:
        """
        Calculate cost based on Euclidean distance to goal

        Args:
            trajectory_end: (x, y, theta) - final pose
            goal: (x, y) - goal position

        Returns:
            cost (not normalized)
        """
        final_x, final_y, _ = trajectory_end
        goal_x, goal_y = goal

        distance = math.sqrt((goal_x - final_x) ** 2 + (goal_y - final_y) ** 2)

        # Can normalize by some expected maximum distance
        # max_expected_distance = 10.0
        # cost = distance / max_expected_distance

        return distance

    # ==============================================================================
    # 5. PATH SMOOTHNESS COST (Optional) - Penalize jerky trajectories
    # ==============================================================================
    def smoothness_cost(self, trajectory_points: List[Tuple[float, float, float]]) -> float:
        """
        Calculate cost based on path curvature/smoothness

        Args:
            trajectory_points: List of (x, y, theta) along trajectory

        Returns:
            cost based on total curvature
        """
        if len(trajectory_points) < 3:
            return 0.0

        total_curvature = 0.0

        for i in range(1, len(trajectory_points) - 1):
            # Get three consecutive points
            p1 = trajectory_points[i - 1]
            p2 = trajectory_points[i]
            p3 = trajectory_points[i + 1]

            # Calculate curvature using three points
            curvature = self._calculate_curvature((p1[0], p1[1]), (p2[0], p2[1]), (p3[0], p3[1]))

            total_curvature += abs(curvature)

        # Normalize by number of segments
        avg_curvature = total_curvature / (len(trajectory_points) - 2)

        return avg_curvature

    def _calculate_curvature(
        self, p1: Tuple[float, float], p2: Tuple[float, float], p3: Tuple[float, float]
    ) -> float:
        """
        Calculate curvature through three points using Menger curvature formula

        Curvature = 4 * Area of triangle / (product of side lengths)
        """
        # Calculate triangle area using cross product
        area = abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1])) / 2.0

        # Calculate side lengths
        a = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
        b = math.sqrt((p3[0] - p2[0]) ** 2 + (p3[1] - p2[1]) ** 2)
        c = math.sqrt((p3[0] - p1[0]) ** 2 + (p3[1] - p1[1]) ** 2)

        # Avoid division by zero
        if a * b * c < 0.001:
            return 0.0

        curvature = 4 * area / (a * b * c)
        return curvature

    # ==============================================================================
    # TOTAL COST CALCULATION
    # ==============================================================================
    def calculate_total_cost(
        self,
        trajectory_points: List[Tuple[float, float, float]],
        velocity: Tuple[float, float, float],
        goal: Tuple[float, float],
        obstacles: np.ndarray,
        weights: dict,
    ) -> float:
        """
        Calculate weighted sum of all cost components

        Args:
            trajectory_points: Trajectory to evaluate
            velocity: Commanded velocity
            goal: Goal position
            obstacles: Obstacle points
            weights: Dictionary of weights for each cost component

        Returns:
            Total weighted cost
        """
        # Calculate individual costs
        h_cost = self.heading_cost(trajectory_points[-1], goal)
        d_cost = self.distance_cost(trajectory_points, obstacles)
        v_cost = self.velocity_cost(velocity, max_vel_x=0.5, max_vel_y=0.0)

        # Optional additional costs
        g_cost = self.goal_distance_cost(trajectory_points[-1], goal)
        s_cost = self.smoothness_cost(trajectory_points)

        # Weighted sum
        total = (
            weights.get("heading", 1.0) * h_cost
            + weights.get("distance", 2.0) * d_cost
            + weights.get("velocity", 0.5) * v_cost
            + weights.get("goal", 0.0) * g_cost
            + weights.get("smoothness", 0.0) * s_cost
        )

        return total
