import math
from typing import Any, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray


class Trajectory:
    def __init__(
        self,
        linear_vel: float = 0.0,
        angular_vel: float = 0.0,
        points: Optional[List[Tuple[float, float, float]]] = None,
    ):
        """
        Initialize a trajectory.
        Points are stored as (x, y, theta) tuples.
        """
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        self.points = points if points is not None else []
        self.cost = float("inf")

    def add_point(self, x: float, y: float, theta: float) -> None:
        """Add a point to the trajectory."""
        self.points.append((x, y, theta))

    def set_velocity(self, velocity: Tuple[float, float]) -> None:
        """Sets the velocity of the trajectory"""
        self.velocity = velocity

    def set_cost(self, cost: int) -> None:
        """Sets the cost of the trajectory"""
        self.cost = cost

    def get_cost(self) -> float:
        """Returns the cost of the trajectory"""
        return self.cost

    def is_valid(self, occupancy_grid: NDArray[np.int8]) -> bool:
        """
        Checks if trajectory object should be removed by checking against points in grid
        Returns false if there is a collision.
        """
        total_cost: int = 0  # use python int to avoid int8 overflow during accumulation
        for point in self.points:
            x, y = int(point[0]), int(point[1])
            cost: np.int8 = occupancy_grid[x, y]
            total_cost += int(cost)
            if cost > np.int8(100):
                return False

        self.set_cost(total_cost)

        return True


class DWAPlanner:
    def __init__(
        self,
        costmap: NDArray[np.int8],
        robot_radius: float,
        current_velocity: Tuple[float, float],  # (left_wheel, right_wheel) velocities
        current_position: Tuple[float, float],  # (x, y) position
        time_delta: float,
        goal: Tuple[float, float],
        theta: float,  # current yaw/heading
    ):
        """
        Initialize the DWA planner with robot parameters.
        """
        # State Variables
        self.costmap = costmap
        self.robot_radius = robot_radius
        self.current_wheel_vel = current_velocity  # (left, right) wheel velocities
        self.current_pos = current_position
        self.time_delta = time_delta
        self.goal = goal
        self.current_theta = theta

        # Robot physical parameters (adjust these to match your rover)
        self.wheel_base = 0.5  # Distance between left and right wheels in meters
        self.wheel_radius = 0.11  # Radius of wheels in meters
    
        # Velocity limits (in m/s and rad/s)
        self.max_linear_vel = 1.0  # Maximum forward velocity
        self.min_linear_vel = -0.5  # Maximum backward velocity (negative)
        self.max_angular_vel = 1.0  # Maximum rotation rate

        # Acceleration limits (in m/s² and rad/s²)
        self.max_linear_accel = 1.0
        self.max_angular_accel = 1.0

        # DWA parameters
        self.prediction_time = 3.0  # How far ahead to simulate trajectories
        self.linear_vel_samples = 11  # Number of linear velocity samples
        self.angular_vel_samples = 21  # Number of angular velocity samples

        # Cost weights (tune these for desired behavior)
        self.weight_goal_heading = 1.0
        self.weight_distance = 0.8
        self.weight_velocity = 0.2
        self.weight_obstacle = 2.0

    def plan(self) -> Tuple[float, float]:
        """
        Main planning method - generates trajectories and returns best wheel velocities.

        Returns:
            (left_wheel_vel, right_wheel_vel) in rad/s
        """

        # Convert current wheel velocities to robot velocities
        current_velocities: Tuple[float, float] = self.wheel_to_robot_velocities(
            self.current_wheel_vel
        )

        # Generate dynamic window to create velocity samples
        dynamic_window = self.calculate_dynamic_window(current_velocities)

        # Get velocity samples
        velocity_samples = self.generate_velocity_samples(dynamic_window)

        # Generate all trajectory candidates
        trajectories = self.generate_trajectories(velocity_samples)

        if not trajectories:
            # Emergency stop if no trajectories generated
            return (0.0, 0.0)

        # TODO: Evaluate trajectories and select best one
        # For now, return first valid trajectory as example
        best_trajectory = trajectories[0]

        # Convert the best trajectory's velocities back to wheel velocities
        left_wheel, right_wheel = self.robot_to_wheel_velocities(
            best_trajectory.linear_vel, best_trajectory.angular_vel
        )

        return (left_wheel / 15.7, right_wheel / 15.7)

    def generate_trajectories(
        self, velocity_samples: List[Tuple[float, float]]
    ) -> List[Trajectory]:
        """
        Generate all trajectory candidates within the dynamic window.

        Returns:
            List of Trajectory objects
        """
        trajectories = []

        # Generate trajectory for each velocity sample
        for linear_vel, angular_vel in velocity_samples:
            trajectory = self.simulate_trajectory(linear_vel, angular_vel)
            trajectories.append(trajectory)

        return trajectories

    def wheel_to_robot_velocities(self, wheel_vels: Tuple[float, float]) -> Tuple[float, float]:
        """
        Convert wheel velocities to robot velocities.

        Args:
            wheel_vels: (left_wheel_vel, right_wheel_vel) in rad/s

        Returns:
            (linear_velocity, angular_velocity) in m/s and rad/s
        """
        left_vel, right_vel = wheel_vels

        # Convert wheel angular velocities to linear velocities
        left_linear = left_vel * self.wheel_radius
        right_linear = right_vel * self.wheel_radius

        # Calculate robot velocities using differential drive kinematics
        linear_vel = (right_linear + left_linear) / 2.0
        angular_vel = (right_linear - left_linear) / self.wheel_base

        return (linear_vel, angular_vel)

    def robot_to_wheel_velocities(
        self, linear_vel: float, angular_vel: float
    ) -> Tuple[float, float]:
        """
        Convert robot velocities to wheel velocities.

        Args:
            linear_vel: Forward velocity in m/s
            angular_vel: Angular velocity in rad/s

        Returns:
            (left_wheel_vel, right_wheel_vel) in rad/s
        """
        # Calculate wheel linear velocities
        left_linear = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_linear = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Convert to wheel angular velocities
        left_wheel = left_linear / self.wheel_radius
        right_wheel = right_linear / self.wheel_radius

        return (left_wheel, right_wheel)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-pi, pi] range.

        Args:
            angle: Angle in radians

        Returns:
            Normalized angle in [-pi, pi]
        """
        while angle >= math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def simulate_trajectory(self, linear_vel: float, angular_vel: float) -> Trajectory:
        """
        Simulate a trajectory for given velocities using differential drive kinematics.

        Args:
            linear_vel: Forward velocity in m/s
            angular_vel: Angular velocity in rad/s

        Returns:
            Trajectory object containing predicted path
        """
        trajectory = Trajectory(linear_vel, angular_vel)

        # Start from current state
        x, y = self.current_pos
        theta = self.current_theta

        # Add starting point
        trajectory.add_point(x, y, theta)

        # Number of simulation steps
        num_steps = int(self.prediction_time / self.time_delta)

        # Simulate forward using differential drive kinematics
        for _ in range(num_steps):
            if abs(angular_vel) < 0.001:  # Nearly straight motion
                # Use simplified model for straight motion
                x_new = x + linear_vel * self.time_delta * math.cos(theta)
                y_new = y + linear_vel * self.time_delta * math.sin(theta)
                theta_new = theta
            else:  # Curved motion
                # Calculate radius of curvature
                radius = linear_vel / angular_vel

                # Calculate new position using arc motion
                x_new = x + radius * (
                    math.sin(theta + angular_vel * self.time_delta) - math.sin(theta)
                )
                y_new = y - radius * (
                    math.cos(theta + angular_vel * self.time_delta) - math.cos(theta)
                )
                theta_new = theta + angular_vel * self.time_delta

            # Normalize theta to [-pi, pi]
            theta_new = self.normalize_angle(theta_new)

            # Update state for next iteration
            x, y, theta = x_new, y_new, theta_new

            # Add point to trajectory
            trajectory.add_point(x, y, theta)

        return trajectory

    def calculate_dynamic_window(
        self, current_velocities: Tuple[float, float]
    ) -> Tuple[float, float, float, float]:
        """
        Calculate the dynamic window based on current velocities and acceleration limits.

        Returns:
            (min_linear_vel, max_linear_vel, min_angular_vel, max_angular_vel)
        """
        # Calculate reachable velocities within one time step
        # Linear velocity window

        current_linear, current_angular = current_velocities

        min_linear = max(
            current_linear - self.max_linear_accel * self.time_delta, self.min_linear_vel
        )
        max_linear = min(
            current_linear + self.max_linear_accel * self.time_delta, self.max_linear_vel
        )

        # Angular velocity window
        min_angular = max(
            current_angular - self.max_angular_accel * self.time_delta, -self.max_angular_vel
        )
        max_angular = min(
            current_angular + self.max_angular_accel * self.time_delta, self.max_angular_vel
        )

        return (min_linear, max_linear, min_angular, max_angular)

    def generate_velocity_samples(
        self, dynamic_window: Tuple[float, float, float, float]
    ) -> List[Tuple[float, float]]:
        """
        Generate velocity samples within the dynamic window.

        Returns:
            List of (linear_vel, angular_vel) tuples to evaluate
        """
        # Get dynamic window bounds
        min_v, max_v, min_w, max_w = dynamic_window

        velocity_samples = []

        # Sample velocities uniformly within the window
        if max_v > min_v:
            linear_samples = np.linspace(min_v, max_v, self.linear_vel_samples)
        else:
            linear_samples = np.array([min_v])

        if max_w > min_w:
            angular_samples = np.linspace(min_w, max_w, self.angular_vel_samples)
        else:
            angular_samples = np.array([max_w])

        # Create all combinations
        for v in linear_samples:
            for w in angular_samples:
                velocity_samples.append((v, w))

        return velocity_samples
