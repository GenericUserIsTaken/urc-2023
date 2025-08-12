import math
from typing import Any, Optional, Tuple

import numpy as np
from numpy.typing import NDArray


class Trajectory:
    def __init__(
        self,
        init_vel: Optional[Tuple[float, float]] = None,
        points: Optional[list[Tuple[float, float]]] = None,
        cost: Optional[int] = None,
        velocity: Optional[Tuple[float, float]] = None,
    ):
        self.points = points if points is not None else []
        self.init_vel = init_vel if points is not None else 0.0, 0.0
        self.cost = cost if cost is not None else 1000
        self.velocity = velocity if velocity is not None else 0.0, 0.0

    def append_point(self, point: Tuple[float, float]) -> bool:
        """Appends a point to the trajectory"""
        if len(self.points) < 10:
            self.points.append(point)
            return True

        return False

    def set_velocity(self, velocity: Tuple[float, float]) -> None:
        """Sets the velocity of the trajectory"""
        self.velocity = velocity

    def set_cost(self, cost: int) -> None:
        """Sets the cost of the trajectory"""
        self.cost = cost

    def get_cost(self) -> int:
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

    def create_velocity_command(
        self,
        initial_pos: Tuple[float, float],
        initial_theta: float,
        initial_vel: Tuple[float, float],
        target_pos: Tuple[float, float],
        wheel_base_radius: float,
    ) -> Tuple[float, float]:
        """
        Creates a velocity command (linear v, angular ω) to move from initial position and orientation towards the target position.
        This is a simplified controller based on forward kinematics principles, using proportional control for alignment and speed.
        Assumes differential drive; wheel_base_radius is half the wheelbase (used if converting to wheel speeds, but here we return (v, ω)).
        Tune Kp_angular and Kp_linear as needed.
        """
        # Unpack inputs
        x0, y0 = initial_pos
        x1, y1 = target_pos
        v_current, ω_current = initial_vel  # Can use for smoothing if desired

        # Compute relative position
        dx = x1 - x0
        dy = y1 - y0
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.01:  # Threshold for arrival
            return (0.0, 0.0)  # Stop

        # Desired heading
        desired_theta = math.atan2(dy, dx)

        # Angular error (normalize to -pi to pi)
        angular_error = desired_theta - initial_theta
        angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi

        # Proportional controllers (tune these gains)
        Kp_angular = 1.0  # For angular_velocity
        Kp_linear = 0.5  # For v, scaled by distance

        # Compute angular velocity
        angular_velocity = Kp_angular * angular_error

        # Clamp ω based on max (example limits; adjust)
        max_angular_velocity = 1.0  # rad/s
        angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)

        # Compute linear velocity (reduce when turning sharply)
        v = Kp_linear * distance * (1 - abs(angular_error) / math.pi)  # Dampen v during large turns

        # Clamp v
        max_v = 0.5  # m/s
        v = max(min(v, max_v), 0.0)  # Assume no reverse for simplicity

        # Optional: Smooth with current vel (e.g., low-pass filter)
        alpha = 0.8  # Smoothing factor
        v = alpha * v + (1 - alpha) * v_current
        angular_velocity = alpha * angular_velocity + (1 - alpha) * ω_current

        # Note: If you need wheel velocities instead, use:
        # left = v - ω * wheel_base_radius
        # right = v + ω * wheel_base_radius
        # return (left, right)
        # But based on signature, returning (v, ω)
        return (v, angular_velocity)


class DWAPlanner:
    def __init__(
        self,
        costmap: NDArray[np.int8],
        robot_radius: float,
        current_velocity: Tuple[float, float],
        current_position: Tuple[float, float],
        time_delta: float,
        goal: Any,
        theta: float,
    ):
        """
        Initialize the DWA planner with robot parameters.

        Args:
            robot_radius: Radius of the robot.
            max_speed: Maximum speed of the robot.
            max_acceleration: Maximum acceleration of the robot.
        """

        # State Variables
        self.costmap = costmap
        self.robot_radius = robot_radius
        self.current_vel = current_velocity
        self.current_pos = current_position
        self.time_delta = time_delta
        self.goal = goal
        self.theta = theta

        # Physical limits
        self.min_linear_vel: float = 0.0
        self.max_linear_vel: float = 1.0  # 15.7 radians per second
        self.max_linear_accel: float = 1.0
        self.max_angular_vel: float = 1.0  # radians per second
        self.max_angular_accel: float = 1.0  # radians per second

    def plan(self) -> Tuple[float, float]:
        """
        Main method for local navigation.

        Uses Dynamic Window Approach logic to navigate through 2D costmap from ROS2.

        Code Flow:
            1. Extract current position and velcoity from current_state.
            2. Create velocity window based on current velocities.
            3. Generate possible trajectories within the velocity window.
            4. Evaluate each trajectory and remove ones with collisions.
            5. Return velocity command that will be published in decision_processing_node.py
        """

        velocity_command: Tuple[float, float] = 0, 0

        # Generate velocity window
        velocity_window: list[Tuple[float, float]] = self.generate_vel_window(
            self.max_linear_accel, self.max_linear_vel
        )

        # Generate trajectories based on velocity window
        trajectories: list[Trajectory] = []
        trajectories = self.generate_trajectories(
            velocity_window, self.time_delta, self.theta, self.current_pos
        )

        # Evaluate all trajectories and determine what to remove
        for trajectory in trajectories:
            if not trajectory.is_valid(self.costmap):
                trajectories.remove(trajectory)

        # Sort Trajectory objects by cost
        trajectories.sort(key=lambda traj: traj.get_cost(), reverse=True)

        return velocity_command

    def generate_vel_window(
        self, max_vel_delta: float, max_vel: float
    ) -> list[Tuple[float, float]]:
        vel_window: list[Tuple[float, float]] = []

        current_velocity_left, current_velocity_right = self.current_vel
        vel_index: int = 0

        max_left = min(current_velocity_left + max_vel_delta, max_vel)
        max_right = min(current_velocity_right + max_vel_delta, max_vel)

        for l_vel in np.arange(
            current_velocity_left - max_vel_delta,
            max_left,
            (max_vel_delta / 2),
        ):
            # For each left velocity, create a velocity for each right
            for r_vel in np.arange(
                current_velocity_right - max_vel_delta,
                max_right,
                (max_vel_delta / 2),
            ):
                vel_window[vel_index] = float(l_vel), float(r_vel)
                vel_index += 1

        return vel_window

    def generate_trajectories(
        self,
        vel_window: list[Tuple[float, float]],
        time_delta: float,
        current_heading: float,
        current_pos: Tuple[float, float],
    ) -> list[Trajectory]:
        """
        Generate possible trajectories based on the velocity window.
        1. left changes while right stays the same
        2. right cycles thru vel window while left stays the same
            Will create several duplicate of both velocities being the same. Must delete one of each

        Args:
            velocity_window: List of velocities to consider.

        Returns:
            List of generated trajectories.
        """
        trajectories: list[Trajectory] = []

        # For each velocity, create a trajectory
        for vel in vel_window:
            prev_l, prev_r = current_pos
            l_vel = vel[0]
            r_vel = vel[1]
            # Create 10 points in trajectory
            trajectory: Trajectory = Trajectory(vel)
            for i in range(10):
                new_l, new_r = prev_l + l_vel * time_delta, prev_r + r_vel * time_delta
                trajectory.append_point((new_l, new_r))
                trajectory.set_velocity((l_vel, r_vel))
                prev_l, prev_r = new_l, new_r

            trajectories.append(trajectory)

        return trajectories

    def output_trajectory(self, trajectory: Trajectory) -> Trajectory:
        """
        Outputs the trajectory to the appropriate interface (e.g., ROS topic, log file).
        """
        return trajectory

    def calculate_dynamic_window(
        self, current_linear_vel: float, current_angular_vel: float, dt: float
    ) -> Tuple[float, float, float, float]:
        """
        Calculate velocities reachable within one timestep.
        Returns: (min_v, max_v, min_w, max_w)
        """
        # Dynamic constraints (what's reachable given acceleration limits)
        min_v = max(current_linear_vel - self.max_linear_accel * dt, self.min_linear_vel)
        max_v = min(current_linear_vel + self.max_linear_accel * dt, self.max_linear_vel)
        min_w = max(current_angular_vel - self.max_angular_accel * dt, -self.max_angular_vel)
        max_w = min(current_angular_vel + self.max_angular_accel * dt, self.max_angular_vel)

        return (min_v, max_v, min_w, max_w)
