"""
Role: Serve as local path planning to navigate to a goal.

Functionality:  Receives goal in float form
                Receives obstacle list as np.ndarray[list]
"""

import math
from typing import Tuple

import numpy as np
from decision_making_node import Obstacle
from dwa_cost_calculator import DWACostCalculations


class Trajectory:
    def __init__(self, points: list[Tuple[float, float, float]]):
        """
        Initialize a trajectory with a list of points.

        Args:
            points: List of points representing the trajectory
        """
        self.points = points


class DWA_planner:
    def __init__(
        self,
        current_state: list,
        goal: Tuple[float, float],
        obstacles: list[Tuple[float, float]],
        wheel_base: float = 0.5,
    ):
        self.current_state = current_state
        self.goal = goal
        self.obstacles = obstacles
        self.wheel_base = wheel_base

    def plan(self, current_state: list, goal: Tuple[float, float], obstacles: np.ndarray) -> tuple:
        """
        Plan a trajectory from the current state to the goal while avoiding obstacles.

        Args:
            current_state: [x, y, theta, vx, vy, omega]
            goal: [x, y] target position
            obstacles: PointCloud2 message containing obstacle points

        Returns:
            [vx, vy, omega] velocity command

        Code Flow:
            1. Extract current position and velcoity from current_state.
            2. Create velocity window based on current velocities.
            3. Generate possible trajectories within the velocity window.
            4. Evaluate each trajectory and remove ones with collisions.
            5. Calculate costs for remaining trajectories.
            6. Select the trajectory with the lowest cost.
            7. Return velocity command that will be published in decision_processing_node.py
        """

        # Step 1: Extract current position and velocities
        current_position = current_state[0:3]
        current_velocity = current_state[3:6]

        # Step 2: Create velocity window based on current velocities
        velocity_window = self.get_velocity_window(current_velocity)

        # Step 3: Generate possible trajectories within the velocity window
        trajectories = self.generate_trajectories(current_position, velocity_window)

        # Step 4: Evaluate each trajectory and remove ones with collisions
        safe_trajectories = self.remove_collisions(trajectories, obstacles)

        if not safe_trajectories:
            # If no safe trajectories, return zero velocity command
            return 0, 0

        # Step 5: Calculate costs for remaining trajectories and store in dict Trajectory -> cost
        costs: dict[Trajectory, float] = {}
        for trajectory in safe_trajectories:
            cost: float = self.calculate_cost(trajectory, current_state, goal, obstacles)
            costs[trajectory] = cost

        # Step 6: Select the trajectory with the lowest cost
        best_trajectory = costs.index(min(costs.values()))

        if best_trajectory is None:
            # If no best trajectory found, return zero velocity command
            return 0.0, 0.0

        # Step 7: Convert the best trajectory to velocity command
        vel_command = self.trajectory_to_velocity_command(best_trajectory)

        return vel_command

    def get_velocity_window(self, current_vel: list) -> dict:
        """
        Create a velocity window based on current velocities.

        Args:
            current_vel: [vx, vy, omega] current velocities

        Returns:
            {'vx': (min, max), 'vy': (min, max), 'omega': (min, max)}
        """
        # Get current velocities
        vx, vy, omega = current_vel

        # Define velocity limits based on robot configuration
        max_accel = 1.5
        max_omega = 1.0

        vx_limits = (vx - max_accel, vx + max_accel)
        vy_limits = (vy - max_accel, vy + max_accel)
        omega_limits = (omega - max_omega, omega + max_omega)

        return {"vx": vx_limits, "vy": vy_limits, "omega": omega_limits}

    def generate_trajectories(self, current_position: list, velocity_window: dict) -> list:
        """
        Generate possible trajectories within the velocity window.

        Args:
            current_position: [x, y, theta] current position
            velocity_window: {'vx': (min, max), 'vy': (min, max), 'omega': (min, max)}

        Returns:
            List of Trajectory objects
        """
        trajectories: list = []
        vx_range = np.linspace(velocity_window["vx"][0], velocity_window["vx"][1], num=10)
        vy_range = np.linspace(velocity_window["vy"][0], velocity_window["vy"][1], num=10)
        omega_range = np.linspace(velocity_window["omega"][0], velocity_window["omega"][1], num=10)

        for vx in vx_range:
            for vy in vy_range:
                for omega in omega_range:
                    trajectory = self.create_trajectory(current_position, vx, vy, omega)
                    trajectories.append(trajectory)

        return trajectories

    def create_trajectory(
        self, current_position: list, vx: float, vy: float, omega: float
    ) -> Trajectory:
        """
        Create a trajectory based on current position and velocities.

        Args:
            current_position: [x, y, theta] current position
            vx: Linear velocity in x direction
            vy: Linear velocity in y direction
            omega: Angular velocity

        Returns:
            Trajectory object
        """
        # Placeholder for trajectory points generation logic
        trajectory_points: list[Tuple[float, float, float]] = []
        for t in np.linspace(0, 1, num=10):  # Generate 10 points along the trajectory
            x: float = current_position[0] + vx * t
            y: float = current_position[1] + vy * t
            theta: float = current_position[2] + omega * t
            trajectory_points.append((x, y, theta))
        return Trajectory(points=trajectory_points)

    def remove_collisions(self, trajectories: list, obstacles: np.ndarray) -> list:
        """
        Evaluate each trajectory and remove ones with collisions.

        Args:
            trajectories: List of Trajectory objects
            obstacles: PointCloud2 message containing obstacle points

        Returns:
            List of safe Trajectory objects
        """
        safe_trajectories: list = []
        for trajectory in trajectories:
            for point in trajectory.points:
                # If this point collides with an obstacle, remove trajectory
                if self.is_collision(point, obstacles):
                    break
            else:
                # If no collision was detected, keep the trajectory
                safe_trajectories.append(trajectory)

        return safe_trajectories

    def is_collision(self, point: list, obstacles: np.ndarray) -> bool:
        """
        Check if a point collides with any obstacle.

        Args:
            point: [x, y, theta] point to check for collision
            obstacles: PointCloud2 message containing obstacle points

        Returns:
            True if point collides with an obstacle, False otherwise
        """

        for obstacle in obstacles:
            if self.is_point_in_obstacle(point, obstacle):
                return True
        return False

    def is_point_in_obstacle(self, point: list, obstacle: Obstacle) -> bool:
        """
        Check if a point is within the radius of an obstacle.

        Args:
            point: [x, y] coordinates of the point
            obstacle: Obstacle object

        Returns:
            True if the point +- the rovers radius is within the obstacle's radius, False otherwise
        """
        distance = np.sqrt(
            (point[0] - obstacle.position[0]) ** 2 + (point[1] - obstacle.position[1]) ** 2
        )
        return distance <= obstacle.radius + self.robot_radius

    def calculate_cost(
        self,
        trajectory: Trajectory,
        current_state: list,
        goal: Tuple[float, float],
        obstacles: np.ndarray,
    ) -> float:
        """
        Calculates the cost of each trajectory returns in float form.
        If trajectory does not reach goal, return cost of 1

        Args:
            trajectory (Trajectory): projected trajectory of rover
            goal (float): goal for the rover to navigate to
            obstalces (np.array): array of list of obstacle points

        Returns:
            float: _description_
        """

        cost = 0.0

        weights: dict = {}
        cost_calculator = DWACostCalculations()

        cost = cost_calculator.calculate_total_cost(
            trajectory_points=trajectory.points,
            velocity=Tuple[current_state[0:3]],
            goal=goal,
            obstacles=obstacles,
            weights=weights,
        )

        # If end pos in trajectory is not goal, return 1.0
        if trajectory.points[9] is not goal:
            return 1.0

        return cost

    def trajectory_to_velocity_command(
        self, trajectory: Trajectory, dt: float = 0.1
    ) -> tuple[float, float]:
        """
        Convert a Trajectory object to differential drive velocity commands.

        This method extracts velocity information from trajectory points and converts
        it to left/right wheel velocities for differential drive control.

        Args:
            trajectory: Trajectory object containing points (x, y, theta)
            dt: Time step between trajectory points (default 0.1 seconds)

        Returns:
            (left_wheel_vel, right_wheel_vel) in m/s
        """
        if not trajectory.points or len(trajectory.points) < 2:
            return 0.0, 0.0

        # Get velocities from the first two points in the trajectory
        # This represents the immediate velocity the robot should follow
        p1 = trajectory.points[0]
        p2 = trajectory.points[1]

        # Calculate linear velocities
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dtheta = p2[2] - p1[2]

        # Normalize angle difference to [-pi, pi]
        while dtheta > math.pi:
            dtheta -= 2 * math.pi
        while dtheta < -math.pi:
            dtheta += 2 * math.pi

        # Convert to velocities
        vx = dx / dt
        vy = dy / dt
        omega = dtheta / dt

        # Convert to differential drive commands
        return self.dwa_holonomic_to_differential(vx, vy, omega)

    def dwa_holonomic_to_differential(
        self, vx: float, vy: float, omega: float
    ) -> tuple[float, float]:
        """
        Convert holonomic DWA output to differential drive (with vy consideration)

        This version combines vx and vy into a single forward velocity,
        useful if your DWA planner assumes holonomic motion but your robot is differential

        Args:
            vx: Linear velocity in x direction (m/s)
            vy: Linear velocity in y direction (m/s)
            omega: Angular velocity (rad/s)

        Returns:
            (left_wheel_vel, right_wheel_vel) in m/s
        """
        # Option 1: Just use magnitude of velocity vector
        v_linear = math.sqrt(vx**2 + vy**2)

        # Option 2: Project onto forward direction based on current motion
        # (uncomment if you want to maintain direction)
        # if vx < 0:  # Moving backward
        #     v_linear = -v_linear

        # Apply differential drive kinematics
        left_wheel_vel = v_linear - (omega * self.wheel_base / 2.0)
        right_wheel_vel = v_linear + (omega * self.wheel_base / 2.0)

        return left_wheel_vel, right_wheel_vel
