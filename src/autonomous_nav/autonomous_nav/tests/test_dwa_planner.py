from typing import List, Optional, Tuple

import pytest
from dwa_planner import DWAPlanner, Trajectory
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid


def test_generate_trajectory(dwa_planner: DWAPlanner) -> Optional[Trajectory]:
    """Test trajectory generation."""
    trajectory = dwa_planner.generate_trajectory()
    return trajectory


def test_generate_dynamic_window(
    dwa_planner: DWAPlanner,
) -> Optional[Tuple[float, float, float, float]]:
    """Test dynamic window generation."""
    dynamic_window = dwa_planner.calculate_dynamic_window(dwa_planner.current_wheel_vel)
    return dynamic_window


def main() -> None:
    """Creates a DWAPlanner object and a mock costmap to test trajectory generation."""
    mock_occupancy_grid = OccupancyGrid()
    mock_costmap: PyCostmap2D = PyCostmap2D(
        mock_occupancy_grid
    )  # Replace with actual mock costmap if needed
    mock_waypoints: List[Tuple[float, float]] = []  # Replace with actual mock waypoints if needed
    dwa_planner = DWAPlanner(mock_costmap, 0.7, (1.0, 1.0), (0.0, 0.0), 0.1, mock_waypoints[1], 0.0)

    # Test methods
    trajectory: Trajectory | None = test_generate_trajectory(dwa_planner)
    if trajectory is not None:
        print(f"Generated trajectory with {len(trajectory.points)} points.")

    # velocity_window: Tuple[float, float, float, float] | None = test_generate_dynamic_window(
    #     dwa_planner
    # )
    # if velocity_window is not None:
    #     print("Dynamic window generation test passed.")
    #     print(f"Velocity window: {velocity_window}")


if __name__ == "__main__":
    main()
