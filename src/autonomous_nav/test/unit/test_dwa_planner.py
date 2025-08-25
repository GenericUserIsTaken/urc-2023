from typing import Tuple

import numpy as np
import pytest

from autonomous_nav.dwa_planner import DWAPlanner


class TestDWACalculator:
    def create_planner(self) -> DWAPlanner:
        return DWAPlanner(
            costmap=np.zeros((100, 100), dtype=np.int8),
            robot_radius=0.25,
            current_velocity=(2.0, 3.0),
            current_position=(0.0, 0.0),
            time_delta=0.1,
            goal=(1.0, 1.0),
            theta=0.0,
        )

    def test_wheel_to_robot_velocities(self) -> None:
        DWAPlanner = self.create_planner()
        velocities: Tuple[float, float] = DWAPlanner.wheel_to_robot_velocities((2.0, 3.0))
        assert velocities == (0.225, 0.44)
