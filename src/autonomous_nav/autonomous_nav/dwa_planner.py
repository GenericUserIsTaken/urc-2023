class DWAPlanner:
    def __init__(self, robot_config: RobotConfig, 
                 dwa_config: DWAConfig, 
                 cost_weights: CostWeights):
        """
        Initialize DWA planner with configuration
        
        PSEUDO CODE:
        - Store robot_config as self.robot_config
        - Store dwa_config as self.dwa_config  
        - Store cost_weights as self.cost_weights
        - Initialize any helper data structures
        """
        
    def plan(self, current_state: list, goal: list, obstacles: np.ndarray) -> list:
        """
        Main planning method
        
        Args:
            current_state: [x, y, theta, vx, vy, omega]
            goal: [x, y] target position
            obstacles: Nx3 array of [x, y, z] points
            
        Returns:
            [vx, vy, omega] velocity command
            
        PSEUDO CODE:
        # Extract current position and velocity
        current_pos = current_state[0:3]  # x, y, theta
        current_vel = current_state[3:6]  # vx, vy, omega
        
        # Generate feasible velocity window
        vel_window = generate_velocity_window(current_vel)
        
        # Generate and evaluate trajectories
        trajectories = generate_trajectories(current_state, vel_window)
        
        # Remove trajectories that collide
        valid_trajectories = []
        FOR each trajectory in trajectories:
            IF obstacle_clearance(trajectory.path_points, obstacles):
                valid_trajectories.append(trajectory)
        
        # If no valid trajectories, return stop command
        IF valid_trajectories is empty:
            RETURN [0, 0, 0]
            
        # Evaluate costs for valid trajectories
        best_trajectory = None
        min_cost = infinity
        
        FOR each trajectory in valid_trajectories:
            cost = evaluate_trajectory(trajectory, goal, obstacles)
            IF cost < min_cost:
                min_cost = cost
                best_trajectory = trajectory
                
        RETURN best_trajectory.velocity_command
        """
        
    def generate_velocity_window(self, current_vel: list) -> dict:
        """
        Calculate dynamic window based on current velocity and constraints
        
        Args:
            current_vel: [vx, vy, omega]
            
        Returns:
            {'vx': (min, max), 'vy': (min, max), 'omega': (min, max)}
            
        PSEUDO CODE:
        # Get current velocities
        vx, vy, omega = current_vel
        
        # Calculate dynamic window based on acceleration limits
        dt = self.dwa_config.dt
        
        # Linear x velocity window
        vx_min = max(vx - self.robot_config.max_acc_x * dt, 
                     -self.robot_config.max_vel_x)
        vx_max = min(vx + self.robot_config.max_acc_x * dt,
                     self.robot_config.max_vel_x)
                     
        # Linear y velocity window (for holonomic robots)
        vy_min = max(vy - self.robot_config.max_acc_y * dt,
                     -self.robot_config.max_vel_y)
        vy_max = min(vy + self.robot_config.max_acc_y * dt,
                     self.robot_config.max_vel_y)
                     
        # Angular velocity window
        omega_min = max(omega - self.robot_config.max_acc_theta * dt,
                       -self.robot_config.max_vel_theta)
        omega_max = min(omega + self.robot_config.max_acc_theta * dt,
                       self.robot_config.max_vel_theta)
                       
        RETURN {
            'vx': (vx_min, vx_max),
            'vy': (vy_min, vy_max),
            'omega': (omega_min, omega_max)
        }
        """
        
    def generate_trajectories(self, current_state: list, vel_window: dict) -> list:
        """
        Generate candidate trajectories from velocity samples
        
        Args:
            current_state: [x, y, theta, vx, vy, omega]
            vel_window: velocity ranges
            
        Returns:
            List of Trajectory objects
            
        PSEUDO CODE:
        trajectories = []
        
        # Sample velocities from the window
        vx_min, vx_max = vel_window['vx']
        vy_min, vy_max = vel_window['vy']
        omega_min, omega_max = vel_window['omega']
        
        # Create velocity samples
        vx_samples = linspace(vx_min, vx_max, self.dwa_config.vx_samples)
        vy_samples = linspace(vy_min, vy_max, self.dwa_config.vy_samples)
        omega_samples = linspace(omega_min, omega_max, self.dwa_config.omega_samples)
        
        # Generate trajectory for each velocity combination
        FOR vx in vx_samples:
            FOR vy in vy_samples:
                FOR omega in omega_samples:
                    velocity = [vx, vy, omega]
                    path = predict_trajectory(current_state, velocity)
                    trajectory = Trajectory(velocity, path, 0.0)
                    trajectories.append(trajectory)
                    
        RETURN trajectories
        """
        
    def predict_trajectory(self, state: list, velocity: list) -> list:
        """
        Forward simulate trajectory for given velocity
        
        Args:
            state: [x, y, theta, vx, vy, omega]
            velocity: [vx, vy, omega] to simulate
            
        Returns:
            List of [x, y, theta] points
            
        PSEUDO CODE:
        trajectory = []
        
        # Initialize state
        x, y, theta = state[0:3]
        vx, vy, omega = velocity
        
        # Simulate for configured time
        time = 0
        dt = self.dwa_config.sim_time_step
        
        WHILE time < self.dwa_config.sim_time:
            # Add current position to trajectory
            trajectory.append([x, y, theta])
            
            # Update position based on motion model
            IF self.robot_config.wheel_base == 0:  # Holonomic
                # Simple integration
                x = x + vx * cos(theta) * dt - vy * sin(theta) * dt
                y = y + vx * sin(theta) * dt + vy * cos(theta) * dt
                theta = theta + omega * dt
            ELSE:  # Non-holonomic (differential drive)
                # Use bicycle model
                IF abs(omega) < 0.001:  # Straight line
                    x = x + vx * cos(theta) * dt
                    y = y + vx * sin(theta) * dt
                ELSE:  # Arc
                    radius = vx / omega
                    x = x + radius * (sin(theta + omega*dt) - sin(theta))
                    y = y + radius * (cos(theta) - cos(theta + omega*dt))
                    theta = theta + omega * dt
                    
            # Normalize theta to [-pi, pi]
            theta = normalize_angle(theta)
            
            time = time + dt
            
        RETURN trajectory
        """
        
    def evaluate_trajectory(self, trajectory: Trajectory, goal: list, 
                          obstacles: np.ndarray) -> float:
        """
        Calculate total cost for a trajectory
        
        Args:
            trajectory: Trajectory object to evaluate
            goal: [x, y] target position
            obstacles: Nx3 array of obstacle points
            
        Returns:
            Total cost (lower is better)
            
        PSEUDO CODE:
        # Get individual cost components
        h_cost = heading_cost(trajectory, goal)
        d_cost = distance_cost(trajectory, obstacles)
        v_cost = velocity_cost(trajectory.velocity_command)
        
        # Weighted sum
        total_cost = (self.cost_weights.heading_weight * h_cost +
                     self.cost_weights.distance_weight * d_cost +
                     self.cost_weights.velocity_weight * v_cost)
                     
        RETURN total_cost
        """
        
    def heading_cost(self, trajectory: Trajectory, goal: list) -> float:
        """
        Cost based on heading alignment with goal
        
        Args:
            trajectory: Trajectory to evaluate
            goal: [x, y] target
            
        Returns:
            Cost value (0-1)
            
        PSEUDO CODE:
        # Get final position from trajectory
        final_x, final_y, final_theta = trajectory.path_points[-1]
        
        # Calculate angle to goal
        dx = goal[0] - final_x
        dy = goal[1] - final_y
        angle_to_goal = atan2(dy, dx)
        
        # Calculate heading difference
        heading_diff = abs(angle_to_goal - final_theta)
        heading_diff = normalize_angle(heading_diff)
        
        # Normalize to 0-1 (0 is aligned, 1 is opposite)
        cost = abs(heading_diff) / pi
        
        RETURN cost
        """
        
    def distance_cost(self, trajectory: Trajectory, obstacles: np.ndarray) -> float:
        """
        Cost based on distance to obstacles
        
        Args:
            trajectory: Trajectory to evaluate
            obstacles: Nx3 array of obstacle points
            
        Returns:
            Cost value (0-1)
            
        PSEUDO CODE:
        IF obstacles is empty:
            RETURN 0.0
            
        min_distance = infinity
        
        # Find minimum distance along trajectory
        FOR point in trajectory.path_points:
            x, y, _ = point
            
            # Calculate distances to all obstacles
            distances = sqrt((obstacles[:, 0] - x)^2 + 
                           (obstacles[:, 1] - y)^2)
            
            # Track minimum
            point_min_dist = min(distances)
            IF point_min_dist < min_distance:
                min_distance = point_min_dist
                
        # Convert to cost (closer = higher cost)
        # Use exponential decay
        safe_distance = self.robot_config.robot_radius * 2
        IF min_distance < safe_distance:
            cost = 1.0 - exp(-2 * (safe_distance - min_distance))
        ELSE:
            cost = 0.0
            
        RETURN cost
        """
        
    def velocity_cost(self, velocity: list) -> float:
        """
        Cost based on forward velocity (prefer faster)
        
        Args:
            velocity: [vx, vy, omega]
            
        Returns:
            Cost value (0-1)
            
        PSEUDO CODE:
        vx, vy, omega = velocity
        
        # Calculate linear speed
        linear_speed = sqrt(vx^2 + vy^2)
        
        # Normalize by maximum possible speed
        max_speed = sqrt(self.robot_config.max_vel_x^2 + 
                        self.robot_config.max_vel_y^2)
        
        # Invert so higher speed = lower cost
        cost = 1.0 - (linear_speed / max_speed)
        
        RETURN cost
        """
        
    def obstacle_clearance(self, path_points: list, obstacles: np.ndarray,
                          clearance_radius: float = None) -> bool:
        """
        Check if trajectory is collision-free
        
        Args:
            path_points: List of [x, y, theta] positions
            obstacles: Nx3 array of obstacle points
            clearance_radius: Minimum clearance (default: robot_radius)
            
        Returns:
            True if trajectory is safe
            
        PSEUDO CODE:
        IF clearance_radius is None:
            clearance_radius = self.robot_config.robot_radius
            
        IF obstacles is empty:
            RETURN True
            
        # Check each point in trajectory
        FOR point in path_points:
            x, y, _ = point
            
            # Calculate distances to all obstacles
            distances = sqrt((obstacles[:, 0] - x)^2 + 
                           (obstacles[:, 1] - y)^2)
            
            # Check if any obstacle is too close
            min_distance = min(distances)
            IF min_distance < clearance_radius:
                RETURN False
                
        RETURN True
        """