# Chapter 3: Nav2 Path Planning for Humanoid Robots

## Introduction

Navigation2 (Nav2) is the official ROS 2 navigation stack that provides comprehensive path planning, localization, and navigation capabilities for mobile robots. While originally designed for wheeled robots, Nav2 can be adapted for humanoid robots with specific modifications to account for bipedal locomotion, balance constraints, and unique kinematic properties.

In this chapter, you'll learn how to:
- Adapt Nav2 for humanoid robot navigation
- Implement custom path planners for bipedal locomotion
- Create footstep planners for stable walking
- Integrate balance control with navigation systems
- Configure Nav2 parameters specifically for humanoid robots

## Understanding Nav2 Architecture

### Core Components
Nav2 consists of several key components that work together:
- **Global Planner**: Creates high-level path from start to goal
- **Local Planner**: Handles real-time obstacle avoidance and path following
- **Controller**: Sends velocity commands to robot base
- **Costmap**: Maintains obstacle and free space information
- **Recovery Behaviors**: Handles navigation failures

### Behavior Trees
Nav2 uses behavior trees for navigation logic, allowing:
- Flexible navigation strategies
- Conditional execution of navigation tasks
- Easy customization of navigation behaviors
- Robust failure handling and recovery

## Nav2 for Humanoid Robots: Key Differences

### Bipedal Locomotion Challenges
Humanoid robots present unique challenges compared to wheeled robots:
- **Balance Constraints**: Must maintain center of mass over support polygon
- **Foot Placement**: Path planning must consider specific footstep locations
- **Gait Patterns**: Different walking styles for various terrains and speeds
- **Turning Mechanisms**: Different from differential drive robots
- **Dynamic Stability**: Balance changes during movement

### Humanoid-Specific Requirements
- **Footstep Planning**: Generate stable footstep sequences
- **Balance Control Integration**: Coordinate with balance controllers
- **Terrain Adaptation**: Handle stairs, slopes, and uneven terrain
- **Human Interaction**: Navigate safely around humans
- **Energy Efficiency**: Optimize for battery-powered operation

## Setting Up Nav2 for Humanoid Robots

### Installation
```bash
# Install Nav2 packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install additional packages for humanoid-specific features
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
```

### Basic Configuration
Create a configuration file for humanoid-specific Nav2 parameters:

```yaml
# humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "nav2_bt_navigator/navigate_through_poses_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_cleared_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Humanoid-specific controller parameters
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      angular_dist_threshold: 0.785
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.0
      max_angular_accel: 3.2
      goal_tolerance: 0.1

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3  # Adjust based on humanoid robot dimensions
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3  # Adjust based on humanoid robot dimensions
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      w_smooth: 0.9
      w_data: 0.1

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors/BackUp"
      backup_dist: 0.15
      backup_speed: 0.025
    wait:
      plugin: "nav2_behaviors/Wait"
      wait_duration: 1.0
```

## Custom Path Planners for Humanoid Robots

### Footstep Planning Integration
Humanoid robots require specialized path planning that considers foot placement:

```cpp
// Example C++ code for a humanoid-specific path planner
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace nav2_humanoid_planner
{

class HumanoidGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  HumanoidGlobalPlanner() = default;
  ~HumanoidGlobalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    RCLCPP_INFO(node_->get_logger(), "Configured humanoid global planner");
  }

  void cleanup() override
  {
    RCLCPP_INFO(node_->get_logger(), "Cleaning up humanoid global planner");
  }

  void activate() override
  {
    RCLCPP_INFO(node_->get_logger(), "Activating humanoid global planner");
  }

  void deactivate() override
  {
    RCLCPP_INFO(node_->get_logger(), "Deactivating humanoid global planner");
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    nav_msgs::msg::Path path = nav_msgs::msg::Path();
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();

    // Check if start and goal are valid
    if (isGoalUnreachable(start, goal)) {
      return path;
    }

    // For humanoid robots, we need to consider:
    // 1. Balance constraints
    // 2. Foot placement requirements
    // 3. Gait patterns
    // 4. Terrain walkability

    // Generate path considering humanoid-specific constraints
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    if (computeHumanoidPath(start, goal, poses)) {
      path.poses = poses;
    }

    return path;
  }

private:
  bool isGoalUnreachable(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    // Check if goal is in collision or unreachable
    unsigned int start_index, goal_index;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_index) ||
        !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_index)) {
      return true;
    }

    unsigned char start_cost = costmap_->getCost(start_index);
    unsigned char goal_cost = costmap_->getCost(goal_index);

    // Consider costs that would make navigation impossible for humanoid
    if (start_cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        goal_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      return true;
    }

    return false;
  }

  bool computeHumanoidPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    // Implement path planning algorithm that considers:
    // - Balance constraints (center of mass)
    // - Foot placement requirements
    // - Walkable terrain analysis
    // - Step height and width constraints

    // This is a simplified example - a real implementation would include
    // sophisticated footstep planning and balance checking

    // For now, use a basic A* planner with humanoid-specific cost adjustments
    // (In practice, you'd integrate with a footstep planner like SBPL)

    // Add start pose
    poses.push_back(start);

    // Add intermediate poses that consider humanoid constraints
    // This would involve calling a footstep planner
    // and generating poses that maintain balance

    // Add goal pose
    poses.push_back(goal);

    return true;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;
};

} // namespace nav2_humanoid_planner

// Register this planner as a plugin
PLUGINLIB_EXPORT_CLASS(nav2_humanoid_planner::HumanoidGlobalPlanner, nav2_core::GlobalPlanner)
```

### Humanoid-Specific Local Planner
For local planning, humanoid robots need to consider balance and footstep constraints:

```python
# Example Python code for humanoid local planner integration
import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

class HumanoidLocalPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_local_planner')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for current robot state
        self.robot_state_sub = self.create_subscription(
            # Robot state message type (depends on your humanoid)
            # For example: sensor_msgs/JointState, or custom humanoid state
            # Replace with appropriate message type
            None,  # Placeholder - define based on your humanoid
            'robot_state',
            self.robot_state_callback,
            10
        )

        # Action server for following paths
        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            self.execute_follow_path
        )

        # Parameters for humanoid-specific navigation
        self.step_size = self.declare_parameter('step_size', 0.3).value
        self.max_step_height = self.declare_parameter('max_step_height', 0.1).value
        self.balance_threshold = self.declare_parameter('balance_threshold', 0.05).value

    def robot_state_callback(self, msg):
        # Update internal state based on robot feedback
        # This would include joint angles, IMU data, etc.
        pass

    def execute_follow_path(self, goal_handle):
        path = goal_handle.request.path
        self.get_logger().info(f'Following path with {len(path.poses)} poses')

        # Convert global path to footstep plan
        footsteps = self.generate_footsteps(path)

        # Execute footsteps while maintaining balance
        for i, footstep in enumerate(footsteps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowPath.Result()

            # Execute single step
            success = self.execute_step(footstep)

            if not success:
                self.get_logger().error('Failed to execute step')
                goal_handle.abort()
                return FollowPath.Result()

        goal_handle.succeed()
        return FollowPath.Result()

    def generate_footsteps(self, path):
        # Generate footstep plan from global path
        # This would integrate with a footstep planner like SBPL or other
        # humanoid-specific planning algorithms
        footsteps = []

        # Simplified example - in practice, this would be much more complex
        for i in range(len(path.poses) - 1):
            start_pose = path.poses[i]
            end_pose = path.poses[i + 1]

            # Calculate intermediate footsteps between poses
            # considering step size constraints
            distance = self.calculate_distance(start_pose, end_pose)
            num_steps = int(distance / self.step_size) + 1

            for j in range(num_steps):
                step_pose = self.interpolate_pose(start_pose, end_pose, j / num_steps)
                footsteps.append(step_pose)

        return footsteps

    def execute_step(self, footstep):
        # Execute a single step while maintaining balance
        # This would interface with the humanoid's walking controller
        # and balance system

        # Example: move to next footstep position
        cmd_vel = Twist()
        # Calculate appropriate velocity to reach footstep
        # while maintaining balance

        self.cmd_vel_pub.publish(cmd_vel)

        # Wait for step completion
        # Check balance constraints
        # Return success/failure

        return True  # Simplified - would check actual success

    def calculate_distance(self, pose1, pose2):
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def interpolate_pose(self, start, end, t):
        # Linear interpolation between two poses
        result = PoseStamped()
        result.header = start.header

        result.pose.position.x = start.pose.position.x + t * (end.pose.position.x - start.pose.position.x)
        result.pose.position.y = start.pose.position.y + t * (end.pose.position.y - start.pose.position.y)
        result.pose.position.z = start.pose.position.z + t * (end.pose.position.z - start.pose.position.z)

        # For orientation, you might want to use quaternion slerp
        # Simplified here:
        result.pose.orientation = start.pose.orientation  # Keep start orientation

        return result

def main(args=None):
    rclpy.init(args=args)
    planner = HumanoidLocalPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Balance Control Integration

### Balance-Aware Navigation
Humanoid robots must maintain balance during navigation:

```yaml
# balance_controller_params.yaml
balance_controller:
  ros__parameters:
    use_sim_time: True
    control_frequency: 100.0  # Higher frequency for balance control

    # Center of mass control
    com_control:
      kp: 10.0
      ki: 1.0
      kd: 1.0
      max_force: 500.0  # Newtons

    # ZMP (Zero Moment Point) control
    zmp_control:
      kp: 5.0
      ki: 0.5
      kd: 0.1
      support_polygon_margin: 0.05  # meters

    # Joint position control for balance
    joint_control:
      hip_pitch_kp: 50.0
      hip_pitch_kd: 5.0
      ankle_pitch_kp: 80.0
      ankle_pitch_kd: 8.0
      hip_roll_kp: 60.0
      hip_roll_kd: 6.0
      ankle_roll_kp: 90.0
      ankle_roll_kd: 9.0
```

## Launching Humanoid Navigation

### Complete Launch File
Create a launch file that brings together all components:

```python
# humanoid_nav2_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Paths
    bringup_dir = FindPackageShare('nav2_bringup')
    params_path = PathJoinSubstitution([
        FindPackageShare('humanoid_navigation'),
        'config',
        'humanoid_nav2_params.yaml'
    ])

    # Launch Nav2
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bringup_dir,
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_path,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn
        }.items()
    )

    # Launch Humanoid-Specific Nodes
    humanoid_planner_node = Node(
        package='humanoid_navigation',
        executable='humanoid_global_planner',
        name='humanoid_global_planner',
        parameters=[params_path, {'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )

    balance_controller_node = Node(
        package='humanoid_balance',
        executable='balance_controller',
        name='balance_controller',
        parameters=[PathJoinSubstitution([
            FindPackageShare('humanoid_balance'),
            'config',
            'balance_controller_params.yaml'
        ])],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_path,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='false',
            description='Whether to launch nodes in composition'),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Whether to respawn if a node crashes'),

        # Launch components
        nav2_bringup_launch,
        humanoid_planner_node,
        balance_controller_node
    ])
```

## Simulation Integration

### Testing in Isaac Sim
To test your humanoid navigation system in Isaac Sim:

```python
# isaac_sim_humanoid_nav.py
import omni
from omni.isaac.kit import SimulationApp

# Simulation configuration
config = {
    "headless": False,
    "render": "core",
    "livesync": True,
    "width": 1280,
    "height": 720
}
simulation_app = SimulationApp(config)

# Import Isaac Sim modules
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.viewports import set_camera_view

# Import ROS bridge
from omni.isaac.ros_bridge import _ros_bridge
import carb

# Initialize ROS bridge
ros_bridge = _ros_bridge.acquire_ros_bridge_interface()

# Create world
world = World(stage_units_in_meters=1.0)

# Get assets
assets_root_path = get_assets_root_path()
if assets_root_path:
    # Add environment
    room_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
    add_reference_to_stage(usd_path=room_path, prim_path="/World/SimpleRoom")

    # Add humanoid robot (using a simple model for this example)
    # In practice, you would use a detailed humanoid model
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd",
        prim_path="/World/MyRobot"
    )

# Set up camera view
set_camera_view(eye=[2.5, 2.5, 2.5], target=[0, 0, 0.5])

# Reset world
world.reset()

# Main simulation loop
while simulation_app.is_running():
    # Step simulation
    world.step(render=True)

    # Process ROS messages
    ros_bridge.publish_ros_transform_tree("/World/MyRobot", "base_link")

    # Check for navigation commands via ROS
    # Process navigation goals
    # Update robot position based on navigation system

# Shutdown
simulation_app.close()
```

## Performance Optimization

### Tuning Navigation Parameters
For optimal humanoid navigation performance:

1. **Costmap Resolution**: Adjust resolution based on step size
2. **Inflation Radius**: Set appropriately for humanoid foot size
3. **Controller Frequency**: Balance between responsiveness and computation
4. **Tolerance Values**: Set realistic values for humanoid movement precision

### Hardware Considerations
- Ensure sufficient computational resources for real-time balance control
- Consider dedicated hardware for balance control (if needed)
- Optimize perception pipeline to reduce navigation latency

## Troubleshooting Common Issues

### Navigation Failures
- **Path Planning**: Verify costmap inflation settings
- **Balance Issues**: Check balance controller parameters
- **Footstep Planning**: Validate step size constraints
- **Sensor Integration**: Ensure proper TF frames and timing

### Simulation vs Real World
- **Dynamics**: Account for differences in simulation vs real dynamics
- **Sensors**: Ensure sensor models match real hardware
- **Timing**: Consider real-time constraints in simulation

## Best Practices

### Safety Considerations
- Implement emergency stop mechanisms
- Set appropriate velocity limits
- Include balance failure detection
- Plan for graceful degradation

### Testing Strategy
- Test in simulation first
- Validate on simplified scenarios
- Gradually increase complexity
- Test edge cases and failure modes

## Summary

In this chapter, you learned how to adapt Nav2 for humanoid robot navigation, including:
- Understanding the unique challenges of humanoid navigation
- Configuring Nav2 with humanoid-specific parameters
- Implementing custom path planners for bipedal locomotion
- Integrating balance control with navigation systems
- Testing navigation systems in simulation

You now have the knowledge to implement a complete navigation system for humanoid robots using Nav2, Isaac ROS, and Isaac Sim. The combination of these technologies provides a powerful platform for developing advanced humanoid robot navigation capabilities.