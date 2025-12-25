# Quickstart: Module 3: The AI-Robot Brain (NVIDIA Isaac™)

## Overview
This quickstart guide provides step-by-step instructions for setting up the NVIDIA Isaac ecosystem for humanoid robotics development. You'll learn how to install Isaac Sim, Isaac ROS, and configure Nav2 for humanoid path planning.

## Prerequisites
- Ubuntu 22.04 LTS
- NVIDIA GPU with RTX capabilities (RTX 3060 or better recommended)
- NVIDIA drivers installed (version 535 or later)
- CUDA 11.8 or later
- Docker and Docker Compose
- ROS 2 Humble Hawksbill installed

## Step 1: Install NVIDIA Isaac Sim

### 1.1 System Preparation
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install NVIDIA drivers (if not already installed)
sudo apt install nvidia-driver-535

# Install CUDA toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /"
sudo apt update
sudo apt -y install cuda-toolkit-11-8
```

### 1.2 Install Isaac Sim
```bash
# Create workspace directory
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws

# Install Isaac Sim using Omniverse Launcher
# Download from NVIDIA Developer website and run the installer
# Follow the installation wizard to complete the setup

# Launch Isaac Sim
isaac-sim.sh
```

### 1.3 Verify Installation
```bash
# Check Isaac Sim version
isaac-sim --version

# Test basic simulation
python3 -c "import omni; print('Isaac Sim Python API available')"
```

## Step 2: Install Isaac ROS

### 2.1 Install ROS 2 Humble
```bash
# Set up ROS 2 repositories
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-ros-base ros-humble-isaac-ros-common
```

### 2.2 Install Isaac ROS Packages
```bash
# Install Isaac ROS perception packages
sudo apt install ros-humble-isaac-ros-image-pipeline
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-apriltag
sudo apt install ros-humble-isaac-ros-detection2d

# Install Isaac ROS navigation packages
sudo apt install ros-humble-isaac-ros-navigation
sudo apt install ros-humble-isaac-ros-manipulation
```

### 2.3 Source ROS 2 Environment
```bash
# Add to ~/.bashrc for persistent sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 3: Install and Configure Nav2

### 3.1 Install Nav2
```bash
# Install Nav2 packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install additional dependencies for humanoid navigation
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
```

### 3.2 Set up Humanoid-Specific Navigation Configuration
```bash
# Create navigation configuration directory
mkdir -p ~/isaac_ws/src/humanoid_nav2_config
cd ~/isaac_ws/src/humanoid_nav2_config

# Create basic configuration files (these will be expanded in the full tutorial)
mkdir -p config maps launch

# Create basic Nav2 parameters for humanoid robot
cat > config/nav2_params.yaml << EOF
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
EOF
```

## Step 4: Create Your First Isaac Simulation

### 4.1 Launch Isaac Sim with a Humanoid Robot
```bash
# Create a simple test script
cat > ~/isaac_ws/test_humanoid_sim.py << EOF
import omni
from omni.isaac.kit import SimulationApp

# Initialize Isaac Sim
config = {
    "headless": False,
    "render": "core",
    "livesync": True,
    "width": 1280,
    "height": 720
}
simulation_app = SimulationApp(config)

# Import necessary modules
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create world instance
world = World(stage_units_in_meters=1.0)

# Add a simple humanoid robot (example using a basic rig)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print("Could not find Isaac Sim assets. Please enable Isaac Sim Nucleus.")
else:
    # Add a simple environment
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd",
        prim_path="/World/SimpleRoom"
    )

# Reset the world
world.reset()

# Run simulation for a few steps
for i in range(100):
    world.step(render=True)

# Shutdown
simulation_app.close()
EOF

# Run the test
python3 ~/isaac_ws/test_humanoid_sim.py
```

## Step 5: Test Isaac ROS Integration

### 5.1 Launch Isaac ROS Perception Pipeline
```bash
# Terminal 1: Launch a simple ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# Launch Isaac ROS image pipeline demo
ros2 launch isaac_ros_image_pipeline isaac_ros_image_pipeline.launch.py
```

### 5.2 Verify Pipeline Operation
```bash
# Terminal 2: Check topics
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# List active topics
ros2 topic list

# Check image topics
ros2 topic echo /image_raw --field data
```

## Step 6: Test Navigation Setup

### 6.1 Launch Nav2 in Simulation
```bash
# Terminal 1: Launch Nav2 bringup
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# Launch Nav2 with a simple map
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=~/isaac_ws/src/humanoid_nav2_config/config/nav2_params.yaml
```

### 6.2 Send Navigation Goal
```bash
# Terminal 2: Send a simple navigation goal
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# Use Nav2 tools to send a goal
ros2 run nav2_test_nav goal_pose.py --ros-args -p x:=1.0 -p y:=1.0 -p z:=0.0 -p w:=1.0
```

## Troubleshooting

### Common Issues and Solutions

1. **Isaac Sim fails to launch**:
   - Ensure NVIDIA drivers are properly installed
   - Check that CUDA is correctly configured
   - Verify GPU is RTX-compatible

2. **ROS 2 packages not found**:
   - Verify ROS 2 Humble is installed
   - Ensure you've sourced the ROS 2 environment
   - Check that Isaac ROS packages are properly installed

3. **GPU acceleration not working**:
   - Confirm CUDA installation
   - Verify Isaac ROS GPU-accelerated packages are installed
   - Check nvidia-smi for GPU availability

## Next Steps

After completing this quickstart, you'll be ready to dive deeper into:
- Chapter 1: NVIDIA Isaac Sim for photorealistic simulation
- Chapter 2: Isaac ROS for VSLAM and navigation
- Chapter 3: Nav2 path planning for humanoid robots

Each chapter will expand on these concepts with detailed tutorials and practical examples.