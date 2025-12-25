---
sidebar_position: 5
---

# Running Physics-Based Simulations

This section covers how to launch and run physics-based simulations for humanoid robots in Gazebo, including launch files, control interfaces, and simulation monitoring.

## Launching Gazebo with Your Robot

### Using ROS 2 Launch Files

Create a launch file to spawn your humanoid robot in Gazebo:

```python
# launch/humanoid_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='humanoid.urdf.xacro',
        description='Robot description file'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': PathJoinSubstitution([
                FindPackageShare('humanoid_description'),
                'urdf',
                LaunchConfiguration('model')
            ])}
        ]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Add actions to launch description
    ld.add_action(model_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld
```

## Gazebo Control Interface

### Joint State Controller
Monitor joint positions, velocities, and efforts:

```yaml
# config/joint_state_controller.yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_controller:
      type: joint_state_controller/JointStateController

joint_state_controller:
  ros__parameters:
    joints:
      - hip_joint
      - knee_joint
      - ankle_joint
      - shoulder_joint
      - elbow_joint
```

### Joint Trajectory Controller
Control joint positions with trajectories:

```yaml
# config/joint_trajectory_controller.yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - hip_joint
      - knee_joint
      - ankle_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

## Simulation Monitoring

### Real-Time Visualization
Use RViz2 to monitor simulation:
- Robot state visualization
- Sensor data display
- Path planning visualization
- Control feedback monitoring

### Performance Metrics
Monitor simulation performance:
- Real-time factor (RTF)
- Update rates
- CPU usage
- Memory consumption

### Physics Validation
Check physics simulation quality:
- Joint position accuracy
- Balance stability
- Collision response
- Contact forces

## Common Simulation Issues and Solutions

### 1. Robot Falls Over Immediately
**Causes**:
- Incorrect initial pose
- Bad inertial properties
- Physics parameter issues

**Solutions**:
- Check initial joint positions
- Verify inertial properties
- Adjust physics parameters

### 2. Joints Behaving Erratically
**Causes**:
- High control gains
- Bad joint limits
- Physics instability

**Solutions**:
- Reduce control gains
- Check joint limits
- Improve physics configuration

### 3. Slow Simulation Performance
**Causes**:
- Complex collision geometry
- High update rates
- Large world size

**Solutions**:
- Simplify collision models
- Adjust update rates
- Optimize world complexity

## Exercise: Launch Your Humanoid Simulation

1. Create a launch file for your humanoid model
2. Configure basic controllers
3. Launch the simulation
4. Monitor the robot's initial behavior
5. Document any issues and solutions

## Best Practices

1. **Incremental Testing**: Start with simple models and gradually add complexity
2. **Parameter Validation**: Verify all model parameters before simulation
3. **Controller Tuning**: Start with conservative control parameters
4. **Performance Monitoring**: Keep track of real-time factor during development
5. **Logging**: Enable appropriate logging for debugging

## Summary

Running physics-based simulations requires proper launch configuration, control interfaces, and monitoring. Understanding how to set up and troubleshoot simulation issues is crucial for successful humanoid robot development. Following best practices ensures stable and efficient simulation runs.