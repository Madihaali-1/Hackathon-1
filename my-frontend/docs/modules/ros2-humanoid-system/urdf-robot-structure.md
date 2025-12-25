---
title: Robot Structure with URDF
sidebar_label: Robot Structure with URDF
---

# Robot Structure with URDF

## Understanding URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical structure of a robot, including links, joints, and their relationships. URDF is essential for:

- Robot simulation
- Robot visualization
- Kinematic analysis
- Collision detection

## URDF Structure for Humanoid Robots

A humanoid robot URDF typically follows this structure:

### Links
Links represent rigid bodies in the robot. For a humanoid robot, these include:
- **Base link**: Usually the pelvis or torso
- **Limb links**: Arms, legs, head
- **End-effector links**: Hands, feet
- **Sensor links**: Camera, IMU mounts

### Joints
Joints connect links and define their motion constraints:
- **Revolute joints**: Rotational joints with limits (e.g., elbow, knee)
- **Continuous joints**: Rotational joints without limits (e.g., head rotation)
- **Prismatic joints**: Linear motion joints (rare in humanoids)
- **Fixed joints**: Rigid connections between links

### Materials and Visual Properties
- **Visual**: How the robot appears in simulation/visualization
- **Collision**: Used for collision detection
- **Inertial**: Mass, center of mass, and inertia properties

## Example URDF Structure for Humanoid

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.3"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Similar definitions for arms, legs, etc. -->
</robot>
```

## Simulation Readiness

To make your URDF simulation-ready, ensure:

1. **Complete kinematic chain**: All links are connected through joints
2. **Proper inertial properties**: Accurate mass, center of mass, and inertia values
3. **Collision geometry**: Appropriate collision models for physics simulation
4. **Transmission definitions**: If using Gazebo simulation
5. **Joint limits**: Properly defined joint limits and effort/velocity constraints

## Visualization and Debugging

### Checking URDF
- Use `check_urdf` tool to validate URDF syntax
- Use `urdf_to_graphiz` to visualize the kinematic tree
- Use RViz to visualize the robot model

### Common Issues
- Missing parent-child relationships
- Incorrect joint axis definitions
- Inconsistent units (meters vs millimeters)
- Invalid mesh file paths

## Advanced URDF Features

### Xacro for Complex Models
Xacro (XML Macros) allows you to:
- Define reusable components
- Use mathematical expressions
- Include other files
- Define parameters

### Gazebo Integration
Add Gazebo-specific tags for simulation:
- Physics parameters
- Sensor definitions
- Plugin configurations
- Material properties

## Best Practices

1. **Start Simple**: Begin with a basic model and add complexity gradually
2. **Use Standard Formats**: Follow ROS conventions for link and joint naming
3. **Validate Regularly**: Check your URDF frequently during development
4. **Document Assumptions**: Include comments about coordinate frames and conventions
5. **Test in Simulation**: Verify your model works in a physics simulator

With a properly defined URDF, you can simulate your humanoid robot, plan trajectories, and visualize its movements in tools like RViz and Gazebo.