---
sidebar_position: 4
---

# Humanoid Robot Models in Gazebo

This section covers how to create and configure humanoid robot models for physics simulation in Gazebo. Humanoid robots have unique characteristics that require special attention in simulation.

## Humanoid Robot Characteristics

Humanoid robots have several key features that differentiate them from other robot types:
- **Bipedal locomotion**: Two legs for walking and balance
- **Multi-joint structure**: Many degrees of freedom for human-like movement
- **Balance requirements**: Need active balance control
- **Anthropomorphic design**: Human-like proportions and capabilities

## Model Description Formats

### URDF (Unified Robot Description Format)
URDF is the most common format for ROS-based robots:
```xml
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### SDF (Simulation Description Format)
SDF is Gazebo's native format and can include simulation-specific elements:
```xml
<sdf version="1.7">
  <model name="humanoid_robot">
    <pose>0 0 1 0 0 0</pose>
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 0.1 0.1</size>
          </box>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.1 0.1</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

## Key Components for Humanoid Models

### 1. Links
Each rigid body in the robot:
- **Torso**: Main body section
- **Head**: For vision and interaction
- **Arms**: For manipulation tasks
- **Legs**: For locomotion
- **Feet**: For ground contact

### 2. Joints
Connections between links:
- **Revolute**: Rotational joints (most common for humanoid)
- **Fixed**: Rigid connections
- **Prismatic**: Linear motion (rarely used)

### 3. Inertial Properties
Critical for realistic physics simulation:
- **Mass**: Total mass of each link
- **Center of Mass**: Location of mass center
- **Inertia Tensor**: Resistance to rotational motion

## Humanoid-Specific Considerations

### Balance and Stability
- Accurate center of mass positioning
- Proper mass distribution
- Appropriate foot geometry for contact

### Joint Configuration
- Realistic joint limits
- Appropriate actuator parameters
- Proper joint ordering and naming

### Collision Geometry
- Simplified geometry for performance
- Accurate contact surfaces for feet
- Appropriate padding to prevent interpenetration

## Example Humanoid Model Structure

```
humanoid_robot/
├── urdf/
│   └── humanoid.urdf.xacro
├── meshes/
│   ├── torso.dae
│   ├── head.dae
│   ├── arm.dae
│   └── leg.dae
├── config/
│   └── controllers.yaml
└── launch/
    └── gazebo.launch.py
```

## Best Practices for Humanoid Models

1. **Accurate Inertial Properties**: Use CAD software to calculate precise inertial properties
2. **Proper Scaling**: Ensure correct proportions for realistic simulation
3. **Simplified Collision Geometry**: Use simple shapes for collision to improve performance
4. **Realistic Joint Limits**: Set limits based on real robot capabilities
5. **Gait-Appropriate Design**: Consider how the model will be used for walking/locomotion

## Exercise: Create a Simple Humanoid Model

Create a basic humanoid model with:
- A torso link
- Head, arms, and legs
- Appropriate joints connecting the links
- Basic inertial properties
- Simple collision and visual geometry

## Summary

Humanoid robot models require special attention to balance, stability, and locomotion requirements. Proper configuration of links, joints, and inertial properties is essential for realistic physics simulation. Following best practices ensures stable and accurate simulation results for humanoid robotics applications.