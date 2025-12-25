---
sidebar_position: 2
---

# Setting Up Simulation Worlds

This section covers how to create and configure simulation environments in Gazebo for humanoid robotics applications.

## Creating a Basic World

A Gazebo world file defines the environment where your robot will operate. World files are written in SDF (Simulation Description Format) and include:
- Physical properties (gravity, magnetic field)
- Models (robots, objects, environment)
- Plugins (controllers, sensors)
- Scene properties (lighting, sky)

### Basic World Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- World properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

## Essential World Components

### 1. Ground Plane
Every humanoid simulation needs a ground plane for the robot to stand and walk on:
```xml
<include>
  <uri>model://ground_plane</uri>
</include>
```

### 2. Lighting
Proper lighting is essential for sensor simulation:
```xml
<include>
  <uri>model://sun</uri>
</include>
```

### 3. Physics Engine Configuration
Configure the physics engine for humanoid dynamics:
```xml
<physics type="ode">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

## Creating Custom Environments

For humanoid robotics, you may want to create custom environments:

### Indoor Environments
- Rooms with furniture
- Corridors and doorways
- Stairs and ramps

### Outdoor Environments
- Parks with obstacles
- Urban environments
- Terrain with varying elevation

## Best Practices for Humanoid Worlds

1. **Collision Geometry**: Use simplified collision geometry for better performance
2. **Visual Fidelity**: Separate visual and collision models for optimization
3. **Physics Tuning**: Adjust physics parameters for stable humanoid simulation
4. **Environment Complexity**: Balance realism with simulation performance

## Exercise: Create Your First Humanoid World

Create a simple world file that includes:
- A ground plane
- Proper lighting
- Physics configuration suitable for humanoid simulation
- A basic humanoid robot model (to be added in the next section)

## Summary

Setting up simulation worlds is the foundation of humanoid robotics simulation. The world provides the environment where your robot will interact, and proper configuration is essential for realistic physics simulation.