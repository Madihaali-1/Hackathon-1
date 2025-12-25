---
sidebar_position: 3
---

# Physics Parameters for Humanoid Simulation

Understanding and configuring physics parameters is crucial for realistic humanoid robot simulation. This section covers the key parameters that affect how your humanoid robot behaves in simulation.

## Key Physics Parameters

### 1. Gravity
The gravity vector determines the downward force on all objects:
```xml
<gravity>0 0 -9.8</gravity>
```

For humanoid simulation:
- Standard Earth gravity (9.8 m/s²) is typically appropriate
- Small variations can be used for testing different environments

### 2. Time Step
The time step controls simulation accuracy and stability:
```xml
<max_step_size>0.001</max_step_size>
```

For humanoid robots:
- Use smaller time steps (0.001s or smaller) for stability
- Smaller steps increase accuracy but reduce performance
- Balance accuracy with computational requirements

### 3. Real-Time Factor
Controls how fast the simulation runs compared to real time:
```xml
<real_time_factor>1.0</real_time_factor>
```

For humanoid simulation:
- 1.0 = real-time (recommended for most applications)
- Values > 1.0 = faster than real-time (useful for testing)
- Values < 1.0 = slower than real-time (more stable but slower)

## Collision Parameters

### 1. Contact Parameters
These affect how objects interact when they collide:
```xml
<ode>
  <solver>
    <type>quick</type>
    <iters>10</iters>
    <sor>1.3</sor>
  </solver>
  <constraints>
    <cfm>0.0</cfm>
    <erp>0.2</erp>
    <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
    <contact_surface_layer>0.001</contact_surface_layer>
  </constraints>
</ode>
```

### 2. Material Properties
Configure surface properties for realistic interaction:
- Friction coefficients
- Bounce properties
- Surface roughness

## Humanoid-Specific Considerations

### Balance and Stability
Humanoid robots require careful tuning for stable balance:
- Higher friction coefficients for feet
- Proper mass distribution
- Accurate inertia tensors

### Joint Constraints
Ensure joints behave realistically:
- Proper joint limits
- Appropriate damping values
- Correct effort and velocity limits

## Common Physics Issues in Humanoid Simulation

### 1. Unstable Joints
**Symptoms**: Robot falls apart or joints behave erratically
**Solutions**:
- Increase solver iterations
- Reduce time step
- Check mass/inertia values

### 2. Penetration Issues
**Symptoms**: Robot parts pass through objects
**Solutions**:
- Increase contact parameters (ERP, CFM)
- Improve collision geometry
- Reduce time step

### 3. Slipping Feet
**Symptoms**: Robot feet slide during standing/walking
**Solutions**:
- Increase friction coefficients
- Adjust contact parameters
- Check ground plane properties

## Exercise: Physics Parameter Tuning

Create a simple humanoid model and experiment with different physics parameters:
1. Start with default parameters
2. Adjust time step and observe stability
3. Modify friction coefficients and note the effect on locomotion
4. Document the optimal settings for your specific humanoid model

## Best Practices

1. **Start Conservative**: Begin with stable parameters and gradually optimize
2. **Iterative Tuning**: Make small changes and test frequently
3. **Model-Specific**: Parameters may vary between different humanoid designs
4. **Validation**: Compare simulation results with real-world behavior when possible

## Summary

Physics parameters significantly impact humanoid robot simulation quality. Proper configuration of gravity, time step, real-time factor, and collision parameters is essential for realistic and stable simulation. Regular testing and tuning ensure optimal performance for your specific humanoid robot model.