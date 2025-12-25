---
sidebar_position: 3
---

# IMU Simulation in Gazebo and Unity

Inertial Measurement Units (IMUs) are critical sensors for humanoid robots, providing information about orientation, angular velocity, and linear acceleration. This section covers how to simulate IMU sensors in both Gazebo and Unity environments.

## Understanding IMU Sensors

### IMU Principles
IMUs typically combine three types of sensors:
- **Accelerometer**: Measures linear acceleration in 3 axes
- **Gyroscope**: Measures angular velocity in 3 axes
- **Magnetometer**: Measures magnetic field (optional, for heading reference)

### Common IMU Specifications
- **Accelerometer range**: ±2g to ±16g
- **Gyroscope range**: ±250°/s to ±2000°/s
- **Update rate**: 10Hz to 1000Hz
- **Bias stability**: Micro-g to milli-g for accelerometers
- **Noise density**: Units of °/s/√Hz for gyroscopes

## IMU Simulation in Gazebo

### Gazebo IMU Plugin
Gazebo provides the `libgazebo_ros_imu_sensor.so` plugin for IMU simulation:

```xml
<!-- Example IMU configuration in URDF/SDF -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.0001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.0001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.0001</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <body_name>imu_link</body_name>
      <update_rate>100</update_rate>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Configuration Parameters

#### 1. Noise Models
Real IMUs have various types of noise:
- **Gaussian noise**: Random noise in measurements
- **Bias**: Systematic offset in measurements
- **Drift**: Slowly changing bias over time

#### 2. Sensor Ranges
Configure appropriate ranges for your application:
- Accelerometer: ±2g to ±16g depending on expected accelerations
- Gyroscope: ±250°/s to ±2000°/s depending on expected rotation rates

#### 3. Update Rates
Higher update rates provide more data but increase computational load:
- Typical rates: 100Hz to 1000Hz
- Consider your control loop requirements

## IMU Simulation in Unity

### Unity IMU Simulation
Unity doesn't have native IMU simulation, but it can be implemented using physics and sensor models:

```csharp
// Example: Unity IMU simulation
using UnityEngine;
using System.Collections.Generic;

[System.Serializable]
public struct IMUData
{
    public Vector3 linearAcceleration;
    public Vector3 angularVelocity;
    public Quaternion orientation;
    public double timestamp;
}

public class UnityIMUSimulation : MonoBehaviour
{
    [Header("IMU Configuration")]
    [SerializeField] private float updateRate = 100f; // Hz
    [SerializeField] private bool simulateNoise = true;
    [SerializeField] private float accelerometerNoiseStdDev = 0.017f;
    [SerializeField] private float gyroscopeNoiseStdDev = 0.001f;
    [SerializeField] private float accelerometerBias = 0.001f;
    [SerializeField] private float gyroscopeBias = 0.0001f;

    [Header("Reference Frame")]
    [SerializeField] private Transform referenceFrame;

    private IMUData currentIMUData;
    private Rigidbody attachedRigidbody;
    private float updateInterval;
    private float lastUpdateTime;
    private System.Random random;

    void Start()
    {
        random = new System.Random();
        updateInterval = 1f / updateRate;
        lastUpdateTime = Time.time;

        // Try to find attached rigidbody for physics data
        attachedRigidbody = GetComponent<Rigidbody>();
        if (attachedRigidbody == null)
        {
            attachedRigidbody = GetComponentInParent<Rigidbody>();
        }
    }

    void Update()
    {
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            UpdateIMUData();
            lastUpdateTime = Time.time;
        }
    }

    void UpdateIMUData()
    {
        // Get the transform relative to reference frame
        Transform targetTransform = referenceFrame != null ? referenceFrame : transform;

        // Calculate linear acceleration
        Vector3 linearAcc = CalculateLinearAcceleration(targetTransform);

        // Calculate angular velocity
        Vector3 angularVel = CalculateAngularVelocity(targetTransform);

        // Get orientation
        Quaternion orientation = targetTransform.rotation;

        // Apply noise if enabled
        if (simulateNoise)
        {
            linearAcc = AddNoiseToVector(linearAcc, accelerometerNoiseStdDev, accelerometerBias);
            angularVel = AddNoiseToVector(angularVel, gyroscopeNoiseStdDev, gyroscopeBias);
        }

        // Store IMU data
        currentIMUData = new IMUData
        {
            linearAcceleration = linearAcc,
            angularVelocity = angularVel,
            orientation = orientation,
            timestamp = Time.timeAsDouble()
        };
    }

    Vector3 CalculateLinearAcceleration(Transform t)
    {
        // If we have a rigidbody, use physics data
        if (attachedRigidbody != null)
        {
            // Get acceleration from change in velocity
            // Note: Unity doesn't directly provide acceleration, so we estimate it
            return attachedRigidbody.velocity / Time.fixedDeltaTime;
        }
        else
        {
            // Estimate from transform changes
            // This is a simplified approach
            return Physics.gravity; // Default to gravity in world frame
        }
    }

    Vector3 CalculateAngularVelocity(Transform t)
    {
        // If we have a rigidbody, use its angular velocity
        if (attachedRigidbody != null)
        {
            return attachedRigidbody.angularVelocity;
        }
        else
        {
            // Estimate from rotation changes
            // This is a simplified approach
            return Vector3.zero;
        }
    }

    Vector3 AddNoiseToVector(Vector3 original, float stdDev, float bias)
    {
        return new Vector3(
            AddNoiseToFloat(original.x, stdDev, bias),
            AddNoiseToFloat(original.y, stdDev, bias),
            AddNoiseToFloat(original.z, stdDev, bias)
        );
    }

    float AddNoiseToFloat(float original, float stdDev, float bias)
    {
        // Box-Muller transform for Gaussian noise
        float u1 = Random.value;
        float u2 = Random.value;
        float normal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Cos(2.0f * Mathf.PI * u2);
        return original + normal * stdDev + bias;
    }

    public IMUData GetIMUData()
    {
        return currentIMUData;
    }

    public Vector3 GetLinearAcceleration()
    {
        return currentIMUData.linearAcceleration;
    }

    public Vector3 GetAngularVelocity()
    {
        return currentIMUData.angularVelocity;
    }

    public Quaternion GetOrientation()
    {
        return currentIMUData.orientation;
    }

    // Visualization for debugging
    void OnDrawGizmosSelected()
    {
        if (Application.isPlaying)
        {
            Transform targetTransform = referenceFrame != null ? referenceFrame : transform;

            // Visualize linear acceleration
            Gizmos.color = Color.red;
            Gizmos.DrawRay(targetTransform.position, currentIMUData.linearAcceleration * 0.1f);

            // Visualize angular velocity
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(targetTransform.position, currentIMUData.angularVelocity * 0.01f);
        }
    }
}
```

### Advanced IMU Simulation with Bias and Drift
For more realistic IMU simulation, include bias and drift effects:

```csharp
// Example: Advanced IMU with bias and drift
using UnityEngine;

public class AdvancedIMUSimulation : MonoBehaviour
{
    [Header("IMU Configuration")]
    [SerializeField] private float updateRate = 100f;
    [SerializeField] private float accelerometerNoiseDensity = 0.01f; // m/s²/√Hz
    [SerializeField] private float gyroscopeNoiseDensity = 0.001f; // rad/s/√Hz
    [SerializeField] private float accelerometerRandomWalk = 0.001f; // m/s²/√s
    [SerializeField] private float gyroscopeRandomWalk = 0.0001f; // rad/s/√s
    [SerializeField] private float accelerometerBiasStability = 0.01f; // m/s²
    [SerializeField] private float gyroscopeBiasStability = 0.001f; // rad/s

    private IMUData currentIMUData;
    private float updateInterval;
    private double lastTimestamp;

    // Bias and drift tracking
    private Vector3 accelBias;
    private Vector3 gyroBias;
    private double simulationTime;

    void Start()
    {
        updateInterval = 1f / updateRate;
        lastTimestamp = Time.timeAsDouble();

        // Initialize with random bias
        accelBias = new Vector3(
            RandomGaussian() * accelerometerBiasStability,
            RandomGaussian() * accelerometerBiasStability,
            RandomGaussian() * accelerometerBiasStability
        );

        gyroBias = new Vector3(
            RandomGaussian() * gyroscopeBiasStability,
            RandomGaussian() * gyroscopeBiasStability,
            RandomGaussian() * gyroscopeBiasStability
        );
    }

    void Update()
    {
        double currentTime = Time.timeAsDouble();
        if (currentTime - lastTimestamp >= 1.0/updateRate)
        {
            UpdateIMUWithBiasAndDrift();
            lastTimestamp = currentTime;
        }
    }

    void UpdateIMUWithBiasAndDrift()
    {
        // Calculate time delta
        float dt = (float)(Time.timeAsDouble() - simulationTime);
        simulationTime = Time.timeAsDouble();

        // Update bias with random walk (first-order Gauss-Markov process)
        float biasDt = 1.0f; // Time constant for bias correlation
        float biasCorrelation = Mathf.Exp(-dt / biasDt);

        // Update accelerometer bias
        Vector3 accelRandomWalk = new Vector3(
            RandomGaussian() * accelerometerRandomWalk * Mathf.Sqrt(dt),
            RandomGaussian() * accelerometerRandomWalk * Mathf.Sqrt(dt),
            RandomGaussian() * accelerometerRandomWalk * Mathf.Sqrt(dt)
        );
        accelBias = accelBias * biasCorrelation + accelRandomWalk;

        // Update gyroscope bias
        Vector3 gyroRandomWalk = new Vector3(
            RandomGaussian() * gyroscopeRandomWalk * Mathf.Sqrt(dt),
            RandomGaussian() * gyroscopeRandomWalk * Mathf.Sqrt(dt),
            RandomGaussian() * gyroscopeRandomWalk * Mathf.Sqrt(dt)
        );
        gyroBias = gyroBias * biasCorrelation + gyroRandomWalk;

        // Get base measurements from physics
        Vector3 trueAccel = GetTrueAcceleration();
        Vector3 trueGyro = GetTrueAngularVelocity();

        // Apply noise, bias, and drift
        Vector3 measuredAccel = trueAccel + accelBias +
            new Vector3(
                RandomGaussian() * accelerometerNoiseDensity * Mathf.Sqrt(1.0f/updateRate),
                RandomGaussian() * accelerometerNoiseDensity * Mathf.Sqrt(1.0f/updateRate),
                RandomGaussian() * accelerometerNoiseDensity * Mathf.Sqrt(1.0f/updateRate)
            );

        Vector3 measuredGyro = trueGyro + gyroBias +
            new Vector3(
                RandomGaussian() * gyroscopeNoiseDensity * Mathf.Sqrt(1.0f/updateRate),
                RandomGaussian() * gyroscopeNoiseDensity * Mathf.Sqrt(1.0f/updateRate),
                RandomGaussian() * gyroscopeNoiseDensity * Mathf.Sqrt(1.0f/updateRate)
            );

        // Store data
        currentIMUData = new IMUData
        {
            linearAcceleration = measuredAccel,
            angularVelocity = measuredGyro,
            orientation = transform.rotation, // Orientation from transform
            timestamp = Time.timeAsDouble()
        };
    }

    Vector3 GetTrueAcceleration()
    {
        // Get true acceleration from physics simulation
        Rigidbody rb = GetComponent<Rigidbody>() ?? GetComponentInParent<Rigidbody>();
        if (rb != null)
        {
            return rb.velocity / Time.fixedDeltaTime;
        }
        return Physics.gravity; // Default to gravity
    }

    Vector3 GetTrueAngularVelocity()
    {
        // Get true angular velocity from physics
        Rigidbody rb = GetComponent<Rigidbody>() ?? GetComponentInParent<Rigidbody>();
        if (rb != null)
        {
            return rb.angularVelocity;
        }
        return Vector3.zero;
    }

    float RandomGaussian()
    {
        // Box-Muller transform
        float u1 = Random.value;
        float u2 = Random.value;
        if (u1 < float.Epsilon) u1 = float.Epsilon;
        return Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Cos(2.0f * Mathf.PI * u2);
    }

    public IMUData GetIMUData()
    {
        return currentIMUData;
    }
}
```

## Sensor Fusion and Orientation Estimation

### Complementary Filter
Combine IMU data to estimate orientation:

```csharp
// Example: Complementary filter for orientation
using UnityEngine;

public class IMUOrientationEstimator : MonoBehaviour
{
    [SerializeField] private UnityIMUSimulation imu;
    [SerializeField] private float filterGain = 0.98f;
    [SerializeField] private Vector3 gravityVector = new Vector3(0, 9.81f, 0);

    private Quaternion estimatedOrientation = Quaternion.identity;
    private float lastUpdateTime;

    void Start()
    {
        lastUpdateTime = Time.time;
    }

    void Update()
    {
        float dt = Time.time - lastUpdateTime;
        lastUpdateTime = Time.time;

        IMUData imuData = imu.GetIMUData();

        // Integrate gyroscope data for orientation prediction
        Vector3 angularVelocity = imuData.angularVelocity;
        Quaternion gyroUpdate = Quaternion.Euler(angularVelocity * dt * Mathf.Rad2Deg);
        Quaternion predictedOrientation = estimatedOrientation * gyroUpdate;

        // Use accelerometer data for orientation correction
        Vector3 measuredGravity = imuData.linearAcceleration;
        if (measuredGravity.magnitude > 0.1f) // Avoid division by zero
        {
            Vector3 gravityDirection = measuredGravity.normalized;
            Quaternion accelOrientation = Quaternion.FromToRotation(Vector3.up, -gravityDirection);

            // Apply complementary filter
            estimatedOrientation = Quaternion.Slerp(
                predictedOrientation,
                accelOrientation,
                1f - filterGain
            );
        }
        else
        {
            // If accelerometer data is invalid, trust gyroscope more
            estimatedOrientation = predictedOrientation;
        }

        // Apply the estimated orientation to a visual indicator
        transform.rotation = estimatedOrientation;
    }

    public Quaternion GetEstimatedOrientation()
    {
        return estimatedOrientation;
    }
}
```

## Sensor Validation Techniques

### 1. Static Validation
Verify IMU readings when the robot is stationary:

```csharp
// Example: Static IMU validation
using UnityEngine;

public class IMUStaticValidation : MonoBehaviour
{
    [SerializeField] private UnityIMUSimulation imu;
    [SerializeField] private float staticThreshold = 0.1f;
    [SerializeField] private float tolerance = 0.5f;

    public float ValidateStaticIMU()
    {
        IMUData imuData = imu.GetIMUData();

        // For static validation:
        // - Acceleration should be close to gravity (9.81 m/s²)
        // - Angular velocity should be close to zero
        float expectedGravity = 9.81f;
        float measuredAccelMagnitude = imuData.linearAcceleration.magnitude;

        float accelError = Mathf.Abs(measuredAccelMagnitude - expectedGravity);
        float gyroError = imuData.angularVelocity.magnitude;

        // Calculate accuracy as average of both measurements
        float accelAccuracy = Mathf.Clamp01(1.0f - (accelError / tolerance));
        float gyroAccuracy = Mathf.Clamp01(1.0f - (gyroError / tolerance));

        return ((accelAccuracy + gyroAccuracy) / 2.0f) * 100f;
    }
}
```

### 2. Dynamic Validation
Verify IMU readings during known motions:

```csharp
// Example: Dynamic IMU validation
using UnityEngine;

public class IMUDynamicValidation : MonoBehaviour
{
    [SerializeField] private UnityIMUSimulation imu;
    [SerializeField] private float tolerance = 0.2f;

    public float ValidateKnownMotion(Vector3 expectedAcceleration, Vector3 expectedAngularVelocity)
    {
        IMUData imuData = imu.GetIMUData();

        float accelError = Vector3.Distance(imuData.linearAcceleration, expectedAcceleration);
        float gyroError = Vector3.Distance(imuData.angularVelocity, expectedAngularVelocity);

        float accelAccuracy = Mathf.Clamp01(1.0f - (accelError / tolerance));
        float gyroAccuracy = Mathf.Clamp01(1.0f - (gyroError / tolerance));

        return ((accelAccuracy + gyroAccuracy) / 2.0f) * 100f;
    }
}
```

## Performance Considerations

### Gazebo Performance
- **Update rate**: Higher rates provide more data but increase CPU load
- **Noise models**: Complex noise models require more computation
- **Integration**: IMU data integration for orientation requires additional processing

### Unity Performance
- **Physics queries**: Frequent physics queries can impact performance
- **Noise generation**: Random number generation for noise simulation
- **Filtering algorithms**: Complex filtering algorithms require processing power

## Exercise: IMU Implementation

1. Create a humanoid robot with an IMU sensor
2. Configure the IMU in Gazebo with realistic parameters
3. Implement IMU simulation in Unity with noise and bias
4. Add sensor fusion for orientation estimation
5. Validate the simulation in both static and dynamic conditions
6. Test performance with different update rates

## Best Practices

1. **Realistic Noise**: Include appropriate noise models that match real IMUs
2. **Calibration**: Ensure proper IMU calibration in the simulation
3. **Coordinate Frames**: Maintain consistent coordinate frame conventions
4. **Validation**: Regularly validate IMU data against expected values
5. **Performance**: Balance realism with computational requirements

## Summary

IMU simulation is crucial for humanoid robotics applications requiring orientation and motion sensing. Both Gazebo and Unity provide tools for simulating IMUs, with Gazebo offering more realistic physics-based simulation and Unity allowing for custom implementations. Proper noise modeling, bias simulation, and validation ensure realistic IMU behavior for robotics development.