---
sidebar_position: 2
---

# Depth Camera Simulation in Gazebo and Unity

Depth cameras provide crucial 3D perception capabilities for humanoid robots, enabling them to understand their environment in three dimensions. This section covers how to simulate depth cameras in both Gazebo and Unity environments.

## Understanding Depth Cameras

### Depth Camera Principles
Depth cameras capture both color and depth information simultaneously:
- **RGB data**: Color image information
- **Depth data**: Distance from camera to objects
- **Point cloud**: 3D representation of the environment
- **Infrared**: Additional data for low-light conditions

### Common Depth Camera Specifications
- **Resolution**: 640x480 to 1920x1080 pixels
- **Frame rate**: 15Hz to 60Hz
- **Depth range**: 0.3m to 10m typically
- **Accuracy**: mm to cm level precision
- **Field of view**: 60° to 120° diagonal

## Depth Camera Simulation in Gazebo

### Gazebo Depth Camera Plugin
Gazebo provides the `libgazebo_ros_openni_kinect.so` plugin for depth camera simulation:

```xml
<!-- Example depth camera configuration in URDF/SDF -->
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <depth_camera>
        <output>depths</output>
      </depth_camera>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>camera</namespace>
        <remapping>~/rgb/image_raw:=image_raw</remapping>
        <remapping>~/depth/image_raw:=depth/image_raw</remapping>
        <remapping>~/depth/camera_info:=depth/camera_info</remapping>
      </ros>
      <baseline>0.2</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <point_cloud_cutoff>0.1</point_cloud_cutoff>
      <point_cloud_cutoff_max>3.0</point_cloud_cutoff_max>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera Parameters in Gazebo

#### 1. Image Parameters
- **Resolution**: Width and height of the image
- **Format**: Color format (R8G8B8, B8G8R8, etc.)
- **Frame rate**: Update rate of the sensor

#### 2. Camera Parameters
- **Field of view**: Horizontal and vertical viewing angle
- **Near/far clipping**: Range of detectable depths
- **Distortion**: Lens distortion coefficients

#### 3. Point Cloud Parameters
- **Cutoff distances**: Near and far limits for point cloud generation
- **Point cloud format**: Output format for 3D data

## Depth Camera Simulation in Unity

### Unity Depth Camera Implementation
Unity doesn't have native depth camera simulation, but it can be implemented using render textures and shader techniques:

```csharp
// Example: Unity depth camera simulation
using UnityEngine;
using System.Collections.Generic;

public class UnityDepthCamera : MonoBehaviour
{
    [Header("Camera Configuration")]
    [SerializeField] private Camera depthCamera;
    [SerializeField] private int width = 640;
    [SerializeField] private int height = 480;
    [SerializeField] private float nearClip = 0.1f;
    [SerializeField] private float farClip = 10.0f;

    [Header("Output Configuration")]
    [SerializeField] private RenderTexture colorTexture;
    [SerializeField] private RenderTexture depthTexture;
    [SerializeField] private Shader depthShader;

    private Texture2D colorOutput;
    private Texture2D depthOutput;
    private float[] depthData;

    void Start()
    {
        InitializeCameras();
        CreateRenderTextures();
        CreateOutputTextures();
    }

    void InitializeCameras()
    {
        if (depthCamera == null)
        {
            depthCamera = GetComponent<Camera>();
        }

        depthCamera.depthTextureMode = DepthTextureMode.Depth;
        depthCamera.nearClipPlane = nearClip;
        depthCamera.farClipPlane = farClip;
    }

    void CreateRenderTextures()
    {
        if (colorTexture != null)
        {
            colorTexture.Release();
        }
        colorTexture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        colorTexture.Create();

        if (depthTexture != null)
        {
            depthTexture.Release();
        }
        depthTexture = new RenderTexture(width, height, 24, RenderTextureFormat.RFloat);
        depthTexture.Create();

        depthCamera.targetTexture = colorTexture;
    }

    void CreateOutputTextures()
    {
        colorOutput = new Texture2D(width, height, TextureFormat.RGB24, false);
        depthOutput = new Texture2D(width, height, TextureFormat.RFloat, false);
    }

    public void CaptureDepthFrame()
    {
        // Render the scene from the depth camera
        depthCamera.Render();

        // Read color data
        RenderTexture.active = colorTexture;
        colorOutput.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        colorOutput.Apply();

        // Read depth data
        RenderTexture.active = depthTexture;
        depthOutput.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        depthOutput.Apply();

        // Convert depth texture to float array
        Color[] depthColors = depthOutput.GetPixels();
        depthData = new float[depthColors.Length];

        for (int i = 0; i < depthColors.Length; i++)
        {
            // Convert color value back to depth (this is a simplified example)
            depthData[i] = depthColors[i].r * farClip; // Scale by far clip
        }

        RenderTexture.active = null;
    }

    public Texture2D GetColorImage()
    {
        return colorOutput;
    }

    public Texture2D GetDepthImage()
    {
        return depthOutput;
    }

    public float[] GetDepthData()
    {
        return depthData;
    }

    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        if (depthShader != null)
        {
            Graphics.Blit(source, destination, depthShader);
        }
        else
        {
            Graphics.Blit(source, destination);
        }
    }
}
```

### Depth Shader for Unity
Creating a custom shader for depth visualization:

```hlsl
// Example: Depth visualization shader
Shader "Custom/DepthVisualization"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _DepthTex ("Depth Texture", 2D) = "white" {}
        _NearClip ("Near Clip", Float) = 0.1
        _FarClip ("Far Clip", Float) = 10.0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float4 screenPos : TEXCOORD1;
            };

            sampler2D _MainTex;
            sampler2D _DepthTex;
            float4 _MainTex_TexelSize;
            float _NearClip;
            float _FarClip;

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                o.screenPos = ComputeScreenPos(o.vertex);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, i.uv);

                // Sample depth
                float depth = tex2D(_DepthTex, i.uv).r;

                // Normalize depth for visualization
                float normalizedDepth = (depth - _NearClip) / (_FarClip - _NearClip);

                // Create depth visualization
                fixed3 depthVis = fixed3(normalizedDepth, normalizedDepth, normalizedDepth);

                // Blend color and depth visualization
                col.rgb = lerp(col.rgb, depthVis, 0.3); // 30% depth overlay

                return col;
            }
            ENDCG
        }
    }
}
```

### Point Cloud Generation from Depth Data
Converting depth data to 3D point clouds:

```csharp
// Example: Point cloud generation from depth data
using UnityEngine;
using System.Collections.Generic;

public class DepthToPointCloud : MonoBehaviour
{
    [SerializeField] private UnityDepthCamera depthCamera;
    [SerializeField] private GameObject pointPrefab;
    [SerializeField] private float pointSpacing = 0.05f;

    private List<GameObject> pointCloud = new List<GameObject>();
    private Camera unityCamera;

    void Start()
    {
        unityCamera = Camera.main;
    }

    public void GeneratePointCloud()
    {
        // Clear existing points
        foreach (GameObject point in pointCloud)
        {
            DestroyImmediate(point);
        }
        pointCloud.Clear();

        float[] depthData = depthCamera.GetDepthData();
        if (depthData == null) return;

        int width = 640; // Match your depth camera resolution
        int height = 480;

        for (int y = 0; y < height; y += 10) // Sample every 10 pixels for performance
        {
            for (int x = 0; x < width; x += 10)
            {
                int index = y * width + x;
                if (index >= depthData.Length) continue;

                float depth = depthData[index];

                // Skip if depth is invalid (0 or beyond range)
                if (depth <= 0.01f || depth >= 9.99f) continue;

                // Convert screen coordinates to world coordinates
                Vector3 screenPoint = new Vector3(x, y, depth);
                Vector3 worldPoint = unityCamera.ScreenToWorldPoint(screenPoint);

                // Create point in point cloud
                GameObject point = Instantiate(pointPrefab, worldPoint, Quaternion.identity);
                pointCloud.Add(point);
            }
        }
    }

    public List<GameObject> GetPointCloud()
    {
        return pointCloud;
    }
}
```

## Sensor Validation Techniques

### 1. Geometric Validation
Verify that depth measurements match geometric expectations:

```csharp
// Example: Depth camera validation
using UnityEngine;

public class DepthCameraValidation : MonoBehaviour
{
    [SerializeField] private UnityDepthCamera depthCam;
    [SerializeField] private Transform validationObject;
    [SerializeField] private float tolerance = 0.05f; // 5cm tolerance

    public float ValidateDepthMeasurement()
    {
        depthCam.CaptureDepthFrame();
        float[] depthData = depthCam.GetDepthData();

        // Calculate expected distance to validation object
        float expectedDistance = Vector3.Distance(
            depthCam.transform.position,
            validationObject.position
        );

        // Find depth value at the center of the image
        int centerX = 640 / 2; // Assuming 640x480 resolution
        int centerY = 480 / 2;
        int centerIndex = centerY * 640 + centerX;

        if (centerIndex < depthData.Length)
        {
            float measuredDepth = depthData[centerIndex];
            float error = Mathf.Abs(measuredDepth - expectedDistance);
            float accuracy = Mathf.Clamp01(1.0f - (error / tolerance));

            return accuracy * 100f; // Return percentage accuracy
        }

        return 0f; // Error in validation
    }
}
```

### 2. Environmental Consistency
Ensure depth data is consistent with the environment:
- Objects should appear at correct distances
- Occlusions should be properly handled
- Depth ranges should be respected
- Noise patterns should match real sensors

## Performance Considerations

### Gazebo Performance
- **Resolution**: Higher resolutions provide more detail but reduce performance
- **Update rate**: Higher rates provide more data but increase CPU load
- **Rendering**: Depth rendering is more expensive than color-only
- **Point clouds**: Real-time point cloud generation is computationally intensive

### Unity Performance
- **Render textures**: Multiple render textures consume GPU memory
- **Shader complexity**: Complex depth shaders impact frame rate
- **Point cloud generation**: CPU-intensive operation
- **Texture sampling**: Frequent texture reads can be expensive

## Exercise: Depth Camera Implementation

1. Create a humanoid robot with a depth camera sensor
2. Configure the depth camera in Gazebo with appropriate parameters
3. Implement depth camera simulation in Unity
4. Generate point clouds from depth data
5. Validate the simulation against geometric expectations
6. Test performance with different configurations

## Best Practices

1. **Parameter Matching**: Ensure simulated parameters match real sensor specifications
2. **Performance Optimization**: Balance quality with computational requirements
3. **Validation Testing**: Regularly validate depth measurements against ground truth
4. **Noise Modeling**: Include realistic noise patterns in simulated data
5. **Calibration**: Maintain proper camera calibration parameters

## Summary

Depth camera simulation is essential for robotics applications requiring 3D perception. Both Gazebo and Unity provide tools for simulating depth cameras, though with different approaches and considerations. Proper implementation and validation ensure that simulated depth data is realistic and useful for robotics development.