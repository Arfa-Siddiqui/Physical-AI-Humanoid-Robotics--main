---
sidebar_position: 3
---

# Unity: Photorealistic Visualization and Synthetic Data

While Gazebo excels at physics simulation, **Unity** provides photorealistic rendering, advanced graphics, and synthetic data generation for computer vision. This chapter integrates Unity with ROS 2 for visualization and training data creation.

## Why Unity for Robotics?

Unity offers capabilities beyond traditional robot simulators:

- **Photorealism**: Ray tracing, global illumination, PBR (Physically Based Rendering) materials
- **Synthetic data**: Labeled images for training perception models
- **VR/AR support**: Immersive teleoperation interfaces
- **Cross-platform**: Deploy to Windows, Linux, macOS, mobile, web

**Use Cases:**
- Training object detection models (bounding boxes, semantic segmentation)
- Testing vision algorithms under varied lighting conditions
- Creating marketing demos with cinematic quality

## Unity Robotics Hub Overview

Unity Technologies provides **ROS-TCP-Connector** for bidirectional communication:

**ROS 2 → Unity**: Send robot state (joint angles, transforms)
**Unity → ROS 2**: Send sensor data (camera images, LiDAR point clouds)

### Architecture

```
[ROS 2 Node] ←→ [ROS TCP Endpoint] ←→ [Unity TCP Connector] ←→ [Unity Scene]
```

**TCP Bridge**: Serializes ROS messages (JSON or MessagePack) over TCP.

## Setting Up Unity-ROS Integration

### Install ROS TCP Endpoint (ROS Side)

```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

**Launch Server:**

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

Server listens on port 10000 for Unity connections.

### Install Unity Packages (Unity Side)

1. Open Unity Editor
2. **Window → Package Manager → Add package from git URL**
3. Add: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Add: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

**Configure Connection:**

- **Robotics → ROS Settings**
- Set **ROS IP Address**: `127.0.0.1`
- Set **ROS Port**: `10000`
- Click **Connect**

## Importing URDF into Unity

Unity's URDF Importer converts robot models to Unity GameObjects:

1. **Assets → Import Robot from URDF**
2. Select your `humanoid.urdf` file
3. Unity creates:
   - **GameObjects** for each link
   - **ArticulationBody** components for joints (Unity's physics joints)
   - **Meshes** and **Materials** from visual geometry

**Adjust Settings:**
- **Axis Type**: Y-Axis (Unity uses Y-up, ROS uses Z-up—importer handles conversion)
- **Mesh Decomposer**: Convex decomposition for collision

## Controlling the Robot from ROS 2

### Publish Joint States (Unity → ROS 2)

Create a C# script `JointStatePublisher.cs`:

```csharp
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class JointStatePublisher : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/joint_states";
    private float publishRate = 50f;
    private float timeElapsed;

    private ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName);

        joints = GetComponentsInChildren<ArticulationBody>();
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1f / publishRate)
        {
            PublishJointStates();
            timeElapsed = 0;
        }
    }

    void PublishJointStates()
    {
        JointStateMsg msg = new JointStateMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg(),
            name = new string[joints.Length],
            position = new double[joints.Length],
            velocity = new double[joints.Length],
            effort = new double[joints.Length]
        };

        for (int i = 0; i < joints.Length; i++)
        {
            msg.name[i] = joints[i].name;
            msg.position[i] = joints[i].jointPosition[0];
            msg.velocity[i] = joints[i].jointVelocity[0];
            msg.effort[i] = joints[i].jointForce[0];
        }

        ros.Publish(topicName, msg);
    }
}
```

Attach to robot root GameObject. Unity now publishes `/joint_states` to ROS 2.

### Subscribe to Commands (ROS 2 → Unity)

Create `VelocitySubscriber.cs`:

```csharp
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class VelocitySubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/cmd_vel";
    private Rigidbody rb;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, ApplyVelocity);
        rb = GetComponent<Rigidbody>();
    }

    void ApplyVelocity(TwistMsg msg)
    {
        Vector3 linear = new Vector3(
            (float)msg.linear.x,
            0,
            (float)msg.linear.y
        );
        rb.velocity = linear;
        rb.angularVelocity = new Vector3(0, (float)msg.angular.z, 0);
    }
}
```

Now ROS 2 `/cmd_vel` messages control the Unity robot.

## Camera Sensor: Generating Synthetic Data

### RGB Camera

Add a Camera GameObject as child of robot's head link.

Create `CameraPublisher.cs`:

```csharp
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class CameraPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/camera/image_raw";
    private Camera cam;
    private Texture2D tex;
    private float publishRate = 10f;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
        cam = GetComponent<Camera>();
        tex = new Texture2D(cam.pixelWidth, cam.pixelHeight, TextureFormat.RGB24, false);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1f / publishRate)
        {
            PublishImage();
            timeElapsed = 0;
        }
    }

    void PublishImage()
    {
        RenderTexture rt = new RenderTexture(cam.pixelWidth, cam.pixelHeight, 24);
        cam.targetTexture = rt;
        cam.Render();

        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();

        RenderTexture.active = null;
        cam.targetTexture = null;

        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg(),
            height = (uint)tex.height,
            width = (uint)tex.width,
            encoding = "rgb8",
            step = (uint)(tex.width * 3),
            data = tex.GetRawTextureData()
        };

        ros.Publish(topicName, msg);
    }
}
```

**View in ROS 2:**

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### Semantic Segmentation

Unity's Perception package generates labeled images for training.

**Install Perception Package:**

Package Manager → Add `com.unity.perception`

**Add Labeling Component:**

1. Select objects (e.g., "Table", "Chair")
2. **Add Component → Labeling**
3. Add label (e.g., "furniture")

**Add Perception Camera:**

1. Select camera GameObject
2. **Add Component → Perception Camera**
3. **Add Semantic Segmentation Labeler**

Unity now outputs segmentation masks (each object class a different color).

## Lighting and Materials for Realism

### HDRP (High Definition Render Pipeline)

For photorealism, use HDRP:

1. **Edit → Project Settings → Graphics**
2. Set **Scriptable Render Pipeline Settings** to HDRP
3. **Window → Rendering → Lighting**
4. Enable **Ambient Occlusion**, **Screen Space Reflections**

### PBR Materials

Use Physically Based Rendering materials:

- **Albedo**: Base color texture
- **Metallic**: Shiny (1) or matte (0)
- **Smoothness**: Glossy (1) or rough (0)
- **Normal Map**: Surface detail

**Example**: Robot metal surfaces → Metallic=0.8, Smoothness=0.6

## Domain Randomization in Unity

For robust vision models, randomize:

**Lighting:**

```csharp
void RandomizeLighting()
{
    Light sun = GameObject.Find("Directional Light").GetComponent<Light>();
    sun.intensity = Random.Range(0.5f, 2.0f);
    sun.color = new Color(Random.Range(0.8f, 1.0f), Random.Range(0.8f, 1.0f), Random.Range(0.8f, 1.0f));
}
```

**Object Placement:**

```csharp
void RandomizeObjectPosition(GameObject obj)
{
    float x = Random.Range(-5f, 5f);
    float z = Random.Range(-5f, 5f);
    obj.transform.position = new Vector3(x, 0, z);
}
```

**Textures:**

Swap materials dynamically for variety.

## Performance Considerations

### GPU Requirements

- **Minimum**: GTX 1060 (6 GB VRAM)
- **Recommended**: RTX 3060 or better (ray tracing, high-res renders)

### Optimization

- Use **Occlusion Culling** (don't render hidden objects)
- **LOD (Level of Detail)**: Simplified meshes at distance
- **Baked Lighting**: Pre-compute static lighting (faster than real-time)

## Conclusion

Unity extends ROS 2 with photorealistic visualization, synthetic data generation, and VR interfaces. While Gazebo handles physics, Unity excels at perception testing and creating training datasets. In the next module, we'll explore NVIDIA Isaac Sim, which combines GPU-accelerated physics with Unity-level graphics.

## References

- Unity Technologies. (2024). "Unity Robotics Hub." Retrieved from https://github.com/Unity-Technologies/Unity-Robotics-Hub
- Unity Technologies. (2024). "Perception Package Documentation." Retrieved from https://github.com/Unity-Technologies/com.unity.perception
