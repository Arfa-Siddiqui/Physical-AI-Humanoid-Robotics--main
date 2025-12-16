---
sidebar_position: 2
---

# Isaac ROS: GPU-Accelerated Perception

Isaac ROS provides hardware-accelerated ROS 2 nodes for perception tasks like object detection, visual SLAM, and depth estimation. By offloading computation to NVIDIA GPUs, Isaac ROS achieves 10-100× speedups over CPU-only implementations.

## Isaac ROS Packages Overview

| Package | Purpose | GPU Speedup |
|---------|---------|-------------|
| **isaac_ros_dnn_inference** | Deep learning inference (TensorRT) | 50-100× |
| **isaac_ros_apriltag** | Fiducial marker detection | 20×  |
| **isaac_ros_image_proc** | Image rectification, debayering | 15× |
| **isaac_ros_visual_slam** | Visual odometry (stereo/mono) | 30× |
| **isaac_ros_object_detection** | YOLO, DetectNet models | 80× |
| **isaac_ros_depth_segmentation** | Depth-based segmentation | 40× |

All packages use **CUDA** and **TensorRT** for GPU acceleration.

## Prerequisites

**Hardware:**
- NVIDIA Jetson (Nano, Xavier, Orin) **or**
- x86 desktop with RTX GPU (2060+)

**Software:**
- ROS 2 Humble
- CUDA 11.8+ (comes with JetPack on Jetson)
- TensorRT 8.5+

**Installation:**

```bash
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## AprilTag Detection (GPU-Accelerated)

AprilTags are fiducial markers used for localization and calibration. Isaac ROS accelerates detection 20× over CPU.

### Launch AprilTag Detector

```bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

**Subscribes:** `/camera/image_rect` (rectified images)
**Publishes:** `/tag_detections` (apriltag_msgs/AprilTagDetectionArray)

### Test with Webcam

```bash
# Launch camera
ros2 run usb_cam usb_cam_node_exe

# Remap image topic
ros2 run image_transport republish raw in:=/usb_cam/image_raw out:=/camera/image_rect
```

Hold an AprilTag (print from https://april.eecs.umich.edu/software/apriltag). Detections appear:

```bash
ros2 topic echo /tag_detections
```

### Visualize in RViz

```bash
ros2 run rviz2 rviz2
```

Add:
- **Image**: `/usb_cam/image_raw`
- **TF**: Shows tag pose in 3D

## Object Detection with YOLO

Isaac ROS integrates YOLOv5/v8 for real-time object detection.

### Convert YOLO to TensorRT

**Export YOLO Model:**

```bash
# Install YOLOv5
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
pip install -r requirements.txt

# Export to ONNX
python export.py --weights yolov5s.pt --include onnx --img 640
```

**Convert ONNX to TensorRT:**

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=yolov5s.onnx \
  --saveEngine=yolov5s.engine \
  --fp16  # Enable FP16 for 2× speedup on RTX GPUs
```

### Launch Detection Node

```python
# yolo_detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_dnn_inference',
            executable='dnn_inference_node',
            parameters=[{
                'model_file_path': '/path/to/yolov5s.engine',
                'engine_file_path': '/path/to/yolov5s.engine',
                'input_tensor_names': ['images'],
                'output_tensor_names': ['output'],
                'input_binding_names': ['images'],
                'output_binding_names': ['output'],
                'input_image_width': 640,
                'input_image_height': 640,
            }],
            remappings=[('/image', '/camera/image_raw')]
        )
    ])
```

**Run:**

```bash
ros2 launch my_package yolo_detection.launch.py
```

Detected objects published to `/detections` with bounding boxes and class labels.

## Visual SLAM (vSLAM)

Isaac ROS Visual SLAM provides GPU-accelerated visual odometry and mapping.

### Stereo Camera Configuration

**Launch vSLAM:**

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**Subscribes:**
- `/left/image_rect` (left stereo image)
- `/right/image_rect` (right stereo image)
- `/left/camera_info`, `/right/camera_info`

**Publishes:**
- `/visual_slam/tracking/odometry` (nav_msgs/Odometry)
- `/visual_slam/tracking/vo_pose` (geometry_msgs/PoseStamped)
- `/visual_slam/vis/landmarks` (visualization)

### Test with RealSense Camera

```bash
# Launch RealSense stereo
ros2 launch realsense2_camera rs_launch.py enable_depth:=false enable_infra1:=true enable_infra2:=true

# Remap topics for vSLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
  left_image_topic:=/camera/infra1/image_rect_raw \
  right_image_topic:=/camera/infra2/image_rect_raw
```

Move camera. vSLAM estimates pose in real-time.

**Visualize in RViz:**

Add:
- **Odometry**: `/visual_slam/tracking/odometry`
- **PointCloud**: `/visual_slam/vis/landmarks`

## Depth Image Processing

Rectify and denoise depth images:

```bash
ros2 launch isaac_ros_image_proc depth_proc.launch.py
```

**Input:** Raw depth image (e.g., from RealSense)
**Output:** Filtered, rectified depth image

Reduces noise by 50% using bilateral filtering on GPU.

## Custom DNN Models with Triton

For custom models (e.g., trained segmentation networks):

### Convert Model to TensorRT

```python
import tensorrt as trt

# Load ONNX
builder = trt.Builder(trt.Logger(trt.Logger.WARNING))
network = builder.create_network()
parser = trt.OnnxParser(network, trt.Logger())

with open('my_model.onnx', 'rb') as f:
    parser.parse(f.read())

# Build engine
config = builder.create_builder_config()
config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB
engine = builder.build_serialized_network(network, config)

with open('my_model.engine', 'wb') as f:
    f.write(engine)
```

### Inference Node

```python
Node(
    package='isaac_ros_dnn_inference',
    executable='dnn_inference_node',
    parameters=[{
        'model_file_path': '/path/to/my_model.engine',
        'input_tensor_names': ['input'],
        'output_tensor_names': ['output']
    }]
)
```

## Performance Benchmarks

**Test System:** RTX 3070, ROS 2 Humble

| Task | CPU (ms) | GPU (ms) | Speedup |
|------|----------|----------|---------|
| YOLOv5s (640×640) | 120 | 8 | 15× |
| AprilTag (1280×720) | 80 | 4 | 20× |
| Visual SLAM (stereo) | 150 | 5 | 30× |
| Image Rectification | 25 | 1.5 | 17× |

**Note:** Speedups increase with higher resolutions and batch processing.

## Troubleshooting

### Issue: "TensorRT Engine Build Failed"

**Solution:** Ensure CUDA and TensorRT versions match:

```bash
nvcc --version  # Should match TensorRT version
```

### Issue: "Image Topics Not Received"

**Solution:** Check QoS settings. Isaac ROS uses "sensor data" QoS:

```bash
ros2 topic info /camera/image_raw --verbose
```

### Issue: High Latency Despite GPU

**Solution:** Enable zero-copy transport:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Reduces overhead by avoiding serialization.

## Conclusion

Isaac ROS transforms perception from a CPU bottleneck to real-time GPU-accelerated processing. Whether detecting objects, tracking fiducials, or estimating pose, Isaac ROS enables humanoid robots to perceive their environment at speeds previously impossible. In the next chapter, we'll use these perception capabilities with Nav2 for autonomous navigation.

## References

- NVIDIA. (2024). "Isaac ROS Documentation." Retrieved from https://nvidia-isaac-ros.github.io/
- He, K., et al. (2024). "TensorRT: High Performance Deep Learning Inference." NVIDIA Developer Documentation.
