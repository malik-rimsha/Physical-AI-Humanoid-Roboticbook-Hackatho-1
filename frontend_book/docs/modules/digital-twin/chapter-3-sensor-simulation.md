---
sidebar_position: 3
title: "Chapter 3: Sensor Simulation"
---

# Chapter 3: Sensor Simulation

## Simulating LiDAR Sensors

LiDAR (Light Detection and Ranging) sensors are crucial for robotics applications, providing 3D spatial information about the environment. In simulation, we create virtual LiDAR sensors that produce realistic point cloud data.

### LiDAR Sensor Characteristics

Virtual LiDAR sensors in simulation include:

- **Range**: Maximum and minimum detection distances
- **Field of view**: Angular coverage (horizontal and vertical)
- **Resolution**: Angular resolution and accuracy
- **Scan rate**: How frequently the sensor updates
- **Noise models**: Realistic error characteristics

### Point Cloud Data

LiDAR sensors generate point cloud data in the format:

- **3D coordinates**: X, Y, Z positions of detected points
- **Intensity**: Reflectance values for each point
- **Timestamps**: When each measurement was taken
- **Frame information**: Coordinate system reference

### Configuring LiDAR Sensors in Gazebo

LiDAR sensors are configured in URDF/SDF files:

```xml
<!-- Example LiDAR sensor configuration -->
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
          <max_angle>3.14159</max_angle>    <!-- 180 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Sensor Types

Different LiDAR configurations for various applications:

1. **2D LiDAR**: Single horizontal plane, typically 270° or 360° FOV
2. **3D LiDAR**: Multiple planes creating full 3D point clouds
3. **Solid-state LiDAR**: No moving parts, faster scanning
4. **Flash LiDAR**: Illuminates entire scene at once

## Simulating Depth Cameras

Depth cameras provide both visual and depth information, essential for perception tasks in robotics.

### Depth Camera Properties

Virtual depth cameras include:

- **RGB data**: Color image information
- **Depth data**: Distance measurements for each pixel
- **Resolution**: Image dimensions (e.g., 640x480, 1280x720)
- **Field of view**: Angular coverage
- **Noise characteristics**: Realistic sensor noise models

### Depth Map Generation

Depth maps are generated with:

- **Distance measurements**: Distance from camera to objects
- **Missing data**: Areas where depth cannot be measured
- **Resolution limitations**: Accuracy based on distance and sensor quality

### Configuring Depth Cameras in Gazebo

Depth cameras are configured in URDF/SDF files:

```xml
<!-- Example depth camera configuration -->
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <format>R8G8B8</format>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>depth_camera</cameraName>
      <imageTopicName>/rgb/image_raw</imageTopicName>
      <depthImageTopicName>/depth/image_raw</depthImageTopicName>
      <pointCloudTopicName>/depth/points</pointCloudTopicName>
      <cameraInfoTopicName>/rgb/camera_info</cameraInfoTopicName>
      <depthImageCameraInfoTopicName>/depth/camera_info</depthImageCameraInfoTopicName>
      <frameName>camera_depth_optical_frame</frameName>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <pointCloudCutoff>0.4</pointCloudCutoff>
      <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
      <CxPrime>0</CxPrime>
      <Cx>0</Cx>
      <Cy>0</Cy>
      <focalLength>0</focalLength>
      <hackBaseline>0</hackBaseline>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera Types

Different depth camera configurations:

1. **Stereo Cameras**: Two cameras for triangulation-based depth
2. **Structured Light**: Projects known patterns for depth calculation
3. **Time-of-Flight**: Measures light travel time for distance
4. **RGB-D Cameras**: Combines color and depth in single sensor

## Simulating IMU Sensors

IMU (Inertial Measurement Unit) sensors provide information about robot motion and orientation.

### IMU Components

Virtual IMUs typically include:

- **Accelerometer**: Linear acceleration measurements
- **Gyroscope**: Angular velocity measurements
- **Magnetometer**: Magnetic field measurements (optional)

### IMU Data Streams

IMU sensors produce:

- **Linear acceleration**: X, Y, Z acceleration values
- **Angular velocity**: Roll, pitch, yaw rate measurements
- **Orientation**: Quaternion or Euler angle representations
- **Covariance**: Uncertainty information for each measurement

### Configuring IMU Sensors in Gazebo

IMU sensors are configured in URDF/SDF files:

```xml
<!-- Example IMU sensor configuration -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>  <!-- ~0.1 deg/s -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>  <!-- ~0.017 m/s² -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <body_name>imu_link</body_name>
      <update_rate>100</update_rate>
      <gaussian_noise>0.0017</gaussian_noise>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensor Types

Different IMU configurations:

1. **6-DOF IMU**: Accelerometer + Gyroscope
2. **9-DOF IMU**: Accelerometer + Gyroscope + Magnetometer
3. **Virtual IMU**: Simulated with realistic noise models
4. **Fused IMU**: Combines multiple sensor inputs

## Sensor Data Flow into ROS 2 Systems

### ROS 2 Message Types

The simulated sensors publish data using standard ROS 2 message types:

- **LiDAR**: `sensor_msgs/LaserScan` or `sensor_msgs/PointCloud2`
- **Depth Camera**: `sensor_msgs/Image` for depth data
- **IMU**: `sensor_msgs/Imu`

### Topic Architecture

Sensor data flows through ROS 2 topics:

```
/robot/lidar_scan → sensor_msgs/LaserScan
/robot/depth_camera/depth/image_raw → sensor_msgs/Image
/robot/imu/data → sensor_msgs/Imu
```

### Data Processing Pipeline

The typical data flow includes:

1. **Sensor simulation**: Virtual sensors generate data
2. **ROS 2 publishing**: Data published to appropriate topics
3. **Perception nodes**: Algorithms process sensor data
4. **Fusion**: Multiple sensor inputs combined
5. **Decision making**: Processed data used for robot behavior

### ROS 2 Sensor Interfaces

Based on the contracts/ros2_interfaces.yaml, here are the specific interfaces:

#### LiDAR Sensor Interface
- **Topic**: `/robot/lidar_scan`
- **Type**: `sensor_msgs/LaserScan`
- **Publish Rate**: 10 Hz
- **Frame**: `laser_frame`

#### Depth Camera Interface
- **Topic**: `/robot/depth_camera/depth/image_raw`
- **Type**: `sensor_msgs/Image`
- **Publish Rate**: 30 Hz
- **Frame**: `depth_camera_frame`

#### IMU Sensor Interface
- **Topic**: `/robot/imu/data`
- **Type**: `sensor_msgs/Imu`
- **Publish Rate**: 100 Hz
- **Frame**: `imu_frame`

#### Additional Interfaces
- **Joint States**: `/robot/joint_states` → `sensor_msgs/JointState` (50 Hz)
- **Transforms**: `/tf` and `/tf_static` → `tf2_msgs/TFMessage` (50 Hz)

### Example Sensor Node Implementation

```python
# Example ROS 2 sensor node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from tf2_ros import TransformBroadcaster
import numpy as np

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Create subscribers for different sensor types
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/robot/lidar_scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/robot/imu/data',
            self.imu_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/robot/depth_camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # Create publisher for processed data
        self.obstacle_pub = self.create_publisher(
            Bool,
            '/robot/obstacle_detected',
            10
        )

        self.get_logger().info('Sensor processor node initialized')

    def lidar_callback(self, msg):
        # Process LiDAR data to detect obstacles
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            if min_distance < 0.5:  # 50 cm threshold
                obstacle_msg = Bool()
                obstacle_msg.data = True
                self.obstacle_pub.publish(obstacle_msg)

    def imu_callback(self, msg):
        # Process IMU data for orientation
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        # Implement your IMU processing logic here
        pass

    def depth_callback(self, msg):
        # Process depth camera data
        # msg.data contains the raw image data
        # msg.width, msg.height for dimensions
        # msg.encoding for data format

        # Implement your depth processing logic here
        pass

def main(args=None):
    rclpy.init(args=args)
    sensor_processor = SensorProcessor()

    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Preparing Perception Pipelines for AI Modules

### Data Preprocessing

Before feeding sensor data to AI modules:

- **Filtering**: Remove noise and outliers
- **Normalization**: Scale data to appropriate ranges
- **Synchronization**: Align data from multiple sensors
- **Calibration**: Correct for sensor-specific characteristics

### Sensor Fusion Techniques

Combining data from multiple sensors:

1. **Early Fusion**: Combine raw sensor data before processing
2. **Late Fusion**: Combine processed results from individual sensors
3. **Deep Fusion**: Use neural networks to learn optimal combination
4. **Kalman Filtering**: Statistically optimal fusion for dynamic systems

### AI Integration

Sensor data feeds into AI modules for:

- **Object detection**: Identifying objects in the environment
- **Localization**: Determining robot position
- **Mapping**: Creating environmental maps
- **Path planning**: Determining safe robot trajectories
- **Behavior prediction**: Anticipating human or object movements

### Example Perception Pipeline

```python
# Example perception pipeline for AI integration
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from cv2 import aruco

class PerceptionPipeline:
    def __init__(self):
        self.lidar_data = None
        self.camera_data = None
        self.imu_data = None

    def process_lidar_data(self, lidar_msg):
        """Process LiDAR point cloud for AI modules"""
        # Convert ROS message to numpy array
        points = np.array(list(pc2.read_points(
            lidar_msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )))

        # Apply noise filtering
        filtered_points = self.filter_point_cloud(points)

        # Create occupancy grid or voxel grid for AI
        occupancy_grid = self.create_occupancy_grid(filtered_points)

        return occupancy_grid

    def process_camera_data(self, image_msg):
        """Process camera data for AI modules"""
        # Convert ROS Image message to OpenCV format
        # Note: This is simplified - actual conversion depends on encoding
        image = self.ros_to_cv2(image_msg)

        # Apply preprocessing for neural networks
        normalized_image = self.normalize_image(image)

        # Run object detection if needed
        detections = self.run_object_detection(normalized_image)

        return detections, normalized_image

    def sensor_fusion(self, lidar_data, camera_data, imu_data):
        """Fuse sensor data for AI modules"""
        # Transform data to common coordinate frame
        lidar_world = self.transform_to_world_frame(lidar_data, imu_data)
        camera_world = self.transform_to_world_frame(camera_data, imu_data)

        # Combine sensor data using appropriate fusion technique
        fused_data = self.combine_sensor_data(lidar_world, camera_world)

        return fused_data

    def prepare_for_ai(self, fused_data):
        """Prepare fused data in format suitable for AI modules"""
        # Format data according to AI model requirements
        ai_input = self.format_for_model(fused_data)

        # Apply any model-specific preprocessing
        preprocessed_input = self.model_specific_preprocessing(ai_input)

        return preprocessed_input

# Example usage in a ROS 2 node
class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Initialize perception pipeline
        self.pipeline = PerceptionPipeline()

        # Create subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/robot/lidar_points', self.lidar_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/robot/camera/image_raw', self.camera_callback, 10
        )

        # Create publisher for AI-ready data
        self.ai_pub = self.create_publisher(
            # Custom message type for AI input
            'ai_msgs/AIInput',
            '/ai/perception_input',
            10
        )

    def lidar_callback(self, msg):
        processed_data = self.pipeline.process_lidar_data(msg)
        self.get_logger().info(f'Processed {len(processed_data)} points')
```

### Best Practices for AI Integration

When preparing sensor data for AI modules:

1. **Consistent Data Formats**: Ensure all sensors provide data in compatible formats
2. **Temporal Alignment**: Synchronize sensor data from the same time window
3. **Spatial Calibration**: Ensure all sensors are properly calibrated to a common frame
4. **Quality Assurance**: Validate sensor data quality before AI processing
5. **Real-time Performance**: Optimize pipeline for real-time operation requirements

## Practical Exercise: Sensor Configuration

Let's configure virtual sensors on a humanoid robot:

1. Set up LiDAR sensor with appropriate parameters
2. Configure depth camera for environment perception
3. Add IMU for motion tracking
4. Observe sensor data streams in ROS 2

```bash
# Configure virtual sensors on the humanoid robot
ros2 launch digital_twin_examples sensor_simulation.launch.py

# Observe sensor data streams in ROS 2
ros2 topic echo /robot/lidar_scan sensor_msgs/msg/LaserScan
```

### Exercise 1: Multi-Sensor Setup
Configure a humanoid robot with:
- 2D LiDAR sensor with 180° FOV
- RGB-D camera with 60° FOV
- 9-DOF IMU
- Joint state publisher

### Exercise 2: Sensor Data Analysis
- Subscribe to all sensor topics
- Analyze the data rates and formats
- Verify temporal synchronization between sensors

### Exercise 3: Perception Pipeline Implementation
- Create a ROS 2 node that subscribes to multiple sensors
- Implement basic sensor fusion
- Publish fused data for AI processing

### Exercise 4: AI Module Integration
- Prepare sensor data in a format suitable for a neural network
- Implement preprocessing pipeline
- Validate data quality and format

## Summary

In this chapter, you learned about simulating various sensors (LiDAR, depth cameras, IMUs) and connecting them to ROS 2 systems. You now understand how sensor data flows into perception pipelines and how to prepare this data for AI modules. This completes the Digital Twin Simulation module, where you've learned about physics simulation, environment building, and sensor simulation for humanoid robots.