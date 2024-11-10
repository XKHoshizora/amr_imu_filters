# AMR IMU Filters

ROS package for advanced IMU data filtering, supporting both Extended Kalman Filter (EKF) and complementary filter algorithms.

## Features

- **Multiple Filter Options**

  - Extended Kalman Filter (EKF)
  - Complementary Filter
  - Runtime switchable filter types

- **Sensor Support**

  - IMU (accelerometer + gyroscope)
  - Optional magnetometer integration

- **Advanced Functionality**
  - Automatic gyroscope bias estimation
  - Dynamic parameter reconfiguration
  - Comprehensive diagnostics
  - Real-time frequency monitoring
  - Static state detection
  - Thread-safe data processing

## Dependencies

- ROS Noetic
- Eigen3
- tf2
- diagnostic_updater
- dynamic_reconfigure

## Installation

1. Clone the repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/your-username/amr_imu_filters.git
```

2. Install dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:

```bash
cd ~/catkin_ws
catkin_make
```

## Usage

1. Launch the IMU filter node with default settings:

```bash
roslaunch amr_imu_filters imu_filter.launch
```

2. Launch with custom topic names:

```bash
roslaunch amr_imu_filters imu_filter.launch custom_topics:=true input_topic:=/custom/imu/data output_topic:=/custom/imu/filtered
```

3. Input/Output Topics:
   - Input:
     - `/imu/data` (sensor_msgs/Imu)
     - `/magnetic` (sensor_msgs/MagneticField) [optional]
   - Output:
     - `/imu/filtered` (sensor_msgs/Imu)
     - `/mag/filtered` (sensor_msgs/MagneticField) [if magnetometer enabled]

## Configuration

### Launch File Parameters

```xml
<!-- Filter Configuration -->
<param name="filter_type" value="EKF" />              <!-- Options: EKF, COMPLEMENTARY -->
<param name="use_mag" value="false" />                <!-- Enable/disable magnetometer -->
<param name="fixed_frame" value="odom" />             <!-- Fixed frame ID -->
<param name="expected_publish_freq" value="20.0" />   <!-- Publishing frequency in Hz -->

<!-- Complementary Filter Parameters -->
<param name="alpha" value="0.96" />                   <!-- Complementary filter coefficient -->

<!-- EKF Parameters -->
<param name="process_noise_gyro" value="0.0001" />    <!-- Process noise for gyroscope -->
<param name="process_noise_accel" value="0.001" />    <!-- Process noise for accelerometer -->
<param name="process_noise_bias" value="0.00001" />   <!-- Process noise for gyro bias -->
<param name="measurement_noise_accel" value="0.1" />  <!-- Measurement noise for accelerometer -->
<param name="measurement_noise_mag" value="0.01" />   <!-- Measurement noise for magnetometer -->

<!-- Static Detection Parameters -->
<param name="static_threshold" value="0.005" />       <!-- Threshold for static state detection -->
<param name="static_samples" value="100" />           <!-- Number of samples for initialization -->
```

### Dynamic Reconfigure Parameters

The following parameters can be adjusted in real-time using rqt_reconfigure:

- **Filter Parameters**
  - `alpha`: Complementary filter coefficient (0.0-1.0)
  - `process_noise_gyro`: Process noise for gyroscope
  - `process_noise_accel`: Process noise for accelerometer
  - `process_noise_bias`: Process noise for gyro bias
  - `measurement_noise_accel`: Measurement noise for accelerometer
  - `measurement_noise_mag`: Measurement noise for magnetometer

## Diagnostics

The node provides diagnostic information through the `/diagnostics` topic:

- Filter status and type
- Input data validity
- Processing frequency
- Static state detection
- Initialization status

View diagnostics using:

```bash
rosrun rqt_robot_monitor rqt_robot_monitor
```

Monitor real-time data:

```bash
# View filtered orientation
rostopic echo /imu/filtered/orientation

# Plot orientation in rqt_plot
rqt_plot /imu/filtered/orientation/x /imu/filtered/orientation/y /imu/filtered/orientation/z /imu/filtered/orientation/w
```

## Testing

Run the unit tests:

```bash
catkin_make run_tests_amr_imu_filters
```

## Contributing

1. Fork the repository
2. Create your feature branch:

```bash
git checkout -b feature/my-new-feature
```

3. Commit your changes:

```bash
git commit -am 'Add some feature'
```

4. Push to the branch:

```bash
git push origin feature/my-new-feature
```

5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- EKF implementation based on principles from "Quaternion kinematics for the error-state Kalman filter" by Joan Solà
- Complementary filter design inspired by best practices in IMU sensor fusion

## Authors

- **Hoshizora** - _Initial work_ - [XKHoshizora](https://github.com/XKHoshizora)

## Project Status

This package is actively maintained. Issues and pull requests are welcome.
