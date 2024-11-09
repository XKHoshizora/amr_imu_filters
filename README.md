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
  - Data validity checking

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

1. Launch the IMU filter node:
```bash
roslaunch amr_imu_filters imu_filter.launch
```

2. Input/Output Topics:
   - Input:
     - `/imu` (sensor_msgs/Imu)
     - `/magnetic` (sensor_msgs/MagneticField) [optional]
   - Output:
     - `/imu_filtered/data` (sensor_msgs/Imu)
     - `/imu_filtered/mag` (sensor_msgs/MagneticField) [if magnetometer enabled]

## Configuration

### Launch File Parameters

```xml
<param name="filter_type" value="EKF" />          <!-- Options: EKF, COMPLEMENTARY -->
<param name="use_mag" value="false" />            <!-- Enable/disable magnetometer -->
<param name="expected_publish_freq" value="20.0" /> <!-- Publishing frequency in Hz -->
```

### Dynamic Reconfigure Parameters

The following parameters can be adjusted in real-time using rqt_reconfigure:

- **Filter Parameters**
  - `filter_type`: Switch between EKF and complementary filter
  - `use_mag`: Enable/disable magnetometer usage
  - `alpha`: Complementary filter coefficient (0.0-1.0)

- **EKF Parameters**
  - `process_noise_gyro`: Process noise for gyroscope
  - `process_noise_accel`: Process noise for accelerometer
  - `process_noise_bias`: Process noise for gyro bias
  - `measurement_noise_accel`: Measurement noise for accelerometer
  - `measurement_noise_mag`: Measurement noise for magnetometer

## Diagnostics

The node provides diagnostic information through the `/diagnostics` topic:

- Filter frequency monitoring
- Data validity checks
- Runtime status

View diagnostics using:
```bash
rosrun rqt_robot_monitor rqt_robot_monitor
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

- Initial IMU filter implementation inspired by [imu_tools](https://github.com/ccny-ros-pkg/imu_tools)
- EKF implementation based on principles from "Quaternion kinematics for the error-state Kalman filter" by Joan Solà

## Authors

* **Hoshizora** - *Initial work* - [XKHoshizora](https://github.com/XKHoshizora)

## Project Status

This package is actively maintained. Issues and pull requests are welcome.