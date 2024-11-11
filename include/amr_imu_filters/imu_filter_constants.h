#ifndef AMR_LAUNCHER_IMU_FILTER_CONSTANTS_H
#define AMR_LAUNCHER_IMU_FILTER_CONSTANTS_H

namespace amr_launcher {
namespace imu_filter {

// 默认参数值
constexpr int DEFAULT_WINDOW_SIZE = 5;
constexpr int DEFAULT_INIT_SAMPLES = 100;
constexpr double DEFAULT_ACC_LPF_ALPHA = 0.2;
constexpr double DEFAULT_GYRO_LPF_ALPHA = 0.2;
constexpr double DEFAULT_GRAVITY_MAGNITUDE = 9.80665;

// 默认话题名称
const char* const DEFAULT_INPUT_TOPIC = "/imu";
const char* const DEFAULT_OUTPUT_TOPIC = "/imu_filtered";
const char* const DEFAULT_BASE_FRAME_ID = "base_link";
const char* const DEFAULT_IMU_FRAME_ID = "imu_link";

// 数据有效性检查范围
constexpr double MIN_ACC_MAGNITUDE = 1.0;
constexpr double MAX_ACC_MAGNITUDE = 30.0;

// 默认协方差值
constexpr double DEFAULT_ORIENTATION_STDDEV = 0.01;
constexpr double DEFAULT_ANGULAR_VELOCITY_STDDEV = 0.01;
constexpr double DEFAULT_LINEAR_ACCELERATION_STDDEV = 0.01;

}  // namespace imu_filter
}  // namespace amr_launcher

#endif  // AMR_LAUNCHER_IMU_FILTER_CONSTANTS_H