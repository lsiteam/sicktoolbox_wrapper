// ROS 2 version of sicklms.cpp (simplified migration base)
#include <memory>
#include <limits>
#include <sicktoolbox/SickLMS2xx.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

using namespace SickToolbox;
using std::placeholders::_1;

SickLMS2xx::sick_lms_2xx_measuring_units_t StringToLmsMeasuringUnits(const std::string &units) {
  if (units == "mm") return SickLMS2xx::SICK_MEASURING_UNITS_MM;
  if (units == "cm") return SickLMS2xx::SICK_MEASURING_UNITS_CM;
  return SickLMS2xx::SICK_MEASURING_UNITS_UNKNOWN;
}

void publish_scan(diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::LaserScan> &pub, uint32_t *range_values,
                  uint32_t n_range_values, uint32_t *intensity_values, uint32_t n_intensity_values, double scale,
                  rclcpp::Time start, double scan_time, bool inverted, float angle_min, float angle_max,
                  const std::string &frame_id) {

  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header.frame_id = frame_id;
  scan_msg.header.stamp = start;
  scan_msg.angle_min = inverted ? angle_max : angle_min;
  scan_msg.angle_max = inverted ? angle_min : angle_max;
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(n_range_values - 1);
  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (2 * M_PI) * scan_msg.angle_increment;
  scan_msg.range_min = 0;
  scan_msg.range_max = (scale == 0.01) ? 81.0 : (scale == 0.001 ? 8.1 : 0);

  scan_msg.ranges.resize(n_range_values);
  for (size_t i = 0; i < n_range_values; ++i) {
    switch (range_values[i]) {
      case 8191: case 8190: case 8189: case 8187: case 8186:
        scan_msg.ranges[i] = std::numeric_limits<float>::quiet_NaN(); break;
      case 8183:
        scan_msg.ranges[i] = std::numeric_limits<float>::infinity(); break;
      default:
        scan_msg.ranges[i] = static_cast<float>(range_values[i]) * static_cast<float>(scale);
    }
  }
  scan_msg.intensities.resize(n_intensity_values);
  for (size_t i = 0; i < n_intensity_values; ++i)
    scan_msg.intensities[i] = static_cast<float>(intensity_values[i]);

  pub.publish(scan_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sicklms");

  // Declare parameters
  std::string port = node->declare_parameter("port", std::string("/dev/lms200"));
  int baud = node->declare_parameter("baud", 38400);
  int delay = node->declare_parameter("connect_delay", 0);
  bool inverted = node->declare_parameter("inverted", false);
  int angle = node->declare_parameter("angle", 0);
  double resolution = node->declare_parameter("resolution", 0.0);
  std::string measuring_units = node->declare_parameter("units", "");
  std::string frame_id = node->declare_parameter("frame_id", "laser");
  double time_offset_sec = node->declare_parameter("time_offset", 0.0);
  rclcpp::Duration time_offset = rclcpp::Duration::from_seconds(time_offset_sec);

  // Diagnostics params
  double desired_freq = node->declare_parameter("desired_frequency", 75.0);
  double min_freq = node->declare_parameter("min_frequency", desired_freq);
  double max_freq = node->declare_parameter("max_frequency", desired_freq);
  double freq_tolerance = node->declare_parameter("frequency_tolerance", 0.3);
  int window_size = node->declare_parameter("window_size", 30);
  double min_delay = node->declare_parameter("min_acceptable_delay", 0.0);
  double max_delay = node->declare_parameter("max_acceptable_delay", 0.2);
  std::string hardware_id = node->declare_parameter("hardware_id", "SICK LMS");

  diagnostic_updater::Updater updater(node);
  updater.setHardwareID(hardware_id);
  auto scan_pub = diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::LaserScan>(
      node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10),
      updater,
      diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, freq_tolerance, window_size),
      diagnostic_updater::TimeStampStatusParam(min_delay, max_delay));

  SickLMS2xx::sick_lms_2xx_baud_t desired_baud = SickLMS2xx::IntToSickBaud(baud);
  if (desired_baud == SickLMS2xx::SICK_BAUD_UNKNOWN) {
    RCLCPP_ERROR(node->get_logger(), "Invalid baud rate");
    return 1;
  }

  SickLMS2xx sick_lms(port.c_str());
  uint32_t range_values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS] = {0};
  uint32_t intensity_values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS] = {0};
  uint32_t n_range_values = 0, n_intensity_values = 0;
  double scale = 0;
  double scan_time = 0, angle_offset = 0;
  float angle_min = 0.0, angle_max = 0.0;
  uint32_t partial_scan_index;

  try {
    sick_lms.Initialize(desired_baud, delay);
    int actual_angle = sick_lms.GetSickScanAngle();
    double actual_resolution = sick_lms.GetSickScanResolution();
    auto actual_units = sick_lms.GetSickMeasuringUnits();

    try {
      if ((angle && actual_angle != angle) || (resolution && actual_resolution != resolution)) {
        sick_lms.SetSickVariant(sick_lms.IntToSickScanAngle(angle), sick_lms.DoubleToSickScanResolution(resolution));
      } else {
        angle = actual_angle;
        resolution = actual_resolution;
      }
    } catch (...) {
      RCLCPP_WARN(node->get_logger(), "Using fallback scan angle/resolution");
    }

    try {
      if (!measuring_units.empty() && (actual_units != StringToLmsMeasuringUnits(measuring_units))) {
        actual_units = StringToLmsMeasuringUnits(measuring_units);
        sick_lms.SetSickMeasuringUnits(actual_units);
      }
    } catch (...) {
      RCLCPP_WARN(node->get_logger(), "Failed to set measuring units");
    }

    if (actual_units == SickLMS2xx::SICK_MEASURING_UNITS_CM) scale = 0.01;
    else if (actual_units == SickLMS2xx::SICK_MEASURING_UNITS_MM) scale = 0.001;
    else {
      RCLCPP_ERROR(node->get_logger(), "Invalid measuring units");
      return 1;
    }

    if (angle == 180 || sick_lms.IsSickLMS2xxFast()) scan_time = 1.0 / 75;
    else {
      auto res_enum = SickLMS2xx::DoubleToSickScanResolution(resolution);
      if (res_enum == SickLMS2xx::SICK_SCAN_RESOLUTION_25) scan_time = 4.0 / 75;
      else if (res_enum == SickLMS2xx::SICK_SCAN_RESOLUTION_50) scan_time = 2.0 / 75;
      else if (res_enum == SickLMS2xx::SICK_SCAN_RESOLUTION_100) scan_time = 1.0 / 75;
      else return 1;
    }
    angle_offset = (180.0 - angle) / 2;
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize LMS");
    return 1;
  }

  rclcpp::Rate loop_rate(75);
  while (rclcpp::ok()) {
    try {
      if (sick_lms.IsSickLMS2xxFast()) {
        sick_lms.GetSickScan(range_values, intensity_values, n_range_values, n_intensity_values);
        angle_min = -M_PI / 4;
        angle_max = M_PI / 4;
      } else if (angle != 180) {
        sick_lms.GetSickScan(range_values, n_range_values);
        angle_min = (-90.0 + angle_offset) * M_PI / 180.0;
        angle_max = (90.0 - angle_offset) * M_PI / 180.0;
      } else {
        sick_lms.GetSickPartialScan(range_values, n_range_values, partial_scan_index);
        double offset = 0.25 * partial_scan_index;
        angle_min = (-90.0 + angle_offset + offset) * M_PI / 180.0;
        angle_max = (90.0 - angle_offset - fmod(1.0 - offset, 1.0)) * M_PI / 180.0;
      }
      auto end_of_scan = node->get_clock()->now();
      auto start = end_of_scan - rclcpp::Duration::from_seconds(scan_time / 2.0) + time_offset;

      publish_scan(scan_pub, range_values, n_range_values, intensity_values, n_intensity_values, scale, start,
                   scan_time, inverted, angle_min, angle_max, frame_id);
      //updater.update();
      rclcpp::spin_some(node);
      loop_rate.sleep();
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Scan error");
      break;
    }
  }

  try { sick_lms.Uninitialize(); } catch (...) {}
  rclcpp::shutdown();
  return 0;
}

