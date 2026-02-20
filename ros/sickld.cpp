// ROS 2 version of sickld.cpp
#include <iostream>
#include <sicktoolbox/SickLD.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <deque>
#include <cmath>

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace std;
using namespace SickToolbox;
using std::placeholders::_1;

class SickLDNode : public rclcpp::Node {
public:
  SickLDNode() : Node("sickld") {
    declare_parameter<int>("port", DEFAULT_SICK_TCP_PORT);
    declare_parameter<std::string>("ipaddress", DEFAULT_SICK_IP_ADDRESS);
    declare_parameter<std::string>("frame_id", "laser");
    declare_parameter<bool>("inverted", true);
    declare_parameter<double>("timer_smoothing_factor", 0.97);
    declare_parameter<double>("timer_error_threshold", 0.5);
    declare_parameter<double>("resolution", 1.0);
    declare_parameter<double>("start_angle", 0.0);
    declare_parameter<double>("stop_angle", 300.0);
    declare_parameter<int>("scan_rate", 10);

    get_parameter("port", port);
    get_parameter("ipaddress", ipaddress);
    get_parameter("frame_id", frame_id);
    get_parameter("inverted", inverted);
    get_parameter("timer_smoothing_factor", smoothing_factor);
    get_parameter("timer_error_threshold", error_threshold);
    get_parameter("resolution", sick_step_angle);
    get_parameter("start_angle", active_sector_start_angle);
    get_parameter("stop_angle", active_sector_stop_angle);
    get_parameter("scan_rate", sick_motor_speed);

    rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos_profile);
    timer_ = this->create_wall_timer(10ms, std::bind(&SickLDNode::timer_callback, this));
  }

private:
  void publish_scan(double *range_values, uint32_t n_range_values, unsigned int *intensity_values,
                    uint32_t n_intensity_values, rclcpp::Time start, double scan_time, bool inverted,
                    float angle_min, float angle_max, const std::string &frame_id) {
    sensor_msgs::msg::LaserScan scan_msg;

    scan_msg.angle_min = inverted ? angle_max : angle_min;
    scan_msg.angle_max = inverted ? angle_min : angle_max;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / static_cast<double>(n_range_values - 1);
    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / n_range_values;
    scan_msg.range_min = 0.5;
    scan_msg.range_max = 250.0;

    scan_msg.ranges.resize(n_range_values);
    for (size_t i = 0; i < n_range_values; i++) {
      scan_msg.ranges[i] = static_cast<float>(range_values[i]);
    }
    scan_msg.intensities.resize(n_intensity_values);
    for (size_t i = 0; i < n_intensity_values; i++) {
      scan_msg.intensities[i] = static_cast<float>(intensity_values[i]);
    }
    scan_msg.header.frame_id = frame_id;
    scan_msg.header.stamp = this->now();
    scan_pub->publish(scan_msg);
  }

  void timer_callback() {
    static SickLD sick_ld(ipaddress.c_str(), port);
    static bool initialized = false;
    static unsigned int last_sector_stop_timestamp = 0;
    static rclcpp::Time last_start_scan_time;
    static double full_duration;
    static averager avg_fulldur, avg_scandur;
    static smoothtime smoothtimer;

    if (!initialized) {
      try {
        sick_ld.Initialize();
        sick_ld.SetSickGlobalParamsAndScanAreas(static_cast<unsigned int>(sick_motor_speed), sick_step_angle,
                                                &active_sector_start_angle, &active_sector_stop_angle, 1);
        smoothtimer.set_smoothing_factor(smoothing_factor);
        smoothtimer.set_error_threshold(error_threshold);
        initialized = true;
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize SickLD");
        rclcpp::shutdown();
        return;
      }
    }

    double range_values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};
    unsigned int intensity_values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};
    unsigned int num_measurements = 0, sector_start_timestamp = 0, sector_stop_timestamp = 0;
    double sector_step_angle = 0, sector_start_angle = 0, sector_stop_angle = 0;

    sick_ld.GetSickMeasurements(range_values, intensity_values, &num_measurements, NULL, NULL,
                                &sector_step_angle, &sector_start_angle, &sector_stop_angle,
                                &sector_start_timestamp, &sector_stop_timestamp);

    auto end_scan_time = this->now();
    double scan_duration = (sector_stop_timestamp - sector_start_timestamp) * 1e-3;
    avg_scandur.add_new(scan_duration);
    scan_duration = avg_scandur.get_mean();

    full_duration = (last_sector_stop_timestamp == 0) ? (1. / sick_motor_speed)
                                                      : ((sector_stop_timestamp - last_sector_stop_timestamp) * 1e-3);
    avg_fulldur.add_new(full_duration);
    full_duration = avg_fulldur.get_mean();

    auto smoothed_end_scan_time = smoothtimer.smooth_timestamp(end_scan_time, rclcpp::Duration::from_seconds(full_duration));
    auto start_scan_time = smoothed_end_scan_time - rclcpp::Duration::from_seconds(scan_duration);

    publish_scan(range_values, num_measurements, intensity_values, num_measurements, start_scan_time, scan_duration,
                 inverted, DEG2RAD(sector_start_angle), DEG2RAD(sector_stop_angle), frame_id);

    last_start_scan_time = start_scan_time;
    last_sector_stop_timestamp = sector_stop_timestamp;
  }

  class smoothtime {
  public:
    rclcpp::Time smooth_timestamp(rclcpp::Time recv_timestamp, rclcpp::Duration expctd_dur) {
      if (smoothtime_prev.nanoseconds() == 0) {
        smoothed_timestamp = recv_timestamp;
      } else {
        smoothed_timestamp = smoothtime_prev + expctd_dur;
        double err = (recv_timestamp - smoothed_timestamp).seconds();
        double time_error_threshold = expctd_dur.seconds() * error_threshold;
        if ((time_smoothing_factor > 0) && (fabs(err) < time_error_threshold)) {
          auto correction = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
  	std::chrono::duration<double>(err * (1 - time_smoothing_factor))));

          smoothed_timestamp += correction;
        } else {
          smoothed_timestamp = recv_timestamp;
        }
      }
      smoothtime_prev = smoothed_timestamp;
      return smoothed_timestamp;
    }

    void set_smoothing_factor(double factor) { time_smoothing_factor = factor; }
    void set_error_threshold(double threshold) { error_threshold = threshold; }

  private:
    rclcpp::Time smoothtime_prev, smoothed_timestamp;
    double time_smoothing_factor{0.95};
    double error_threshold{0.5};
  };

  class averager {
  public:
    averager(int max_len = 50) : max_len(max_len), sum(0.0) {}
    void add_new(double data) {
      deq.push_back(data);
      sum += data;
      if (deq.size() > max_len) {
        sum -= deq.front();
        deq.pop_front();
      }
    }
    double get_mean() const { return sum / deq.size(); }

  private:
    std::deque<double> deq;
    unsigned int max_len;
    double sum;
  };

  int port;
  std::string ipaddress;
  std::string frame_id;
  bool inverted;
  double sick_step_angle, active_sector_start_angle, active_sector_stop_angle;
  double smoothing_factor, error_threshold;
  int sick_motor_speed;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SickLDNode>());
  rclcpp::shutdown();
  return 0;
}

