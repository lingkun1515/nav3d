// Standalone IMU gyro yaw integrator from a rosbag2 (no rosbag2_py needed).
// Reads /livox/imu over [t_start, t_end] (seconds, bag-relative) and integrates
// the gyro z-axis to get the yaw delta. Prints YAWDELTA <rad>.
//
// Usage: imu_yaw_delta --bag <dir> --imu-topic /livox/imu --start 16 --end 18
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  std::string bag, imu_topic = "/livox/imu";
  double t_start = 0, t_end = 0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](int& k){ return (k+1<argc)?argv[++k]:""; };
    if (a=="--bag") bag=next(i);
    else if (a=="--imu-topic") imu_topic=next(i);
    else if (a=="--start") t_start=std::stod(next(i));
    else if (a=="--end") t_end=std::stod(next(i));
  }
  if (bag.empty() || t_end<=t_start) { std::cerr<<"--bag --start --end required\n"; return 1; }
  rclcpp::init(argc, argv);
  rosbag2_cpp::readers::SequentialReader reader;
  reader.open({bag,"sqlite3"},{"cdr","cdr"});
  reader.set_filter(rosbag2_storage::StorageFilter{{imu_topic}});
  auto ser = rclcpp::Serialization<sensor_msgs::msg::Imu>();
  int64_t t0=-1, prev_t=-1;
  double yaw=0;
  while (reader.has_next()) {
    auto m = reader.read_next();
    if (t0<0) t0=m->time_stamp;
    double rel=(m->time_stamp-t0)/1e9;
    if (rel<t_start) continue;
    if (rel>t_end) break;
    sensor_msgs::msg::Imu imu;
    rclcpp::SerializedMessage s(*m->serialized_data);
    ser.deserialize_message(&s, &imu);
    if (prev_t>=0) yaw += imu.angular_velocity.z * (m->time_stamp-prev_t)/1e9;
    prev_t = m->time_stamp;
  }
  std::cout<<"YAWDELTA "<<yaw<<"\n";
  return 0;
}
