// Copyright 2023 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "path_to_trajectory/csv_to_trajectory.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>


CsvToTrajectory::CsvToTrajectory() : Node("csv_to_trajectory_node") {
  using std::placeholders::_1;
  this->declare_parameter<std::string>("csv_file_path", "");
  this->declare_parameter<float>("velocity_rate", 1.0f);
  this->declare_parameter<float>("trajectory_length", 100.0f);
  this->declare_parameter<float>("trajectory_margin", 2.0f);
  this->declare_parameter<float>("next_point_threshold", 30.0f);

  std::string csv_file_path;
  this->get_parameter("csv_file_path", csv_file_path);
  this->get_parameter("velocity_rate", this->velocity_rate_);
  this->get_parameter("trajectory_length", this->trajectory_length_);
  this->get_parameter("trajectory_margin", this->trajectory_margin_);
  this->get_parameter("next_point_threshold", this->next_point_threshold_);

  if (csv_file_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No CSV file path provided");
      rclcpp::shutdown();
      return;
  }
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "in_odom", qos, std::bind(&CsvToTrajectory::odomCallback, this, _1));
  this->pub_ = this->create_publisher<Trajectory>("output", 1);
  this->readCsv(csv_file_path);

}

void CsvToTrajectory::readCsv(const std::string& file_path) {
  std::ifstream file(file_path);
  std::string line;
  while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') continue; // Skip empty lines and comments
      std::istringstream s(line);
      std::string field;
      std::vector<double> values;

      while (getline(s, field, ';')) {
          values.push_back(std::stod(field));
      }
      // s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
      TrajectoryPoint point;
      point.pose.position.x = values[1];
      point.pose.position.y = values[2];
      point.pose.position.z = 0.0;
      double yaw = values[3]; // Z軸周りの回転角度（ラジアン単位）
      point.pose.orientation.x = 0.0;
      point.pose.orientation.y = 0.0;
      point.pose.orientation.z = sin(yaw / 2);
      point.pose.orientation.w = cos(yaw / 2);
      point.longitudinal_velocity_mps = values[5] * this->velocity_rate_;
      point.acceleration_mps2 = 0.0; //values[6];

      trajectory_points_.push_back(point);
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points", trajectory_points_.size());
}

void CsvToTrajectory::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
  if (current_point_index_ >= trajectory_points_.size()) return;

  Trajectory trajectory;
  // Set trajectory header
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = this->now();
  while(true){
    const float dis = std::hypot(
        trajectory_points_[current_point_index_].pose.position.x - odometry->pose.pose.position.x,
        trajectory_points_[current_point_index_].pose.position.y - odometry->pose.pose.position.y);
    if(dis > next_point_threshold_){
      break;
    }
    current_point_index_++;
  }
  const int start_index = std::max(0, int(current_point_index_ - next_point_threshold_/trajectory_margin_));
  const int end_index = std::min(int(trajectory_points_.size()), int(current_point_index_ + trajectory_length_/trajectory_margin_));
  for(int i = start_index; i < end_index; i++){
    trajectory.points.push_back(trajectory_points_[i]);
  }

  pub_->publish(trajectory);
}

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvToTrajectory>());
  rclcpp::shutdown();
  return 0;
}
