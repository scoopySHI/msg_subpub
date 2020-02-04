#include<rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>
using std::placeholders::_1;


class Subpub : public rclcpp::Node
{
public:
  Subpub() : Node("subpub_node")
  {
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", 10, std::bind(&Subpub::imuCallback, this, _1));

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "imu", 10);
}

private:

  sensor_msgs::msg::Imu modified_msg_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_sub_;

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  modified_msg_ = *msg;
  modified_msg_.angular_velocity_covariance[0] = 0.0004;
  modified_msg_.angular_velocity_covariance[4] = 0.0004;
  modified_msg_.angular_velocity_covariance[8] = 0.0004;
  modified_msg_.linear_acceleration_covariance[0] = 0.0004;
  modified_msg_.linear_acceleration_covariance[4] = 0.0004;
  modified_msg_.linear_acceleration_covariance[8] = 0.0004;

  modified_msg_.header.stamp = this->get_clock()->now();

  imu_pub_->publish(modified_msg_);
}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto subpub_node = std::make_shared<Subpub>();
  rclcpp::spin(subpub_node);
  rclcpp::shutdown();
  return 0;
}
