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



void hexDump (const void * data, const int len)
{
    int i;
    char buff[17];
    const unsigned char * pc = (const unsigned char *) data;
    // Length checks
    if (len == 0) {
        printf("  ZERO LENGTH\n"); return;
    }
    else if (len < 0) {
        printf("  NEGATIVE LENGTH: %d\n", len); return;
    }

    // Process every byte in the data
    for (i = 0; i < len; i++) 
    {
        // Multiple of 16 means new line (with line offset).
        if ((i % 16) == 0) 
        {
            // Don't print ASCII buffer for the "zeroth" line.
            if (i != 0)
                printf ("  %s\n", buff);
            // Output the offset.
            printf ("  %04x ", i);
        }

        // Now the hex code for the specific character.
        printf (" %02x", pc[i]);

        // And buffer a printable ASCII character for later.
        if ((pc[i] < 0x20) || (pc[i] > 0x7e)) // isprint() may be better.
            buff[i % 16] = '.';
        else
            buff[i % 16] = pc[i];
        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0) {
        printf ("   ");
        i++;
    }

    // And print the final ASCII buffer.
    printf ("  %s\n\n", buff);
}

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{

    sensor_msgs::msg::Imu* dataPtr = msg.get();

    hexDump(dataPtr, sizeof(sensor_msgs::msg::Imu));


    modified_msg_ = *msg;
    //modified_msg_.angular_velocity_covariance[0] = 0.0004;
    //modified_msg_.angular_velocity_covariance[4] = 0.0004;
    //modified_msg_.angular_velocity_covariance[8] = 0.0004;
    //modified_msg_.linear_acceleration_covariance[0] = 0.0004;
    //modified_msg_.linear_acceleration_covariance[4] = 0.0004;
    //modified_msg_.linear_acceleration_covariance[8] = 0.0004;

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
