#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class SensorPosePublisher : public rclcpp::Node
{
  public:
    SensorPosePublisher() : Node("sensor_pose_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/sensor/pose", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&SensorPosePublisher::callback, this));
    }
  
  private:
    void callback()
    {
      auto message = geometry_msgs::msg::Pose();
      message.position.x = 0.5;
      message.position.y = 1.0;
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
