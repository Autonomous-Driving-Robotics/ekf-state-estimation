#include <chrono>

#include "geometry_msgs/msg/pose.hpp"
#include "pose_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SensorPosePublisher : public rclcpp::Node
{
  public:
    SensorPosePublisher() : Node("sensor_pose_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/sensor/pose", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&SensorPosePublisher::callback, this));
    }

  private:
    void callback()
    {
        auto message = geometry_msgs::msg::Pose();
        pose_publisher::Position pose{generator.GetPosition(0.05)};
        message.position.x = pose.x;
        message.position.y = pose.y;
        publisher_->publish(message);
    }
    pose_publisher::DataGenerator generator{0.0, 0.0, 0.01, 0.01};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
