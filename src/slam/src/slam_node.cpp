#include <chrono>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Slam : public rclcpp::Node
{
  public:
    Slam() : Node("slam_node")
    {
        sensor_data_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/sensor/pose", 10, std::bind(&Slam::sensor_pose_callback, this, _1));
        slam_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/slam/pose", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&Slam::slam_publisher_callback, this));
    }

  private:
    void sensor_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        sensor_pose_valid = true;
        latest_sensor_pose_msg = *msg;
    }

    void slam_publisher_callback()
    {
        geometry_msgs::msg::Pose slam_pose{};
        if (sensor_pose_valid)
        {
            slam_pose = latest_sensor_pose_msg;
        }
        slam_publisher_->publish(slam_pose);
        sensor_pose_valid = false;
    }

    bool sensor_pose_valid{false};
    geometry_msgs::msg::Pose latest_sensor_pose_msg{};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr slam_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sensor_data_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Slam>());
    rclcpp::shutdown();
    return 0;
}