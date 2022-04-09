// Include the C++ standard library headers
#include <memory> // Dynamic memory management
 
// Dependencies
#include "rclcpp/rclcpp.hpp" // ROS Clienty Library for C++
#include "std_msgs/msg/string.hpp" // Handles String messages in ROS 2
using std::placeholders::_1;
 
class MinimalSubscriber : public rclcpp::Node
{
  public:
    // Constructor
    // The name of the node is minimal_subscriber
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      // Create the subscription.
      // The topic_callback function executes whenever data is published
      // to the 'addison' topic.
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "addison", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }
 
  private:
    // Receives the String message that is published over the topic
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      // Write the message that was received on the console window
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    // Declare the subscription attribute
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
 
int main(int argc, char * argv[])
{
  // Launch ROS 2
  rclcpp::init(argc, argv);
   
  // Prepare to receive messages that arrive on the topic
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
   
  // Shutdown routine for ROS2
  rclcpp::shutdown();
  return 0;
}

