// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
 
// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"
 
// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;
 
// Create the node class named MinimalPublisher which inherits the attributes
// and methods of the rclcpp::Node class.
class MinimalPublisher : public rclcpp::Node
{
  public:
    // Constructor creates a node named minimal_publisher. 
    // The published message count is initialized to 0.
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      // Publisher publishes String messages to a topic named "addison". 
      // The size of the queue is 10 messages.
      publisher_ = this->create_publisher<std_msgs::msg::String>("addison",10);
       
      // Initialize the timer. The timer_callback function will execute every
      // 500 milliseconds.
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
 
  private:
    // This method executes every 500 milliseconds
    void timer_callback()
    {
      // Create a new message of type String
      auto message = std_msgs::msg::String();
       
      // Set our message's data attribute and increment the message count by 1
      message.data = "Hi Automatic Addison! " + std::to_string(count_++);
 
      // Print every message to the terminal window      
      RCLCPP_INFO(this->get_logger(),"Publishing: '%s'", message.data.c_str());
       
      // Publish the message to the topic named "addison"
      publisher_->publish(message);
    }
     
    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_;
  
    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
   
    // Declaration of the count_ attribute
    size_t count_;
};
 
// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Start processing data from the node as well as the callbacks and the timer
  rclcpp::spin(std::make_shared<MinimalPublisher>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}
