// Include important C++ header files that provide class
// templates for useful operations.
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
 
// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"
 
using std::placeholders::_1;
 
// Create the node class named PublishingSubscriber which inherits the attributes
// and methods of the rclcpp::Node class.
class PublishingSubscriber : public rclcpp::Node
{
  public:
    // Constructor creates a node named publishing_subscriber. 
    // The published message count is initialized to 0.
    PublishingSubscriber()
    : Node("publishing_subscriber")
    {
      // Create the subscription.
      // The topic_callback function executes whenever data is published
      // to the 'addison' topic.
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "addison", 10, std::bind(&PublishingSubscriber::topic_callback, this, _1));
             
      // Publisher publishes String messages to a topic named "addison2". 
      // The size of the queue is 10 messages.
      publisher_ = this->create_publisher<std_msgs::msg::String>("addison2",10);
       
    }
 
  private:
    // Receives the String message that is published over the topic
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      // Create a new message of type String
      auto message = std_msgs::msg::String();
       
      // Set our message's data attribute
      message.data = "I heard " + msg->data;
 
      // Publish the message to the topic named "addison2"
      publisher_->publish(message);
    }
    // Declare the subscription attribute
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
         
    // Declaration of the publisher_ attribute      
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
   
};
 
// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Start processing data from the node as well as the callbacks
  rclcpp::spin(std::make_shared<PublishingSubscriber>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}
