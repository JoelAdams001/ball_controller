#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
 
// Built-in message type that will be used to publish data
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"

//OpenCV and bridge for converting between ROS and OpenCV images
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <utility>
 
using std::placeholders::_1;
 
// Create the node class named PublishingSubscriber which inherits the attributes
// and methods of the rclcpp::Node class.
class TraceCommand : public rclcpp::Node
{
  public:
    // Constructor creates a node named publishing_subscriber. 
    // The published message count is initialized to 0.
    TraceCommand()
    : Node("trace_command")
    {
      // Create the subscription.
      // The topic_callback function executes whenever data is published
      // to the 'image_raw' topic.
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, std::bind(&TraceCommand::topic_callback, this, _1));
             
      // Publisher publishes String messages to a topic named "addison2". 
      // The size of the queue is 10 messages.
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
       
    }
 
  private:
    // Receives the String message that is published over the topic
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        cv_bridge::CvImagePtr image;
        //image = cv_bridge::toCvCopy(msg,'BGR8');
   //     image = cv_bridge::toCvCopy(msg);
        geometry_msgs::msg::Twist vel = geometry_msgs::msg::Twist();
        vel.linear.x = 1.0;
       
 
        publisher_->publish(std::move(vel));
    }
    // Declare the subscription attribute
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
         
    // Declaration of the publisher_ attribute      
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
   
};
 
// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Start processing data from the node as well as the callbacks
  auto node = std::make_shared<TraceCommand>();
  rclcpp::spin(node);
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}
