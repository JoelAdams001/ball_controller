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

cv::Point computeCentroid(const cv::Mat &mask) {
	// find moments of the image
	cv::Moments m = cv::moments(mask, true);
		if( m.m00 <= 1e-4  )
			return cv::Point(-1, -1);
	return cv::Point(m.m10/m.m00, m.m01/m.m00);
}

cv::Mat createMask( const cv::Mat &image, 
                    const cv::Scalar &lower,
                    const cv::Scalar &upper,
                    bool clean = true ) {
	// Convert frame image from BGR to HSV color space
	cv::Mat hsv, kernel;
	cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	// Create a mask using calibrated color
    cv::Mat mask;
    cv::inRange(hsv, lower, upper, mask);
    if (clean) {
        // Clean isolated pixels
        cv::erode(mask, mask, kernel, cv::Point(-1,-1), 4);
        // Enhance surviving pixels
        cv::dilate(mask, mask, kernel, cv::Point(-1,-1), 4);
    }
    return mask;
}

cv::Point traceBall(const cv::Mat &image) {
   // Define heu level of treshold color
	int color = 46; // Some sort of lime color!!!
	// Define lower and upper treshold color range
	cv::Scalar lower(color - 24, 127, 76);
	cv::Scalar upper(color + 48, 182, 255);
	// Create mask	
	auto mask = createMask(image, lower, upper);
	// Compute center of the mask
	return computeCentroid(mask);
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
 {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_demo",10);
       
    }
 
  private:
    // Receives the String message that is published over the topic
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        //Working with the image
        cv_bridge::CvImagePtr image;
        image = cv_bridge::toCvCopy(msg);
        cv::Point c =  traceBall(image->image);
        
        //Creating the command velocity
        geometry_msgs::msg::Twist vel = geometry_msgs::msg::Twist();
        float speed = map(c.y,300,0,0,2);
        vel.linear.x = speed;
       
 
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
