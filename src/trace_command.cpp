#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include "rclcpp/rclcpp.hpp" //ROS
 
// Message data
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

//Map values from an input range to an output range
float map(float x, float in_min, float in_max, float out_min, float out_max)
 {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Node class which contains both the publisher and subscriber
class TraceCommand : public rclcpp::Node
{
  public:
    // Constructor for creating and naming the node
    TraceCommand()
    : Node("trace_command")
    {
        //Create subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10, std::bind(&TraceCommand::topic_callback, this, _1));
             
        //Create publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    }
 
  private:
    // Callback function that is entered everytime the subscriber recieves a message
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        //Working with the image
        cv_bridge::CvImagePtr image;
        image = cv_bridge::toCvCopy(msg);
        cv::Point c =  traceBall(image->image);
    
        //Creating the command velocity
        float speed_lin;
        float speed_ang;
        auto vel = geometry_msgs::msg::Twist(); //command velocity message creation
        
        //If no ball is detected, no speed, otherwise map the y-axis coordinate to the speed
        if ((c.x == -1) & (c.y == -1)){
            speed_lin = 0;
            speed_ang = 0;
        }
        else{
            speed_lin = map(c.y,300,-1,-1,2);
            speed_ang = map(c.x,300,-1,-0.2,0.2);
        }
        vel.linear.x = speed_lin;
        vel.angular.z = speed_ang;
       
        publisher_->publish(std::move(vel)); //Publish the velocity message
    }
    // Declare the subscription attribute
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
         
    // Declaration of the publisher attribute      
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
   
};
 

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
