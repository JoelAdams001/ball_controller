#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <string>

void info(void) {
	std::cout << "EML 4840 Robot Design" << std::endl
		  << "Florida International University" << std::endl
		  << "Department of Mechanical and Materials Engineering" << std::endl
		  << "Anthony Abrahao <aabra037@fiu.edu>" << std::endl
		  << "Miami, Spring 2019" << std::endl
		  << std::endl;
}

#define DEFAULT_CAMERA 0
#define KEY_ESC 27

// Windows
const std::string window = "FIU-EML4840: Image";
float g_zoom = 0.75f;

// Remember track
std::vector<cv::Point> g_track;

cv::Mat resize(const cv::Mat &image, float factor) {
    cv::Size s = image.size();
    cv::Mat resized;
    cv::resize( image, resized, cv::Size( factor*s.width, factor*s.height ) );
    return resized;  
}

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
	cv::Scalar lower(color - 20, 80, 80);
	cv::Scalar upper(color + 20, 255, 255);
	// Create mask	
	auto mask = createMask(image, lower, upper);
	// Compute center of the mask
	return computeCentroid(mask);
}

bool shouldSave(const cv::Point &c) { 
	// Is empty?
	if ( g_track.empty() )
       		return true;
	// Is the same?
	if ( c.x == g_track.back().x and c.y == g_track.back().y ) 
		return false;
	return true;
}

void processImage(cv::Mat &image) {
    // Fit to screen
    image = resize(image, g_zoom);
	// Trace the ball
	cv::Point c =  traceBall(image);
	// Clear track or remember point?
	if ( c.x < 0 or c.y < 0 ) // Outside?
		g_track.clear();
	else if ( shouldSave(c) ) // Remember?
		g_track.push_back(c);
	// Draw track over image
	cv::polylines(image, g_track, false, cv::Scalar(0,0,255), 3);
	// Display image
	cv::imshow(window, image);
}

int main(int argc, char **argv) {
	info();
	// Create a capture object
	cv::VideoCapture cap;
    // Check arguments
    if (argc > 1)
        cap.open( argv[1] ); // Open filename
    else
        cap.open( DEFAULT_CAMERA ); // Open camera
	// Check if any error happened
	if( !cap.isOpened() ) {
		std::cout << "Ops, capture cannot be created!" << std::endl;
		return -1;
	}
	std::cout << "Press 'Esc' to quit (or 'c' to clear tracks)..." << std::endl;
	while( true ) {
		// Create a image object 
		cv::Mat image;
		// Capture frame
		bool okay = cap.read( image );
		// Skip if any capture error happened (or video ended)
		if( okay )
            processImage(image); // Implement our image processing
            // Get pressed key
		int key = cv::waitKey(30);
		// Break loop if the key 'ESC' was pressed
		if( key == KEY_ESC )
			break;
		else if( key == 'c' )
			g_track.clear();
	}
	return 0;
}
