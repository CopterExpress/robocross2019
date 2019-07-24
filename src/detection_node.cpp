#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

image_transport::Publisher debug_image;

void image_callback(const sensor_msgs::ImageConstPtr& image)
{

	ROS_INFO_THROTTLE(1, "red_dead_detection: callback called");
	cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(image, "mono8");

	cv::Mat outMat = img->image;
	//cv::adaptiveThreshold(img->image, outMat, 1.0, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 7, 0.0);

	cv_bridge::CvImage output(image->header, "mono8", outMat);
	debug_image.publish(output.toImageMsg());
	//debug_image.publish(output.toImageMsg());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "red_dead_detection_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	ROS_INFO("red_dead_detection: Subscribing to input_image");
	image_transport::Subscriber in_image = it.subscribe("input_image", 1, image_callback);
	ROS_INFO("red_dead_detection: Advertising debug topic");
	debug_image = it.advertise("debug_image", 1);
	ROS_INFO("red_dead_detection: Starting spinning");
	ros::spin();

	return 0;
}