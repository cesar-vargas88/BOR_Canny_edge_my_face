#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::Mat gray;
		cv::Mat canny_edges;
		cv::cvtColor(cv_bridge::toCvShare(msg, "bgr8")->image, gray, cv::COLOR_BGR2GRAY);
		cv::blur(gray, canny_edges, cv::Size(3,3));
		cv::Canny(canny_edges, canny_edges, 100, 200, 3);
		cv::imshow("image_source", cv_bridge::toCvShare(msg, "bgr8")->image);
		cv::imshow("canny_edge_my_face", canny_edges);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	cv::namedWindow("image_source");
	cv::namedWindow("canny_edge_my_face");
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("cv_camera/image_raw", 1, imageCallback);
	ros::spin();
	cv::destroyWindow("image_source");
	cv::destroyWindow("canny_edge_my_face");
}
