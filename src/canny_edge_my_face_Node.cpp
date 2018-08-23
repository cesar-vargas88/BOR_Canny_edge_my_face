#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::Mat canny_edges;
		cv::cvtColor(cv_bridge::toCvShare(msg, "bgr8")->image, canny_edges, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(canny_edges, canny_edges, cv::Size(3,3),0,0);
		cv::Canny(canny_edges, canny_edges, 100, 200, 3);
		cv::cvtColor(canny_edges, canny_edges, cv::COLOR_GRAY2BGR);

		cv_bridge::CvImage imgBridge = cv_bridge::CvImage(std_msgs::Header(), "bgr8", canny_edges);
		sensor_msgs::Image img_msg;		
		imgBridge.toImageMsg(img_msg);
		pub.publish(img_msg);

		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'. %s", msg->encoding.c_str(), e.what());
	}
}
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "canny_edge_my_face_Node");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/cv_camera/image_raw", 1, imageCallback);
	pub = it.advertise("/camera/canny_edge_my_face", 1);

	ros::spin();
}
