#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <dji_sdk/dji_drone.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <guidance/OffsetData.h>
#include <guidance/yaw_direction.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <std_msgs/Int8.h>


using namespace cv;
using namespace std;


void imageReceived(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_im;

	try
	{
		cv_im = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat img = cv_im->image;

	Mat detected_edges;
	Canny(img, detected_edges, 80, 100, 3, true); //find edges in the image
	imshow( "video", img);  
    imshow( "Edges", detected_edges);                  // Show our image inside it.
	
	

    // std::map<int, int>count;
    // for (int i = 0; i<)
	cv::waitKey(1);
	

	// src.convertTo(src, CV_32F); 
	// src=src/256.0;
 }


int main(int argc, char **argv)
{
  
  ros::init(argc,argv,"canal_detection_image");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/stereo/camera/left/image_raw",1,&imageReceived);

  ros::spin();
  
  return 0;
}
