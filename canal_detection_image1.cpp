#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <time.h>
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
#include "opencv2/calib3d/calib3d.hpp"

//using namespace cv;
//using namespace std;

#define PI 3.14159265

using namespace cv;
using namespace std;

struct RansacOuput{
	double slope;
	double intercept;
	std::vector<cv::Point2f> best_outliers;
};

RansacOuput  Ransac_func(std::vector<cv::Point2f> edgePoints, int iter, double threshDist)
{
	if (edgePoints.size() == 0)
	{
		RansacOuput Param;
		Param.slope = 0;
		Param.intercept = 0;
		
		return Param;
	}
	int number = edgePoints.size();
	int bestInNum = 0;
	double bestParam1 = 0;
	double bestParam2 = 0;
	int inlierNum = 0;
	srand(time(NULL));
	std::vector<cv::Point2f> best_outliers;
	RansacOuput Param;
	
	
	cout<<"total valid edge points: "<< number << endl;

	for (int i = 0; i<iter; i++)
	{
		std::vector<cv::Point2f> outliers;
		int idx1 = rand()%number;
		int idx2 = rand()%number;
		//cout<<	idx1 << " " <<idx2 <<endl;
		cv::Point2f p1 = edgePoints[idx1];
		cv::Point2f p2 = edgePoints[idx2];
		
		//compute distances
		double kLineX = (double)(p2.x - p1.x);
		double kLineY = (double)(p2.y - p1.y);

		double distance = 0;
	
		inlierNum = 0;
		for (int z = 0 ; z<number;z++)
		{
			
			double dist = abs((kLineY*edgePoints[z].x) - (kLineX*edgePoints[z].y) 
								+ (p2.x*p1.y) - (p1.x*p2.y));
			dist = (dist) / sqrt(pow(kLineX,2.0) + pow(kLineY,2.0));
			//cout<<"distance "<<dist<<endl;
			if (dist <= threshDist)
			{
				inlierNum++;
				//inlierIdx.push_back(dist);
			}
			else
			{
				outliers.push_back(edgePoints[z]);
			}
			
		}
		
		if (inlierNum > bestInNum)
		{
			double param1 = kLineY/kLineX;
			double param2 = p1.y - (param1*p1.x);
			 double angle = abs(atan(param1)*(180/PI));
			 if (angle>10 &&  angle<80)
			 {
				bestInNum = inlierNum;
				bestParam1 = param1;
				bestParam2 = param2;
				best_outliers = outliers;
			}
		}
	}

	Param.slope = bestParam1;
	Param.intercept = bestParam2;
	Param.best_outliers = best_outliers;
	cout<<"best Inleris Number " <<bestInNum<<endl;
	cout<<"Outliers Number " <<best_outliers.size()<<endl;

	return Param;	
}


void imageReceived(const sensor_msgs::ImageConstPtr& msg)
{
	clock_t t1, t2;
	t1 = clock();

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

	cv::Mat img, img_gray;
	cv::Mat detected_edges, dst;
	//int lowThreshold;
	int const lowThreshold = 30;
	int ratio = 3;
	int kernel_size = 3;
	//const char* window_name = "Edge_Map";

	img = cv_im->image;
	int rows = img.rows;
	int cols = img.cols;
	cout<<"Image rows: "<<rows<<endl;
	cout<<"Image cols: "<<cols<<endl;
	//img=img/256.0;

	cout <<endl<< "Size : " << img.size()<<endl;
	cout<<"img channels "<<img.channels()<<endl;
	
	//create empty dst image
	//cout<<"create image size "<<endl;
	dst.create(img.size(), img.type());
	
	//change image to grayscale
	//cout<<"img to grayscale "<<endl;
	if (img.channels()!= 1)
	{
		cvtColor(img, img_gray, COLOR_BGR2GRAY);
	}
	else
	{
		img_gray = img;
	}

	//window size
	//cout<<"name window "<<endl;
	//namedWindow(window_name, WINDOW_AUTOSIZE);
	
	//blur
	//cout<<"blur grayscale image "<<endl;
	blur(img_gray, detected_edges, Size(3,3));

	//canny
	//cout<<"canny edge detector"<<endl;
	Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size, true);
	
	// set dst
	//cout<<"setting dst"<<endl;
	dst = Scalar::all(0);
	img.copyTo(dst, detected_edges);
	
	// convert the points to 0-255 range
	dst.convertTo(dst, CV_64F); 
	// convert it 0-1 range
	dst = dst/256.0;
	//cout<<img<<endl;
	


////////////////////////Something new/////////////////
	


	imshow( "Original Video", img);

    imshow( "Canny Edge Detection", detected_edges);                  // Show our image inside it.
	
	imshow("Processed Edges", dst);

	// Get valid edge points
	std::vector<cv::Point2f> edgePoints;
	cv::Point2f ePoint;
    for (int i=0; i < rows; i++)
    {
    	for (int j = 0; j < cols; j++)
    	{
    		if (dst.at<double>(i, j) > 0.00)
    		{
    			ePoint.x = j;
    			ePoint.y = i;

    			edgePoints.push_back(ePoint);
    			
    		}	
    	}
    }
    
    ///////////////////////////////

   	int iterations1 = 1500;
   	int threshDist1 = 5;

   	int iterations2 = 1000;
   	int threshDist2 = 10;

    RansacOuput Param1 = Ransac_func(edgePoints, iterations1, threshDist1);
    cout<<"First Slope "<<Param1.slope<< ", First intercept "<<Param1.intercept<<endl;
    RansacOuput Param2 = Ransac_func(Param1.best_outliers, iterations2, threshDist2);
    cout<<"Second Slope "<<Param2.slope<< ", Second intercept "<<Param2.intercept<<endl;
    //check second tme points
    /*cout<<"size edgePoints "<<Param1.best_outliers.size()<<endl;
    for (int k = 0; k<Param1.best_outliers.size(); k++)
    {
    	cout <<setprecision(3)<<dst.at<double>(Param1.best_outliers[k])<<", ";
    }*/
    //cout<<endl;

    cv::Point2f p1,q1, p2, q2;
    p1.x = 0.0;
    p1.y = 0.0;
    q1.x = 0.0;
    q1.y = 0.0;

    p2.x = 0.0;
    p2.y = 0.0;
    q2.x = 0.0;
    q2.y = 0.0;

    if (Param1.slope !=0 && Param1.intercept != 0)
	{

		int y1[2] = {100, rows};
		double x1[2];
		for (int i = 0; i < 2; i++) {
			x1[i] = (y1[i] - Param1.intercept)/Param1.slope;
		}
		p1.x = x1[0];
		p1.y = y1[0];
		q1.x = x1[1];
		q1.y = y1[1];

		cout<<"p1 ("<< p1.x<< " "<< p1.y<<")"<<endl;
		cout<<"q1 ("<< q1.x<< " "<< q1.y<<")"<<endl;
	}

    

    if (Param2.slope !=0 && Param2.intercept != 0)
	{

		int y2[2] = {100, rows};
		double x2[2];
		for (int i = 0; i < 2; i++) {
			x2[i] = (y2[i] - Param2.intercept)/Param2.slope;
		}
		p2.x = x2[0];
		p2.y = y2[0];
		q2.x = x2[1];
		q2.y = y2[1];

		cout<<"p2 ("<< p2.x<< " "<< p2.y<<")"<<endl;
		cout<<"q2 ("<< q2.x<< " "<< q2.y<<")"<<endl;
	   
	}

	//////Homography Function//////////

	cv::Point2f intersect, p3;
	vector<Point2f> pts_src, pts_dst;

	intersect.x = (Param2.intercept - Param1.intercept)/(Param1.slope - Param2.slope);
	intersect.y = (Param1.slope*intersect.x) + Param1.intercept;
	if (intersect.x >= 0 && intersect.y >= 0) {
		cout<<"points from if part"<<endl;

		p3.x = intersect.x;
		p3.y = intersect.y;
		cout<<"p3" << p3.x << ","<<p3.y<<endl;
		cout<<"intersect" << intersect.x << ","<<intersect.y<<endl;
	    pts_src.push_back(Point2f(p3.x-10, p3.y));
	    pts_src.push_back(Point2f(q1.x, q1.y));
	    pts_src.push_back(Point2f(p3.x+10, p3.y));
	    pts_src.push_back(Point2f(q2.x, q2.y));
	    
	    pts_dst.push_back(Point2f(q1.x, p3.y));
	    pts_dst.push_back(Point2f(q1.x, q1.y));
	    pts_dst.push_back(Point2f(q2.x, p3.y));
	    pts_dst.push_back(Point2f(q2.x, q2.y));
	}
   	
	else {
		cout<<"points from else part"<<endl;
		pts_src.push_back(Point2f(p2.x, p1.y));
	    pts_src.push_back(Point2f(q1.x, q1.y));
	    pts_src.push_back(Point2f(p1.x, p2.y));
	    pts_src.push_back(Point2f(q2.x, q2.y));
	    
	    pts_dst.push_back(Point2f(q1.x, p1.y));
	    pts_dst.push_back(Point2f(q1.x, q1.y));
	    pts_dst.push_back(Point2f(q2.x, p2.y));
	    pts_dst.push_back(Point2f(q2.x, q2.y));
	}
   	
    

    // Calculate Homography
   	Mat h = cv::findHomography(pts_src, pts_dst, CV_RANSAC, 5);
 	
 	cout<<"Homography" <<endl;
   	for (int i = 0; i<3; i ++) {
   		for (int j = 0; j<3; j ++){
   			cout<<h.at<double>(i,j)<<", ";
   		}
   		cout<<endl;
   	}
   		

    // Output image
   	Mat img_out;

   	// Warp source image to destination based on homography
   	//perspectiveTransform( img, img_out, h);
   	warpPerspective(img, img_out, h, img.size());
   	cv::Point2f p3_left, p3_right;
   	std::vector<Point2f> obj_corners(4);
   	p3_left.x = p3.x - 10;
   	p3_left.y = p3.y;
   	p3_right.x = p3.x + 10;
   	p3_right.y = p3.y;
   	obj_corners[0] = p3_left;
   	obj_corners[1] = q1;
   	obj_corners[2] = p3_right;
   	obj_corners[3] = q2;
   	std::vector<Point2f> scene_corners(4);
   	perspectiveTransform( obj_corners, scene_corners, h);
   	cout<<endl;
   	cout<<"obj_corners 0: " <<obj_corners[0].x <<","<<obj_corners[0].y<<endl;
    cout<<"obj_corners 1: " <<obj_corners[1].x <<","<<obj_corners[1].y<<endl;
    cout<<"obj_corners 2: " <<obj_corners[2].x <<","<<obj_corners[2].y<<endl;
    cout<<"obj_corners 3: " <<obj_corners[3].x <<","<<obj_corners[3].y<<endl<<endl;;
   	
    cout<<"scene_corners 0: " <<scene_corners[0].x <<","<<scene_corners[0].y<<endl;
    cout<<"scene_corners 1: " <<scene_corners[1].x <<","<<scene_corners[1].y<<endl;
    cout<<"scene_corners 2: " <<scene_corners[2].x <<","<<scene_corners[2].y<<endl;
    cout<<"scene_corners 3: " <<scene_corners[3].x <<","<<scene_corners[3].y<<endl;

    cv::line(img_out, scene_corners[0], scene_corners[1],(1,255,255), 2);
    cv::line(img_out, scene_corners[2], scene_corners[3],(1,255,255), 2);
   	// Display images
   	imshow("Warped Image", img_out);

   	//Plotting lines on output images
   	/*if (p1.x != 0 && q1.x != 0 && q1.y != 0) {
		cv::line(img, p1, q1,(1,255,255), 2);
	}

	if (p2.x != 0 && q2.x != 0 && q2.y != 0) {	
	    cv::line(img, p2, q2,(0,1,1), 2);
	}*/
   	imshow("Output Results", img);

	cv::waitKey(1);
	
	t2 = clock();
	float diff = (float)t2 - (float)t1;
	float seconds = diff/CLOCKS_PER_SEC;
	cout<<"time for 1 frame "<<seconds<<endl;
	cout<<endl;
 }


int main(int argc, char **argv)
{
  
  ros::init(argc,argv,"canal_detection_image");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/stereo/camera/left/image_raw",1,&imageReceived);
 
  //ros::Subscriber sub = nh.subscribe("/videofile/image_raw",1,&imageReceived);

  ros::spin();
  
  return 0;
}