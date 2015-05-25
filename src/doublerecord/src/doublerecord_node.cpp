#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "camshift");
	ros::NodeHandle nh;  

	VideoCapture cap1(1); 
	if(!cap1.isOpened()){
		cout << "Error opening video stream 1" << endl;
	}

	VideoCapture cap2(2); 
	if(!cap2.isOpened()){
		cout << "Error opening video stream 2" << endl;
	}

	int frame_width_1=   cap1.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height_1=   cap1.get(CV_CAP_PROP_FRAME_HEIGHT);
	VideoWriter video1("/home/deanzaka/datatemp/video1.avi",CV_FOURCC('M','J','P','G'),25, Size(frame_width_1,frame_height_1),true);

	int frame_width_2=   cap2.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height_2=   cap2.get(CV_CAP_PROP_FRAME_HEIGHT);
	VideoWriter video2("/home/deanzaka/datatemp/video2.avi",CV_FOURCC('M','J','P','G'),25, Size(frame_width_2,frame_height_2),true);

	while(nh.ok()){
		ros::Time start = ros::Time::now();

		Mat frame1, frame2;
		cap1 >> frame1;
		cap2 >> frame2;
		video1.write(frame1);
		video2.write(frame2);
		imshow( "Frame 1", frame1 );
		imshow( "Frame 2", frame2 );
		char c = (char)waitKey(1);
		if( c == 27 ) break;

		float delay = ros::Time::now().toSec() - start.toSec();
		while(delay < 0.040) {
			delay = ros::Time::now().toSec() - start.toSec();
		}

		cout << "\n\n Delay = \t" << delay << "\n\n";
        	
        	ros::spinOnce();
	}
}