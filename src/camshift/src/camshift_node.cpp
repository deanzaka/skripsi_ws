
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <ctype.h>

#define _USE_MATH_DEFINES

using namespace cv;
using namespace std;

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;

static void help()
{
    	cout << "\nUsage: \n"
            "   ./camshiftdemo [camera number]\n";

    	cout << "\n\nHot keys: \n"
            "\tESC - quit the program\n"
            "\tc - stop the tracking\n"
            "\tb - switch to/from backprojection view\n"
            "\th - show/hide object histogram\n"
            "\tp - pause video\n"
            "To initialize tracking, select the object with mouse\n";
}

static void onMouse( int event, int x, int y, int, void* )
{
    	if( selectObject )
    	{
        	selection.x = MIN(x, origin.x);
        	selection.y = MIN(y, origin.y);
        	selection.width = std::abs(x - origin.x);
        	selection.height = std::abs(y - origin.y);

        	selection &= Rect(0, 0, image.cols, image.rows);
    	}

    	switch( event )
    	{
    	case CV_EVENT_LBUTTONDOWN:
        	origin = Point(x,y);
        	selection = Rect(x,y,0,0);
        	selectObject = true;
        	break;
    	case CV_EVENT_LBUTTONUP:
        	selectObject = false;
        	if( selection.width > 0 && selection.height > 0 )
            		trackObject = -1;
        	break;
    	}
}

int main (int argc, char** argv)
{
    	// Initialize ROS
    	ros::init (argc, argv, "camshift");
    	ros::NodeHandle nh;  

    	//help();

    	VideoCapture cap1(1); //capture the video from webcam
	VideoCapture cap2(2); //capture the video from webcam

    	while(nh.ok()) {

      	    	Mat imgOriginal1;
	       bool bSuccess1 = cap1.read(imgOriginal1); // read a new frame from video
	       Mat imgOriginalCopy1; // make copy of imgOriginal
	       cap1.read(imgOriginalCopy1);
	       
	       Mat imgOriginal2;
	       bool bSuccess2 = cap2.read(imgOriginal2); // read a new frame from video
	       Mat imgOriginalCopy2; // make copy of imgOriginal
	       cap2.read(imgOriginalCopy2);

    		cap1.read(imgOriginal1);
    		cap2.read(imgOriginal2);

    		if (!bSuccess1) //if not success, break loop
      		{
      			cout << "\nCannot read a frame from video stream 1\n";
      			break;
      		}

        	if (!bSuccess2) //if not success, break loop
        	{
            		cout << "\nCannot read a frame from video stream 1\n";
            		break;
        	}

      	    	imshow("Camera 1", imgOriginal1);
		imshow("Camera 2", imgOriginal2);

		ros::spinOnce();
	      if (waitKey(1) == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
	      {
		       cout << "\nESC: Closing Application\n";
		       break;
	      }
  	}
}
