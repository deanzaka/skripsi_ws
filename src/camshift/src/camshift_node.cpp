
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

Mat image1, image2;

bool backprojMode1 = false;
bool selectObject1 = false;
int trackObject1 = 0;
bool showHist1 = true;
Point origin1;
Rect selection1;
int vmin1 = 10, vmax1 = 256, smin1 = 30;

bool backprojMode2 = false;
bool selectObject2 = false;
int trackObject2 = 0;
bool showHist2 = true;
Point origin2;
Rect selection2;
int vmin2 = 10, vmax2 = 256, smin2 = 30;

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

static void onMouse1( int event, int x, int y, int, void* )
{
    	if( selectObject1 )
    	{
        	selection1.x = MIN(x, origin1.x);
        	selection1.y = MIN(y, origin1.y);
        	selection1.width = std::abs(x - origin1.x);
        	selection1.height = std::abs(y - origin1.y);

        	selection1 &= Rect(0, 0, image1.cols, image1.rows);
    	}

    	switch( event )
    	{
    	case CV_EVENT_LBUTTONDOWN:
        	origin1 = Point(x,y);
        	selection1 = Rect(x,y,0,0);
        	selectObject1 = true;
        	break;
    	case CV_EVENT_LBUTTONUP:
        	selectObject1 = false;
        	if( selection1.width > 0 && selection1.height > 0 )
            		trackObject1 = -1;
        	break;
    	}
}

static void onMouse2( int event, int x, int y, int, void* )
{
    	if( selectObject2 )
    	{
        	selection2.x = MIN(x, origin2.x);
        	selection2.y = MIN(y, origin2.y);
        	selection2.width = std::abs(x - origin2.x);
        	selection2.height = std::abs(y - origin2.y);

        	selection2 &= Rect(0, 0, image2.cols, image2.rows);
    	}

    	switch( event )
    	{
    	case CV_EVENT_LBUTTONDOWN:
        	origin2 = Point(x,y);
        	selection2 = Rect(x,y,0,0);
        	selectObject2 = true;
        	break;
    	case CV_EVENT_LBUTTONUP:
        	selectObject2 = false;
        	if( selection2.width > 0 && selection2.height > 0 )
            		trackObject2 = -1;
        	break;
    	}
}

int main (int argc, char** argv)
{
    	// Initialize ROS
    	ros::init (argc, argv, "camshift");
    	ros::NodeHandle nh;  

    	//help();

    	int hsize = 16;
    	float hranges[] = {0,180};
    	const float* phranges = hranges;
    	Rect trackWindow;
    
    	VideoCapture cap1(1); //capture the video from webcam
	VideoCapture cap2(2); //capture the video from webcam

	namedWindow( "Histogram 1", 0 );
    	namedWindow( "CamShift 1", 0 );
    	setMouseCallback( "CamShift 1", onMouse1, 0 );

    	namedWindow("CamShift 1", 0);
    	createTrackbar( "Vmin", "CamShift 1", &vmin1, 256, 0 );
    	createTrackbar( "Vmax", "CamShift 1", &vmax1, 256, 0 );
    	createTrackbar( "Smin", "CamShift 1", &smin1, 256, 0 );
    	
    	namedWindow( "Histogram 2", 0 );
    	namedWindow( "CamShift 2", 0 );
    	setMouseCallback( "CamShift 2", onMouse2, 0 );

    	namedWindow("CamShift 2", 0);
    	createTrackbar( "Vmin", "CamShift 2", &vmin2, 256, 0 );
    	createTrackbar( "Vmax", "CamShift 2", &vmax2, 256, 0 );
    	createTrackbar( "Smin", "CamShift 2", &smin2, 256, 0 );

    	Mat frame1, hsv1, hue1, mask1, hist1, histimg1 = Mat::zeros(200, 320, CV_8UC3), backproj1;
    	Mat frame2, hsv2, hue2, mask2, hist2, histimg2 = Mat::zeros(200, 320, CV_8UC3), backproj2;
    	bool paused = false;

    	while(nh.ok()) {

        	if( !paused )
        	{
            		cap1 >> frame1;
            		cap2 >> frame2;
            		if( frame1.empty() || frame2.empty())
                		break;
        	}

        	frame1.copyTo(image1);
        	frame2.copyTo(image2);

        	//========================================= CAMERA 1 SELECTION ===================================//

        	if( !paused )
        	{
            		cvtColor(image1,hsv1, COLOR_BGR2HSV);
            		//cvtColor(image2,hsv2, COLOR_BGR2HSV);

            		if( trackObject1 )
            		{
                		int _vmin1 = vmin1, _vmax1 = vmax1;

                		inRange(hsv1, Scalar(0, smin1, MIN(_vmin1,_vmax1)),
                        	Scalar(180, 256, MAX(_vmin1, _vmax1)), mask1);
                		int ch[] = {0, 0};
                		hue1.create(hsv1.size(), hsv1.depth());
                		mixChannels(&hsv1, 1, &hue1, 1, ch, 1);

                		if( trackObject1 < 0 )
                		{
                    		Mat roi(hue1, selection1), maskroi(mask1, selection1);
		                    calcHist(&roi, 1, 0, maskroi, hist1, 1, &hsize, &phranges);
		                    normalize(hist1, hist1, 0, 255, CV_MINMAX);

		                    trackWindow = selection1;
		                    trackObject1 = 1;

		                    histimg1 = Scalar::all(0);
		                    int binW = histimg1.cols / hsize;
		                    Mat buf(1, hsize, CV_8UC3);
                    		for( int i = 0; i < hsize; i++ )
                        				buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                    		cvtColor(buf, buf, CV_HSV2BGR);

		                    for( int i = 0; i < hsize; i++ )
		                    {
		                        int val = saturate_cast<int>(hist1.at<float>(i)*histimg1.rows/255);
		                        rectangle( histimg1, Point(i*binW,histimg1.rows),
		                                   Point((i+1)*binW,histimg1.rows - val),
		                                   Scalar(buf.at<Vec3b>(i)), -1, 8 );
		                    }
                		}

                		calcBackProject(&hue1, 1, 0, hist1, backproj1, &phranges);
                		backproj1 &= mask1;
                		RotatedRect trackBox = CamShift(backproj1, trackWindow,
                                    	TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                		if( trackWindow.area() <= 1 )
                		{
		                    int cols = backproj1.cols, rows = backproj1.rows, r = (MIN(cols, rows) + 5)/6;
		                    trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                       	trackWindow.x + r, trackWindow.y + r) &
                                 				Rect(0, 0, cols, rows);
                		}

                		if( backprojMode1 )
                    			cvtColor( backproj1, image1, COLOR_GRAY2BGR );
                		ellipse( image1, trackBox, Scalar(0,0,255), 3, CV_AA );
            		}
        	}
        	else if( trackObject1 < 0 )
            			paused = false;

        	if( selectObject1 && selection1.width > 0 && selection1.height > 0 )
        	{
            		Mat roi(image1, selection1);
            		bitwise_not(roi, roi);
        	}

            	//========================================= camera 1 selection ===================================//

        	imshow( "CamShift 1", image1 );
        	imshow( "Histogram 1", histimg1 );

        	imshow( "CamShift 2", image2 );
        	imshow( "Histogram 2", histimg2 );

        	char c = (char)waitKey(10);
        	if( c == 27 )
            			break;
        	switch(c)
        	{
        		case 'b':
            				backprojMode1 = !backprojMode1;
            				break;
        		case 'c':
            				trackObject1 = 0;
            				histimg1 = Scalar::all(0);
            				break;
        		case 'h':
            				showHist1 = !showHist1;
            				if( !showHist1 ) destroyWindow( "Histogram 1" );
            				else namedWindow( "Histogram 1", 1 );
            				break;
        		case 'p':
            				paused = !paused;
            				break;
        		default:
            		;
        	}
        	
		ros::spinOnce();
	      if (waitKey(1) == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
	      {
		       cout << "\nESC: Closing Application\n";
		       break;
	      }
  	}
}
