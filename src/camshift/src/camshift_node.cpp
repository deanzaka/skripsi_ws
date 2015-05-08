
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

Mat image1;

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

        	selection &= Rect(0, 0, image1.cols, image1.rows);
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

    	int hsize = 16;
    	float hranges[] = {0,180};
    	const float* phranges = hranges;
    	Rect trackWindow;
    
    	VideoCapture cap1(1); //capture the video from webcam
	VideoCapture cap2(2); //capture the video from webcam

	namedWindow( "Histogram", 0 );
    	namedWindow( "CamShift 1", 0 );
    	setMouseCallback( "CamShift 1", onMouse, 0 );
    	namedWindow("CamShift Panel", 0);
    	createTrackbar( "Vmin", "CamShift Panel", &vmin, 256, 0 );
    	createTrackbar( "Vmax", "CamShift Panel", &vmax, 256, 0 );
    	createTrackbar( "Smin", "CamShift Panel", &smin, 256, 0 );

    	Mat frame1, hsv1, hue1, mask1, hist1, histimg1 = Mat::zeros(200, 320, CV_8UC3), backproj1;
    	bool paused = false;

    	while(nh.ok()) {

        	if( !paused )
        	{
            		cap1 >> frame1;
            		if( frame1.empty() )
                		break;
        	}

        	frame1.copyTo(image1);

        	if( !paused )
        	{
            		cvtColor(image1,hsv1, COLOR_BGR2HSV);

            		if( trackObject )
            		{
                		int _vmin = vmin, _vmax = vmax;

                		inRange(hsv1, Scalar(0, smin, MIN(_vmin,_vmax)),
                        	Scalar(180, 256, MAX(_vmin, _vmax)), mask1);
                		int ch[] = {0, 0};
                		hue1.create(hsv1.size(), hsv1.depth());
                		mixChannels(&hsv1, 1, &hue1, 1, ch, 1);

                		if( trackObject < 0 )
                		{
                    		Mat roi(hue1, selection), maskroi(mask1, selection);
		                    calcHist(&roi, 1, 0, maskroi, hist1, 1, &hsize, &phranges);
		                    normalize(hist1, hist1, 0, 255, CV_MINMAX);

		                    trackWindow = selection;
		                    trackObject = 1;

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

                		if( backprojMode )
                    			cvtColor( backproj1, image1, COLOR_GRAY2BGR );
                		ellipse( image1, trackBox, Scalar(0,0,255), 3, CV_AA );
            		}
        	}
        	else if( trackObject < 0 )
            			paused = false;

        	if( selectObject && selection.width > 0 && selection.height > 0 )
        	{
            		Mat roi(image1, selection);
            		bitwise_not(roi, roi);
        	}

        	imshow( "CamShift 1", image1 );
        	imshow( "Histogram", histimg1 );

        	char c = (char)waitKey(10);
        	if( c == 27 )
            			break;
        	switch(c)
        	{
        		case 'b':
            				backprojMode = !backprojMode;
            				break;
        		case 'c':
            				trackObject = 0;
            				histimg1 = Scalar::all(0);
            				break;
        		case 'h':
            				showHist = !showHist;
            				if( !showHist ) destroyWindow( "Histogram" );
            				else namedWindow( "Histogram", 1 );
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
