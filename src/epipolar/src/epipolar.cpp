#include <epipolar/epipolar.h>

namespace epipolar {
	Epipolar::Epipolar(ros::NodeHandle nh): nh_(nh) {

  	}
  	Epipolar::~Epipolar() {

  	}

  	void Epipolar::run(ros::Rate loop_rate) {
    		using namespace std;
    		using namespace cv;

    		VideoCapture cap1(1); //capture the video from webcam
    		VideoCapture cap2(2); //capture the video from webcam

	    	if ( !cap1.isOpened() )  // if not success, exit program
	    	{
	        	ROS_INFO("Cannot open web cam 1");
	    	}

	    	if ( !cap2.isOpened() )  // if not success, exit program
	    	{
	        	ROS_INFO("Cannot open web cam 2");
	    	}

	    	while (ros::ok()) {
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
            			ROS_INFO("Cannot read a frame from video stream");
        		}

	        	if (!bSuccess2) //if not success, break loop
	        	{
	            		ROS_INFO("Cannot read a frame from video stream");
	        	}

      			imshow("Camera 1", imgOriginal1);
			imshow("Camera 2", imgOriginal2);

			ros::spinOnce();
		      loop_rate.sleep();
		      if (waitKey(1) == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
		      {
			       ROS_INFO_STREAM("ESC: Closing Application");
			       break;
		      }	    		
	    	}
    		
    	}

}