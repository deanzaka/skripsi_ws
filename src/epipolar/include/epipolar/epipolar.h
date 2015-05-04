#ifndef EPIPOLAR_H_
#define EPIPOLAR_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>

#include <opencv2/video/background_segm.hpp>
#include <opencv2/legacy/blobtrack.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>

#include "std_msgs/String.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <math.h>

#define PI 3.14159265

namespace epipolar {

	typedef void (*voidFuncPtr)(void);

	class Epipolar{
	  public:
	  	Epipolar(ros::NodeHandle nh);
    		~Epipolar();

    		void run(ros::Rate loop_rate);

    		void epipolar_init();

	  private:	
	  	ros::NodeHandle nh_;
	  	cv::Mat imgOriginal1, imgOriginal2;
	};
}

#endif