#ifndef EPIPOLAR_H_
#define EPIPOLAR_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <math.h>

namespace epipolar {

	typedef void (*voidFuncPtr)(void);

	class epipolar{
	 public:
	 private:	
	};
}

#endif