#include <epipolar/epipolar.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "epipolar");
  ros::NodeHandle nh;
  
  epipolar::Epipolar epipolar(nh);

  epipolar.run(ros::Rate(10));

  return(0);
}