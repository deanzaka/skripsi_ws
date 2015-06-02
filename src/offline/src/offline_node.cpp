
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

#include <iostream>
#include <ctype.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <fstream>

#include "std_msgs/String.h"

#define _USE_MATH_DEFINES
#define PI 3.14159265

using namespace cv;
using namespace std;

Mat image1, image2, imageOriginal1, imageOriginal2;
float xCam1, yCam1, xCam2, yCam2;

visualization_msgs::Marker marker, lines, points;
geometry_msgs::Pose pose, trans_pose;
geometry_msgs::Point last, p;
ros::Publisher marker_pub;

bool step = true;
bool showHist = true;
bool backprojMode = false;

bool selectObject1 = false;
int trackObject1 = 0;
Point origin1;
Rect selection1;
int vmin1 = 10, vmax1 = 256, smin1 = 30;

bool selectObject2 = false;
int trackObject2 = 0;
Point origin2;
Rect selection2;
int vmin2 = 10, vmax2 = 256, smin2 = 30;

Mat fgMaskMOG1; //fg mask generated by MOG method
Mat merged1;
Ptr<BackgroundSubtractor> pMOG1; //MOG Background subtractor
Mat channel1[3];

Mat fgMaskMOG2; //fg mask generated by MOG method
Mat merged2;
Ptr<BackgroundSubtractor> pMOG2; //MOG Background subtractor
Mat channel2[3];

static void help()
{
    	
            "\tESC - quit the program\n"
            "\tc - clear tracking\n"
            "\ts - switch to/from step mode\n"
            "\tb - switch to/from backprojection view\n"
            "\th - show/hide object histogram\n"
            "\tp - pause video\n"
            "To initialize tracking, select the object with mouse\n";
}

void csv_init(const std::string& filepath) {
    using namespace std;
    using namespace boost;

    ofstream csv;
    csv.open(filepath.c_str(),ios::app);
    if ( csv.is_open() ) csv << ","; 
    else {
        assert(false && "csv.open(filepath.c_str()): FALSE");
    }
    csv.close();
}

// void csv_write(const geometry_msgs::PoseStamped& pose,
//         const std::string& filepath) {
//     using namespace std;
//     using namespace boost;

//     ofstream csv;
//     csv.open(filepath.c_str(),ios::app);
//     if ( csv.is_open() ) {
//         // csv << lexical_cast<string>(pose.header.stamp.toSec()); csv << ",";
//         // csv << lexical_cast<string>(pose.pose.position.x); csv << ",";
//         // csv << lexical_cast<string>(pose.pose.position.y); csv << ",";
//         // csv << lexical_cast<string>(pose.pose.position.z); csv << ",";
//     }
//     else {
//         assert(false && "csv.open(filepath.c_str()): FALSE");
//     }

//     csv.close();
// }

void marker_init() {
 // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
    uint32_t line_strip = visualization_msgs::Marker::LINE_STRIP;
    uint32_t points_mark =  visualization_msgs::Marker::POINTS;

    points.header.frame_id = lines.header.frame_id = marker.header.frame_id = "/map";
    points.header.stamp = lines.header.stamp = marker.header.stamp = ros::Time::now();
    marker.type = shape;
    lines.type = line_strip;
    points.type = points_mark;
    points.action = lines.action = marker.action = visualization_msgs::Marker::ADD;

    lines.id = 0;
    points.id = 1;
    marker.id = 2;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    points.scale.x = 0.1;
    points.scale.y = 0.1;

    points.color.r = 0.0f;
    points.color.g = 0.0f;
    points.color.b = 1.0f;
    points.color.a = 1.0;

    lines.scale.x = 0.05;  
    lines.color.r = 1.0f;
    lines.color.g = 0.0f;
    lines.color.b = 0.0f;
    lines.color.a = 1.0;

    marker.lifetime = ros::Duration();
}

void transformer (const geometry_msgs::Pose& sPose)
{

    float r, th;
    float x_temp = (float) sPose.position.x / 100;
    float y_temp = (float) sPose.position.y / 100;
    float z_temp = (float) sPose.position.z / 100;

    trans_pose.position.x = y_temp;
    trans_pose.position.y = -(x_temp);
    trans_pose.position.z = z_temp;

    marker.pose = trans_pose;

    p.x = trans_pose.position.x; // backward - forward
    p.y = trans_pose.position.y; // right - left
    p.z = trans_pose.position.z; // down - up

    // debug
    cout << "\n\n" << p.x << "\t";
    cout << p.y << "\t";
    cout << p.z << "\n\n";

    // lines.lifetime = ros::Duration(5);
    // points.lifetime = ros::Duration(5);

    lines.pose.orientation.w = 1.0;
    points.pose.orientation.w = 1.0;
    lines.points.push_back(p);
    points.points.push_back(p);

    marker_pub.publish(marker);
    marker_pub.publish(lines);
    marker_pub.publish(points);
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
    ros::init (argc, argv, "camshift_offline");
    ros::NodeHandle nh;
    marker_init();

    string csv_filepath = "/home/deanzaka/datatemp/red_net.csv";  
    float timestamp = 0;
    help();

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    KalmanFilter KF1(4, 2, 0);

    // intialization of KF
    KF1.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
    Mat_<float> measurement1(2,1); measurement1.setTo(Scalar(0));
     
    KF1.statePre.at<float>(0) = 0; // initial value
    KF1.statePre.at<float>(1) = 0; // initial value
    KF1.statePre.at<float>(2) = 0;
    KF1.statePre.at<float>(3) = 0;
    setIdentity(KF1.measurementMatrix);
    setIdentity(KF1.processNoiseCov, Scalar::all(.005));
    setIdentity(KF1.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF1.errorCovPost, Scalar::all(.1));

    KalmanFilter KF2(4, 2, 0);

    // intialization of KF
    KF2.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
    Mat_<float> measurement2(2,1); measurement2.setTo(Scalar(0));
     
    KF2.statePre.at<float>(0) = 0; // initial value
    KF2.statePre.at<float>(1) = 0; // initial value
    KF2.statePre.at<float>(2) = 0;
    KF2.statePre.at<float>(3) = 0;
    setIdentity(KF2.measurementMatrix);
    setIdentity(KF2.processNoiseCov, Scalar::all(.005));
    setIdentity(KF2.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF2.errorCovPost, Scalar::all(.1));

    VideoCapture inputVideo1("/home/deanzaka/datatemp/red_net_1/red_net_1-cut-02.avi");              // Open input
    if (!inputVideo1.isOpened())
    {
        cout  << "Could not open the input video 1" << endl;
        return -1;
    }

    VideoCapture inputVideo2("/home/deanzaka/datatemp/red_net_2/red_net_2-cut-02.avi");              // Open input
    if (!inputVideo2.isOpened())
    {
        cout  << "Could not open the input video 2" << endl;
        return -1;
    }

    int ex1 = static_cast<int>(inputVideo1.get(CV_CAP_PROP_FOURCC));     // Get Codec Type- Int form
    int ex2 = static_cast<int>(inputVideo2.get(CV_CAP_PROP_FOURCC));     // Get Codec Type- Int form

    // Transform from int to char via Bitwise operators
    char EXT1[] = {(char)(ex1 & 0XFF) , (char)((ex1 & 0XFF00) >> 8),(char)((ex1 & 0XFF0000) >> 16),(char)((ex1 & 0XFF000000) >> 24), 0};
    char EXT2[] = {(char)(ex2 & 0XFF) , (char)((ex2 & 0XFF00) >> 8),(char)((ex2 & 0XFF0000) >> 16),(char)((ex2 & 0XFF000000) >> 24), 0};

    Size S1 = Size((int) inputVideo1.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                  (int) inputVideo1.get(CV_CAP_PROP_FRAME_HEIGHT));
    Size S2 = Size((int) inputVideo2.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                  (int) inputVideo2.get(CV_CAP_PROP_FRAME_HEIGHT));

    int posZ;
    double posX1, posX2, posY1, posY2;

    Rect trackWindow1;
    Rect trackWindow2;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;

    namedWindow( "Histogram 1", 0 );
    namedWindow( "Histogram 2", 0 );

    namedWindow( "CamShift 1", 0 );
    setMouseCallback( "CamShift 1", onMouse1, 0 );
    // createTrackbar( "Vmin", "CamShift 1", &vmin1, 256, 0 );
    // createTrackbar( "Vmax", "CamShift 1", &vmax1, 256, 0 );
    // createTrackbar( "Smin", "CamShift 1", &smin1, 256, 0 );


    namedWindow( "CamShift 2", 0 );
    setMouseCallback( "CamShift 2", onMouse2, 0 );
    // createTrackbar( "Vmin", "CamShift 2", &vmin2, 256, 0 );
    // createTrackbar( "Vmax", "CamShift 2", &vmax2, 256, 0 );
    // createTrackbar( "Smin", "CamShift 2", &smin2, 256, 0 );

    namedWindow("Input 1", 0);
    namedWindow("Input 2", 0);

    Mat frame1, hsv1, hue1, mask1, hist1, histimg1 = Mat::zeros(200, 320, CV_8UC3), backproj1;
    Mat frame2, hsv2, hue2, mask2, hist2, histimg2 = Mat::zeros(200, 320, CV_8UC3), backproj2;
    bool paused = false;

    //create Background Subtractor objects
    pMOG1 = new BackgroundSubtractorMOG2(); //MOG2 approach
    pMOG2 = new BackgroundSubtractorMOG2(); //MOG2 approach

    while(nh.ok()) 
    {
        ros::Time start = ros::Time::now();

        // First predict, to update the internal statePre variable
        Mat prediction1 = KF1.predict();
        Point predictPt1(prediction1.at<float>(0),prediction1.at<float>(1));
        Mat prediction2 = KF2.predict();
        Point predictPt2(prediction2.at<float>(0),prediction2.at<float>(1));


        if( !paused )
        {
            inputVideo1 >> frame1;
            if( frame1.empty() )
                break;
        }
        if( !paused )
        {
            inputVideo2 >> frame2;
            if( frame2.empty() )
                break;
        }
        if( !paused ) timestamp = timestamp + 0.04; //adding 40 ms for 25 fps

        frame1.copyTo(image1);
        frame2.copyTo(image2);
        frame1.copyTo(imageOriginal1);
        frame2.copyTo(imageOriginal2);

        pMOG1->operator()(image1, fgMaskMOG1);
        fgMaskMOG1.copyTo(channel1[0]);
        channel1[1] = Mat::zeros(480, 640, CV_8UC1 );
        channel1[2] = Mat::zeros(480, 640, CV_8UC1 );
        merge(channel1, 3, image1);

        //morphological opening (removes small objects from the foreground)
        erode(image1, image1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate(image1, image1, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );

        // //morphological closing (removes small holes from the foreground)
        // dilate(image1, image1, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) );
        // erode(image1, image1, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) );

        pMOG2->operator()(image2, fgMaskMOG2);
        fgMaskMOG2.copyTo(channel2[2]);
        channel2[1] = Mat::zeros(480, 640, CV_8UC1 );
        channel2[0] = Mat::zeros(480, 640, CV_8UC1 );
        merge(channel2, 3, image2);

        //morphological opening (removes small objects from the foreground)
        erode(image2, image2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate(image2, image2, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );

        // //morphological closing (removes small holes from the foreground)
        // dilate(image2, image2, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) );
        // erode(image2, image2, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) );

        if( !paused )
        {
            cvtColor(image1, hsv1, COLOR_BGR2HSV);
            cvtColor(image2, hsv2, COLOR_BGR2HSV);

            if( trackObject1 )
            {
                int _vmin1 = vmin1, _vmax1 = vmax1;

                inRange(hsv1, Scalar(0, smin1, MIN(_vmin1,_vmax1)),
                        Scalar(180, 256, MAX(_vmin1, _vmax1)), mask1);
                int ch1[] = {0, 0};
                hue1.create(hsv1.size(), hsv1.depth());
                mixChannels(&hsv1, 1, &hue1, 1, ch1, 1);

                if( trackObject1 < 0 )
                {
                    Mat roi1(hue1, selection1), maskroi(mask1, selection1);
                    calcHist(&roi1, 1, 0, maskroi, hist1, 1, &hsize, &phranges);
                    normalize(hist1, hist1, 0, 255, CV_MINMAX);

                    trackWindow1 = selection1;
                    trackObject1 = 1;

                    histimg1 = Scalar::all(0);
                    int binW = histimg1.cols / hsize;
                    Mat buf1(1, hsize, CV_8UC3);
                    for( int i = 0; i < hsize; i++ )
                            buf1.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                    cvtColor(buf1, buf1, CV_HSV2BGR);

                    for( int i = 0; i < hsize; i++ )
                    {
                        int val2 = saturate_cast<int>(hist1.at<float>(i)*histimg1.rows/255);
                        rectangle( histimg1, Point(i*binW,histimg1.rows),
                           Point((i+1)*binW,histimg1.rows - val2),
                           Scalar(buf1.at<Vec3b>(i)), -1, 8 );
                    }
                }

                calcBackProject(&hue1, 1, 0, hist1, backproj1, &phranges);
                backproj1 &= mask1;
                int flagShift1 = meanShift(backproj1, trackWindow1,
                        TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                RotatedRect trackBox1;
                if(flagShift1 != 0) {
                    trackBox1 = CamShift(backproj1, trackWindow1,
                        TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                }
                else if(posZ > 0){
                    Rect preWindow1(predictPt1.x, predictPt1.y, 10, 10); 
                    trackBox1 = CamShift(backproj1, preWindow1,
                                        TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                }
                else {
                    Rect preWindow1(xCam1, yCam1, 50, 50); 
                    trackBox1 = CamShift(backproj1, preWindow1,
                                        TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                }
                if( trackWindow1.area() <= 1 )
                {
                    int cols = backproj1.cols, rows = backproj1.rows, r = (MIN(cols, rows) + 5)/6;
                    trackWindow1 = Rect(trackWindow1.x - r, trackWindow1.y - r,
                           trackWindow1.x + r, trackWindow1.y + r) &
                      Rect(0, 0, cols, rows);
                }

                if( backprojMode )
                    cvtColor( backproj1, image1, COLOR_GRAY2BGR );
                if(trackBox1.size.width != 0 && trackBox1.size.height != 0 )
                    ellipse( image1, trackBox1, Scalar(0,255,0), 3, CV_AA );

                cout << "\nCam 1 XY Position: \t\t";
                cout << trackBox1.center.x << "\t" << trackBox1.center.y;
                if(trackBox1.center.x != 0 || trackBox1.center.y != 0) {
                    xCam1 = trackBox1.center.x;
                    yCam1 = trackBox1.center.y;

                    measurement1(0) = xCam1;
                    measurement1(1) = yCam1; 

                    // The update phase 
                    Mat estimated1 = KF1.correct(measurement1);
                    Point statePt1(estimated1.at<float>(0),estimated1.at<float>(1));
                }

                if(trackBox1.center.x != 0 || trackBox1.center.y != 0) {
                    posX1 = trackBox1.center.x;
                    posY1 = trackBox1.center.y;
                }
                else {
                    posX1 = predictPt1.x;
                    posY1 = predictPt1.y;
                }

                prediction1 = KF1.predict();
                Point predictPt1(prediction1.at<float>(0),prediction1.at<float>(1));
                cout << "\nCam 1 Predicted Position: \t";
                cout << predictPt1.x << "\t" << predictPt1.y;
                circle(image1 , Point(  predictPt1.x,predictPt1.y), 16.0, Scalar( 0, 155, 255), 3, 8 );
            }


            if( trackObject2 )
            {
                int _vmin2 = vmin2, _vmax2 = vmax2;

                inRange(hsv2, Scalar(0, smin2, MIN(_vmin2,_vmax2)),
                Scalar(180, 256, MAX(_vmin2, _vmax2)), mask2);
                int ch2[] = {0, 0};
                hue2.create(hsv2.size(), hsv2.depth());
                mixChannels(&hsv2, 1, &hue2, 1, ch2, 1);

                if( trackObject2 < 0 )
                {
                    Mat roi2(hue2, selection2), maskroi(mask2, selection2);
                    calcHist(&roi2, 1, 0, maskroi, hist2, 1, &hsize, &phranges);
                    normalize(hist2, hist2, 0, 255, CV_MINMAX);

                    trackWindow2 = selection2;
                    trackObject2 = 1;

                    histimg2 = Scalar::all(0);
                    int binW = histimg2.cols / hsize;
                    Mat buf2(1, hsize, CV_8UC3);
                    for( int i = 0; i < hsize; i++ )
                    buf2.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                    cvtColor(buf2, buf2, CV_HSV2BGR);

                    for( int i = 0; i < hsize; i++ )
                    {
                        int val2 = saturate_cast<int>(hist2.at<float>(i)*histimg2.rows/255);
                        rectangle( histimg2, Point(i*binW,histimg2.rows),
                           Point((i+1)*binW,histimg2.rows - val2),
                           Scalar(buf2.at<Vec3b>(i)), -1, 8 );
                    }
                }

                calcBackProject(&hue2, 1, 0, hist2, backproj2, &phranges);
                backproj2 &= mask2;
                int flagShift2 = meanShift(backproj2, trackWindow2,
                    TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                RotatedRect trackBox2;
                if(flagShift2 != 0) {
                    trackBox2 = CamShift(backproj2, trackWindow2,
                        TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                }
                else if (posZ > 0) {
                    Rect preWindow2(predictPt2.x, predictPt2.y, 10, 10); 
                    trackBox2 = CamShift(backproj2, preWindow2,
                                        TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                }
                else {
                    Rect preWindow2(xCam2, yCam2, 50, 50); 
                    trackBox2 = CamShift(backproj2, preWindow2,
                                        TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                }
                
                if( trackWindow2.area() <= 1 )
                {
                    int cols = backproj2.cols, rows = backproj2.rows, r = (MIN(cols, rows) + 5)/6;
                    trackWindow2 = Rect(trackWindow2.x - r, trackWindow2.y - r,
                           trackWindow2.x + r, trackWindow2.y + r) &
                      Rect(0, 0, cols, rows);
                }

                if( backprojMode )
                        cvtColor( backproj2, image2, COLOR_GRAY2BGR );
                if(trackBox2.size.width != 0 && trackBox2.size.height != 0 )
                        ellipse( image2, trackBox2, Scalar(0,255,0), 3, CV_AA );

                cout << "\nCam 2 XY Position: \t\t";
                cout << trackBox2.center.x << "\t" << trackBox2.center.y;
                if(trackBox2.center.x != 0 || trackBox2.center.y != 0) {
                    xCam2 = trackBox2.center.x;
                    yCam2 = trackBox2.center.y;

                    measurement2(0) = xCam2;
                    measurement2(1) = yCam2; 

                    // The update phase 
                    Mat estimated2 = KF2.correct(measurement2);
                    Point statePt2(estimated2.at<float>(0),estimated2.at<float>(1));
                }

                prediction2 = KF2.predict();
                Point predictPt2(prediction2.at<float>(0),prediction2.at<float>(1));
                cout << "\nCam 2 Predicted Position: \t";
                cout << predictPt2.x << "\t" << predictPt2.y << "\n";
                circle(image2 , Point(  predictPt2.x,predictPt2.y), 16.0, Scalar( 0, 155, 255), 3, 8 );

                if(trackBox2.center.x != 0 || trackBox2.center.y != 0) {
                    posX2 = trackBox2.center.x;
                    posY2 = trackBox2.center.y;
                }
                else {
                    posX2 = predictPt2.x;
                    posY2 = predictPt2.y;
                }

            }

        }
        //  else if( trackObject1 < 0 && trackObject2 < 0 )
        //  paused = false;

          //==================== DISTANCE ESTIMATION ========================================================================//

        if( trackObject1 > 0 && trackObject2 > 0) {
           
            double dist = 600;

            // convert pixel position to angle
            double angleX1 = 90 - ((posX1*64) / 640);
            double angleX2 = (((posX2*64) / 640) + 26);

            // calculate tangensial value for angles
            double tan1 = tan( angleX1 * PI / 180.0 );
            double tan2 = tan( angleX2 * PI / 180.0 );

            // calculate object position
            int posX, posY;
            posX = (tan1 * dist) / (tan1 + tan2);
            posY = (tan2 * posX);

        //==================== distance estimation ========================================================================//

        //==================== HEIGHT ESTIMATION ========================================================================//
            
            double stand = 102.0;
            double posR, angleZ, tanZ;

            posR = sqrt(posX*posX + posY*posY);

            if(posY2 > 240) {
                angleZ = ((posY2*48) / 480) - 24;
                tanZ = tan(angleZ * PI / 180.0);

                posZ = posR * tanZ;
                posZ = stand - posZ;
            }
            else if (posY2 < 240){
                angleZ = 24 - ((posY2*48)/480);
                tanZ = tan(angleZ * PI / 180.0);

                posZ = posR * tanZ;
                posZ = stand + posZ;
            }
            else posZ = stand;

            cout << "\nObject position: \t";
            
            cout << 600 - posX << "\t";
            cout << posY << "\t";
            cout << posZ << "\n";
            cout << "Timestamp: \t";
            cout << timestamp << "\n\n";

            pose.position.x = posX;
            pose.position.y = posY;
            pose.position.z = posZ;

            transformer(pose);


        }
        //==================== height estimation ========================================================================//


        if( selectObject1 && selection1.width > 0 && selection1.height > 0 )
        {
            Mat roi1(image1, selection1);
            bitwise_not(roi1, roi1);
        }

        if( selectObject2 && selection2.width > 0 && selection2.height > 0 )
        {
            Mat roi2(image2, selection2);
            bitwise_not(roi2, roi2);
        }

        imshow( "CamShift 1", image1 );
        imshow( "CamShift 2", image2 );
        imshow( "Input 1", imageOriginal1 );
        imshow( "Input 2", imageOriginal2 );
        imshow( "Histogram 1", histimg1 );
        imshow( "Histogram 2", histimg2 );

        char c;
        if(step) c = (char)waitKey();
        else c = (char)waitKey(1);
        
        if( c == 27 )
                break;
        switch(c)
        {
            case 'b':
                backprojMode = !backprojMode;
                break;
            case 'c':
                trackObject1 = 0;
                histimg1 = Scalar::all(0);
                trackObject2 = 0;
                histimg2 = Scalar::all(0);
                break;
            case 'h':
                showHist = !showHist;
                if( !showHist ) {
                    destroyWindow( "Histogram 1" );
                    destroyWindow( "Histogram 2" );
                }
                else {
                    namedWindow( "Histogram 1", 1 );
                    namedWindow( "Histogram 2", 1 );
                }
                break;
            case 'p':
                paused = !paused;
                if(paused) step = false;
                else step = true;
                break;
            case 's':
                step = !step;
            default:
            ;
        }

        float delay = ros::Time::now().toSec() - start.toSec();
        while(delay < 0.040) {
            delay = ros::Time::now().toSec() - start.toSec();
        }
        
        // cout << "\n\n Delay = \t" << delay << "\n\n";

        ros::spinOnce();
    }
}