#include <iostream>

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/ccalib/omnidir.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16MultiArray.h>

# define PI 3.14159265358979323846

using namespace std;
using namespace cv;

string path = ros::package::getPath("rostu_v2");
YAML::Node calibration_data = YAML::LoadFile(path + "/cfg/rostu/rostu_vision.yaml");

ros::Publisher scanPub;
ros::Time current_time, last_time;
sensor_msgs::LaserScan lineScan;

Mat kernel = Mat::ones(1, 1, CV_8UC1);
Mat frame, mask[3], hsv;

//Omnidirectional Camera Parameter Declaration
double K[3][3] = {{1.3543423650372035e+03 , 3.0622404481811563e+00, 2.8313230176743360e+02},
            {0, 1.3530679383107395e+03, 2.7597773946639654e+02},
            {0, 0, 1}};
double D[4] = {-6.8260594840136530e+00, 6.3615894572318972e+01, 1.6242358740954328e-02, 2.3056383278755574e-02};
double XI = 4.0106204115209874e+00;
double KNEW[3][3] = {{600/6, 0, 600/2},
                    {0, 600/6, 600/2},
                    {0, 0, 1}};

Mat knew = Mat(3, 3, CV_64F, KNEW);
Mat k = Mat(3, 3, CV_64F, K);
Mat d = Mat(1, 4, CV_64F, D);
Mat xi = Mat(1, 1, CV_64F, XI);
//End Omnidirectional Camera Parameter Declaration

int lower[3][3] = {
  {calibration_data["ball_h_low"].as<int>(), calibration_data["ball_s_low"].as<int>(), calibration_data["ball_v_low"].as<int>()},
  {calibration_data["field_h_low"].as<int>(), calibration_data["field_s_low"].as<int>(), calibration_data["field_v_low"].as<int>()},
  {calibration_data["line_h_low"].as<int>(), calibration_data["line_s_low"].as<int>(), calibration_data["line_v_low"].as<int>()}
};

int upper[3][3] = {
  {calibration_data["ball_h_up"].as<int>(), calibration_data["ball_s_up"].as<int>(), calibration_data["ball_v_up"].as<int>()},
  {calibration_data["field_h_up"].as<int>(), calibration_data["field_s_up"].as<int>(), calibration_data["field_v_up"].as<int>()},
  {calibration_data["line_h_up"].as<int>(), calibration_data["line_s_up"].as<int>(), calibration_data["line_v_up"].as<int>()}
};

int x_resize_value = calibration_data["scanning_matching_adjustment_x"].as<int>();
int y_resize_value = calibration_data["scanning_matching_adjustment_y"].as<int>();

vector<double> findDistanceNAngle(int x1, int y1, int x2, int y2, double sinH, double cosH) {
  double dis1 = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
  double dis2 = sqrt(pow(((x1 + int(sinH * 20)) - x1), 2) + pow(((y1 + int(cosH * 20)) - y1), 2));
  double dis3 = sqrt(pow((x2 - (x1 + int(sinH * 20))), 2) + pow((y2 - (y1 + int(cosH * 20))), 2));

  double angle = atan2((y1 - (y1 + int(cosH * 20))), (x1 - (x1 + int(sinH * 20)))) - atan2(y2-y1, x2-x1);
  angle = angle * 360 / (2 * PI);

  if (angle < 0) {
    angle = angle + 360;
  }

  vector<double> distanceNAngle(2);
  distanceNAngle[0] = dis1;
  distanceNAngle[1] = angle;

  return distanceNAngle;
}

void resize_frame_callback(const std_msgs::Int16MultiArray& msg) {
  x_resize_value = msg.data[0];
  y_resize_value = msg.data[1];
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    frame = cv_bridge::toCvShare(msg, "8UC3")->image;
    current_time = ros::Time::now();

    lineScan.header.stamp = current_time;

    double heading = 180 - 0;
    double cosHeading = cos(heading * 2 * PI / 360);
    double sinHeading = sin(heading * 2 * PI / 360);

    //Undistort Omnidirectional Camera
    omnidir::undistortImage(frame, frame, k, d, xi, omnidir::RECTIFY_PERSPECTIVE, knew);
    flip(frame, frame, 1);
    resize(frame, frame, Size(600 * (float(x_resize_value) / 100), 600 * (float(y_resize_value) / 100)), 0, 0, cv::INTER_AREA);

    //Convert Captured Frame From BGR to HSV
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    //Thresholding For Convex Hull
    for (int i = 0; i < 3; i++) {
      inRange(hsv, Scalar(lower[i][0],lower[i][1],lower[i][2]), Scalar(upper[i][0],upper[i][1],upper[i][2]), mask[i]);
      erode(mask[i], mask[i], 0, Point(-1,-1), 2);
      dilate(mask[i], mask[i], 0, Point(-1, -1), 2);
      morphologyEx(mask[i] ,mask[i], MORPH_CLOSE, kernel);
    }

    //Find Contours For Convex Hull
    Mat maskMerge = mask[0] + mask[1] + mask[2];
    vector<vector<Point> > hullCnts;
    vector<vector<Point> > cnts;
    vector<vector<Point> > largestHull(1);
    findContours(maskMerge.clone(), hullCnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    //Drawing Convex Hull
    vector<Vec4i> lines;
    vector<vector<double> > distanceNAngle;

    Mat cropped_img, hsv_cropped_img;

    if (hullCnts.size() > 0) {
      vector<vector<Point> > hull(hullCnts.size());

      int largest_area = 0;
      int largest_contour_index = 0;

      for (int i = 0; i < hullCnts.size(); i++) {
        convexHull(Mat(hullCnts[i]), hull[i], false);
      }

      for (int i = 0; i < hull.size(); i++) {
        double a = contourArea(hull[i], false);
        if (a > largest_area) {
          largest_area = a;
          largest_contour_index = i;
        }
      }

      largestHull[0] = hull[largest_contour_index];
      // drawContours(frame, largestHull, 0, Scalar(0, 0, 255), 2, 8);

      //Convex Hull Crop
      Mat mask_for_crop = Mat::zeros(frame.size(), CV_8UC3);
      fillPoly(mask_for_crop, largestHull, Scalar(255, 255, 255));
      bitwise_and(mask_for_crop, frame, cropped_img);

      cvtColor(cropped_img, hsv_cropped_img, COLOR_BGR2HSV);

      //Thresholding For Line Tracking
      inRange(hsv_cropped_img, Scalar(lower[2][0],lower[2][1],lower[2][2]), Scalar(upper[2][0],upper[2][1],upper[2][2]), mask[2]);

      //Line Tracking With Hough Lines
      HoughLinesP(mask[2], lines, 100, PI/180, 1);
      for (int i = 0; i < lines.size(); i++) {
          int x1 = lines[i][0], y1 = lines[i][1], x2 = lines[i][2], y2 = lines[i][3];
          vector<double> disNang(2);
          disNang  = findDistanceNAngle(frame.cols/2, frame.rows/2, x1, y1, sinHeading, cosHeading);
          distanceNAngle.push_back(disNang);
          // line(frame, Point(frame.cols/2, frame.rows/2), Point(x1, y1), Scalar(255, 0, 255), 1);
          line(frame, Point(x1, y1), Point(x2, y2), Scalar(0, 0, 255), 2);
      }
    }

    lineScan.ranges.resize(360);
    for (int i = 0; i < 360; i++) {
      lineScan.ranges[i] = 0;
    }

    for (int i = 0; i < distanceNAngle.size(); i++) {
      lineScan.ranges[int(distanceNAngle[i][1])] = distanceNAngle[i][0] * 0.05 * 0.280898876;
    }

    // imshow("mask", mask[2]);
    // imshow("frame", frame);

    scanPub.publish(lineScan);

    if ((char)27 == (char)waitKey(1)) ros::shutdown();

    last_time = current_time;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to '8UC3'.", msg->encoding.c_str());
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_line_tracking");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber camSub = it.subscribe("rostu/camera/image", 1, imageCallback);
  ros::Subscriber sub = nh.subscribe("rostu/camera/resize_frame", 1, resize_frame_callback);
  scanPub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  lineScan.header.frame_id = "base_laser_link";
  lineScan.angle_min = -3.14159;
  lineScan.angle_max = 3.14159;
  lineScan.angle_increment = 0.0174533;
  lineScan.range_min = 0.1;
  lineScan.range_max = 10;
  lineScan.scan_time = 0.0;
  lineScan.time_increment = 0.0;
  lineScan.intensities.resize(360);

  for (int i = 0; i < 360; i++) {
      lineScan.intensities[i] = 126.0;
  }

  ros::spin();
  destroyAllWindows();
  return 0;
}

