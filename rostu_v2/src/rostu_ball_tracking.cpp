#include <iostream>
#include <fstream>

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/ccalib/omnidir.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <rostu_v2/BallCoor.h>

# define PI 3.14159265358979323846

using namespace std;
using namespace cv;

string path = ros::package::getPath("rostu_v2");
YAML::Node calibration_data = YAML::LoadFile(path + "/cfg/rostu/rostu_vision.yaml");

ros::Publisher ballCoorPub;
rostu_v2::BallCoor ballCoor;

Mat frame, depth_image, mask[3], hsv;

//Omnidirectional Camera Parameter Declaration
double K[3][3] = {{1.3543423650372035e+03 , 3.0622404481811563e+00, 2.8313230176743360e+02},
            {0, 1.3530679383107395e+03, 2.7597773946639654e+02},
            {0, 0, 1}};
double D[4] = {-6.8260594840136530e+00, 6.3615894572318972e+01, 1.6242358740954328e-02, 2.3056383278755574e-02};
double XI = 4.0106204115209874e+00;
double KNEW[3][3] = {{600/16, 0, 600/2},
                    {0, 600/16, 600/2},
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

Mat kernel = Mat::ones(1, 1, CV_8UC1);

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    frame = cv_bridge::toCvShare(msg, "8UC3")->image;

    omnidir::undistortImage(frame, frame, k, d, xi, omnidir::RECTIFY_PERSPECTIVE, knew);

    flip(frame, frame, 1);

    cvtColor(frame, hsv, COLOR_BGR2HSV);

    for (int i = 0; i < 3; i++) {
      inRange(hsv, Scalar(lower[i][0],lower[i][1],lower[i][2]), Scalar(upper[i][0],upper[i][1],upper[i][2]), mask[i]);
      erode(mask[i], mask[i], 0, Point(-1,-1), 2);
      dilate(mask[i], mask[i], 0, Point(-1, -1), 2);
      morphologyEx(mask[i] ,mask[i], MORPH_CLOSE, kernel);
    }

    Mat maskMerge = mask[0] + mask[1] + mask[2];
    vector<vector<Point> > hullCnts;
    vector<vector<Point> > cnts;
    vector<vector<Point> > largestHull(1);
    findContours(maskMerge.clone(), hullCnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // imshow("convex_mask", maskMerge);

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

      inRange(hsv_cropped_img, Scalar(lower[0][0],lower[0][1],lower[0][2]), Scalar(upper[0][0],upper[0][1],upper[0][2]), mask[0]);
      erode(mask[0], mask[0], 0, Point(-1,-1), 2);
      dilate(mask[0], mask[0], 0, Point(-1, -1), 2);
      morphologyEx(mask[0] ,mask[0], MORPH_CLOSE, kernel);

      //Find Contours
      Mat maskClone = mask[0].clone();
      vector<vector<Point> > cnts;
      findContours(maskClone, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      //Drawing
      if (cnts.size() > 0) {
        Moments M;
        Point2f center;
        float radius;
        int largest_area = 0;
        int largest_contour_index = 0;

        for (int i = 0; i < cnts.size(); i++) {
          double a = contourArea(cnts[i], false);
          if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;
          }
        }

        minEnclosingCircle(cnts[largest_contour_index], center, radius);
        M = moments(cnts[largest_contour_index]);

        if (radius > 0) {
          // circle(frame, center, int(radius), Scalar(0, 255, 0), 2);
          circle(cropped_img, center, int(radius), Scalar(0, 255, 0), 2);
        }

        double length = sqrt(pow(center.x - 300, 2) + pow(center.y - 300, 2));
        double angle = (atan2(300 - 0, 300 - 300) - atan2(300 - center.y, 300 - center.x)) * 360 / (2 * PI) * -1;
        ballCoor.ball_detect = true;
        ballCoor.distance = length;
        ballCoor.angle = angle;
        line(frame, Point(300, 300), center, Scalar(0, 0, 255), 1);
        putText(frame, to_string(length), Point(center.x - 25, center.y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 2);
        putText(frame, to_string(angle), Point(center.x - 25, center.y + 25), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 2);
      }
      else {
        ballCoor.ball_detect = false;
        ballCoor.distance = 0;
        ballCoor.angle = 0;
      }

      ballCoorPub.publish(ballCoor);
      line(frame, Point(300, 0), Point(300, 600), Scalar(255, 0, 0), 1);
    }

    imshow("frame", frame);
    // imshow("cropped_img", cropped_img);

    if ((char)27 == (char)waitKey(1)) ros::shutdown();
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to '8UC3'.", msg->encoding.c_str());
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_ball_tracking");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber camSub = it.subscribe("rostu/camera/image", 1, imageCallback);
  ballCoorPub = nh.advertise<rostu_v2::BallCoor>("rostu/ball_coor", 1);

  ros::spin();
  destroyAllWindows();
  return 0;
}
