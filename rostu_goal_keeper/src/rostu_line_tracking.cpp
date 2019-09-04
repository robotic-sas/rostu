#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16MultiArray.h>
#include <rostu_goal_keeper/LineCoor.h>

# define PI 3.14159265358979323846

using namespace std;
using namespace cv;

string path = ros::package::getPath("rostu_goal_keeper");
YAML::Node calibration_data = YAML::LoadFile(path + "/cfg/rostu/rostu_vision.yaml");

ros::Time current_time, last_time;
ros::Publisher line_coor_pub;
rostu_goal_keeper::LineCoor line_coor_msg;

Mat kernel = Mat::ones(1, 1, CV_8UC1);
Mat frame, mask[3], hsv;

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

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    frame = cv_bridge::toCvShare(msg, "8UC3")->image;
    current_time = ros::Time::now();

    double heading = 180 - 0;
    double cosHeading = cos(heading * 2 * PI / 360);
    double sinHeading = sin(heading * 2 * PI / 360);


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

      Mat maskClone = mask[2].clone();
      vector<vector<Point> > cnts;
      findContours(maskClone, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      //Drawing
      if (cnts.size() > 0) {
        Moments M;
        Point2f center;
        int largest_area = 0;
        int largest_contour_index = 0;

        for (int i = 0; i < cnts.size(); i++) {
          double a = contourArea(cnts[i], false);
          // cout << a << endl;
          if (a > 20000) {
            if (a > largest_area) {
              largest_area = a;
              largest_contour_index = i;
            }
          }
        }

        if (largest_area > 20000) {
          M = moments(cnts[largest_contour_index]);
          center.x = int(M.m10/M.m00);
          center.y = int(M.m01/M.m00);

          Point corners[1][4];
          corners[0][0] = Point( center.x, 1 );
          corners[0][1] = Point( 600, 1 );
          corners[0][2] = Point( 600, 600 );
          corners[0][3] = Point( center.x, 600 );
          vector<vector<Point> > corner_list(1);
          corner_list[0].resize(4);
          for (int i = 0; i < 4; i++) {
            corner_list[0][i] = corners[0][i];
          }
          mask_for_crop = Mat::zeros(frame.size(), CV_8UC3);
          fillPoly(mask_for_crop, corner_list, Scalar(255, 255, 255));
          bitwise_and(mask_for_crop, cropped_img, cropped_img);
          line_coor_msg.y1 = center.y;
        }
      }

      cvtColor(cropped_img, hsv_cropped_img, COLOR_BGR2HSV);
      inRange(hsv_cropped_img, Scalar(lower[2][0],lower[2][1],lower[2][2]), Scalar(upper[2][0],upper[2][1],upper[2][2]), mask[2]);
      maskClone = mask[2].clone();
      findContours(maskClone, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      if (cnts.size() > 0) {
        Moments M;
        Point2f center;
        int largest_area = 0;
        int largest_contour_index = 0;

        for (int i = 0; i < cnts.size(); i++) {
          double a = contourArea(cnts[i], false);
          if (a > 5000) {
            if (a > largest_area) {
              largest_area = a;
              largest_contour_index = i;
            }
          }
        }

        if (largest_area > 5000) {
          M = moments(cnts[largest_contour_index]);
          center.x = int(M.m10/M.m00);
          center.y = int(M.m01/M.m00);
          cout << M.m10 << " , " << M.m01 << " , " << M.m00 << endl;

          putText(frame, "Point", center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 2);
          line_coor_msg.y2 = center.y;
        }
      }
    }

    imshow("frame", frame);

    if ((char)27 == (char)waitKey(1)) ros::shutdown();
    line_coor_pub.publish(line_coor_msg);

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
  line_coor_pub = nh.advertise<rostu_goal_keeper::LineCoor>("rostu/line_coor", 1);

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::spin();
  destroyAllWindows();
  return 0;
}
