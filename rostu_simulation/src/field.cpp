#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16MultiArray.h>

# define PI 3.14159265358979323846

using namespace std;
using namespace cv;

string path = ros::package::getPath("rostu_simulation");
YAML::Node calibration_data = YAML::LoadFile(path + "/cfg/rostu/map_adjustment.yaml");
YAML::Node odom_adj = YAML::LoadFile(path + "/cfg/rostu/odom_adj.yaml");
YAML::Node robot_speed = YAML::LoadFile(path + "/cfg/rostu/robot_speed.yaml");

int posX = 140, posY = 560, degree = 90;
double pX = 140.0, pY = 560.0, deg = 90.0;
bool spawn = false;

int obstaclePosX = 0, obstaclePosY = 0;


double vx;
double vy;
double vth;

double vx_speed = float(robot_speed["x_robot_speed"].as<int>()) / float(100);
double vy_speed = float(robot_speed["y_robot_speed"].as<int>()) / float(100);
double vth_speed = float(robot_speed["th_robot_speed"].as<int>()) / float(100);

double odom_x;
double odom_y;
double odom_th;

double odom_vx;
double odom_vy;
double odom_vth;

double vx_odom_adj = float(odom_adj["x_odom_adj"].as<int>()) / float(100);
double vy_odom_adj = float(odom_adj["y_odom_adj"].as<int>()) / float(100);
double vth_odom_adj = float(odom_adj["th_odom_adj"].as<int>()) / float(100);

double dt;
double delta_x;
double delta_y;
double delta_th;

int scan_resize_value = calibration_data["scanning_matching_adjustment"].as<int>();

void resize_frame_callback(const std_msgs::Int8& msg) {
  scan_resize_value = msg.data;
}

void odom_adj_callback(const std_msgs::Int16MultiArray& msg) {
  vx_odom_adj = float(msg.data[0]) / float(100);
  vy_odom_adj = float(msg.data[1]) / float(100);
  vth_odom_adj = float(msg.data[2]) / float(100);
}

void robot_speed_callback(const std_msgs::Int16MultiArray& msg) {
  vx_speed = float(msg.data[0]) / float(100);
  vy_speed = float(msg.data[1]) / float(100);
  vth_speed = float(msg.data[2]) / float(100);
}

static void leftClick( int event, int x, int y, int, void* ) {
  if (event == EVENT_LBUTTONDOWN) {
    spawn = true;
  }
  else if (event == EVENT_LBUTTONUP) {
    if (spawn) {
        obstaclePosX = x;
        obstaclePosY = y;
    }
  }
}

void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  double heading = 180 - degree;
  double cosHeading = cos(heading * 2 * PI / 360);
  double sinHeading = sin(heading * 2 * PI / 360);

  vx = msg.linear.x * vx_speed;
  vy = msg.linear.y * vy_speed;
  vth = msg.angular.z * vth_speed;

  odom_vx = msg.linear.x * vx_odom_adj;
  odom_vy = msg.linear.y * vy_odom_adj;
  odom_vth = msg.angular.z * vth_odom_adj;

  delta_x = (vx * cos(odom_th) - vy * sin(odom_th)) * dt;
  delta_y = (vx * sin(odom_th) + vy * cos(odom_th)) * dt;
  delta_th = vth * dt;

  pX += ((vx * sinHeading + vy * cosHeading) * dt) * 100;
  pY += ((vx * cosHeading - vy * sinHeading) * dt) * 100;

  odom_x += delta_x;
  odom_y += delta_y;
  odom_th += delta_th;
  
  posX = int(pX);
  posY = int(pY);

  deg -= delta_th * 180 / PI;

  degree = int(deg);

  delta_x = 0;
  delta_y = 0;
  delta_th = 0;
}

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "rostu_simulation");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cmd_vel", 1, cmd_vel_callback);
  ros::Subscriber sub2 = nh.subscribe("rostu/camera/resize_frame", 1, resize_frame_callback);
  ros::Subscriber sub3 = nh.subscribe("odom_adj", 1, odom_adj_callback);
  ros::Subscriber sub4 = nh.subscribe("robot_speed", 1, robot_speed_callback);
  ros::Publisher scanPub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  ros::Publisher obstaclePub = nh.advertise<sensor_msgs::LaserScan>("obstacle_scan", 1);
  ros::Publisher nav_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);

  ros::Rate r(30);

  tf::TransformBroadcaster bll_broadcaster;
  tf::TransformBroadcaster bl_broadcaster;
  tf::TransformBroadcaster bf_broadcaster;

  odom_x = 0.0;
  odom_y = 0.0;
  odom_th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  sensor_msgs::LaserScan obstacleScan;
  obstacleScan.header.frame_id = "base_laser_link";
  obstacleScan.angle_min = -3.14159;
  obstacleScan.angle_max = 3.14159;
  obstacleScan.angle_increment = 0.0174533;
  obstacleScan.range_min = 0.1;
  obstacleScan.range_max = 10;
  obstacleScan.scan_time = 0.0;
  obstacleScan.time_increment = 0.0;
  obstacleScan.intensities.resize(360);

  for (int i = 0; i < 360; i++) {
      obstacleScan.intensities[i] = 126.0;
  }

  sensor_msgs::LaserScan lineScan;
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

  namedWindow("field", WINDOW_AUTOSIZE);
  // namedWindow("mask", WINDOW_AUTOSIZE);
  // namedWindow("cropped_img", WINDOW_AUTOSIZE);
  setMouseCallback("field", leftClick);

  Mat field;

  while (ros::ok()) {
    current_time = ros::Time::now();
    obstacleScan.header.frame_id = "base_laser_link";
    obstacleScan.header.stamp = current_time;

    lineScan.header.frame_id = "base_laser_link";
    lineScan.header.stamp = current_time;

    dt = (current_time - last_time).toSec();

    double heading = 180 - degree;
    double cosHeading = cos(heading * 2 * PI / 360);
    double sinHeading = sin(heading * 2 * PI / 360);

    field = imread(path + "/model/field/field.png", CV_LOAD_IMAGE_COLOR);
    resize(field, field, Size(field.cols / 4, field.rows / 4));

    // Drawing Robot
    circle(field, Point(posX, posY), 20, Scalar(255, 0, 0), -1);
    line(field, Point(posX, posY), Point(posX + int(sinHeading * 20), posY + int(cosHeading * 20)), Scalar(0, 0, 255), 2);
    // End of Drawing

    // Drawing Obstacle
    if (spawn) {
      circle(field, Point(obstaclePosX, obstaclePosY), 20, Scalar(100, 100, 100), -1);
    }
    // End of Drawing

    // Drawing Area of Scanning Box
    Point Points[5] = {Point(posX + int(cosHeading * 200) - int(sinHeading * 200) , posY - int(sinHeading * 200) - int(cosHeading * 200)),
                  Point(posX + int(cosHeading * 200) + int(sinHeading * 200), posY - int(sinHeading * 200) + int(cosHeading * 200)),
                  Point(posX - int(cosHeading * 200) + int(sinHeading * 200), posY + int(sinHeading * 200) + int(cosHeading * 200)),
                  Point(posX - int(cosHeading * 200) - int(sinHeading * 200) , posY + int(sinHeading * 200) - int(cosHeading * 200)),
                  Point(posX + int(cosHeading * 200) - int(sinHeading * 200), posY - int(sinHeading * 200) - int(cosHeading * 200))
    };

    vector<vector<Point> > rect(1);
    rect[0].resize(5);

    for (int i = 0; i < 5; i++) {
      rect[0][i] = Points[i];
    }

    drawContours(field, rect, 0, Scalar(0, 0, 255), 2, 8);
    // End of Drawing

    Mat cropped_img, mask, mask_obstacle;
    Mat mask_for_crop = Mat::zeros(field.size(), CV_8UC3);
    fillPoly(mask_for_crop, rect, Scalar(255, 255, 255));
    bitwise_and(mask_for_crop, field, cropped_img);

    inRange(cropped_img, Scalar(150, 150, 150), Scalar(255, 255, 255), mask);

    vector<Vec4i> lines;
    HoughLinesP(mask, lines, 100, PI/180, 1);

    vector<vector<double> > distanceNAngle;

    for (int i = 0; i < lines.size(); i++) {
        int x1 = lines[i][0], y1 = lines[i][1], x2 = lines[i][2], y2 = lines[i][3];
        vector<double> disNang(2);
        disNang  = findDistanceNAngle(posX, posY, x1, y1, sinHeading, cosHeading);
        distanceNAngle.push_back(disNang);
        // line(field, Point(posX, posY), Point(x1, y1), Scalar(255, 0, 255), 1);
        line(field, Point(x1, y1), Point(x2, y2), Scalar(0, 0, 255), 2);
    }

    lineScan.ranges.resize(360);
    for (int i = 0; i < 360; i++) {
      lineScan.ranges[i] = 0;
    }

    for (int i = 0; i < distanceNAngle.size(); i++) {
      lineScan.ranges[int(distanceNAngle[i][1])] = distanceNAngle[i][0] * float(scan_resize_value) / float(1000);
    }

    inRange(cropped_img, Scalar(80, 80, 80), Scalar(110, 110, 110), mask_obstacle);

    vector<Vec4i> lines_obstacle;
    HoughLinesP(mask_obstacle, lines_obstacle, 100, PI/180, 1);

    vector<vector<double> > distanceNAngle_obstacle;

    for (int i = 0; i < lines_obstacle.size(); i++) {
        int x1 = lines_obstacle[i][0], y1 = lines_obstacle[i][1], x2 = lines_obstacle[i][2], y2 = lines_obstacle[i][3];
        vector<double> disNang(2);
        disNang  = findDistanceNAngle(posX, posY, x1, y1, sinHeading, cosHeading);
        distanceNAngle_obstacle.push_back(disNang);
        // line(field, Point(posX, posY), Point(x1, y1), Scalar(255, 0, 255), 1);
        line(field, Point(x1, y1), Point(x2, y2), Scalar(50, 50, 50), 2);
    }

    obstacleScan.ranges.resize(360);
    for (int i = 0; i < 360; i++) {
      obstacleScan.ranges[i] = 9.9;
    }

    for (int i = 0; i < distanceNAngle_obstacle.size(); i++) {
      obstacleScan.ranges[int(distanceNAngle_obstacle[i][1])] = distanceNAngle_obstacle[i][0] * 0.05 * 0.280898876;
    }

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_th);

    geometry_msgs::TransformStamped bll_trans;
    bll_trans.header.stamp = current_time;
    bll_trans.header.frame_id = "base_link";
    bll_trans.child_frame_id = "base_laser_link";

    bll_trans.transform.translation.x = 0;
    bll_trans.transform.translation.y = 0;
    bll_trans.transform.translation.z = 0.2;
    bll_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    bll_broadcaster.sendTransform(bll_trans);

    geometry_msgs::TransformStamped bl_trans;
    bl_trans.header.stamp = current_time;
    bl_trans.header.frame_id = "base_footprint";
    bl_trans.child_frame_id = "base_link";

    bl_trans.transform.translation.x = 0;
    bl_trans.transform.translation.y = 0;
    bl_trans.transform.translation.z = 0.0;
    bl_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    bl_broadcaster.sendTransform(bl_trans);

    geometry_msgs::TransformStamped bf_trans;
    bf_trans.header.stamp = current_time;
    bf_trans.header.frame_id = "odom";
    bf_trans.child_frame_id = "base_footprint";

    bf_trans.transform.translation.x = odom_x;
    bf_trans.transform.translation.y = odom_y;
    bf_trans.transform.translation.z = 0.0;
    bf_trans.transform.rotation = odom_quat;
    bf_broadcaster.sendTransform(bf_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = odom_x;
    odom.pose.pose.position.y = odom_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "";
    odom.twist.twist.linear.x = odom_vx;
    odom.twist.twist.linear.y = odom_vy;
    odom.twist.twist.angular.z = odom_vth;

    imshow("field", field);
    // imshow("mask", mask);
    // imshow("cropped_img", cropped_img);

    scanPub.publish(lineScan);
    obstaclePub.publish(obstacleScan);
    nav_odom.publish(odom);

    if ((char)27 == (char)waitKey(1)) ros::shutdown();

    last_time = current_time;
    ros::spinOnce();
    r.sleep();
  }

  ros::spin();
  destroyAllWindows();
  return 0;
}
