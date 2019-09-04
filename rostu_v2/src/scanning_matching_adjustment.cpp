#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <std_msgs/Int16MultiArray.h>

using namespace std;
using namespace cv;

string path = ros::package::getPath("rostu_v2");
YAML::Node scan_adj = YAML::LoadFile(path + "/cfg/rostu/rostu_vision.yaml");
ros::Publisher resize_frame;
std_msgs::Int16MultiArray r_frame;

int x_slider_value = scan_adj["scanning_matching_adjustment_x"].as<int>();
int y_slider_value = scan_adj["scanning_matching_adjustment_y"].as<int>();
int x_slider_max = 200, y_slider_max = 200;

void on_x_trackbar( int, void* ) {
  if (x_slider_value < 1) {
    x_slider_value = 1;
  }

  scan_adj["scanning_matching_adjustment_x"] = x_slider_value;
  ofstream scan_adj_fout(path + "/cfg/rostu/rostu_vision.yaml");
  scan_adj_fout << scan_adj;
  scan_adj_fout.close();

  r_frame.data[0] = x_slider_value;
  r_frame.data[1] = y_slider_value;
  resize_frame.publish(r_frame);
}

void on_y_trackbar( int, void* ) {
  if (y_slider_value < 1) {
    y_slider_value = 1;
  }

  scan_adj["scanning_matching_adjustment_y"] = y_slider_value;
  ofstream scan_adj_fout(path + "/cfg/rostu/rostu_vision.yaml");
  scan_adj_fout << scan_adj;
  scan_adj_fout.close();

  r_frame.data[0] = x_slider_value;
  r_frame.data[1] = y_slider_value;
  resize_frame.publish(r_frame);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_scan_adj");
  ros::NodeHandle nh;
  resize_frame = nh.advertise<std_msgs::Int16MultiArray>("rostu/camera/resize_frame", 1);

  ros::Rate r(30);
  r_frame.data.resize(2);

  namedWindow("scan_adj", 1);
  createTrackbar("X Resize", "scan_adj", &x_slider_value, x_slider_max, on_x_trackbar);
  createTrackbar("Y Resize", "scan_adj", &y_slider_value, y_slider_max, on_y_trackbar);

  while (ros::ok()) {
    if ((char)27 == (char)waitKey(1)) ros::shutdown();

    ros::spinOnce();
    r.sleep();
  }

  destroyAllWindows();
  return 0;
}
