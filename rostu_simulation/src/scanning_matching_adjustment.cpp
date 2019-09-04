#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <std_msgs/Int8.h>

using namespace std;
using namespace cv;

string path = ros::package::getPath("rostu_simulation");
YAML::Node scan_adj = YAML::LoadFile(path + "/cfg/rostu/map_adjustment.yaml");
ros::Publisher resize_frame;
std_msgs::Int8 r_frame;

int scan_slider_value = scan_adj["scanning_matching_adjustment"].as<int>();
int scan_slider_max = 100;

void on_scan_trackbar( int, void* ) {
  if (scan_slider_value < 1) {
    scan_slider_value = 1;
  }

  scan_adj["scanning_matching_adjustment"] = scan_slider_value;
  ofstream scan_adj_fout(path + "/cfg/rostu/map_adjustment.yaml");
  scan_adj_fout << scan_adj;
  scan_adj_fout.close();

  r_frame.data = scan_slider_value;
  resize_frame.publish(r_frame);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_scan_adj");
  ros::NodeHandle nh;
  resize_frame = nh.advertise<std_msgs::Int8>("rostu/camera/resize_frame", 1);

  ros::Rate r(30);

  namedWindow("scan_adj", 1);
  createTrackbar("Scan Adjust", "scan_adj", &scan_slider_value, scan_slider_max, on_scan_trackbar);

  while (ros::ok()) {
    if ((char)27 == (char)waitKey(1)) ros::shutdown();

    ros::spinOnce();
    r.sleep();
  }

  destroyAllWindows();
  return 0;
}
