#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <std_msgs/Int16MultiArray.h>

using namespace std;
using namespace cv;

string path = ros::package::getPath("rostu_simulation");
YAML::Node odom_adj = YAML::LoadFile(path + "/cfg/rostu/odom_adj.yaml");

int x_slider_value = odom_adj["x_odom_adj"].as<int>();
int y_slider_value = odom_adj["y_odom_adj"].as<int>();
int th_slider_value = odom_adj["th_odom_adj"].as<int>();
int x_slider_max = 1000, y_slider_max = 1000, th_slider_max = 1000;

void on_x_trackbar( int, void* ) {
  if (x_slider_value < 1) {
    x_slider_value = 1;
  }

  odom_adj["x_odom_adj"] = x_slider_value;
  ofstream odom_adj_fout(path + "/cfg/rostu/odom_adj.yaml");
  odom_adj_fout << odom_adj;
  odom_adj_fout.close();
}

void on_y_trackbar( int, void* ) {
  if (y_slider_value < 1) {
    y_slider_value = 1;
  }

  odom_adj["y_odom_adj"] = y_slider_value;
  ofstream odom_adj_fout(path + "/cfg/rostu/odom_adj.yaml");
  odom_adj_fout << odom_adj;
  odom_adj_fout.close();
}

void on_th_trackbar( int, void* ) {
  if (th_slider_value < 1) {
    th_slider_value = 1;
  }

  odom_adj["th_odom_adj"] = th_slider_value;
  ofstream odom_adj_fout(path + "/cfg/rostu/odom_adj.yaml");
  odom_adj_fout << odom_adj;
  odom_adj_fout.close();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_odom_adj");
  ros::NodeHandle nh;
  ros::Publisher odom_adj = nh.advertise<std_msgs::Int16MultiArray>("odom_adj", 1);

  ros::Rate r(30);

  namedWindow("odom_adj", 1);

  createTrackbar("X Speed", "odom_adj", &x_slider_value, x_slider_max, on_x_trackbar);
  createTrackbar("Y Speed", "odom_adj", &y_slider_value, y_slider_max, on_y_trackbar);
  createTrackbar("TH Speed", "odom_adj", &th_slider_value, th_slider_max, on_th_trackbar);

  std_msgs::Int16MultiArray o_adj;
  o_adj.data.resize(2);

  while (ros::ok()) {
    o_adj.data[0] = x_slider_value;
    o_adj.data[1] = y_slider_value;

    odom_adj.publish(o_adj);

    if ((char)27 == (char)waitKey(1)) ros::shutdown();

    ros::spinOnce();
    r.sleep();
  }

  destroyAllWindows();
  return 0;
}
