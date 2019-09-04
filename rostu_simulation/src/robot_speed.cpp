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
YAML::Node robot_speed = YAML::LoadFile(path + "/cfg/rostu/robot_speed.yaml");

int x_slider_value = robot_speed["x_robot_speed"].as<int>();
int y_slider_value = robot_speed["y_robot_speed"].as<int>();
int th_slider_value = robot_speed["th_robot_speed"].as<int>();
int x_slider_max = 1000, y_slider_max = 1000, th_slider_max = 1000;

void on_x_trackbar( int, void* ) {
  if (x_slider_value < 1) {
    x_slider_value = 1;
  }

  robot_speed["x_robot_speed"] = x_slider_value;
  ofstream robot_speed_fout(path + "/cfg/rostu/robot_speed.yaml");
  robot_speed_fout << robot_speed;
  robot_speed_fout.close();
}

void on_y_trackbar( int, void* ) {
  if (y_slider_value < 1) {
    y_slider_value = 1;
  }

  robot_speed["y_robot_speed"] = y_slider_value;
  ofstream robot_speed_fout(path + "/cfg/rostu/robot_speed.yaml");
  robot_speed_fout << robot_speed;
  robot_speed_fout.close();
}

void on_th_trackbar( int, void* ) {
  if (th_slider_value < 1) {
    th_slider_value = 1;
  }

  robot_speed["th_robot_speed"] = th_slider_value;
  ofstream robot_speed_fout(path + "/cfg/rostu/robot_speed.yaml");
  robot_speed_fout << robot_speed;
  robot_speed_fout.close();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_robot_speed");
  ros::NodeHandle nh;
  ros::Publisher robot_speed = nh.advertise<std_msgs::Int16MultiArray>("robot_speed", 1);

  ros::Rate r(30);

  namedWindow("robot_speed", 1);

  createTrackbar("X Speed", "robot_speed", &x_slider_value, x_slider_max, on_x_trackbar);
  createTrackbar("Y Speed", "robot_speed", &y_slider_value, y_slider_max, on_y_trackbar);
  createTrackbar("TH Speed", "robot_speed", &th_slider_value, th_slider_max, on_th_trackbar);

  std_msgs::Int16MultiArray o_adj;
  o_adj.data.resize(2);

  while (ros::ok()) {
    o_adj.data[0] = x_slider_value;
    o_adj.data[1] = y_slider_value;

    robot_speed.publish(o_adj);

    if ((char)27 == (char)waitKey(1)) ros::shutdown();

    ros::spinOnce();
    r.sleep();
  }

  destroyAllWindows();
  return 0;
}
