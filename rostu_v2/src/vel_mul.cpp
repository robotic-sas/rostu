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
YAML::Node speed_mul = YAML::LoadFile(path + "/cfg/rostu/speed_mul.yaml");

int x_slider_value = speed_mul["x_speed_mul"].as<int>();
int y_slider_value = speed_mul["y_speed_mul"].as<int>();
int w_slider_value = speed_mul["w_speed_mul"].as<int>();
int x_slider_max = 500, y_slider_max = 500, w_slider_max = 250;

void on_x_trackbar( int, void* ) {
  if (x_slider_value < 1) {
    x_slider_value = 1;
  }

  speed_mul["x_speed_mul"] = x_slider_value;
  ofstream speed_mul_fout(path + "/cfg/rostu/speed_mul.yaml");
  speed_mul_fout << speed_mul;
  speed_mul_fout.close();
}

void on_y_trackbar( int, void* ) {
  if (y_slider_value < 1) {
    y_slider_value = 1;
  }

  speed_mul["y_speed_mul"] = y_slider_value;
  ofstream speed_mul_fout(path + "/cfg/rostu/speed_mul.yaml");
  speed_mul_fout << speed_mul;
  speed_mul_fout.close();
}

void on_w_trackbar( int, void* ) {
  if (w_slider_value < 1) {
    w_slider_value = 1;
  }

  speed_mul["w_speed_mul"] = w_slider_value;
  ofstream speed_mul_fout(path + "/cfg/rostu/speed_mul.yaml");
  speed_mul_fout << speed_mul;
  speed_mul_fout.close();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_vel_mul");
  ros::NodeHandle nh;
  ros::Publisher vel_mul = nh.advertise<std_msgs::Int16MultiArray>("vel_mul", 1);

  ros::Rate r(30);

  namedWindow("vel", 1);

  createTrackbar("X Speed", "vel", &x_slider_value, x_slider_max, on_x_trackbar);
  createTrackbar("Y Speed", "vel", &y_slider_value, y_slider_max, on_y_trackbar);
  createTrackbar("W Speed", "vel", &w_slider_value, w_slider_max, on_w_trackbar);

  std_msgs::Int16MultiArray v_mul;
  v_mul.data.resize(3);

  while (ros::ok()) {
    v_mul.data[0] = x_slider_value;
    v_mul.data[1] = y_slider_value;
    v_mul.data[2] = w_slider_value;

    vel_mul.publish(v_mul);

    if ((char)27 == (char)waitKey(1)) ros::shutdown();

    ros::spinOnce();
    r.sleep();
  }

  destroyAllWindows();
  return 0;
}
