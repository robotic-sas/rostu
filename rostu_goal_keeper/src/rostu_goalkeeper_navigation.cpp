#include <iostream>
#include <ros/ros.h>

#include <rostu_goal_keeper/LineCoor.h>
#include <geometry_msgs/Twist.h>

using namespace std;

ros::Publisher rostu_cmdvel_pub;
geometry_msgs::Twist rostu_cmdvel_msg;

int lineCoorPointCenter, lineCoorPoint1, lineCoorPoint2;
double kp1 = 0.5, kd1 = 0.2, lastError1 = 0;
double kp2 = 0.25, kd2 = 0.1, lastError2 = 0;

double PD_Controller(double input, double KP, double KD, double& lError) {
  double error = input;
  double P_Value = KP * error;
  double D_Value = (lError - error) * KD;
  double PD_Value = P_Value + D_Value;
  lError = error;

  return PD_Value;
}

void lineFollower() {
  double lineError = lineCoorPoint1 - lineCoorPointCenter;

  if (lineError <= 10 && lineError >= -10) {
    lineError = 0;
  }
  else {
    if (lineError > 10) {
      lineError -= 10;
    }
    else if (lineError < -10) {
      lineError += 10;
    }
  }

  double output = PD_Controller(lineError, kp1, kd1, lastError1) * -1;
  if (output <= 5 && output >= -5) {
    output = 0;
  }
  else if (output > 150) {
    output = 150;
  }
  else if (output < -150) {
    output = -150;
  }
  else if (output > 5 && output < 95) {
    output = 95;
  }
  else if (output < -5 && output > -95) {
    output = -95;
  }

  double output2 = PD_Controller(lineCoorPointCenter - 150, kp2, kd2, lastError2) * -1;
  if (output2 <= 30 && output2 >= -30) {
    output2 = 0;
  }
  else if (output2 > 150) {
    output2 = 150;
  }
  else if (output2 < -150) {
    output2 = -150;
  }
  else if (output2 > 30 && output2 < 70) {
    output2 = 70;
  }
  else if (output2 < -30 && output2 > -70) {
    output2 = -70;
  }

  rostu_cmdvel_msg.angular.z = output;
  rostu_cmdvel_msg.linear.x = output2;
  rostu_cmdvel_pub.publish(rostu_cmdvel_msg);
}

void lineCallback(const rostu_goal_keeper::LineCoor& msg) {
  lineCoorPointCenter = msg.y1;
  lineCoorPoint1 = msg.y2;
  lineCoorPoint2 = msg.y3;
}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  rostu_cmdvel_msg.linear.y = msg.linear.y;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_goalkeeper_navigation");
  ros::NodeHandle nh;
  ros::Rate r(30);
  ros::Subscriber lineSub = nh.subscribe("rostu/line_coor", 1, lineCallback);
  ros::Subscriber cmdvelSub = nh.subscribe("cmd_vel", 1, cmdVelCallback);
  rostu_cmdvel_pub = nh.advertise<geometry_msgs::Twist>("rostu/cmd_vel", 1);

  while (ros::ok()) {
    lineFollower();

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
