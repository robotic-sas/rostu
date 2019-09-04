#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <rostu_v2/BallCoor.h>
#include <rostu_v2/Dribling.h>
#include <rostu_v2/Kicker.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#define PI 3.14159265358979323846

using namespace std;

string path = ros::package::getPath("rostu_v2");
YAML::Node team_config = YAML::LoadFile(path + "/cfg/rostu/team.yaml");

double x;
double y;
double th = 0;

double vx;
double vy;
double vth;

double dt;
double delta_x;
double delta_y;
double delta_th;

bool step_back = false;
bool find_ball = false;
bool ball_detect, got_ball;
double ball_distance_from_frame = 200;
double ball_angle_from_frame;
double kicker_volt;
string referee_command = "S";
string robot_team = team_config["team"].as<string>();
bool startGame = false;
// int session = 0; //Babak Pertandingan

geometry_msgs::Twist rostu_cmdvel;
geometry_msgs::Twist cmdvel;
rostu_v2::Dribling dribling;
rostu_v2::Kicker kicker_msg;
geometry_msgs::Quaternion robot_orientation;
geometry_msgs::PoseStamped goal_publish_msg;
actionlib_msgs::GoalID cancel_goal_msg;;

ros::Publisher cancel_goal_pub;
ros::Publisher publish_goal;

double lastGoalPosX, lastGoalPosY, lastGoalPosZ, lastGoalPosW;
bool waitDelayCondition = false;
double waitDelaySec = 0.0;
bool robot_kickoff = false, robot_freekick = false, robot_goalkick = false, robot_throw_in = false, robot_corner = false, robot_penalty = false;
bool enemy_kickoff = false, enemy_freekick = false, enemy_goalkick = false, enemy_throw_in = false, enemy_corner = false, enemy_penalty = false;

ros::Time current_time, last_time, counter, waitDelay, delayUniversal;

double kp = 0.05, kd = 0.03;
double setpoint = 0.0;
double error;
double P_Value;
double D_Value;
double PD_Value;
double lastError;

double robot_center_pose_x, robot_center_pose_y, robot_angle;
double robot_end_pose_x, robot_end_pose_y;
double robot_angle_from_goal;
int robot_status = 3, robot_find_ball_pose = 0;
double gawang_point_x, gawang_point_y;

double PD_Controller(double th) {
  error = th;
  P_Value = kp * error;
  D_Value = (lastError - error) * kd;
  PD_Value = P_Value + D_Value;
  lastError = error;

  return PD_Value;
}

void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  vx = msg.linear.x;
  vy = msg.linear.y;
  vth = msg.angular.z;

  delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  delta_x = 0;
  delta_y = 0;
  delta_th = 0;

  rostu_cmdvel = msg;
}

void ball_coor_callback(const rostu_v2::BallCoor& msg) {
  ball_detect = msg.ball_detect;
  ball_distance_from_frame = msg.distance;
  ball_angle_from_frame = msg.angle;

  if (ball_angle_from_frame < 5 && ball_angle_from_frame > -5) {
    ball_angle_from_frame = 0;
  }
  if (ball_detect) {
    if (ball_distance_from_frame < 50) {
      dribling.d1pwm1 = 200;
      dribling.d1pwm2 = 0;
      dribling.d2pwm1 = 200;
      dribling.d2pwm2 = 0;
    }
  }
  else {
    if (!step_back) {
      dribling.d1pwm1 = 0;
      dribling.d1pwm2 = 0;
      dribling.d2pwm1 = 0;
      dribling.d2pwm2 = 0;
    }
  }
}

void dribling_callback(const rostu_v2::Dribling& msg) {
  got_ball = msg.got_ball;
}

void kicker_callback(const rostu_v2::Kicker& msg) {
  kicker_volt = msg.cap_volt;
  kicker_msg.cap_volt = kicker_volt;
}

void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  robot_center_pose_x = msg.pose.pose.position.x;
  robot_center_pose_y = msg.pose.pose.position.y;
  robot_orientation = msg.pose.pose.orientation;
  robot_angle = tf::getYaw(robot_orientation);
  robot_end_pose_x = robot_center_pose_x - sin(robot_angle) * 0.25;
  robot_end_pose_y = robot_center_pose_y - cos(robot_angle) * 0.25;
  double angle = atan2(robot_center_pose_y - robot_end_pose_y, robot_center_pose_x - robot_end_pose_x) - atan2(gawang_point_x - robot_center_pose_x, gawang_point_y - robot_center_pose_y);
  robot_angle_from_goal = angle * 360 / (2 * PI);
  if (robot_angle_from_goal < 10 && robot_angle_from_goal > -10) {
    robot_angle_from_goal = 0;
  }
}

void move_base_status_callback(const actionlib_msgs::GoalStatusArray& msg) {
  if (msg.status_list.size() > 0) {
    int msg_size = msg.status_list.size();
    robot_status = msg.status_list[msg_size - 1].status;
  }

  // if (robot_status == 3 || robot_status == 2) {
  //   lastGoalPosX = 0; lastGoalPosY = 0; lastGoalPosZ = 0; lastGoalPosW = 0;
  // }

  if (startGame) {
    if (!ball_detect) {
      if (robot_status == 3 || robot_status == 2) {
        find_ball = true;
        if ((current_time - counter).toSec() > 1.5) {
          goal_publish_msg.header.frame_id = "map";
          if (robot_find_ball_pose == 0) {
            goal_publish_msg.pose.position.x = 3;
            goal_publish_msg.pose.position.y = 3.75;
            goal_publish_msg.pose.orientation.w = 1.0;
          }
          else if (robot_find_ball_pose == 1) {
            goal_publish_msg.pose.position.x = 5.25;
            goal_publish_msg.pose.position.y = 6.125;
            goal_publish_msg.pose.orientation.w = 1.0;
          }
          else if (robot_find_ball_pose == 2) {
            goal_publish_msg.pose.position.x = 7.5;
            goal_publish_msg.pose.position.y = 3.75;
            goal_publish_msg.pose.orientation.w = 1.0;
          }
          else if (robot_find_ball_pose == 3) {
            goal_publish_msg.pose.position.x = 5.25;
            goal_publish_msg.pose.position.y = 1.375;
            goal_publish_msg.pose.orientation.w = 1.0;
          }
          robot_find_ball_pose++;
          publish_goal.publish(goal_publish_msg);
        }
      }
      else {
        counter = ros::Time::now();
      }
    }
    else if (ball_detect && robot_status == 1 && find_ball) {
      find_ball = false;
      cancel_goal_pub.publish(cancel_goal_msg);
    }

    if (robot_find_ball_pose > 3) {
      robot_find_ball_pose = 0;
    }
  }
}

void referee_command_callback(const std_msgs::String& msg) {
  referee_command = msg.data;
}

void publish_goal_pos(double x, double y, double z, double w) {
  if (lastGoalPosX != x || lastGoalPosY != y || lastGoalPosZ != z || lastGoalPosW != w) {
    goal_publish_msg.header.frame_id = "map";
    goal_publish_msg.pose.position.x = x;
    goal_publish_msg.pose.position.y = y;
    goal_publish_msg.pose.orientation.z = z;
    goal_publish_msg.pose.orientation.w = w;
    publish_goal.publish(goal_publish_msg);
    lastGoalPosX = x;
    lastGoalPosY = y;
    lastGoalPosZ = z;
    lastGoalPosW = w;
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_navigation");
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("cmd_vel", 1, cmd_vel_callback);
  ros::Subscriber sub2 = nh.subscribe("rostu/ball_coor", 1, ball_coor_callback);
  ros::Subscriber sub3 = nh.subscribe("rostu/dribling", 1, dribling_callback);
  ros::Subscriber sub4 = nh.subscribe("rostu/kicker", 1, kicker_callback);
  ros::Subscriber sub5 = nh.subscribe("amcl_pose", 1, amcl_pose_callback);
  ros::Subscriber sub6 = nh.subscribe("move_base/status", 1, move_base_status_callback);
  ros::Subscriber sub7 = nh.subscribe("/rostu/gs/referee_command", 1, referee_command_callback);
  ros::Publisher nav_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Publisher cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher rostu_vel = nh.advertise<geometry_msgs::Twist>("rostu/cmd_vel", 1);
  ros::Publisher dribling_pub = nh.advertise<rostu_v2::Dribling>("rostu/dribling", 1);
  ros::Publisher kicker_pub = nh.advertise<rostu_v2::Kicker>("rostu/kicker", 1);
  cancel_goal_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
  publish_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

  ros::Rate r(30);

  tf::TransformBroadcaster bll_broadcaster;
  tf::TransformBroadcaster bl_broadcaster;
  tf::TransformBroadcaster bf_broadcaster;

  x = 0.0;
  y = 0.0;
  th = 0.0;

  ros::Time kickoff_pass_time;

  current_time = ros::Time::now();
  last_time = ros::Time::now();
  counter = ros::Time::now();
  waitDelay = ros::Time::now();
  kickoff_pass_time = ros::Time::now();

  double angular_z_vel;

  if( robot_team == "M") {
    gawang_point_x = 1.25;
    gawang_point_y = 3.875;
  }
  else if( robot_team == "C") {
    gawang_point_x = 9.75;
    gawang_point_y = 3.875;
  }

  while (ros::ok()) {
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    if (referee_command == "S") { //Robot Stop Or Dropball
      lastGoalPosX = 0; lastGoalPosY = 0; lastGoalPosZ = 0; lastGoalPosW = 0;
      cancel_goal_pub.publish(cancel_goal_msg);
      kicker_msg.kick_pwm = 0;
      cmdvel.linear.x = 0;
      cmdvel.linear.y = 0;
      cmdvel.angular.z = 0;
      cmd_vel.publish(cmdvel);
      rostu_cmdvel.linear.x = 0;
      rostu_cmdvel.linear.y = 0;
      rostu_cmdvel.angular.z = 0;
      dribling.d1pwm1 = 0;
      dribling.d1pwm2 = 0;
      dribling.d2pwm1 = 0;
      dribling.d2pwm2 = 0;
      startGame = false;
      robot_kickoff = false; robot_freekick = false; robot_goalkick = false; robot_throw_in = false; robot_corner = false; robot_penalty = false;
      enemy_kickoff = false; enemy_freekick = false; enemy_goalkick = false; enemy_throw_in = false; enemy_corner = false; enemy_penalty = false;
      step_back = false;
    }
    else if (referee_command == "N") {
      dribling.d1pwm1 = 0;
      dribling.d1pwm2 = 0;
      dribling.d2pwm1 = 0;
      dribling.d2pwm2 = 0;
      startGame = false;
      robot_kickoff = false; robot_freekick = false; robot_goalkick = false; robot_throw_in = false; robot_corner = false; robot_penalty = false;
      enemy_kickoff = false; enemy_freekick = false; enemy_goalkick = false; enemy_throw_in = false; enemy_corner = false; enemy_penalty = false;
      if (robot_team == "C") {
        publish_goal_pos(3.73551344872, 2.82067728043, 0.257607173491, 0.966249731781);
      }
      else if (robot_team == "M") {
        publish_goal_pos(6.61199522018, 2.70544242859, 0.928864407302, 0.370419914215);
      }
    }
    else if(referee_command == "K") { //Cyan Kickoff
      if (robot_team == "C") {
        publish_goal_pos(5.25402927399, 4.76900291443, -0.711607695391, 0.702577033399);
        robot_kickoff = true;
        enemy_kickoff = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
        kickoff_pass_time = ros::Time::now();
      }
      else if (robot_team == "M") {
        publish_goal_pos(6.61199522018, 2.70544242859, 0.928864407302, 0.370419914215);
        robot_kickoff = false;
        enemy_kickoff = true;
        waitDelayCondition = true;
        waitDelaySec = 8.0;
        waitDelay = ros::Time::now();
        kickoff_pass_time = ros::Time::now();
      }
    }
    else if(referee_command == "k") { //Magenta Kickoff
      if (robot_team == "C") {
        publish_goal_pos(3.73551344872, 2.82067728043, 0.257607173491, 0.966249731781);
        robot_kickoff = false;
        enemy_kickoff = true;
        waitDelayCondition = true;
        waitDelaySec = 8.0;
        waitDelay = ros::Time::now();
        kickoff_pass_time = ros::Time::now();
      }
      else if (robot_team == "M") {
        publish_goal_pos(5.25402927399, 4.76900291443, -0.711607695391, 0.702577033399);
        robot_kickoff = true;
        enemy_kickoff = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
        kickoff_pass_time = ros::Time::now();
      }
    }
    else if(referee_command == "F") { //Cyan Freekick
      if (robot_team == "C") {
        publish_goal_pos(3.73551344872, 2.82067728043, 0.257607173491, 0.966249731781);
        robot_freekick = true;
        enemy_freekick = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
      }
      else if (robot_team == "M") {
        publish_goal_pos(6.61199522018, 2.70544242859, 0.928864407302, 0.370419914215);
        robot_freekick = false;
        enemy_freekick = true;
        waitDelayCondition = true;
        waitDelaySec = 4.0;
        waitDelay = ros::Time::now();
      }
    }
    else if(referee_command == "f") { //Magenta Freekick
      if (robot_team == "C") {
        publish_goal_pos(3.73551344872, 2.82067728043, 0.257607173491, 0.966249731781);
        robot_freekick = false;
        enemy_freekick = true;
        waitDelayCondition = true;
        waitDelaySec = 4.0;
        waitDelay = ros::Time::now();
      }
      else if (robot_team == "M") {
        publish_goal_pos(6.61199522018, 2.70544242859, 0.928864407302, 0.370419914215);
        robot_freekick = true;
        enemy_freekick = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
      }
    }
    else if(referee_command == "G") { //Cyan Goalkick
      if (robot_team == "C") {
        publish_goal_pos(2.31663513184, 3.87645721436, 0.0, 1);
        robot_goalkick = true;
        enemy_goalkick = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
      }
      else if (robot_team == "M") {
        publish_goal_pos(6.61199522018, 2.70544242859, 0.928864407302, 0.370419914215);
        robot_goalkick = false;
        enemy_goalkick = true;
        waitDelayCondition = true;
        waitDelaySec = 4.0;
        waitDelay = ros::Time::now();
      }
    }
    else if(referee_command == "g") { //Magenta Goalkick
      if (robot_team == "C") {
        publish_goal_pos(3.73551344872, 2.82067728043, 0.257607173491, 0.966249731781);
        robot_goalkick = false;
        enemy_goalkick = true;
        waitDelayCondition = true;
        waitDelaySec = 4.0;
        waitDelay = ros::Time::now();
      }
      else if (robot_team == "M") {
        publish_goal_pos(8.20516967773, 3.8259832859, 1, 0.0);
        robot_goalkick = true;
        enemy_goalkick = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
      }
    }
    else if(referee_command == "T") { //Cyan Throw In
      if (robot_team == "C") {
        publish_goal_pos(3.73551344872, 2.82067728043, 0.257607173491, 0.966249731781);
        robot_throw_in = true;
        enemy_throw_in = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
      }
      else if (robot_team == "M") {
        publish_goal_pos(6.61199522018, 2.70544242859, 0.928864407302, 0.370419914215);
        robot_throw_in = false;
        enemy_throw_in = true;
        waitDelayCondition = true;
        waitDelaySec = 4.0;
        waitDelay = ros::Time::now();
      }
    }
    else if(referee_command == "t") { //Magenta Throw In
      if (robot_team == "C") {
        publish_goal_pos(3.73551344872, 2.82067728043, 0.257607173491, 0.966249731781);
        robot_throw_in = false;
        enemy_throw_in = true;
        waitDelayCondition = true;
        waitDelaySec = 4.0;
        waitDelay = ros::Time::now();
      }
      else if (robot_team == "M") {
        publish_goal_pos(6.61199522018, 2.70544242859, 0.928864407302, 0.370419914215);
        robot_throw_in = true;
        enemy_throw_in = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
      }
    }
    else if(referee_command == "C") { //Cyan Corner
      if (robot_team == "C") {
        publish_goal_pos(9.77654838562, 6.95783042908, 0.909825772742, -0.414990437545);
        robot_corner = true;
        enemy_corner = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
      }
      else if (robot_team == "M") {
        publish_goal_pos(6.61199522018, 2.70544242859, 0.928864407302, 0.370419914215);
        robot_corner = false;
        enemy_corner = true;
        waitDelayCondition = true;
        waitDelaySec = 4.0;
        waitDelay = ros::Time::now();
      }
    }
    else if(referee_command == "c") { //Magenta Corner
      if (robot_team == "C") {
        publish_goal_pos(3.73551344872, 2.82067728043, 0.257607173491, 0.966249731781);
        robot_corner = false;
        enemy_corner = true;
        waitDelayCondition = true;
        waitDelaySec = 4.0;
        waitDelay = ros::Time::now();
      }
      else if (robot_team == "M") {
        publish_goal_pos(0.629880428314, 0.816191673279, 0.310069670694, 0.950713836712);
        robot_corner = true;
        enemy_corner = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
      }
    }
    else if(referee_command == "P") { //Cyan Penalty
      if (robot_team == "C") {
        robot_penalty = true;
        enemy_penalty = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
      }
      else if (robot_team == "M") {
        robot_penalty = false;
        enemy_penalty = true;
        waitDelayCondition = true;
        waitDelaySec = 4.0;
        waitDelay = ros::Time::now();
      }
    }
    else if(referee_command == "p") { //Magenta Penalty
      if (robot_team == "C") {
        robot_penalty = false;
        enemy_penalty = true;
        waitDelayCondition = true;
        waitDelaySec = 4.0;
        waitDelay = ros::Time::now();
      }
      else if (robot_team == "M") {
        robot_penalty = true;
        enemy_penalty = false;
        waitDelayCondition = false;
        waitDelaySec = 0.0;
        waitDelay = ros::Time::now();
      }
    }
    else if(referee_command == "s") { //Robot Start

      if (!waitDelayCondition) {
        startGame = true;
      }
      else if(waitDelayCondition) {
        if (enemy_kickoff) {
          if (robot_team == "C") {
            publish_goal_pos(4.47390794754, 3.27618312836, 0.256563103513, 0.966527482235);
          }
          else if (robot_team == "M") {
            publish_goal_pos(6.026092032, 3.27618312836, 0.966527482235, 0.256563103513);
          }
          enemy_kickoff = false;
        }

        if ((current_time - waitDelay).toSec() > waitDelaySec) {
          waitDelayCondition = false;
          startGame = true;
        }
      }

      if ((current_time - kickoff_pass_time).toSec() > 2) {
        robot_kickoff = false;
      }

      if (startGame) {
        if (got_ball && robot_status != 1) {
          if (robot_kickoff) {
            dribling.d1pwm1 = 0;
            dribling.d1pwm2 = 255;
            dribling.d2pwm1 = 0;
            dribling.d2pwm2 = 255;
            dribling_pub.publish(dribling);
          }
          // else if (robot_team == "M" && robot_center_pose_x <= 3) {
          //   publish_goal_pos(4.56785202026, 1.98737204075, 0.97437167892, 0.224944062643);
          //   step_back = true;
          //   delayUniversal = ros::Time::now();
          // }
          // else if (robot_team == "C" && robot_center_pose_x >= 7.5) {
          //   publish_goal_pos(6.13059663773, 2.24405288696, 0.1711920153, 0.985237683961);
          //   step_back = true;
          //   delayUniversal = ros::Time::now();
          // }
          else if (robot_angle_from_goal != 0) {
            double output = PD_Controller(robot_angle_from_goal);
            if (output > 2) {
              output = 2;
            }

            if (output < -2) {
              output = -2;
            }

            if (output > 0 ) {
              cmdvel.angular.z = output;
            }
            else if (output < 0) {
              cmdvel.angular.z = output;
            }
            else {
              cmdvel.angular.z = 0;
            }
            dribling.d1pwm1 = 200;
            dribling.d1pwm2 = 0;
            dribling.d2pwm1 = 200;
            dribling.d2pwm2 = 0;
          }
          else {
            if (kicker_volt > 250) {
              kicker_msg.kick_pwm = 255;
            }
            cmdvel.angular.z = 0;
            dribling.d1pwm1 = 0;
            dribling.d1pwm2 = 0;
            dribling.d2pwm1 = 0;
            dribling.d2pwm2 = 0;
          }
          cmdvel.linear.x = 0.0;
          cmd_vel.publish(cmdvel);
        }
        else if (ball_detect && robot_status != 1) {
          lastGoalPosX = 0; lastGoalPosY = 0; lastGoalPosZ = 0; lastGoalPosW = 0;
          double output = PD_Controller(ball_angle_from_frame);

          if (output > 0 ) {
            angular_z_vel = PD_Value * -1;
          }
          else if (output < 0) {
            angular_z_vel = PD_Value * -1;
          }
          else {
            angular_z_vel = 0;
          }

          if (angular_z_vel > 3) {
            angular_z_vel = 3;
          }

          if (angular_z_vel < -3) {
            angular_z_vel = -3;
          }

          if (ball_angle_from_frame < 45 && ball_angle_from_frame > -45) {
            if (ball_distance_from_frame > 100) {
              cmdvel.linear.x = 1.5;
            }
            else if (ball_distance_from_frame <= 100 && ball_distance_from_frame > 25) {
              cmdvel.linear.x = 1.25;
            }
            else if (ball_distance_from_frame <= 25) {
              if (ball_angle_from_frame < 20 && ball_angle_from_frame > -20) {
                cmdvel.linear.x = 0.5;
              }
            }
          }
          else {
    	       cmdvel.linear.x = 0;
          }

          cmdvel.angular.z = angular_z_vel;
          cmd_vel.publish(cmdvel);
        }
        else if (!step_back) {
          dribling.d1pwm1 = 0;
          dribling.d1pwm2 = 0;
          dribling.d2pwm1 = 0;
          dribling.d2pwm2 = 0;
        }

        // if ((current_time - delayUniversal).toSec() > 1) {
        //   if (robot_status == 3) {
        //     step_back = false;
        //   }
        // }
        //
        // if (step_back) {
        //   dribling.d1pwm1 = 200;
        //   dribling.d1pwm2 = 0;
        //   dribling.d2pwm1 = 200;
        //   dribling.d2pwm2 = 0;
        //   dribling_pub.publish(dribling);
        // }
      }
    }

    /*if (ball_detect) {
      double output = PD_Controller();

      if (output > 0 ) {
        angular_z_vel = PD_Value * -1;
      }
      else if (output < 0) {
        angular_z_vel = PD_Value * -1;
      }
      else {
        angular_z_vel = 0;
      }

      if (angular_z_vel > 3) {
        angular_z_vel = 3;
      }

      if (angular_z_vel < -3) {
        angular_z_vel = -3;
      }

      if (ball_angle_from_frame < 45 && ball_angle_from_frame > -45) {
        if (ball_distance_from_frame > 100) {
          cmdvel.linear.x = 2.0;
        }
        else if (ball_distance_from_frame <= 100 && ball_distance_from_frame > 50) {
          cmdvel.linear.x = 1.0;
        }
        else if (ball_distance_from_frame <= 50 && ball_distance_from_frame > 20) {
          if (ball_angle_from_frame < 20 && ball_angle_from_frame > -20) {
            cmdvel.linear.x = 0.5;
          }
          else {
            cmdvel.linear.x = 0.0;
          }
        }
        else if (ball_distance_from_frame <= 20) {
          cmdvel.linear.x = 0.0;
        }
      }

      cmdvel.angular.z = angular_z_vel;
      cmd_vel.publish(cmdvel);
    }*/

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

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

    bf_trans.transform.translation.x = x;
    bf_trans.transform.translation.y = y;
    bf_trans.transform.translation.z = 0.0;
    bf_trans.transform.rotation = odom_quat;
    bf_broadcaster.sendTransform(bf_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    rostu_vel.publish(rostu_cmdvel);
    dribling_pub.publish(dribling);
    kicker_pub.publish(kicker_msg);

    kicker_msg.kick_pwm = 0;
    rostu_cmdvel.linear.x = 0;
    rostu_cmdvel.linear.y = 0;
    rostu_cmdvel.linear.z = 0;
    rostu_cmdvel.angular.x = 0;
    rostu_cmdvel.angular.y = 0;
    rostu_cmdvel.angular.z = 0;

    last_time = current_time;
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

