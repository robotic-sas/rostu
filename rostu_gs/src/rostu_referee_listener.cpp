#include <iostream>
#include <signal.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <std_msgs/String.h>

using namespace std;

string path = ros::package::getPath("rostu_gs");
YAML::Node host_ip = YAML::LoadFile(path + "/cfg/rostu/host_ip.yaml");

int PORT = 28097;
string HOST = host_ip["host_ip"].as<string>();

int sock = socket(AF_INET, SOCK_STREAM, 0);

void sighandler(int sig) {
  ros::shutdown();
  close(sock);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_gs_referee_listener");
  ros::NodeHandle nh;
  ros::Publisher referee_pub = nh.advertise<std_msgs::String>("/rostu/gs/referee_command", 1);
  std_msgs::String referee_msg;
  ros::Rate r(30);

  if (sock == -1) {
    return 1;
  }

  sockaddr_in hint;
  hint.sin_family = AF_INET;
  hint.sin_port = htons(PORT);
  inet_pton(AF_INET, HOST.c_str(), &hint.sin_addr);

  int connectRes = connect(sock, (sockaddr*)&hint, sizeof(hint));
  if (connectRes == -1) {
    return 1;
  }

  char buf[4096];

  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGINT, &sighandler);

  while(ros::ok()) {
    memset(buf, 0, 4096);
    int bytesReceived = recv(sock, buf, 4096, 0);

    referee_msg.data = string(buf, bytesReceived);
    referee_pub.publish(referee_msg);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
