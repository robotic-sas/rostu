#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {
  VideoCapture cap(1);

  if(!cap.isOpened())
      return -1;

  cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);

  ros::init(argc, argv, "rostu_camera_pub");
  ros::NodeHandle nh;
  ros::Rate r(30);
  image_transport::ImageTransport it(nh);
  image_transport::Publisher camPub = it.advertise("rostu/camera/image", 1);

  Mat frame;

  while (ros::ok()) {
    cap >> frame;

    Rect myROI(100 , 0 / 2, 600, 600);
    frame = frame(myROI);

    sensor_msgs::ImagePtr camMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    if ((char)27 == (char)waitKey(1)) ros::shutdown();

    camPub.publish(camMsg);

    //imshow("frame", frame);

    ros::spinOnce();
    r.sleep();
  }

  destroyAllWindows();
  return 0;
}
