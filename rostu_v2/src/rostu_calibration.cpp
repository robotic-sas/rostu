#include <iostream>
#include <fstream>]

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"

using namespace cv;
using namespace std;

string path = ros::package::getPath("rostu_v2");
YAML::Node calibration_data = YAML::LoadFile(path + "/cfg/rostu/rostu_vision.yaml");
string calib = "";

int x_start = 0;
int y_start = 0;
int x_end = 0;
int y_end = 0;

bool sampling = false;
bool getROI = true;
bool saveROI = false;
bool Undistort = false;

int Lower[3][3] = {
  {calibration_data["ball_h_low"].as<int>(), calibration_data["ball_s_low"].as<int>(), calibration_data["ball_v_low"].as<int>()},
  {calibration_data["field_h_low"].as<int>(), calibration_data["field_s_low"].as<int>(), calibration_data["field_v_low"].as<int>()},
  {calibration_data["line_h_low"].as<int>(), calibration_data["line_s_low"].as<int>(), calibration_data["line_v_low"].as<int>()}
};

int Upper[3][3] = {
  {calibration_data["ball_h_up"].as<int>(), calibration_data["ball_s_up"].as<int>(), calibration_data["ball_v_up"].as<int>()},
  {calibration_data["field_h_up"].as<int>(), calibration_data["field_s_up"].as<int>(), calibration_data["field_v_up"].as<int>()},
  {calibration_data["line_h_up"].as<int>(), calibration_data["line_s_up"].as<int>(), calibration_data["line_v_up"].as<int>()}
};

double lower[3], upper[3];
int intLow[3], intUp[3];

static void leftClick( int event, int x, int y, int, void* ) {
  if (event == EVENT_LBUTTONDOWN) {
    x_start = x;
    y_start = y;
    x_end = x;
    y_end = y;
    sampling = true;
    saveROI = false;
    getROI = false;
  }
  else if (event == EVENT_MOUSEMOVE) {
    if (sampling) {
      x_end = x;
      y_end = y;
    }
  }
  else if (event == EVENT_LBUTTONUP) {
    x_end = x;
    y_end = y;
    if (x_start > x_end) {
      int x_temp = x_start;
      x_start = x_end;
      x_end = x_temp;
    }
    if (y_start > y_end) {
      int y_temp = y_start;
      y_start = y_end;
      y_end = y_temp;
    }

    if (x_start == x_end || y_start == y_end) saveROI = false;
    else saveROI = true;
    sampling = false;
  }
}

void on_trackbar(int, void*) {
  ofstream fo(path + "/cfg/rostu/rostu_vision.yaml");
  if (!fo.is_open()) {
      cout << "unable to save calibration data to " << path << endl;
  }
  else {
    if (calib == "ball") {
      calibration_data["ball_h_low"] = intLow[0]; calibration_data["ball_h_up"] = intUp[0];
      calibration_data["ball_s_low"] = intLow[1]; calibration_data["ball_s_up"] = intUp[1];
      calibration_data["ball_v_low"] = intLow[2]; calibration_data["ball_v_up"] = intUp[2];
    }
    else if (calib == "field") {
    calibration_data["field_h_low"] = intLow[0]; calibration_data["field_h_up"] = intUp[0];
    calibration_data["field_s_low"] = intLow[1]; calibration_data["field_s_up"] = intUp[1];
    calibration_data["field_v_low"] = intLow[2]; calibration_data["field_v_up"] = intUp[2];
    }
    else if (calib == "line") {
      calibration_data["line_h_low"] = intLow[0]; calibration_data["line_h_up"] = intUp[0];
      calibration_data["line_s_low"] = intLow[1]; calibration_data["line_s_up"] = intUp[1];
      calibration_data["line_v_low"] = intLow[2]; calibration_data["line_v_up"] = intUp[2];
    }

    fo << calibration_data;
    fo.close();
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2 || argc > 3) {
      cout << "Usage : " << endl;
      cout << " first arg ball     For Ball Calibration" << endl <<
              " first arg field    For Field Calibration" << endl <<
              " first arg line     For Line Calibration" << endl <<
              " second arg -u      For Enabling Undistort Mode" << endl;
      return 0;
  }

  VideoCapture cap(0);

  if(!cap.isOpened())
      return -1;

  cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);

  Mat kernel = Mat::ones(1, 1, CV_8UC1);

  Mat frame, mask, hsv;

  double K[3][3] = {{1.3543423650372035e+03 , 3.0622404481811563e+00, 2.8313230176743360e+02},
              {0, 1.3530679383107395e+03, 2.7597773946639654e+02},
              {0, 0, 1}};
  double D[4] = {-6.8260594840136530e+00, 6.3615894572318972e+01, 1.6242358740954328e-02, 2.3056383278755574e-02};
  double XI = 4.0106204115209874e+00;
  double KNEW[3][3] = {{600/12, 0, 600/2},
                      {0, 600/12, 600/2},
                      {0, 0, 1}};

  Mat knew = Mat(3, 3, CV_64F, KNEW);
  Mat k = Mat(3, 3, CV_64F, K);
  Mat d = Mat(1, 4, CV_64F, D);
  Mat xi = Mat(1, 1, CV_64F, XI);

  if (strcmp("ball", argv[1]) != 0 &&
      strcmp("field", argv[1]) != 0 &&
      strcmp("line", argv[1]) != 0) {
      cout << "Usage : " << endl;
      cout << " first arg ball     For Ball Calibration" << endl <<
              " first arg field    For Field Calibration" << endl <<
              " first arg line     For Line Calibration" << endl <<
              " second arg -u      For Enabling Undistort Mode" << endl;
    return 0;
  }
  else if (strcmp("ball", argv[1]) == 0) {
    calib = "ball";
    intLow[0] = Lower[0][0]; lower[0] = double(intLow[0]); intUp[0] = Upper[0][0]; upper[0] = double(intUp[0]);
    intLow[1] = Lower[0][1]; lower[1] = double(intLow[1]); intUp[1] = Upper[0][1]; upper[1] = double(intUp[1]);
    intLow[2] = Lower[0][2]; lower[2] = double(intLow[2]); intUp[2] = Upper[0][2]; upper[2] = double(intUp[2]);
  }
  else if (strcmp("field", argv[1]) == 0) {
    calib = "field";
    intLow[0] = Lower[1][0]; lower[0] = double(intLow[0]); intUp[0] = Upper[1][0]; upper[0] = double(intUp[0]);
    intLow[1] = Lower[1][1]; lower[1] = double(intLow[1]); intUp[1] = Upper[1][1]; upper[1] = double(intUp[1]);
    intLow[2] = Lower[1][2]; lower[2] = double(intLow[2]); intUp[2] = Upper[1][2]; upper[2] = double(intUp[2]);
  }
  else if (strcmp("line", argv[1]) == 0) {
    calib = "line";
    intLow[0] = Lower[2][0]; lower[0] = double(intLow[0]); intUp[0] = Upper[2][0]; upper[0] = double(intUp[0]);
    intLow[1] = Lower[2][1]; lower[1] = double(intLow[1]); intUp[1] = Upper[2][1]; upper[1] = double(intUp[1]);
    intLow[2] = Lower[2][2]; lower[2] = double(intLow[2]); intUp[2] = Upper[2][2]; upper[2] = double(intUp[2]);
  }

  if (argc > 2) {
    if (strcmp("-u=1", argv[2]) != 0 && strcmp("-u=0", argv[2]) != 0) {
      cout << "Usage : " << endl;
      cout << " first arg ball     For Ball Calibration" << endl <<
              " first arg field    For Field Calibration" << endl <<
              " first arg line     For Line Calibration" << endl <<
              " second arg -u      For Enabling Undistort Mode" << endl;
      return 0;
    }
    else if (strcmp("-u=0", argv[2]) == 0) {
      Undistort = false;
    }
    else if (strcmp("-u=1", argv[2]) == 0) {
      cout << "Undistort Camera Enable..." << endl;
      Undistort = true;
    }
  }

  namedWindow("frame");
  namedWindow("mask");
  namedWindow("trackbar", CV_WINDOW_FREERATIO);
  setMouseCallback("frame", leftClick);

  createTrackbar("H Low", "trackbar", &intLow[0], 255, on_trackbar);
  createTrackbar("S Low", "trackbar", &intLow[1], 255, on_trackbar);
  createTrackbar("V Low", "trackbar", &intLow[2], 255, on_trackbar);

  createTrackbar("H Up", "trackbar", &intUp[0], 255, on_trackbar);
  createTrackbar("S Up", "trackbar", &intUp[1], 255, on_trackbar);
  createTrackbar("V Up", "trackbar", &intUp[2], 255, on_trackbar);

  for(;;) {
    cap >> frame;

    Rect myROI(100 , 0 / 2, 600, 600);
    frame = frame(myROI);

    if (Undistort) {
      omnidir::undistortImage(frame, frame, k, d, xi, omnidir::RECTIFY_PERSPECTIVE, knew);
    }

    flip(frame, frame, 1);

    if (!getROI) {
      if (!sampling && !saveROI) {
        imshow("frame", frame);
      }
      else if (sampling && !saveROI) {
        rectangle(frame, Point(x_start, y_start), Point(x_end, y_end), Scalar(0, 255, 0), 2);
        imshow("frame", frame);
      }
      else if (saveROI) {
        rectangle(frame, Point(x_start, y_start), Point(x_end, y_end), Scalar(0, 255, 0), 2);
        imshow("frame", frame);

        Mat roi = frame(Range(y_start, y_end), Range(x_start, x_end));
        Mat hsvRoi, splitHsv[3];
        cvtColor(roi, hsvRoi, COLOR_BGR2HSV);
        split(hsvRoi, splitHsv);
        minMaxLoc(splitHsv[0], &lower[0], &upper[0]);
        minMaxLoc(splitHsv[1], &lower[1], &upper[1]);
        minMaxLoc(splitHsv[2], &lower[2], &upper[2]);

        setTrackbarPos("H Low", "trackbar", (int)lower[0]);
        setTrackbarPos("S Low", "trackbar", (int)lower[1]);
        setTrackbarPos("V Low", "trackbar", (int)lower[2]);

        setTrackbarPos("H Up", "trackbar", (int)upper[0]);
        setTrackbarPos("S Up", "trackbar", (int)upper[1]);
        setTrackbarPos("V Up", "trackbar", (int)upper[2]);

        ofstream fo(path.c_str());
        if (!fo.is_open()) {
            cout << "unable to save calibration data to " << path << endl;
        }
        else {
          fo << lower[0] << "," << lower[1] << "," << lower[2] << ",";
          fo << upper[0] << "," << upper[1] << "," << upper[2] << ";";
          fo.close();
        }
        saveROI = false;
        getROI = true;
      }
    }
    else {
      for (int i = 0; i < 3; i++) {
          lower[i] = (double)intLow[i];
          upper[i] = (double)intUp[i];
      }

      cvtColor(frame, hsv, COLOR_BGR2HSV);
      inRange(hsv, Scalar(lower[0],lower[1],lower[2]), Scalar(upper[0],upper[1],upper[2]), mask);
      erode(mask, mask, 0, Point(-1,-1), 2);
      dilate(mask, mask, 0, Point(-1, -1), 2);
      morphologyEx(mask ,mask, MORPH_CLOSE, kernel);

      //Find Contours
      Mat maskColone = mask.clone();
      vector<vector<Point> > cnts;
      findContours(maskColone, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      //Drawing
      if (cnts.size() > 0) {
        Moments M;
        Point2f center;
        float radius;
        int largest_area = 0;
        int largest_contour_index = 0;

        for (int i = 0; i < cnts.size(); i++) {
          double a = contourArea(cnts[i], false);
          if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;
          }
        }

        minEnclosingCircle(cnts[largest_contour_index], center, radius);
        M = moments(cnts[largest_contour_index]);

        if (radius > 0) {
          circle(frame, center, int(radius), Scalar(0, 255, 0), 2);
        }
      }

      imshow("mask", mask);
      imshow("frame", frame);
    }

    if ((char)27 == (char)waitKey(1)) break;
  }

  destroyAllWindows();
  return 0;
}
