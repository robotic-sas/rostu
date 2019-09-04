#define USE_TEENSY_HW_SERIAL

#include <PID_v1.h>

#include <Wire.h>
#include <VL53L0X.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <rostu_v2/Dribling.h>

VL53L0X sensor;

#define enc1A 24
#define enc1B 25
#define enc2A 26
#define enc2B 27
#define enc3A 28
#define enc3B 29

#define M1PWM1 5
#define M1PWM2 6
#define M2PWM1 7
#define M2PWM2 8
#define M3PWM1 9
#define M3PWM2 10

#define MD1PWM1 21
#define MD1PWM2 20
#define MD2PWM1 22
#define MD2PWM2 23

class NewHardware : public ArduinoHardware {
  public:
  NewHardware():ArduinoHardware(&Serial1, 57600) {};
};

ros::NodeHandle_<NewHardware> nh;

float holonomicEquation[3][3] {
  { -0.333333, -0.57735, -0.333333},
  { -0.333333, 0.57735, -0.333333},
  { 0.666667, 0, -0.333333}
};

bool forwardM1, forwardM2, forwardM3;

int enc_count1 = 0;
int enc_count2 = 0;
int enc_count3 = 0;

double Setpoint1 = 0, Input1, Output1;
double Kp1 = 0.125, Ki1 = 3, Kd1 = 0.0016;
double Setpoint2 = 0, Input2, Output2;
double Kp2 = 0.125, Ki2 = 3, Kd2 = 0.0016;
double Setpoint3 = 0, Input3, Output3;
double Kp3 = 0.125, Ki3 = 3, Kd3 = 0.0016;

int rpm1;
int rpm2;
int rpm3;

PID m1PID(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID m2PID(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);
PID m3PID(&Input3, &Output3, &Setpoint3, Kp3, Ki3, Kd3, DIRECT);

int multipler[3] = {1, 1, 1};
int d1pwm1 = 0, d1pwm2 = 0, d2pwm1 = 0, d2pwm2 = 0;

void vel_mull_callback(const std_msgs::Int16MultiArray& msg) {
  for (int i = 0; i < 3; i++) {
    multipler[i] = msg.data[i];
  }
}

void cmd_vel_callback(const geometry_msgs::Twist& vel) {
  Setpoint1 = int((holonomicEquation[0][0] * vel.linear.y * multipler[1] * -1) + (holonomicEquation[0][1] * vel.linear.x * multipler[0]) + (holonomicEquation[0][2] * vel.angular.z * multipler[2] * -1));
  Setpoint2 = int((holonomicEquation[1][0] * vel.linear.y * multipler[1] * -1) + (holonomicEquation[1][1] * vel.linear.x * multipler[0]) + (holonomicEquation[1][2] * vel.angular.z * multipler[2] * -1));
  Setpoint3 = int((holonomicEquation[2][0] * vel.linear.y * multipler[1] * -1) + (holonomicEquation[2][1] * vel.linear.x * multipler[0]) + (holonomicEquation[2][2] * vel.angular.z * multipler[2] * -1));

  if (Setpoint1 != 0) {
    if (Setpoint1 < 0) {
      Setpoint1 *= -1;
      forwardM1 = false;
    }
    else {
      forwardM1 = true;
    }
  }

  if (Setpoint2 != 0) {
    if (Setpoint2 < 0) {
      Setpoint2 *= -1;
      forwardM2 = false;
    }
    else {
      forwardM2 = true;
    }
  }

  if (Setpoint3 != 0) {
    if (Setpoint3 < 0) {
      Setpoint3 *= -1;
      forwardM3 = false;
    }
    else {
      forwardM3 = true;
    }
  }
}

void dribling_callback(const rostu_v2::Dribling& msg) {
  d1pwm1 = msg.d1pwm1;
  d1pwm2 = msg.d1pwm2;
  d2pwm1 = msg.d2pwm1;
  d2pwm2 = msg.d2pwm2;
}

ros::Subscriber<std_msgs::Int16MultiArray> sub1("vel_mul", &vel_mull_callback );
ros::Subscriber<geometry_msgs::Twist> sub2("rostu/cmd_vel", &cmd_vel_callback );
ros::Subscriber<rostu_v2::Dribling> sub3("rostu/dribling", &dribling_callback );

rostu_v2::Dribling drib_msg;

ros::Publisher drib_pub("rostu/dribling", &drib_msg);

void enc_a_counter1() {
  enc_count1++;
}

void enc_a_counter2() {
  enc_count2++;
}

void enc_a_counter3() {
  enc_count3++;
}

void setup() {
  Serial1.begin(57600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(5);
  sensor.setMeasurementTimingBudget(1000);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);

  nh.advertise(drib_pub);

  pinMode(M1PWM1, OUTPUT);
  pinMode(M1PWM2, OUTPUT);
  analogWriteFrequency(M1PWM1, 10000);
  analogWriteFrequency(M1PWM2, 10000);
  pinMode(M2PWM1, OUTPUT);
  pinMode(M2PWM2, OUTPUT);
  analogWriteFrequency(M2PWM1, 10000);
  analogWriteFrequency(M2PWM2, 10000);
  pinMode(M3PWM1, OUTPUT);
  pinMode(M3PWM2, OUTPUT);
  analogWriteFrequency(M3PWM1, 10000);
  analogWriteFrequency(M3PWM2, 10000);

  pinMode(MD1PWM1, OUTPUT);
  pinMode(MD1PWM2, OUTPUT);
  analogWriteFrequency(MD1PWM1, 1000);
  analogWriteFrequency(MD1PWM2, 1000);
  pinMode(MD2PWM1, OUTPUT);
  pinMode(MD2PWM2, OUTPUT);
  analogWriteFrequency(MD2PWM1, 1000);
  analogWriteFrequency(MD2PWM2, 1000);

  attachInterrupt(digitalPinToInterrupt(enc1A), enc_a_counter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(enc2A), enc_a_counter2, FALLING);
  attachInterrupt(digitalPinToInterrupt(enc3A), enc_a_counter3, FALLING);

  m1PID.SetSampleTime(5); //200Hz PID Sample
  m1PID.SetMode(AUTOMATIC);
  m1PID.SetOutputLimits(0, 255);
  m2PID.SetSampleTime(5); //200Hz PID Sample
  m2PID.SetMode(AUTOMATIC);
  m2PID.SetOutputLimits(0, 255);
  m3PID.SetSampleTime(5); //200Hz PID Sample
  m3PID.SetMode(AUTOMATIC);
  m3PID.SetOutputLimits(0, 255);
}

unsigned long printTime = millis();
int readTOF, lastReadTOF;

void loop() {
  readTOF = sensor.readRangeSingleMillimeters();
  if (readTOF > 500) {
    readTOF = lastReadTOF;
  }
  lastReadTOF = readTOF;

  findRpm();

  Input1 = rpm1;
  m1PID.Compute();
  Input2 = rpm2;
  m2PID.Compute();
  Input3 = rpm3;
  m3PID.Compute();

  motor1(Output1);
  motor2(Output2);
  motor3(Output3);

  analogWrite(MD1PWM1, d1pwm1);
  analogWrite(MD1PWM2, d1pwm2);
  analogWrite(MD2PWM1, d2pwm1);
  analogWrite(MD2PWM2, d2pwm2);

  if (readTOF < 60) {
    drib_msg.got_ball = true;
    drib_msg.d1pwm1 = d1pwm1;
    drib_msg.d1pwm2 = d1pwm2;
    drib_msg.d2pwm1 = d2pwm1;
    drib_msg.d2pwm2 = d2pwm2;
  }
  else {
    drib_msg.got_ball = false;
    drib_msg.d1pwm1 = d1pwm1;
    drib_msg.d1pwm2 = d1pwm2;
    drib_msg.d2pwm1 = d2pwm1;
    drib_msg.d2pwm2 = d2pwm2;
  }

  if ((millis() - printTime) > 20) {
    drib_pub.publish(&drib_msg);
    printTime = millis();
  }

  //  if ((millis() - printTime) > 50) {
  //    Serial.print("Distance (mm): ");
  //    Serial.print(readTOF);
  //    Serial.print("\t");
  //    Serial.print("RPM 1 : ");
  //    Serial.print(rpm1);
  //    Serial.print("\tRPM 2 : ");
  //    Serial.print(rpm2);
  //    Serial.print("\tRPM 3 : ");
  //    Serial.println(rpm3);
  //
  //    printTime = millis();
  //  }

  nh.spinOnce();
}

unsigned long lastMillis1;
unsigned long lastMillis2;
unsigned long lastMillis3;

void findRpm() {
  if ((unsigned long)millis() - lastMillis1 >= 50) {
    unsigned long pulses;
    detachInterrupt(digitalPinToInterrupt(enc1A));
    pulses = enc_count1;
    enc_count1 = 0;
    attachInterrupt(digitalPinToInterrupt(enc1A), enc_a_counter1, FALLING);
    rpm1 = (pulses * (60000.f / ((unsigned long)millis() - lastMillis1))) / 134;
    lastMillis1 = (unsigned long)millis();
  }

  if ((unsigned long)millis() - lastMillis2 >= 50) {
    unsigned long pulses;
    detachInterrupt(digitalPinToInterrupt(enc2A));
    pulses = enc_count2;
    enc_count2 = 0;
    attachInterrupt(digitalPinToInterrupt(enc2A), enc_a_counter2, FALLING);
    rpm2 = (pulses * (60000.f / ((unsigned long)millis() - lastMillis2))) / 134;
    lastMillis2 = (unsigned long)millis();
  }

  if ((unsigned long)millis() - lastMillis3 >= 50) {
    unsigned long pulses;
    detachInterrupt(digitalPinToInterrupt(enc3A));
    pulses = enc_count3;
    enc_count3 = 0;
    attachInterrupt(digitalPinToInterrupt(enc3A), enc_a_counter3, FALLING);
    rpm3 = (pulses * (60000.f / ((unsigned long)millis() - lastMillis3))) / 134;
    lastMillis3 = (unsigned long)millis();
  }
}
