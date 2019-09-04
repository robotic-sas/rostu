void motor1(float x) {
  if (forwardM1) {
    analogWrite(M1PWM1, x);
    analogWrite(M1PWM2, 0);
  }
  else {
    analogWrite(M1PWM1, 0);
    analogWrite(M1PWM2, x);
  }
}

void motor2(float x) {
  if (forwardM2) {
    analogWrite(M2PWM1, x);
    analogWrite(M2PWM2, 0);
  }
  else {
    analogWrite(M2PWM1, 0);
    analogWrite(M2PWM2, x);
  }

}

void motor3(float x) {
  if (forwardM3) {
    analogWrite(M3PWM1, x);
    analogWrite(M3PWM2, 0);
  }
  else {
    analogWrite(M3PWM1, 0);
    analogWrite(M3PWM2, x);
  }
Serial.println(forwardM3);
}
