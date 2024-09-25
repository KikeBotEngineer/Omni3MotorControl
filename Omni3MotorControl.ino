//  Código para controlar los motores de un robot
//  omnidireccional de tres ruedas a través de un 
//  microcontrolador ESP32 WROOM 
#include "ESP32Encoder.h"
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

unsigned long lastTime, sampleTime = 100;//50

String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 3;
float data[dataLength];

const int ENA1 = 25;
const int INA1 = 26;
const int INB1 = 27;
int outValueB = 0;
double wB = 0;
double wRefB = 0;

const int ENA2 = 13;
const int INA2 = 14;
const int INB2 = 12;
int outValueL = 0;
double wL = 0;
double wRefL = 0;

const int ENA3 = 23;
const int INA3 = 22;
const int INB3 = 21;
int outValueR = 0;
double wR = 0;
double wRefR = 0;

ESP32Encoder encoderB, encoderL, encoderR;
const double constValue = 1.5866;

double vfRobot  = 0;
double vlRobot  = 0;
double wRobot  = 0;
double phi = 0;
double R = 0.04;
double L = 0.12;

double kp1 = 1.2;
double ki1 = 25.0;
double kd1 = 0.025;

double kp2 = 1.2;
double ki2 = 25.0;
double kd2 = 0.025;

double kp3 = 1.2;
double ki3 = 25.0;
double kd3 = 0.025;

double integralTerm1 = 0.0;
double lastError1 = 0.0;
double integralTerm2 = 0.0;
double lastError2 = 0.0;
double integralTerm3 = 0.0;
double lastError3 = 0.0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ROCKY");

  pinMode(ENA1, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);

  pinMode(ENA2, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);

  pinMode(ENA3, OUTPUT);
  pinMode(INA3, OUTPUT);
  pinMode(INB3, OUTPUT);

  encoderB.attachFullQuad(36, 39);
  encoderL.attachFullQuad(34, 35);
  encoderR.attachFullQuad(16, 17);

  lastTime = millis();
}

void loop() {
  if (SerialBT.available()) {
    char incomingChar = SerialBT.read();
    inputString += incomingChar;
    if (incomingChar == '\n') {
      stringComplete = true;
    }
  }
  if (stringComplete) {
    for (int i = 0; i < dataLength; i++) {
      int index = inputString.indexOf(separator);
      data[i] = inputString.substring(0, index).toFloat();
      inputString = inputString.substring(index + 1);
    }
    velocityMotor(data[0], data[1], data[2]);
    inputString = "";
    stringComplete = false;
  }

  if (millis() - lastTime >= sampleTime) {
    wB = constValue * encoderB.getCount() / (millis() - lastTime);
    wL = constValue * encoderL.getCount() / (millis() - lastTime);
    wR = constValue * encoderR.getCount() / (millis() - lastTime);

    lastTime = millis();
    encoderB.clearCount();
    encoderL.clearCount();
    encoderR.clearCount();

    velocityRobot(wB, wL, wR);

    phi = phi + wRobot * 0.1;

    SerialBT.print(vfRobot);
    SerialBT.print(",");
    SerialBT.print(vlRobot);
    SerialBT.print(",");
    SerialBT.println(wRobot);

    double error1 = wRefB - wB;
    integralTerm1 += error1 * sampleTime / 1000.0;
    double derivativeTerm1 = (error1 - lastError1) / (sampleTime / 1000.0);

    outValueB = kp1 * error1 + ki1 * integralTerm1 + kd1 * derivativeTerm1;
    outValueB = constrain(outValueB, -255, 255);

    double error2 = wRefL - wL;
    integralTerm2 += error2 * sampleTime / 1000.0;
    double derivativeTerm2 = (error2 - lastError2) / (sampleTime / 1000.0);

    outValueL = kp2 * error2 + ki2 * integralTerm2 + kd2 * derivativeTerm2;
    outValueL = constrain(outValueL, -255, 255);

    double error3 = wRefR - wR;
    integralTerm3 += error3 * sampleTime / 1000.0;
    double derivativeTerm3 = (error3 - lastError3) / (sampleTime / 1000.0);

    outValueR = kp3 * error3 + ki3 * integralTerm3 + kd3 * derivativeTerm3;
    outValueR = constrain(outValueR, -255, 255);

    if (outValueB > 0) clockwiseB(abs(outValueB)); else anticlockwiseB(abs(outValueB));
    if (outValueL > 0) clockwiseL(abs(outValueL)); else anticlockwiseL(abs(outValueL));
    if (outValueR > 0) clockwiseR(abs(outValueR)); else anticlockwiseR(abs(outValueR));

    lastError1 = error1;
    lastError2 = error2;
    lastError3 = error3;
  }
}

void clockwiseB(int pwm) {
  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, LOW);
  analogWrite(ENA1, pwm);
}

void clockwiseL(int pwm) {
  digitalWrite(INA2, HIGH);
  digitalWrite(INB2, LOW);
  analogWrite(ENA2, pwm);
}

void clockwiseR(int pwm) {
  digitalWrite(INA3, HIGH);
  digitalWrite(INB3, LOW);
  analogWrite(ENA3, pwm);
}

void anticlockwiseB(int pwm) {
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, HIGH);
  analogWrite(ENA1, pwm);
}

void anticlockwiseL(int pwm) {
  digitalWrite(INA2, LOW);
  digitalWrite(INB2, HIGH);
  analogWrite(ENA2, pwm);
}

void anticlockwiseR(int pwm) {
  digitalWrite(INA3, LOW);
  digitalWrite(INB3, HIGH);
  analogWrite(ENA3, pwm);
}

void velocityRobot(double w1, double w2, double w3) {
  vfRobot = (0.5774 * R) * (w2 - w3);
  vlRobot = -(R / 3) * (w2 - (2 * w1) + w3);
  wRobot = -(R / (3 * L)) * (w1 + w2 + w3);
}

void velocityMotor(double vf, double vl, double w) {
  wRefB = (vl - (L * w)) / R;
  wRefL = -(vl + (2 * L * w) - 1.7321 * vf) / (2 * R);
  wRefR = -(vl + (2 * L * w) + 1.7321 * vf) / (2 * R);
}
