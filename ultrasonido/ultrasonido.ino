#include <PID_v1.h>

#define PIN_OUTPUT A3

const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor
long duration, inches, cm;
unsigned long t1, t2;
double Speed;
unsigned long timeold = millis();
int suma, i = 0;
int distance2 = 0;


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(9600); // Starting Serial Terminal
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //initialize the variables we're linked to
  Input = sonicRead();
  Setpoint = 10;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}


int sonicRead()
{
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);
  return cm;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void loop()
{

  // put your main code here, to run repeatedly:
  Input = sonicRead();
  myPID.Compute();
  Serial.println(Input);
  analogWrite(PIN_OUTPUT, Output);
//  suma += distance;
//  i ++;
//  if ((millis() - timeold) > 50)
//  {
//    distance = suma / i;
//    Speed = (distance2 - distance) * 1.0 / (millis() - timeold); // force float division result = cm/s
//    Serial.println(Speed);
//    int distance2 = distance;
//    timeold = millis();
//    suma, i = 0;
//  }


}
