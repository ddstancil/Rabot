#include "VarSpeedServo.h"
#include "pitches.h"

#define TRIGGER_PIN  2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     4  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // max distance in cm for ultrasonic sensor
#define LED    7  // Arduino pin driving the red LED
#define LEDGreen  6 // Arduino pin driving the green LED
#define LEDBlue 5 // Arduino pin driving the blue LED
#define SPKR    8  // Arduino pin driving the speaker
#define EN12  11 // enable for first half-bridge in the motor driver
#define EN34  3 // enable for second half-bridge in the motor driver
#define M4  13 // half-bridge input 4A (left motor)
#define M3  12 // half-bridge input 3A (left motor)
#define M2  A2 // half-bridge input 2A (right motor)
#define M1  A1 // half-bridge input 1A (right motor)


// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

int fadeValueG = 0;
int fadeValueB = 0;
int nodValueLast = 0;
int nod = -50;
int flag = 0;
int waketime = 15000;
int sleeptime = 30000;
int dittime = 100;
int steptime = 75;

float theta = 0;
float dtheta = 0.05;
float pi = 3.1415926;
float distance,distance1,distance2;
float maxrange = 30;

VarSpeedServo myservoAZ;  // define azimuth servo
VarSpeedServo myservoEL;  // define elevation servo

int posAZ = 0;    // variable to store the servo position
int posEL = 0;

int i, n, nmin;
long counter = 0;
unsigned long starttime, now;

int azbias = -10;

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(LEDGreen, OUTPUT);
  pinMode(LEDBlue, OUTPUT);
  pinMode(SPKR, OUTPUT);
  pinMode(EN12, OUTPUT);
  pinMode(EN34, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);

  pinMode(TRIGGER_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);

  myservoEL.attach(10, 700, 2500); // attaches the servo on pin 10 to the servo object
  myservoAZ.attach(9, 600, 2400); // attaches the servo on pin 9 to the servo object

  //for debugging
  pinMode(0, OUTPUT);
  Serial.begin(9600);

  digitalWrite(LED, HIGH);
  digitalWrite(LEDBlue, HIGH);
  digitalWrite(LEDGreen, HIGH);
  myservoEL.slowmove(100, 40);
  myservoAZ.slowmove(90 + azbias, 40);
  starttime = millis();
  uhoh();

}


void loop() {
  myservoEL.write(90,40,true);
  myservoEL.write(90 - nod, 7);
  digitalWrite(LEDGreen, HIGH);
  digitalWrite(LED,HIGH);
  for (theta=0;theta<pi/2;theta=theta+dtheta){
    fadeValueB = (255.) * sin(theta) * sin(theta);
    analogWrite(LEDBlue, fadeValueB);
    delay(steptime);
    //Serial.println(theta);
  }
  delay(500);
  myservoEL.write(90,7);
  for (theta=pi/2;theta<pi;theta = theta+dtheta){
    fadeValueB = (255.) * sin(theta) * sin(theta);
    analogWrite(LEDBlue, fadeValueB);
    tone(SPKR, NOTE_C1, steptime);
    delay(steptime);
  }
  delay(500);
  /*Serial.println(fadeValueB);
  //Serial.println(fadeValueLast);
  //delay(5000);
  if (fadeValueB<2){
    myservoEL.write(100,5);
  }
  if (fadeValueB>254){
    myservoEL.write(100-nod,5);
  }
  /*nod = fadeValueB * 50 / 255;
  myservoEL.write(100 + nod, 20,true);*/
  /*theta = theta + dtheta;*/
  /*Serial.print("nodValueLast = ");
  Serial.println(nodValueLast);
  Serial.print("nod = ");
  Serial.println(nod);
  if ((fadeValueB < 200) && (nod < nodValueLast)) {
    tone(SPKR, NOTE_C1, 200);
    //Serial.println(fadeValueB);
    //Serial.println(fadeValueLast);
    //delay(5000);
    //delay(100);
  }
  delay(100);
  nodValueLast = nod;*/
  now = millis();
  Serial.println(now-starttime);
  if ((now - starttime) > sleeptime) {
    starttime = millis();
    wake();
    for (int i=1;i<waketime/200;i++){
      distance1 = getDistance();
      distance2 = getDistance();
      distance = (distance1+distance2)/2;
      Serial.println(distance);
      if ((distance>0)&&(distance<maxrange)){
        sayhi();
      }
      delay(200);
     }
   }
    //delay(waketime);
    //starttime = millis();
    nodValueLast = 0;
    theta = 0;
  }


void wake() {
  myservoEL.slowmove(90, 40);
  digitalWrite(LEDBlue, HIGH);
  digitalWrite(LEDGreen, LOW);
  huh();
  myservoAZ.slowmove(45 + azbias, 40);
  delay(1000);
  distance1 = getDistance();
  distance2 = getDistance();
  distance = (distance1+distance2)/2;
  Serial.println(distance);
  if ((distance>0)&&(distance<maxrange)){
    sayhi();
  }
  myservoAZ.slowmove(90 + azbias, 40);
  delay(1000);
  distance1 = getDistance();
  distance2 = getDistance();
  distance = (distance1+distance2)/2;
  Serial.println(distance);
  if ((distance>0)&&(distance<maxrange)){
    sayhi();
  }
  myservoAZ.slowmove(135 + azbias, 40);
  delay(1000);
  distance1 = getDistance();
  distance2 = getDistance();
  distance = (distance1+distance2)/2;
  Serial.println(distance);
  if ((distance>0)&&(distance<maxrange)){
    sayhi();
  }
  myservoAZ.slowmove(90 + azbias, 40);
  delay(1000);
  distance1 = getDistance();
  distance2 = getDistance();
  distance = (distance1+distance2)/2;
  Serial.println(distance);
  if ((distance>0)&&(distance<maxrange)){
    sayhi();
  }
  //delay(1000);
}

void uhoh() {
  tone(SPKR, NOTE_F6, 125);
  digitalWrite(LED, LOW);
  delay(125);
  digitalWrite(LED, HIGH);
  delay(150);
  tone(SPKR, NOTE_CS6, 125);
  digitalWrite(LED, LOW);
  delay(125);
  digitalWrite(LED, HIGH);
  delay(50);
}

void huh() {
  tone(SPKR, NOTE_CS6, 125);
  digitalWrite(LED, LOW);
  delay(125);
  digitalWrite(LED, HIGH);
  delay(150);
  tone(SPKR, NOTE_F6, 125);
  digitalWrite(LED, LOW);
  delay(125);
  digitalWrite(LED, HIGH);
  delay(50);
}
void sayhi(){
  tone(SPKR, NOTE_C7, 50);
  delay(dittime);
  tone(SPKR, NOTE_C7, 50);
  delay(dittime);
  tone(SPKR, NOTE_C7, 50);
  delay(dittime);
  tone(SPKR, NOTE_C7, 50);
  delay(dittime);
  delay(3*dittime);
  tone(SPKR, NOTE_C7, 50);
  delay(dittime);
  tone(SPKR, NOTE_C7, 50);
  delay(3*dittime);
}
float getDistance() 
{
  unsigned long pulseWidth = 0;
  digitalWrite(TRIGGER_PIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN,LOW);
  pulseWidth = pulseIn(ECHO_PIN, HIGH, 50000);
  float range_cm = (((float)pulseWidth)/58.2);
  //Serial.println(pulseWidth);
  //Serial.println(range_cm);
  //if (range_cm>100) range_cm = 0; //not accurate over 50cm
  
  return range_cm;
}
