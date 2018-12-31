// This version is for micro servos.

//#include "Servo.h"
#include "VarSpeedServo.h"
#include "pitches.h"
//#include "NewPing.h"

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

int az[] = {180,165,150,135,120,105,90,75,60,45,30,15,05,60,45,30,15,0};
int dx[5], dmin;
int fadeValueG = 0;
int fadeValueB = 0;
int flag = 0;

float theta = 0;
float dtheta = 0.05;
float pi = 3.1415926;
int offset = -10; // -25 more negative makes it turn to the right
//float fadeoffset = 0;
unsigned long time;
int LookAngle = 45;
int LookAngle2 = 45;


VarSpeedServo myservoAZ;  // define azimuth servo
VarSpeedServo myservoEL;  // define elevation servo


int speed = 150; //0-255
//int leftOffset = 90 + (11); //-90 (more reverse) to 90 (more forward)
//int rightOffset = 90 + (11); //-90 (more forward) to 90 (more reverse)
//#define LEFT_ZERO_SPEED (90 + (0)) //-90 (more reverse) to 90 (more forward)
//#define RIGHT_ZERO_SPEED (90 - (0)) //-90 (more forward) to 90 (more reverse)

int posAZ = 0;    // variable to store the servo position
int posEL = 0;

int i,n,nmin;
unsigned long val, valRight, valLeft;
long randNumber;
long counter = 0;
int obstacleCounter = 0;
//int azbias = -12;
int azbias = 0;
int azscan = 0;
//int spkr = 12;
//int led = 11;
unsigned long pulseWidth = 0;
int distance = 50;
int hopduration = 2;
int period = 2000;
int turntime = 650;

void setup() 
{ 
  pinMode(LED,OUTPUT);
  pinMode(LEDGreen,OUTPUT);
  pinMode(LEDBlue, OUTPUT);
  pinMode(SPKR,OUTPUT);
  pinMode(EN12,OUTPUT);
  pinMode(EN34,OUTPUT);
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(M3,OUTPUT);
  pinMode(M4,OUTPUT);
  
  //digitalWrite(EN12,HIGH);
  //digitalWrite(EN34,HIGH);
  
  playSong();
  
  delay(1000); //time to get set up!
  myservoEL.attach(10);  // attaches the servo on pin 10 to the servo object
  myservoAZ.attach(9);  // attaches the servo on pin 9 to the servo object

  pinMode(TRIGGER_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);

  //for debugging
  pinMode(0,OUTPUT);
  Serial.begin(9600);

  randomSeed(analogRead(1));
  
} 

void playSong()
{
  digitalWrite(LEDGreen,HIGH); // HIGH turns off the LED
  digitalWrite(LEDBlue,HIGH);
  digitalWrite(LED,HIGH);
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    if (melody[thisNote]>0)
      digitalWrite(LED,LOW);
    tone(SPKR, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    delay(noteDuration);
    digitalWrite(LED,HIGH);
    int pauseBetweenNotes = noteDuration * 0.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(SPKR);
  }
}

void loop() 
{
  dmin = 1000;

  //Set look direction
  time = millis() % period;
  //Serial.print("Time: ");
  //Serial.println(time);
  //azscan = 35*square(time,1000);
  azscan = LookAngle*tristate(time,period);
  myservoEL.write(100);
  myservoAZ.slowmove(90+azbias+azscan, 40);

  //Check for obstacles over a sweep
  int angles[5];
  float ranges[5];
  for(n=0;n<=4;n++)
  {
    ranges[n]=getDistance();
    angles[n]=myservoAZ.read();
    //Serial.println(ranges[n]);
  }
  int stopObstacleCounter = 0; //obstacles so close we need to stop
  int turnObstacleCounter = 0; //obstacles we may have time to turn for
  float avgTurnObstacleAngle = 0;
  float avgObstacleDistance_cm = 50.0;
  for(n=0;n<=4;n++)
  {
    if(ranges[n]>0 && ranges[n]<25 && abs(angles[n]-90)<25)
    {
      stopObstacleCounter++;
      avgObstacleDistance_cm += ranges[n];
    }
    if(ranges[n]>0 && ranges[n]<50)
    {
      turnObstacleCounter++;
      avgTurnObstacleAngle += (float)angles[n];
      avgObstacleDistance_cm += ranges[n];
    }
  } 
  if(stopObstacleCounter+turnObstacleCounter>0) avgObstacleDistance_cm = avgObstacleDistance_cm/(float)(stopObstacleCounter+turnObstacleCounter);
  if(turnObstacleCounter>0) avgTurnObstacleAngle = avgTurnObstacleAngle/(float)turnObstacleCounter;

  setLEDByDistance(avgObstacleDistance_cm);

  //Drive while avoiding obstacles!
  if(stopObstacleCounter>3) 
  {
    handleStopObstacle();
  }
  else if(turnObstacleCounter>3) //If we see an obstacle far away, try to avoid it
  {
    //float normalizedHeadingFromZero = (1.0-(abs(avgTurnObstacleAngle-90)/90.0));
    //float normalizedDistanceToObstacle = (1.0-(avgObstacleDistance_cm/50.0));
    //float curvature = normalizedHeadingFromZero*normalizedDistanceToObstacle*10;
    if(avgTurnObstacleAngle>90) forwardRight(speed, 5);
    else forwardLeft(speed, 5);
  }
  else
  {
    forward(speed);
  }
}

void setLEDByDistance(float distance_cm)
{
  float gain = (distance_cm-25.0)/25.0;
  if(gain>1.0) gain=1.0;
  if(gain<0.0) gain=0.0;

  int red = 0;
  int green = gain*255.0;
  int blue = 0;

  if(gain<0.5) red=255;

  Serial.print(red);
  Serial.print(",");
  Serial.print(green);
  Serial.print(",");
  Serial.println(blue);

  analogWrite(LED,255-red);
  analogWrite(LEDGreen,255-green);
  analogWrite(LEDBlue,255-blue);
}

// Motion routines for forward, reverse, turns, and stop
void reverse(int localSpeed) {
  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  digitalWrite(M4,HIGH);
  digitalWrite(M3,LOW);
  analogWrite(EN12,localSpeed+offset);
  analogWrite(EN34,localSpeed);
}

void backRight(int localSpeed) {
//  servoLeft.write(LEFT_ZERO_SPEED-(localSpeed/2));
//  servoRight.write(RIGHT_ZERO_SPEED+localSpeed);
  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  digitalWrite(M4,HIGH);
  digitalWrite(M3,LOW);
  analogWrite(EN12,localSpeed+offset);
  analogWrite(EN34,localSpeed/2);
}

void backLeft(int localSpeed) {
//  servoLeft.write(LEFT_ZERO_SPEED-localSpeed);
//  servoRight.write(RIGHT_ZERO_SPEED+(localSpeed/2));
  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  digitalWrite(M4,HIGH);
  digitalWrite(M3,LOW);
  analogWrite(EN12,localSpeed/2+offset);
  analogWrite(EN34,localSpeed);
}

void forward(int localSpeed) {
  //Serial.println("forward");
//  servoLeft.write(LEFT_ZERO_SPEED+localSpeed);
//  servoRight.write(RIGHT_ZERO_SPEED-localSpeed);
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  digitalWrite(M4,LOW);
  digitalWrite(M3,HIGH);
  analogWrite(EN12,localSpeed+offset);
  analogWrite(EN34,localSpeed);
}

void forwardLeft(int localSpeed, float curvature) {
  //Serial.println("forwardLeft");
//  servoLeft.write(LEFT_ZERO_SPEED+localSpeed);
//  servoRight.write(RIGHT_ZERO_SPEED-localSpeed+(localSpeed/5));
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  digitalWrite(M4,LOW);
  digitalWrite(M3,HIGH);
  analogWrite(EN12,localSpeed+offset);
  analogWrite(EN34,(int)((float)localSpeed/curvature));
}

void forwardRight(int localSpeed, float curvature) {
  //Serial.println("forwardRight");
//  servoRight.write(RIGHT_ZERO_SPEED-localSpeed);
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  digitalWrite(M4,LOW);
  digitalWrite(M3,HIGH);
  analogWrite(EN12,(int)((float)localSpeed/curvature)+offset);
  analogWrite(EN34,localSpeed);
}

void turnLeft(int localSpeed) {
  //Serial.println("turnLeft");
//  servoLeft.write(LEFT_ZERO_SPEED-localSpeed);
//  servoRight.write(RIGHT_ZERO_SPEED-localSpeed);
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  digitalWrite(M4,HIGH);
  digitalWrite(M3,LOW);
  analogWrite(EN12,localSpeed+offset);
  analogWrite(EN34,localSpeed);
}
void turnRight(int localSpeed) {
  //Serial.println("turnRight");
//  servoLeft.write(LEFT_ZERO_SPEED+localSpeed);
//  servoRight.write(RIGHT_ZERO_SPEED+localSpeed);
  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  digitalWrite(M4,LOW);
  digitalWrite(M3,HIGH);
  analogWrite(EN12,localSpeed+offset);
  analogWrite(EN34,localSpeed);
}

void stopRobot() {
//  servoLeft.write(LEFT_ZERO_SPEED);
//  servoRight.write(RIGHT_ZERO_SPEED);
  analogWrite(M1,0);
  analogWrite(M2,0);
  analogWrite(M4,0);
  analogWrite(M3,0);
}
unsigned long mydistance() {
        digitalWrite(TRIGGER_PIN,HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN,LOW);
        pulseWidth = pulseIn(ECHO_PIN, HIGH, 50000);
        //Serial.println(pulseWidth);
        if (pulseWidth==0){
          pulseWidth=11000;
        }
        //pulseWidth = digitalRead(ECHO_PIN);
        //Serial.print("pulseWidth = ");
        //Serial.println(pulseWidth);
       // distance = pulseWidth/58.2;
        return pulseWidth/58.2;
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
  if (range_cm>50) range_cm = 0; //not accurate over 50cm
  
  return range_cm;
}

void hop(int localSpeed,int localhoptime){
forward(localSpeed);
delay(localhoptime*1000/3);
forward(2*localSpeed);
delay(localhoptime*1000/3);
forward(localSpeed);
delay(localhoptime*1000/3);
}
int square(int localt, int localT) {
  if ( localt % localT < localT/2) {
    return 1;
  }
  else {
    return -1;
  }
}
int tristate(int localt, int localT) {
/*  if ( localt % localT < localT/3) {
    return 1;
  }
  else if (localt % localT < localT*2/3) {
    return 0;
  }
  else {
    return -1;
  }*/
    if ( localt % localT < localT/4) {
    return 1;
  }
  else if (localt % localT < localT/2) {
    return 0;
  }
  else if (localt % localT < localT*3/4){
    return -1;
  }
  else {
    return 0;
  }
}

void handleStopObstacle()
{
  stopRobot();
  analogWrite(LEDGreen,255);
  analogWrite(LEDBlue,255);
  tone(SPKR,NOTE_F6,125);
  digitalWrite(LED,LOW);
  delay(125);
  digitalWrite(LED,HIGH);
  delay(150);
  tone(SPKR,NOTE_CS6,125);
  digitalWrite(LED,LOW);
  delay(125);
  digitalWrite(LED,HIGH);
  delay(50);
  
  //Determine how to avoid obstacle
  // Begin scan

  myservoAZ.slowmove(90-LookAngle2+azbias, 0);
  delay(500);
  valRight = mydistance();
  delay(50);
  valRight =(valRight + mydistance())/2;

  myservoAZ.slowmove(90+LookAngle2+azbias, 0);
  delay(500);
  valLeft = mydistance();
  delay(50);
  valLeft = (valLeft + mydistance())/2;
  
  myservoAZ.slowmove(90+azbias, 0);
  delay(500);
  val = mydistance();
  delay(50);
  val = (val+mydistance())/2;
  
  Serial.print("valRight = ");
  Serial.println(valRight);
  Serial.print(" val = ");
  Serial.println(val);
  Serial.print(" valLeft = ");
  Serial.println(valLeft);
  
  // reset head direction
  //   myservoAZ.write(90+azbias);
  //myservoEL.write(90);
  //find direction of minimum distance
  
  /*dmin = 100;
  for(n=0;n<=13;n++){
    if(dx[n]<dmin){
      dmin = dx[n];
      nmin = n;
    }*/
 
  //Serial.print(nmin);
  //Serial.println(dmin);
  // choose motion based on object position
  
  if((valLeft<val)&&(valLeft<valRight)){ // object is on left
    Serial.println("object on left");
    //reverse(speed/2);
    //delay(1000);
    turnRight(speed);
    delay(turntime);
  }
  else if((valRight<val)&&(valRight<valLeft)){ // object is on right
    Serial.println("object on right");
    //reverse(speed/2);
    //delay(1000);
    turnLeft(speed);
    delay(turntime);
  }
  else if((val<valRight)&&(val<valLeft)){
      Serial.println("object straight ahead");
    randNumber = random(2);
  
    if(randNumber == 0)
    {
      turnRight(speed);
      delay(turntime);
    }
    else if(randNumber == 1)
    {
      turnLeft(speed);
      delay(turntime);
    }
    /*else if(randNumber == 2)
    {
      backRight(speed/2);
      delay(1000);
    }
    else if(randNumber == 3)
    {
      backLeft(speed/2);
      delay(1000);
    }*/
  }
  else if(valRight<valLeft){ // object is on right
    Serial.println("object on right");
    //reverse(speed/2);
    //delay(1000);
    turnLeft(speed);
    delay(turntime);
  }
  else if(valRight>valLeft){ // object is on left
          Serial.println("object on left");
    //reverse(speed/2);
    //delay(1000);
    turnRight(speed);
    delay(turntime);
  }
  else {
    randNumber = random(2);
  
    if(randNumber == 0)
    {
      turnRight(speed);
      delay(turntime);
    }
    else if(randNumber == 1)
    {
      turnLeft(speed);
      delay(turntime);
    }
  }

  stopRobot();
//   obstacleCounter = 0;
  delay(50);
}


    
  


