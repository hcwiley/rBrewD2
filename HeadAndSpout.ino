// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 

// this constant won't change.  It's the pin number
// of the sensor's output:
const int trigger = 2;
const int echo = 4;

const int head_1 = 5;
const int head_2 = 6;

const int relay = 13;

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 

const int max_angle = 120;
const int min_angle = 55;

const int max_motor = 210;
const int min_motor = 195;


int curDir = -1;
int nextDir = 0;
int tick = 0;

void setup() 
{
  // initialize serial communication:
  Serial.begin(9600);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  
  pinMode(head_1, OUTPUT);
  pinMode(head_2, OUTPUT);
  
  headStop();
  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  myservo.write(min_angle);

  pinMode(relay, OUTPUT);

  tick = millis();
} 

void headLeft(){
  digitalWrite(head_1, LOW);
  analogWrite(head_2, random(min_motor,max_motor));
}

void headRight(){
  analogWrite(head_1, random(min_motor,max_motor));
  digitalWrite(head_2, LOW);
}

void headStop(){
  digitalWrite(head_1, LOW);
  digitalWrite(head_2, LOW);
}

void loop() 
{
  if ( shouldPour() ) {
    digitalWrite(relay, LOW);
    myservo.write(max_angle);
  }
  else {
    digitalWrite(relay, HIGH);
    myservo.write(min_angle);
  }
  
  int tock = millis();
  if( tock - tick >= random(1500,5000)){
    Serial.println("tick tock");
    Serial.print("curDir: ");
    Serial.println(curDir);
    Serial.print("nextDir: ");
    Serial.println(nextDir);
    tick = tock;
    // stopped 
    if( curDir == 0 ){
      if( nextDir > 0 ){
        Serial.println("right");
        curDir = 1;
        nextDir = 0;
        headRight();
      }
      else if( nextDir < 0 ){
        Serial.println("left");
        curDir = -1;
        nextDir = 0;
        headLeft();
      }
    }
    // going right
    else if( curDir > 0 ){
      Serial.println("stop right!");
      nextDir = -1;
      curDir = 0;
      headStop();
    }
    // going left
    else if( curDir < 0 ){
      Serial.println("stop left!");
      nextDir = 1;
      curDir = 0;
      headStop();
    }
  }
  delay(50);
}

boolean shouldPour() {
  
  int cm = getDistance();
  delay(25);
  cm += getDistance();
  delay(25);
  cm += getDistance();
  delay(25);
  
  cm /= 3;
  
//  Serial.print(cm);
//  Serial.println();
  
  if( cm < 14 )
    return true;
  else
    return false;
}

int getDistance() {
    // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, inches;
  
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigger, LOW);
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echo, HIGH);
  return microsecondsToCentimeters(duration);
  
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}