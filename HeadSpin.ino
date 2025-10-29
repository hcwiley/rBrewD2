
// ADJUST THIS NUMBER BETWEEN 0-255
int spead = 105; //gets update randomly

int pinI1=10;//define I1 interface
int pinI2=11;//define I2 interface 

int del = 1000;
void setup()
{
  Serial.begin(9600);
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
}
 
void forward()
{
  Serial.println("forward");
  analogWrite(pinI1,spead);
  digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
}
void backward()//
{
  Serial.println("backward");
  analogWrite(pinI2,spead);//turn DC Motor A move clockwise
  digitalWrite(pinI1,LOW);
}

void stop()//
{
  Serial.println("stop");
  digitalWrite(pinI1,LOW);// Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
  digitalWrite(pinI2,LOW);
  del = random(5,20) * 100;
  delay(del);
}

void getRandom(){
  del = random(5,20) * 100;
  spead = random(110, 180);
  Serial.print("delay: ");
  Serial.println(del);
}

void loop()
{
  getRandom();
  forward();
  delay(del);
  stop();
  getRandom();
  backward();
  delay(del); 
  stop(); 
}