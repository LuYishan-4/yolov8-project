
#include <library.h>

#include <HCSR04.h>
#include <Wire.h>
#include <Servo.h>

Servo sc;
int rightSpeed = 0;
int leftSpeed = 0;
bool think = false;
bool think2 = false;
bool reve = false;
int x1,x2,y1,y2,cx,cy,z;
int mx = 320; 
int my = 180;
float setpoint1,setpoint2;
int uy = 10;



void setup() {
Wire.begin(0x60);                // join i2c bus with address #0x40
Wire.onReceive(receiveEvent); // register event
sc.attach(7,0,180);
Wire.onRequest(requestEvent);


   int pins[] = {1,2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
   int pd[] = {1,2,3,5,6,9,10,11};
  for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++) {
    pinMode(pins[i], OUTPUT);
  }
   for (int x = 0; x < sizeof(pd)/sizeof(pd[0]); x++) {
    digitalWrite(pd[x], LOW);
  }
}



void loop() {

PIDController pid1(1.0,0,0, cx);
PIDController pid2(1.0,0,0, cy); 
float output1 = pid1.compute(mx, 1);
float output2 = pid2.compute(my, 1);
rightSpeed = abs(output1 + output2);
leftSpeed = abs(output1 - output2);
rightSpeed = constrain(rightSpeed, 0, 255);
leftSpeed = constrain(leftSpeed, 0, 255);
 analogWrite(5, leftSpeed); // lf
 analogWrite(9, rightSpeed); // rb
 analogWrite(13, rightSpeed);  // rf
 analogWrite(4, leftSpeed);  // lb
 Serial.print("left:");
 Serial.print(leftSpeed);
 Serial.print(",");
 Serial.print("right:");
 Serial.println(rightSpeed);


  
servo(think,think2);

}

void servo(bool think,bool think2){
if(think == true){
  sc.write(0);
}else{
  sc.write(0);
}
if(think2 == true){
  sc.write(0);
}else{
  sc.write(0);
}
}
void Front(bool reve){
  if(reve == false){
   digitalWrite(10, LOW);
   digitalWrite(11, HIGH);

   digitalWrite(1, HIGH);
   digitalWrite(6, LOW);
   
   digitalWrite(12, HIGH);
   digitalWrite(8, LOW);

   digitalWrite(3, LOW);
   digitalWrite(2, HIGH);
  }else{
    digitalWrite(10, HIGH);
   digitalWrite(11, LOW);

   digitalWrite(1, LOW);
   digitalWrite(6, HIGH);
   
   digitalWrite(12, LOW);
   digitalWrite(8, HIGH);

   digitalWrite(3, HIGH);
   digitalWrite(2, LOW);
  }
}
void receiveEvent(int len) {
  if (len < 18) return;

  x1 = (Wire.read() << 8) | Wire.read();
  x2 = (Wire.read() << 8) | Wire.read();
  y1 = (Wire.read() << 8) | Wire.read();
  y2 = (Wire.read() << 8) | Wire.read();
  cx = (Wire.read() << 8) | Wire.read();
  cy = (Wire.read() << 8) | Wire.read();
  z = (Wire.read() << 8) | Wire.read();
  think = Wire.read() > 0;
  think2 = Wire.read() > 0;
}
void requestEvent() {

  Wire.write(leftSpeed & 0xFF);
  Wire.write(rightSpeed & 0xFF);
}