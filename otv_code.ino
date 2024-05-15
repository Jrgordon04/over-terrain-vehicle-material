/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * Arduino Mecanum Omni Direction Wheel Robot Car
 * Tutorial URL http://osoyoo.com/?p=30022
 * CopyRight www.osoyoo.com

 * After running the code, smart car will 
 * go forward and go backward for 2 seconds, 
 * left turn and right turn for 2 seconds, 
 * right shift and left shift for 2 seconds,
 * left diagonal back and right diagonal forward for 2 seconds,
 * left diagonal forward and right diagonal back for 2 seconds,
 * then stop. 
 * 
 */

#include "Enes100.h"

int SPEED = 200;    
int TURN_SPEED = 100; 

int speedPinR = 44;   //  Front Wheel PWM pin connect Right MODEL-X ENA 
int RightMotorDirPin1 = 12;    //Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
int RightMotorDirPin2 = 11;   //Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)   
int RightMotorDirPin1B = 10;    //Rear Right Motor direction pin 1 to Right  MODEL-X IN1 ( K1)
int RightMotorDirPin2B = 9;    //Rear Right Motor direction pin 2 to Right  MODEL-X IN2 ( K1) 
int speedPinRB = 46;   //  Front Wheel PWM pin connect Right MODEL-X ENB

int speedPinL = 2;   //  Rear Wheel PWM pin connect Left MODEL-X ENA 
int LeftMotorDirPin1 = 6;    //Rear Left Motor direction pin 1 to Left MODEL-X IN3 (K3)
int LeftMotorDirPin2 = 5;   //Rear Left Motor direction pin 2 to Left MODEL-X IN4 (K3)
int LeftMotorDirPin1B = 4;    //Front Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
int LeftMotorDirPin2B = 3;  //Front Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)
int speedPinLB = 45;    //  Rear Wheel PWM pin connect Left MODEL-X ENB

const int trigPin = 38;
const int echoPin = 7;
const int trigPinb = 39;
const int echoPinb = 8;
// defines variables

long duration;
float distance;
float lengthblock;
float currentTheta;
float threshold = 0.05;
float value;
float blockDist = 100;

/*motor control*/
void go_advance(int speed1, int speed2, int speed3, int speed4){
   RL_fwd(speed1);
   RR_fwd(speed2);
   FR_fwd(speed3);
   FL_fwd(speed4); 
   return;
}
void go_back(int speed){
   RL_bck(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_bck(speed); 
}
void right_shift(int speed_fl_fwd,int speed_rl_bck ,int speed_rr_fwd,int speed_fr_bck) {
  FL_fwd(speed_fl_fwd); 
  RL_bck(speed_rl_bck); 
  RR_fwd(speed_rr_fwd);
  FR_bck(speed_fr_bck);
  return;
}
void left_shift(int speed_fl_bck,int speed_rl_fwd ,int speed_rr_bck,int speed_fr_fwd){
  FL_bck(speed_fl_bck);
  RL_fwd(speed_rl_fwd);
  RR_bck(speed_rr_bck);
  FR_fwd(speed_fr_fwd);
  return;
}

void left_turn(int speed){
   RL_bck(0);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_bck(0); 
}
void right_turn(int speed){
  RL_fwd(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_fwd(speed); 
}
void left_back(int speed){
   RL_fwd(0);
   RR_bck(speed);
   FR_bck(speed);
   FL_fwd(0); 
}
void right_back(int speed){
   RL_bck(speed);
   RR_fwd(0);
   FR_fwd(0);
   FL_bck(speed); 
}
void clockwise(int speed){
   RL_fwd(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_fwd(speed); 
}
void countclockwise(int speed){
   RL_bck(speed);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_bck(speed); 
}


void FR_fwd(int speed)  //front-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1,LOW);
  digitalWrite(RightMotorDirPin2,HIGH); 
  analogWrite(speedPinR,speed);
}
void FR_bck(int speed) // front-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1,HIGH);
  digitalWrite(RightMotorDirPin2,LOW); 
  analogWrite(speedPinR,speed);
}
void FL_fwd(int speed) // front-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1B,HIGH);
  digitalWrite(LeftMotorDirPin2B,LOW);
  analogWrite(speedPinLB,speed);
}
void FL_bck(int speed) // front-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1B,LOW);
  digitalWrite(LeftMotorDirPin2B,HIGH);
  analogWrite(speedPinLB,speed);
}

void RR_fwd(int speed)  //rear-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B,LOW); 
  analogWrite(speedPinRB,speed);
}
void RR_bck(int speed)  //rear-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B,HIGH); 
  analogWrite(speedPinRB,speed);
}
void RL_fwd(int speed)  //rear-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
}
void RL_bck(int speed)    //rear-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
}
 
void stop_Stop()    //Stop
{
 digitalWrite(LeftMotorDirPin1,LOW);
 digitalWrite(LeftMotorDirPin2,LOW);
 digitalWrite(LeftMotorDirPin1B,LOW);
 digitalWrite(LeftMotorDirPin2B,LOW);
 digitalWrite(RightMotorDirPin1,LOW);
 digitalWrite(RightMotorDirPin2,LOW);
 digitalWrite(RightMotorDirPin1B,LOW);
 digitalWrite(RightMotorDirPin2B,LOW);
}


void setAngle(float target) {
    //print("Targeting angle: "); println(int target);
    // The following line runs our targeting code WHILE the DIFFERENCE (subtraction is taking the difference) is between -thresh and thresh. 
    // We take the absolute value of the difference in order to compare it to a single threshold.
    while (abs(target - Enes100.getTheta()) > threshold) {
      // Enes100.print("condition = ");
      // Enes100.println(target - Enes100.getTheta());
  
     // Enes100.println(abs(target - Enes100.getTheta()));
      right_turn(100);

  //turnValue = someConstant * (target - currentTheta);
    }
    stop_Stop();
    delay(4000);
}

//Pins initialize
void init_GPIO()
{
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT); 
  pinMode(RightMotorDirPin2B, OUTPUT); 
  pinMode(speedPinLB, OUTPUT);  
 
  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT); 
  pinMode(speedPinRB, OUTPUT);
   
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinb, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinb, INPUT); // Sets the echoPin as an Input
  //stop_Stop();
}

void setup()
{
  delay(1000);
  Enes100.begin("Septapi", MATERIAL, 467, 52, 53);
  delay(1000);
  init_GPIO();

//  go_advance(SPEED);
//go_back(SPEED);
       //delay(1000);
//       stop_Stop();
       //delay(1000);

//left_turn(TURN_SPEED);
      //delay(1000);
//       stop_Stop();
     //delay(1000);
	  
//right_turn(TURN_SPEED);
      //delay(1000);
//      stop_Stop();
      //delay(1000);
  
//right_shift(200,200,200,200); //right shift
      //delay(1000);
//      stop_Stop();
      //delay(1000);

//left_shift(200,200,200,200); //left shift
      //delay(1000);
//      stop_Stop();
      //delay(1000);
	 
//left_shift(200,0,200,0); //left diagonal back
      //delay(1000);
//      stop_Stop();
 	 //delay(1000);
 
//right_shift(200,0,200,0); //right diagonal ahead
      //delay(1000);
//      stop_Stop();
 	 //delay(1000);

//left_shift(0,200,0,200); //left diagonal ahead
     // delay(1000);
//      stop_Stop();
      //delay(1000);

//right_shift(0,200,0,200); //right diagonal back
     //delay(1000);
//      stop_Stop();
   	 //delay(1000);
}


void loop(){
//  Enes100.println(Enes100.getX());

//   Enes100.println(Enes100.getX());
  // Enes100.println("Y: ");
  // Enes100.println(Enes100.getY());
  // go_advance(255,255,220,255);

  if (Enes100.getY()>1) 
  {
    setAngle(-1.42);
    while (Enes100.getY() > 1)
    {
      // Enes100.println(Enes100.getY());
      go_advance(255,255,220,255);
    }
    stop_Stop();
    delay(2000);
  } 
  else if (Enes100.getY()<1)
  {
    setAngle(1.74);
    while (Enes100.getY() < 1)
    {
      // Enes100.println(Enes100.getY());
      go_advance(255,255,220,255);
    }
    stop_Stop();
    delay(2000);
  }

 while(Enes100.getX() > 0.49)
  {
    left_shift(100,100,100,100);
  }
    stop_Stop();
    delay(2000);

 while(Enes100.getX() < 0.44)
  {
    right_shift(100,100,100,100);
  }
    stop_Stop();
    delay(2000);    
  

 //  material id
  while(1){
  delay(2000);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  //  Calculating the distance
  distance = duration*(0.034/2);
 
  delay(2000);
  
  if (distance <= 11) 
  {
    Enes100.println("Plastic. ");
    Enes100.println(distance);
    delay(2000);
  } 
  else if (distance > 11) 
  {
    Enes100.print("Foam. ");
    Enes100.println(distance);
    delay(2000);
  }else{
    Enes100.println(distance);
    delay(2000);
  }
  }
  // Enes100.println(distance);

//   Enes100.println(distance);
//   delay(2000);
//   value = analogRead(A0);
//   Enes100.println(value);
//   if (value < 500) {
//     go_advance(255,255,255,255);
//   }
//   else {
//     delay(2000);
//     stop_Stop();
//     delay(2000);
//   }

//   force sensor values

//   if (value >= 500 && value <= 600) 
//   {
//     delay(2000);
//     Enes100.println(value);
//     Enes100.println(", Light Block");
//     delay(2000);
//   }
//   else if (value > 600 && value <= 700) 
//   {
//     delay(2000);
//     Enes100.println(value);
//     Enes100.println(", Medium Block");
//     delay(2000);
//   }
//   else if (value > 700)
//   {
//     delay(2000);
//     Enes100.println(value);
//     Enes100.println(", Heavy Block");
//     delay(2000);
//   } 

// reverse navigation
  // delay(4000);
  // digitalWrite(trigPinb, LOW);
  // delayMicroseconds(2);
  // //Sets the trigPin on HIGH state for 10 micro seconds
  // digitalWrite(trigPinb, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(trigPinb, LOW);
  // // Reads the echoPinb, returns the sound wave travel time in microseconds
  // duration = pulseIn(echoPinb, HIGH);
  // //  Calculating the distance
  // distance = duration*(0.034/2);

}


