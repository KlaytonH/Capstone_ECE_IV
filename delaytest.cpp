#include <DueTimer.h>
//If object is within 200cm of the front of the golf cart
//stop the golf cart, else go. 
/* ------CURRENT BUGS-----------
This is where i will report current bugs with the program.
1.)when cart is stopped due to sensors being blocked, need to trigger main sensers to go again.
2.)need to check in the middle of dodging for obstacles.

------------------------------*/
#define trigPinR 30 //Send high to this pin to trigger sound wave or low to not trigger sound wave
#define echoPinR 32 //Send high to this pin to listen for sound wave or low to not listen. 
#define trigPinL 28 //Send high to this pin to trigger sound wave or low to not trigger sound wave
#define echoPinL 26 //Send high to this pin to listen for sound wave or low to not listen. 
#define trigPinFR 52 //Send high to this pin to trigger sound wave or low to not trigger sound wave
#define echoPinFR 53 //Send high to this pin to listen for sound wave or low to not listen.
#define trigPinFL 50 //Send high to this pin to trigger sound wave or low to not trigger sound wave
#define echoPinFL 51 //Send high to this pin to listen for sound wave or low to not listen. 


#define Brake_actuator_extend 37 //send high to this & low to other wire (of the same actuator) to extend
#define Brake_actuator_retract 36 //send high to this & low to other wire (of the same actuator) to retract
#define Go_actuator_extend 41 //send high to this & low to other wire (of the same actuator) to extend
#define Go_actuator_retract 40 //send high to this & low to other wire (of the same actuator) to retract

int durationFR, distanceFR;
int durationFL, distanceFL;
int durationR, distanceR;
int durationL, distanceL;

//stepper motor variables
unsigned long motorFreq = 0;
long L_duration;
long R_duration;
const int stepper_pulse = 9;
const int stepper_direction = 6;
bool pulseUp = false;

int turningdirection=5; //turning direction 0="we turned left",1="we turned right"
bool Stopped;
int previousturn=0;

//current position variables
int GoPosition;
int BrakePosition;
// these should range from 0-1800 for the brake actuator and 0-2200 for the go actuator
int NewPosition;
int time_to_delay;
bool add=true;
bool subtract=false;


//mydelay variables
bool booltemp;
int delaytime;
int StartTime;
int CurrentTime;
int StartPlusDelay;
int actualdelaytime;
int errortest;


void setup() {
//  Serial.begin (9600); //Board Rate
 // -----------Designate INPUT/OUTPUT Pins Here--------------
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  
  pinMode(trigPinFR, OUTPUT);
  pinMode(echoPinFR, INPUT);
  pinMode(trigPinFL, OUTPUT);
  pinMode(echoPinFL, INPUT);
  
  pinMode(Brake_actuator_extend, OUTPUT);
  pinMode(Brake_actuator_retract, OUTPUT);
  pinMode(Go_actuator_extend, OUTPUT);
  pinMode(Go_actuator_retract, OUTPUT);
  
  //stepper motor code
  Serial.begin(57600);
  pinMode(stepper_pulse,OUTPUT);
  pinMode(stepper_direction,OUTPUT);
  digitalWrite(stepper_pulse,LOW);
  digitalWrite(stepper_direction,LOW);
  Timer3.attachInterrupt(motorPulse); //will need to comment out and see if its needed, forgot to look before i deleted unused old matrial in functions
  //turnStop();
    retractboth();
  delay(2500);
  StopActuator();
  GoPosition = 0;
  BrakePosition = 0;

}
//---------------Main Code Here----------------------
// This code is going from A to B in stright line 
void loop() {
//---------------------------------------*/
GoCart();
booltemp = MyGoDelay(1800);
if (booltemp == true)
      {  /*whole delay occured, pause actuators and calculate positions*/
        StopActuator();
        // CalcActuatorPosition(previous position,delay that occured,1=addition, any other int = subtraction)
        GoPosition = CalcActuatorPosition(GoPosition,actualdelaytime,add);  //GoPosition = GoPosition+actualdelay
        BrakePosition = 0;
        Serial.println("full delay ran, no object detected");
        Serial.print("Actual Delay Time: ");
        Serial.println(actualdelaytime);
        Serial.print("GoPosition = ");
        Serial.println(GoPosition);
        Serial.print("BrakePosition = ");
        Serial.println(BrakePosition);
        delay(5000);
      }
      else
      {  //only part of delay occurred, pause actuators and calculate positions
        StopActuator();
        //extending go implies addition, and visa versa for brake
        GoPosition = CalcActuatorPosition(GoPosition,actualdelaytime,add);//GoPosition = GoPosition+actualdelaytime
        BrakePosition = CalcActuatorPosition(BrakePosition,actualdelaytime,subtract);
        
        Serial.println("Detected object while accelerating");
        Serial.print("Actual Delay Time: ");
        Serial.println(actualdelaytime);
        Serial.print("GoPosition = ");
        Serial.println(GoPosition);
        Serial.print("BrakePosition = ");
        Serial.println(BrakePosition);
        delay(5000);
        
      }


}//end main loop

//---------------Functions----------
void rightsensor(){
  //-------Right Sensor--------
  digitalWrite(trigPinR, HIGH);//triggers sonar
  delayMicroseconds(1000);
  digitalWrite(trigPinR, LOW);//turn off the sonar sender
  durationR = pulseIn(echoPinR, HIGH);//measures distance
  distanceR = (durationR/2) / 29.1;//converts to cm
  if (distanceR<=10)
    rightsensor();
  delay(5);
}

void leftsensor(){
  //-------Left Sensor-------- 
  digitalWrite(trigPinL, HIGH); //triggers sonar
  delayMicroseconds(1000);
  digitalWrite(trigPinL, LOW); //turn off the sonar sender
  durationL = pulseIn(echoPinL, HIGH); //measures distance
  distanceL = (durationL/2) / 29.1; //converts to cm
  if (distanceL<=10)
    leftsensor();
  delay(5);
}
void frontrightsensor(){
  //———Front Right Sensor--------
  digitalWrite(trigPinFR, HIGH);//triggers sonar
  delayMicroseconds(1000);
  digitalWrite(trigPinFR, LOW);//turn off the sonar sender
  durationFR = pulseIn(echoPinFR, HIGH);//measures distance
  distanceFR = (durationFR/2) / 29.1;//converts to cm
//  Serial.print("distanceFR = ");
//  Serial.println(distanceFR);
  if (distanceFR<=10)
    frontrightsensor();
  delay(5);
}
void frontleftsensor(){
  //———Front Left Sensor-------- 
  digitalWrite(trigPinFL, HIGH); //triggers sonar
  delayMicroseconds(1000);
  digitalWrite(trigPinFL, LOW); //turn off the sonar sender
  durationFL = pulseIn(echoPinFL, HIGH);//measures distance
  distanceFL = (durationFL/2) / 29.1; //converts to cm
 //Serial.print("distanceFL = ");
 //Serial.println(distanceFL);
  if (distanceFL<=10)
    frontleftsensor();
  delay(5);
}
//---Actuator functions-----
void StopCart() {
  //this will move extend the actuator
  digitalWrite(Brake_actuator_extend, HIGH);
  digitalWrite(Brake_actuator_retract, LOW);
  digitalWrite(Go_actuator_extend, LOW);
  digitalWrite(Go_actuator_retract, HIGH);
}
void ExtendBrake()  {
  digitalWrite(Brake_actuator_extend, HIGH);
  digitalWrite(Brake_actuator_retract, LOW);
}
void RetractBrake()  {
  digitalWrite(Brake_actuator_extend, LOW);
  digitalWrite(Brake_actuator_retract, HIGH);
}
void GoCart() {
  //this will move retract the actuator
    digitalWrite(Brake_actuator_extend, LOW);
    digitalWrite(Brake_actuator_retract, HIGH);
    digitalWrite(Go_actuator_extend, HIGH);
    digitalWrite(Go_actuator_retract, LOW);
}
void ExtendGo()  {
    digitalWrite(Go_actuator_extend, HIGH);
    digitalWrite(Go_actuator_retract, LOW);
}
void RetractGo()  {
    digitalWrite(Go_actuator_extend, LOW);
    digitalWrite(Go_actuator_retract, HIGH);
}
void StopActuator() {
  //pause both pedal's actuator's movement
    digitalWrite(Brake_actuator_extend, LOW);
    digitalWrite(Brake_actuator_retract, LOW);
    digitalWrite(Go_actuator_extend, LOW);
    digitalWrite(Go_actuator_retract, LOW);
}
void StopBrake() {
  //pause brake pedal actuator's movement
    digitalWrite(Brake_actuator_extend, LOW);
    digitalWrite(Brake_actuator_retract, LOW);
}
void StopGo() {
  //pause go pedal actuator's movement
    digitalWrite(Go_actuator_extend, LOW);
    digitalWrite(Go_actuator_retract, LOW);
}
void retractboth() {
  //this will retract both actuators, for moving the actuators for assembly/disassembly
    digitalWrite(Brake_actuator_extend, LOW);
    digitalWrite(Brake_actuator_retract, HIGH);
    digitalWrite(Go_actuator_extend, LOW);
    digitalWrite(Go_actuator_retract, HIGH);
 }
 //-----stepper motor functions-----
 void motorPulse() {
  pulseUp = !pulseUp;  
  digitalWrite(stepper_pulse, pulseUp); // Pulse on, off, on, off...
}

void turnLeft() {
//  turns the CART RIGHT
    digitalWrite(stepper_direction,LOW);
    Timer3.setFrequency(100000).start();
//  }
}

void turnRight() {
//  turns the CART LEFT
    digitalWrite(stepper_direction,HIGH);
    Timer3.setFrequency(100000).start();
// 
}

void turnStop() {
  Timer3.setFrequency(100000).stop();
}

bool MyGoDelay(int delaytime)  {
StartTime  = millis();
CurrentTime= millis();
StartPlusDelay = StartTime+delaytime;

while (CurrentTime < StartPlusDelay)
{
  rightsensor();
  leftsensor();
  frontrightsensor();
  frontleftsensor();
  if (distanceL <= 200 || distanceFL <= 75 || distanceR <=200 || distanceFR <= 75)
  {  break;  }
  CurrentTime=millis();
}

if (CurrentTime >= StartPlusDelay)
  { return true; }//delay ran in full
else
{
  CurrentTime=millis();
  actualdelaytime = CurrentTime-StartTime;  
  return false; //object detected, need to use goto in main function
}
}//end GoCartdelay function


bool MyStopDelay(int delaytime)  {
StartTime  = millis();
CurrentTime= millis();
StartPlusDelay = StartTime+delaytime;

while (CurrentTime < StartPlusDelay)
{
  rightsensor();
  leftsensor();
  frontrightsensor();
  frontleftsensor();
  if (distanceL > 200 && distanceFL > 75 && distanceR >200 && distanceFR > 75)
  {  break;  }
  CurrentTime=millis();
}

if (CurrentTime >= StartPlusDelay)
  { return true; }//delay ran in full
else
{
  CurrentTime=millis();
  actualdelaytime = CurrentTime-StartTime;  
  return false; //object detected, need to use goto in main function
}
}//end GoCartdelay function

int CalcActuatorPosition(int previous, int dist_moved, bool addition)
{
  if (addition == true)
  {
    //add
    NewPosition = previous + dist_moved;
  }
  else
  {
    //subtract
    NewPosition = previous - dist_moved;
    if (NewPosition < 0)
    {//minimum position can be is 0
      NewPosition = 0;
    }
    
  }
  return NewPosition;
}
