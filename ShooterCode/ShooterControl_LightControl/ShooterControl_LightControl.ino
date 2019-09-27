#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <PID_v1.h>

#define DEBUG 0
#define PRINTDEBUG(STR) \
  {  \
    if (DEBUG) Serial.print(STR); \
  }
#define PRINTLNDEBUG(STR) \
  {  \
    if (DEBUG) Serial.println(STR); \
  }
//Define Pins

// Motor0 -> Azimuth
#define Motor0PinA 39
#define Motor0PinB 47
#define Motor0EncoderPinA 19//20   // Encoder Pin A pin 2 and pin 3 are inturrpt pins (MUST BE 2 OR 3)
#define Motor0EncoderPinB 14//25  // Encoder Pin B
#define Motor0PWM 46
#define Motor0INT 4

// Motor1 -> Elevation
#define Motor1PinA 51
#define Motor1PinB 49
#define Motor1EncoderPinA 21   // Encoder Pin A pin 2 and pin 3 are inturrpt pins (MUST BE 2 OR 3)
#define Motor1EncoderPinB 31   // Encoder Pin B
#define Motor1PWM 44
#define Motor1INT 2

// Motor1 -> Reload
#define Motor2PinA 43
#define Motor2PinB 41
#define Motor2EncoderPinA 18   // Encoder Pin A pin 2 and pin 3 are inturrpt pins (MUST BE 2 OR 3)
#define Motor2EncoderPinB 27   // Encoder Pin B
#define Motor2PWM 45
#define Motor2INT 5

#define laserPin 37
#define airPressurePin 35
#define airPressurePinLow 33
#define shootButton 2
#define shootButtonInt 0

#define stepMotorStepPin 6
#define stepMotorDirPin 13

#define PIDLIM 50000
#define EL_MAXLIM_ENC 500 
#define EL_MINLIM_ENC -500 

#define MOTOR_C_QRT_RND 1050
#define SHOOT_DELAY 500

//Initialize Variables
int encoder0PinA = -1;
int encoder0PinB = -1;
int encoder1PinA = -1;
int encoder1PinB = -1;
int encoder2PinA = -1;
int encoder2PinB = -1;
long counts0 = 0; //counts the encoder counts.
long counts1 = 0; //counts the encoder counts.
long counts2 = 0; //counts the encoder counts.

volatile boolean shootFlag = false;
boolean readyToShoot = true;;
boolean firstShot = true;

unsigned long compressorStartTime = millis();
#define compressorPin 8
boolean compressorFlag = false;
#define COMPRESSOR_TIME 3000

long counts1Last = 0;
long commandAz = 0;
long commandEl = 0;
long commandEnc = 0;
long commandTestUpdate = -60000;//millis();

long shootTime = 0;

// PID0 - YAW
double Setpoint0 = 0;
double Input0 = 0;
double Output0 = 0;
int kp0 = 50;
int ki0 = 0;
int kd0 = 0;//18;
double k_att0 = 1;//3.75;


// PID1 - PITCH
double Setpoint1 = 0;
double Input1 = 0;
double Output1 = 0;
int kp1 = 10;
int ki1 = 0;//20;
int kd1 = 0;//30;
double k_att1 = 1;//3.75;

// PID2
double Setpoint2 = 0;
double Input2 = 0;
double Output2 = 0;
int kp2 = 150;
int ki2 = 5;
int kd2 = 15;
double k_att2 = 2;//3.75;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete


#include "classes.h"

motor motor0(Motor0PinA, Motor0PinB, Motor0EncoderPinA, Motor0EncoderPinB , Motor0PWM,0);
motor motor1(Motor1PinA, Motor1PinB, Motor1EncoderPinA, Motor1EncoderPinB , Motor1PWM,1);
motor motor2(Motor2PinA, Motor2PinB, Motor2EncoderPinA, Motor2EncoderPinB , Motor2PWM,2);
PID myPID0(&Input0, &Output0, &Setpoint0, kp0, ki0, kd0, P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
PID myPID1(&Input1, &Output1, &Setpoint1, kp1, ki1, kd1, P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
PID myPID2(&Input2, &Output2, &Setpoint2, kp2, ki2, kd2, P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(airPressurePin,OUTPUT);
  pinMode(airPressurePinLow,OUTPUT);
  pinMode(compressorPin,OUTPUT);
  digitalWrite(airPressurePinLow,LOW);
  digitalWrite(airPressurePin,LOW);  
  digitalWrite(compressorPin,HIGH);
  pinMode(laserPin,OUTPUT);
  digitalWrite(laserPin,HIGH);    
  pinMode(shootButton,INPUT_PULLUP);  
  attachInterrupt(Motor0INT, readEncoder0, CHANGE); //attach interrupt to PIN 2
  attachInterrupt(Motor1INT, readEncoder1, CHANGE); //attach interrupt to PIN 3
  attachInterrupt(Motor2INT, readEncoder2, CHANGE); //attach interrupt to PIN 21
  attachInterrupt(shootButtonInt, shootCommand, FALLING); //attach interrupt to PIN 20
  myPID0.SetMode(AUTOMATIC);
  myPID0.SetOutputLimits(-PIDLIM, PIDLIM);
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-PIDLIM, PIDLIM);
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-PIDLIM, PIDLIM);
  inputString.reserve(200);
}

void loop() {

  if ((shootFlag) && (readyToShoot)) {
    //Serial.println("SHOOT");
    if (firstShot) firstShot = false;
    else {
      shoot();
      shootFlag = false;
      readyToShoot = false;
      commandEnc = commandEnc + MOTOR_C_QRT_RND;
    }
    compressorFlag = true;
    compressorStartTime = millis();
  }

  if (!firstShot && (millis() - compressorStartTime) < COMPRESSOR_TIME) {
     //Serial.println("LOW");
     digitalWrite(compressorPin,LOW);  
  }
  else {
    compressorFlag = false;
    //Serial.println("HIGH");
    digitalWrite(compressorPin,HIGH);  
  }


  Setpoint2 = (double)commandEnc;
  Input2 = counts2;   //<===========================
  myPID2.Compute();
  double nOutput2 = constrain(map(Output2, -PIDLIM*k_att2, PIDLIM*k_att2, -255, 255),-255,255);
  motor2.moveMotor(nOutput2); //<===========================

  
  if (motor2.getState() == 0 && (millis() - compressorStartTime) > COMPRESSOR_TIME) readyToShoot = true;
  else readyToShoot = false;

  if ((millis() - compressorStartTime) < COMPRESSOR_TIME + 200) shootFlag = false;
  
  // put your main code here, to run repeatedly:
  Setpoint0 = (double)commandAz;
  
  while (Serial.available() > 0 ) {
        while (Serial.available() < 2){
          delay(10);
        }
        char pitch_yaw_flag  = Serial.read();
        if (pitch_yaw_flag == 0){
          int8_t pwm = Serial.read();
          motor0.moveMotor(pwm*2); //<===========================
          delay(50);
          motor0.moveMotor(0); //<===========================
        } else if (pitch_yaw_flag == 1) {
          int8_t pwm2 = Serial.read();
          motor1.moveMotor(pwm2*2); //<===========================
          delay(25);
          motor1.moveMotor(0); //<===========================
        }
        //Serial.println(pwm);
        //Serial.println(pwm2);


  }

}


void readEncoder0() //this function is triggered by the encoder CHANGE, and increments the encoder counter
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB) ) counts0++;
  else counts0--;
}

void readEncoder1() //this function is triggered by the encoder CHANGE, and increments the encoder counter
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB) ) counts1--;
  else counts1++;
}

void readEncoder2() //this function is triggered by the encoder CHANGE, and increments the encoder counter
{
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB) ) counts2--;
  else counts2++;
}

void shootCommand() //this function is triggered by the encoder falling, and increments the encoder counter
{
  if ((millis() - shootTime) > SHOOT_DELAY) {
      shootFlag = true;
      shootTime = millis();
  }
}

void shoot() {
  digitalWrite(airPressurePin,HIGH);
  delay(200);
  digitalWrite(airPressurePin,LOW);
  delay(10);
  digitalWrite(airPressurePin,HIGH);
  delay(10);
  digitalWrite(airPressurePin,LOW);
  PRINTLNDEBUG("SHOOT2!")
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
        //PRINTDEBUG("got input from oculus : ");
        //PRINTLNDEBUG( inputString)
    }
  }
}
