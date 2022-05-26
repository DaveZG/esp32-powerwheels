/*
  Serial1.begin(115200,SERIAL_8N1,16,17);  where DEAFULT Rx is 16 Tx is 17 so we can change the pins that the car serial port will be on
*/

#include "SteeringSerial.h"
#include "utils.h"
#define MOTOR_RX_BYTES 8
#include <ESP32Encoder.h> //Not needed unless you have an encoder on the steering wheel

//Make the encoder object
ESP32Encoder encoder; //Not needed unless you have an encoder on the steering wheel

//Define the min and Max steering encorder counts
#define MIN_STEER                       -21 //Not needed unless you have an encoder on the steering wheel
#define MAX_STEER                       21  //Not needed unless you have an encoder on the steering wheel

#define STEERING_ROTARY_SWITCH_PIN      33  //Not needed unless you have an encoder on the steering wheel
#define FORWARDS_GEAR_PIN               12  //Not needed unless you have forward and reverse switch
#define REVERSE_GEAR_PIN                14  //Not needed unless you have forward and reverse switch
#define ROTARY_ENCODER_PIN_A            25  //Not needed unless you have an encoder on the steering wheel
#define ROTARY_ENCODER_PIN_B            26  //Not needed unless you have an encoder on the steering wheel

#define DRIVING_STATE_NEUTRAL            0  //Not needed unless you have forward and reverse switch
#define DRIVING_STATE_FORWARDS           1  //Not needed unless you have forward and reverse switch
#define DRIVING_STATE_REVERSE            2  //Not needed unless you have forward and reverse switch

#define SNAIL_MODE          1
#define SLOW_MODE           2
#define NORMAL_MODE         3
#define SPORTS_MODE         4
#define LIMP_MODE           5   //Used when we are between almost empty and UVLO, to limp home on one set of motors and low max speed

bool emergency = false;
uint8_t carMode = SNAIL_MODE;
uint16_t ModeMax = 1625;
uint16_t ModeMin = 1375;

// Define variables to store incoming readings - start them in a defined nuteral state
uint16_t incomingFrontSpeed = 1500;
uint16_t incomingBackSpeed = 1500;
uint16_t incomingFrontSteer = 1500;
uint16_t incomingBackSteer = 1500;
bool incomingEmergency = false;
uint16_t incomingModeMax = 1500;
uint16_t incomingModeMin = 1500;

bool rxData = false;
char motorDataBuffer[MOTOR_RX_BYTES];
uint8_t motorDataIndex = 0;
uint16_t batteryVoltage = 0;
uint16_t realSpeed = 0;
uint16_t realSpeedSlave = 0;

bool rxDataRear = false;
char motorDataBufferRear[MOTOR_RX_BYTES];
uint8_t motorDataIndexRear = 0;
uint16_t batteryVoltageRear = 0;
uint16_t realSpeedRear = 0;
uint16_t realSpeedSlaveRear = 0;

// Variable to store connected state of if sending data was successful
String success;
uint8_t failedDelivery = 0;
uint8_t maxfailedDelivery = 3;
//volatile bool remoteConnected = false;
uint8_t drivingState = DRIVING_STATE_NEUTRAL; //0 is neutral, 1 is forward, 2 is reverse

void changeMode(uint8_t newMode)
{
  switch (newMode) 
  {
    case SNAIL_MODE:
      carMode = SNAIL_MODE;
      ModeMax = 1625;
      ModeMin = 1375;
      break;
    case SLOW_MODE:
      carMode = SLOW_MODE;
      ModeMax = 1750;
      ModeMin = 1250;
      break;
    case NORMAL_MODE:
      carMode = NORMAL_MODE;
      ModeMax = 1875;
      ModeMin = 1125;
      break;
    case SPORTS_MODE:
      carMode = SPORTS_MODE;
      ModeMax = 2000;
      ModeMin = 1000;
      break;
    case LIMP_MODE:
      carMode = LIMP_MODE;
      ModeMax = 1600;
      ModeMin = 1400;
      break;
    default:
      // statements
      break;
  }
}

void setup() {
  // Initialize steering serial
  InitSteeringSerial();

  pinMode(STEERING_ROTARY_SWITCH_PIN, INPUT_PULLUP);    // sets the joystick button as input //Normally pulled high
  pinMode(FORWARDS_GEAR_PIN, INPUT_PULLUP);    // sets the joystick button as input //Normally pulled high
  pinMode(REVERSE_GEAR_PIN, INPUT_PULLUP);    // sets the joystick button as input //Normally pulled high

  // use pin 19 and 18 for the first encoder
  encoder.attachFullQuad(ROTARY_ENCODER_PIN_A, ROTARY_ENCODER_PIN_B);

  // set starting count value after attaching
  encoder.setCount(0);

  //changeMode <-- use this function to set your speed mode
  changeMode(SLOW_MODE);
}
 
void loop() {  

  //Get our speed and steer locally from the car pedal and steering
  int32_t count = encoder.getCount();
  uint16_t throttle = analogRead(A7);
  //Serial.println("Throttle: " + String(throttle));
  uint16_t throttlerear;

  //Make sure we don't go over our min or max settings
  if(count<MIN_STEER){
    encoder.setCount(MIN_STEER);
  }
  if(count>MAX_STEER){
    encoder.setCount(MAX_STEER);
  }

  //Map the raw steering count and constrain it to our steering min and max
  count = map(count, MIN_STEER, MAX_STEER, 1000, 2000);
  count = constrain(count,1000,2000);
  
  //constrain our thottle then set it as forward or reverse
  throttle = constrain(throttle,912,3060);
  throttlerear = throttle;
  
  //if we are in forwards gear selector else use reverse
  if(!digitalRead(REVERSE_GEAR_PIN)){
    drivingState = DRIVING_STATE_REVERSE;
    throttle = map(throttle, 912, 3060, 1500, ModeMin);
    throttlerear = map(throttlerear, 912, 3060, 1500, ModeMax);
  }
  else if(!digitalRead(FORWARDS_GEAR_PIN)){
    drivingState = DRIVING_STATE_FORWARDS;
    throttle = map(throttle, 912, 3060, 1500, ModeMax);
    throttlerear = map(throttlerear, 912, 3060, 1500, ModeMin);
  }
  else
  {
    drivingState = DRIVING_STATE_NEUTRAL;
    throttle = 1500;
    throttlerear = 1500;
  }

  // slow us down if we are in reverse
  float factor = throttle < 1500 ? 0.5 : 1; 
  float rearfactor = throttlerear > 1500 ? 0.5 : 1; //Rear motor in reverse orientation

//  Serial.println("Incoming ModeMax: " + String(incomingModeMax));

  //Allow local Car control if not an emergency
  if(!incomingEmergency)
  {
    // Set speed and steering for Front and Rear motors
    SetSpeed(throttle, factor); //channel values between 1000 and 2000 for min and max
    SetSpeedRear(throttlerear, rearfactor); //channel values between 1000 and 2000 for min and max
    SetSteerRear(count);
    //reverse count for front motors as they are facing the other way meaning forwards is reverse and visa versa 
    count = map(count, 1000, 2000, 2000, 1000);
    SetSteer(count);   
  }
    
  // Receive data from front motor controller
  if (Serial1.available())
  {
    char character = Serial1.read();

    if(character == '/')
    {
      //We got the start of a message
      motorDataBuffer[motorDataIndex] = character;
      motorDataIndex = 1;
      rxData = true;
    }

    //We got to the end of the message so reply, print out the values and reset the index
    else if (character == '\n')
    {
      SendAnswer();
      motorDataBuffer[motorDataIndex] = character;
      motorDataIndex = 0;
      rxData = false;
      batteryVoltage = (uint16_t)((motorDataBuffer[1] << 8) | motorDataBuffer[2]);
      realSpeed = (uint16_t)((motorDataBuffer[3] << 8) | motorDataBuffer[4]);
      realSpeedSlave = (uint16_t)((motorDataBuffer[5] << 8) | motorDataBuffer[6]);
    }

    //We have data and we are recieving now so collect the bytes
    else if ((motorDataIndex<MOTOR_RX_BYTES) && (rxData))
    {
      //Store our data
      motorDataBuffer[motorDataIndex] = character;
      motorDataIndex++;
    }
  }

  // Receive data from rear motor controller
  if (Serial2.available())
  {
    char character2 = Serial2.read();

    if(character2 == '/')
    {
      //We got the start of a message
      motorDataBufferRear[motorDataIndexRear] = character2;
      motorDataIndexRear = 1;
      rxDataRear = true;
    }

    //We got to the end of the message so reply, print out the values and reset the index
    else if (character2 == '\n')
    {
      SendAnswerRear();
      motorDataBufferRear[motorDataIndexRear] = character2;
      motorDataIndexRear = 0;
      rxDataRear = false;
      batteryVoltageRear = (uint16_t)((motorDataBufferRear[1] << 8) | motorDataBufferRear[2]);
      realSpeedRear = (uint16_t)((motorDataBufferRear[3] << 8) | motorDataBufferRear[4]);
      realSpeedSlaveRear = (uint16_t)((motorDataBufferRear[5] << 8) | motorDataBufferRear[6]);
    }

    //We have data and we are recieving now so collect the bytes
    else if ((motorDataIndexRear<MOTOR_RX_BYTES) && (rxDataRear))
    {
      //Store our data
      motorDataBufferRear[motorDataIndexRear] = character2;
      motorDataIndexRear++;
    }
  }
}
