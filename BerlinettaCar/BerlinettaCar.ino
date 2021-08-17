/*
  ESPNow two way communication example
  30:AE:A4:96:BA:70 - Car ESP32 MAC Address
  30:AE:A4:96:FF:74 - Controller ESP32 MAC Address

  Serial1.begin(115200,SERIAL_8N1,16,17);  where DEAFULT Rx is 16 Tx is 17 so we can change the pins that the car serial port will be on

  Will use two core tasks, will put the espNow tasks in default Core0, and push the Serial Comms to the Motor controllers into Core1
  This will allow us to prioritize the motor controller comms 
*/

#include <esp_now.h>
#include <WiFi.h>
#include "SteeringSerial.h"
#include "utils.h"
#define MOTOR_RX_BYTES 8
#include <ESP32Encoder.h>

//Make the encoder object
ESP32Encoder encoder;

//Define the min and Max steering encorder counts
#define MIN_STEER                       -21
#define MAX_STEER                       21

#define STEERING_ROTARY_SWITCH_PIN      33
#define FORWARDS_GEAR_PIN               12
#define REVERSE_GEAR_PIN                14
#define ROTARY_ENCODER_PIN_A            25
#define ROTARY_ENCODER_PIN_B            26     

#define DRIVING_STATE_NEUTRAL            0
#define DRIVING_STATE_FORWARDS           1
#define DRIVING_STATE_REVERSE            2

// REPLACE WITH THE MAC Address of your receiver - this will be the Controller MAC
uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x96, 0xFF, 0x74};

// Define variables to store steer and speed values to be sent
uint16_t battv = 4100;
uint16_t frontleftrealspeed = 0;
uint16_t frontrightrealspeed = 0;
uint16_t backleftrealspeed = 0;
uint16_t backrightrealspeed = 0;

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
volatile bool remoteConnected = false;
uint8_t drivingState = DRIVING_STATE_NEUTRAL; //0 is neutral, 1 is forward, 2 is reverse

//Structure example to send data
//Must match the receiver structure
typedef struct struct_send_message {
    uint16_t batteryvoltage;
    uint16_t frontleftspeed;
    uint16_t frontrightspeed;
    uint16_t backleftspeed;
    uint16_t backrightspeed;
} struct_send_message;

typedef struct struct_recieve_message {
    uint16_t frontmotorsteer;
    uint16_t frontmotorspeed;
    uint16_t backmotorsteer;
    uint16_t backmotorspeed;
    bool emergency;
    uint16_t ModeMax;
    uint16_t ModeMin;
} struct_recieve_message;

// Create a struct_send_message called motorReadings to hold batt voltage and motor speed readings
struct_send_message motorReadings;

// Create a struct_recieve_message to hold incoming sensor readings
struct_recieve_message incomingspeedSteerSettings;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
    failedDelivery = 0;
    remoteConnected = true;
  }
  else{
    success = "Delivery Fail :(";
    failedDelivery++;
    if(failedDelivery>=maxfailedDelivery)
    {
      remoteConnected = false;
      incomingFrontSpeed = 1500;
      incomingBackSpeed = 1500;
      incomingFrontSteer = 1500;
      incomingBackSteer = 1500;
      incomingModeMax = 1500;
      incomingModeMin = 1500;
    }
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingspeedSteerSettings, incomingData, sizeof(incomingspeedSteerSettings));
//  Serial.print("Bytes received: ");
//  Serial.println(len);
  incomingEmergency = incomingspeedSteerSettings.emergency;
  incomingFrontSpeed = incomingspeedSteerSettings.frontmotorspeed;
  incomingBackSpeed = incomingspeedSteerSettings.backmotorspeed;
  incomingFrontSteer = incomingspeedSteerSettings.frontmotorsteer;
  incomingBackSteer = incomingspeedSteerSettings.backmotorsteer;
  incomingModeMax = incomingspeedSteerSettings.ModeMax;
  incomingModeMin = incomingspeedSteerSettings.ModeMin;
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
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  // slow us down if we are in reverse
  float factor = incomingFrontSpeed < 1500 ? 0.5 : 1; 
  float rearfactor = incomingBackSpeed > 1500 ? 0.5 : 1; //Rear motor in reverse orientation

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
    throttle = map(throttle, 912, 3060, 1500, incomingModeMin);
    throttlerear = map(throttlerear, 912, 3060, 1500, incomingModeMax);
  }
  else if(!digitalRead(FORWARDS_GEAR_PIN)){
    drivingState = DRIVING_STATE_FORWARDS;
    throttle = map(throttle, 912, 3060, 1500, incomingModeMax);
    throttlerear = map(throttlerear, 912, 3060, 1500, incomingModeMin);
  }
  else
  {
    drivingState = DRIVING_STATE_NEUTRAL;
    throttle = 1500;
    throttlerear = 1500;
  }

//  Serial.println("Incoming ModeMax: " + String(incomingModeMax));

  //Allow local Car control if the remote is connected,and not an emergency and not any setting other than idle
  if((remoteConnected) && (!incomingEmergency) && (incomingFrontSpeed < 1505) && (incomingFrontSpeed > 1480)){
    //only allow local car control when remote is connected for safety
    // Set speed and steering for Front and Rear motors
//    Serial.println("Throttle: " + String(throttle));
//    Serial.println("Rear Throttle: " + String(throttlerear));
    factor = throttle < 1500 ? 0.5 : 1;
    rearfactor = throttlerear > 1500 ? 0.5 : 1;
    SetSpeed(throttle, factor); //channel values between 1000 and 2000 for min and max
    SetSpeedRear(throttlerear, rearfactor); //channel values between 1000 and 2000 for min and max
    SetSteerRear(count);
    //reverse count for front motors as they are facing the other way
    count = map(count, 1000, 2000, 2000, 1000);
    SetSteer(count);   
  }
  else //only let the parent RC control the car
  {
//    Serial.println("Incoming FrontSpeed: " + String(incomingFrontSpeed));
//    Serial.println("Incoming BackSpeed: " + String(incomingBackSpeed));
    // Set speed and steering for Front and Rear motors
    SetSpeed(incomingFrontSpeed, factor); //channel values between 1000 and 2000 for min and max
    SetSteer(incomingFrontSteer);
    SetSpeedRear(incomingBackSpeed, rearfactor); //channel values between 1000 and 2000 for min and max
    SetSteerRear(incomingBackSteer);
  }
 
  // Set values to send
  motorReadings.batteryvoltage = batteryVoltageRear;
  motorReadings.frontleftspeed = frontleftrealspeed;
  motorReadings.frontrightspeed = frontrightrealspeed;
  motorReadings.backleftspeed = backleftrealspeed;
  motorReadings.backrightspeed = backrightrealspeed;
    
  // Reply only when you receive data for Front motors
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
      
      // Send message via ESP-NOW
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &motorReadings, sizeof(motorReadings));
       
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
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
