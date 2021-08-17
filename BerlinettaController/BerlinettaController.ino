/*
  ESPNow two way communication example
  30:AE:A4:96:BA:70 - Car ESP32 MAC Address
  30:AE:A4:96:FF:74 - Controller ESP32 MAC Address
*/

#include <esp_now.h>
#include <WiFi.h>
#include <driver/adc.h>
#include <Adafruit_GFX.h>           //Core Graphics Library
#include <Adafruit_ST7735.h>        //Hardware-Specific library for ST7735
#include <EEPROM.h>               
//#include <SdFat.h>                  //DF card & FAT filesystem library
//#include <Adafruit_ImageReader.h>   //Image reading functions


//-------------------- Display + SD Setup Items ---------------------------//
//Define pins for the display 
#define TFT_CS              5
#define TFT_RST             -1
#define TFT_DC              0
#define TFT_LIT             15

//Define pin for SD Card and settings locations on EEPROM
#define EEPROM_SIZE                           1
#define AUDIO_SETTING_LOCATION                0
#define DISPLAY_BACKLIGHT_SETTING_LOCATION    1
#define SD_CS                                 2

#define DISPLAY_UPDATE_INTERVAL 100

//create the TFT object
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST); //Hardware

//Creade SD reading object
//SdFat SDF;
//Adafruit_ImageReader reader(SDF);

//Pin for Controller battery measurement
uint8_t controllerbatterypin = 33;
uint16_t controllerbatteryvoltage = 4200;

#define SOUND_PWM_CHANNEL   0
#define SOUND_RESOLUTION    8 // 8 bit resolution
#define SOUND_ON            (1<<(SOUND_RESOLUTION-1)) // 50% duty cycle
#define SOUND_OFF           0                         // 0% duty cycle

#define SNAIL_MODE          1
#define SLOW_MODE           2
#define NORMAL_MODE         3
#define SPORTS_MODE         4
#define LIMP_MODE           5   //Used when we are between almost empty and UVLO, to limp home on one set of motors and low max speed

//Timer setup
volatile int interruptCounter;
int totalInterruptCounter;
int soundTime;
 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}

//Pin Declarations
int freq1 = 4000; //for buzzer (peak resonance)
uint8_t piezoPin = 16;
bool soundState = false;
uint8_t leftstickforwardbackPin = A0; // potentiometer wiper (middle terminal) connected to analog pin 3 outside leads to ground and +3V3
uint8_t leftsticksidetosidePin = A3;
uint8_t rightstickforwardbackPin = A6;
uint8_t rightsticksidetosidePin = A7;
uint8_t leftstickbuttonPin = A4; //Joystick button Treat as digital input set high and if it comes low its triggered
uint8_t rightstickbuttonPin = 25; //Digital
uint8_t emergencystopbuttonPin = A17; //Emergency stop pin

bool emergency = false;
uint8_t carMode = SNAIL_MODE;
uint16_t ModeMax = 1625;
uint16_t ModeMin = 1375;


// REPLACE WITH THE MAC Address of your receiver - this will be the Car MAC
uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x96, 0xBA, 0x70};

// Define variables to store steer and speed values to be sent
uint16_t frontspeed = 1500;
uint16_t backspeed = 1500;
uint16_t frontsteer = 1500;
uint16_t backsteer = 1500;

int frontspeedOffset = 6;
int frontsteerOffset = -17;
int backspeedOffset = -6;
int backsteerOffset = 17;

// Define variables to store incoming battery voltage, and real speed values for all four motors
uint16_t incomingBattV = 4100;
uint16_t incomingFrontLeftRealSpeed = 0;
uint16_t incomingFrontRightRealSpeed = 0;
uint16_t incomingBackLeftRealSpeed = 0;
uint16_t incomingBackRightRealSpeed = 0;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_send_message {
    uint16_t frontmotorsteer;
    uint16_t frontmotorspeed;
    uint16_t backmotorsteer;
    uint16_t backmotorspeed;
    bool emergency;
    uint16_t ModeMax;
    uint16_t ModeMin;
} struct_send_message;

typedef struct struct_recieve_message {
    uint16_t batteryvoltage;
    uint16_t frontleftspeed;
    uint16_t frontrightspeed;
    uint16_t backleftspeed;
    uint16_t backrightspeed;
} struct_recieve_message;

// Create a struct_send_message called speedSteerSettings to hold Speed and Steer Settings to be sent
struct_send_message speedSteerSettings;

// Create a struct_recieve_message to hold incoming motor readings
struct_recieve_message incomingMotorReadings;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingMotorReadings, incomingData, sizeof(incomingMotorReadings));
//  Serial.print("Bytes received: ");
//  Serial.println(len);
  incomingBattV = incomingMotorReadings.batteryvoltage;
  incomingFrontLeftRealSpeed = incomingMotorReadings.frontleftspeed;
  incomingFrontRightRealSpeed = incomingMotorReadings.frontrightspeed;
  incomingBackLeftRealSpeed = incomingMotorReadings.backleftspeed;
  incomingBackRightRealSpeed = incomingMotorReadings.backrightspeed;
  
//  Serial.print("Car Battery Voltage: ");          // debug value
//  Serial.println(incomingBattV);          // debug value
//  Serial.print("Car Front Left Speed: ");          // debug value
//  Serial.println(incomingFrontLeftRealSpeed);          // debug value
//  Serial.print("Car Front Right Speed: ");          // debug value
//  Serial.println(incomingFrontRightRealSpeed);          // debug value
//  Serial.print("Car Back Left Speed: ");          // debug value
//  Serial.println(incomingBackLeftRealSpeed);          // debug value
//  Serial.print("Car Back Right Speed: ");          // debug value
//  Serial.println(incomingBackRightRealSpeed);          // debug value
}

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
   // Init Serial Monitor
  Serial.begin(19200);

  pinMode(leftstickbuttonPin, INPUT_PULLUP);    // sets the joystick button as input //Normally pulled high 
  pinMode(rightstickbuttonPin, INPUT_PULLUP);    // sets the joystick button as input //Normally pulled high
  pinMode(emergencystopbuttonPin, INPUT_PULLUP);    // sets the emergency stop button as input //Normally pulls low

  //Decide what mode we are in - default is Snail mode
  // left pressed only is SLOW_MODE
  // right pressed only is NORMAL_MODE
  // both pressed together is SPORTS_MODE
  if((!digitalRead(leftstickbuttonPin)) && (digitalRead(rightstickbuttonPin)))
  {
    changeMode(SLOW_MODE);
  }
  else if((digitalRead(leftstickbuttonPin)) && (!digitalRead(rightstickbuttonPin)))
  {
    changeMode(NORMAL_MODE);
  }
  else if((!digitalRead(leftstickbuttonPin)) && (!digitalRead(rightstickbuttonPin)))
  {
    changeMode(SPORTS_MODE);
  }

  //Setup Display
  pinMode(TFT_LIT, OUTPUT);
  digitalWrite(TFT_LIT, HIGH);

  tft.initR(INITR_MINI160x80);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextWrap(false);
  tft.setCursor(0,0);
  tft.setTextSize(2);
  tft.println("Yellow Racer!");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0,20);
  tft.println("Batt:  v|   v");
  tft.setCursor(0,40);
  tft.println("SpeedL ----");
  tft.setCursor(0,60);
  tft.print("SpeedR ----");

  //ImageReturnCode stat; //Status from image-reading functions

  //stat = reader.drawBMP("/Lightboy.bmp", tft, 0, 0);

  //digitalWrite(TFT_LIT, LOW);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_MAX);
 
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

  ledcSetup(SOUND_PWM_CHANNEL+0, freq1, SOUND_RESOLUTION);  // Set up PWM channel
  ledcAttachPin(piezoPin, SOUND_PWM_CHANNEL+0);        // Attach channel to pin

  if((carMode==SNAIL_MODE) || (carMode==SLOW_MODE) || (carMode==NORMAL_MODE) || (carMode==SPORTS_MODE))
  {
    ledcWrite(SOUND_PWM_CHANNEL, SOUND_ON);
    delay(100);
    ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
    delay(100);
  }
  if((carMode==SLOW_MODE) || (carMode==NORMAL_MODE) || (carMode==SPORTS_MODE))
  {
    ledcWrite(SOUND_PWM_CHANNEL, SOUND_ON);
    delay(100);
    ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
    delay(100);
  }
  if((carMode==NORMAL_MODE) || (carMode==SPORTS_MODE))
  {
    ledcWrite(SOUND_PWM_CHANNEL, SOUND_ON);
    delay(100);
    ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
    delay(100);
  }
  if((carMode==SPORTS_MODE)){
    ledcWrite(SOUND_PWM_CHANNEL, SOUND_ON);
    delay(200);
    ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
  }

  //digitalWrite(TFT_LIT, LOW); //turn off display
  //tft.fillScreen(ST77XX_BLACK);

  //Setup our timer
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 500000, true); //500us per increment
  timerAlarmEnable(timer);
}
 
void loop() {

  //First thing is to check do we have an emergency stop condition 
  if(digitalRead(emergencystopbuttonPin))
  {
    //Stop the car and break from this loop, probably need to check that the speed is at 0 or low before we renable power etc.
    frontspeed = 1500;
    backspeed = 1500;
    frontsteer = 1500;
    backsteer = 1500;
    ModeMax = 1500;
    ModeMin = 1500;

    emergency = true;
  }
  
  if (interruptCounter > 0) {
 
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
 
    totalInterruptCounter++;
 
    Serial.print("An interrupt as occurred. Total number: ");
    Serial.println(totalInterruptCounter);
 
  }
  
  getReadings();
 
  // Set values to send
  speedSteerSettings.emergency = emergency;
  speedSteerSettings.frontmotorsteer = frontsteer;
  speedSteerSettings.frontmotorspeed = frontspeed;
  speedSteerSettings.backmotorsteer = backsteer;
  speedSteerSettings.backmotorspeed = backspeed;
  speedSteerSettings.ModeMax = ModeMax;
  speedSteerSettings.ModeMin = ModeMin;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &speedSteerSettings, sizeof(speedSteerSettings));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  delay(50);
}

void getReadings(){
  //This is where we will read the emergency stop button, 
  // the analog joysticks for their positions, 
  // map the joystick readings to speed and steering values,
  // and provide audio feedback if things are not good
  // min is 1000 and max is 2000
  //left stick speed is reversed i.e. we get 0 for full speed ahead and 4096 for full reverse - ok because we use right stick for speed
  //right stick steer is reversed i.e. we get 0 for full right and 4096 for full left - ok because we use left stick for steering

  if(digitalRead(emergencystopbuttonPin))
  {
    //Stop the car and break from this loop, probably need to check that the speed is at 0 or low before we renable power etc.
    frontspeed = 1500;
    backspeed = 1500;
    frontsteer = 1500;
    backsteer = 1500;

    emergency = true;
    
    tft.fillScreen(ST77XX_RED);
  }

  if(!emergency)
  {
    //read the joysticks  
    //measure and map to our min and max speed settings
    frontspeed = map(adc1_get_raw(ADC1_CHANNEL_6), 0, 4095, ModeMin, ModeMax); 
    frontsteer = map(adc1_get_raw(ADC1_CHANNEL_3), 0, 4095, 2000, 1000);
    backspeed = map(adc1_get_raw(ADC1_CHANNEL_6), 0, 4095, ModeMax, ModeMin); //Motors are in reverse orientation so mapped reverse here!
    backsteer = map(adc1_get_raw(ADC1_CHANNEL_3), 0, 4095, 1000, 2000); //motors are in reverse orientation
    controllerbatteryvoltage = adc1_get_raw(ADC1_CHANNEL_5);

    Serial.println(controllerbatteryvoltage);

    //Our offsets
    frontspeed = constrain(frontspeed+frontspeedOffset,1000,2000);
    frontsteer = constrain(frontsteer+frontsteerOffset,1000,2000);
    backspeed = constrain(backspeed+backspeedOffset,1000,2000);
    backsteer = constrain(backsteer+backsteerOffset,1000,2000);
    
    //do any adjustments based on battery or real speed
    if((incomingBattV < 3300) && (incomingBattV > 3000))
    {
      //We are at less than 33.0V lets make each beep 2 seconds apart and reduce to a limp mode
      if(totalInterruptCounter>soundTime)
      {
        if(!soundState){
          ledcWrite(SOUND_PWM_CHANNEL, SOUND_ON);
          soundTime = 1;
          soundState = true;
          totalInterruptCounter = 0;
        }
        else
        {
          ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
          soundTime = 5;
          soundState = false;
          totalInterruptCounter = 0;
        }
      }
//      changeMode(LIMP_MODE);
    }  
    else if(incomingBattV < 3500)
    {
      //we are at less than 35.0V lets make a gentle beep every 10 seconds
      if(totalInterruptCounter>soundTime)
      {
        if(!soundState){
          ledcWrite(SOUND_PWM_CHANNEL, SOUND_ON);
          soundTime = 1;
          soundState = true;
          totalInterruptCounter = 0;
        }
        else
        {
          ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
          soundTime = 20;
          soundState = false;
          totalInterruptCounter = 0;
        }
      }
    }
    else
    {
      ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
    }
  
//    //Debug the results
//    Serial.print("Front Motor Speed: ");          // debug value
//    Serial.println(frontspeed);          // debug value
//    Serial.print("Back Motor Speed: ");          // debug value
//    Serial.println(backspeed);          // debug value
//    Serial.print("Front Motor Steer: ");          // debug value
//    Serial.println(frontsteer);          // debug value
//    Serial.print("Back Motor Steer: ");          // debug value
//    Serial.println(backsteer);          // debug value

    //Update the display with Car Battery Voltage, Speed and with Controller Battery voltage and speed steer settings
    tft.fillRect(60,20,20,15,ST77XX_BLACK);
    tft.setCursor(60,20);
    tft.println(incomingBattV/100);
    tft.fillRect(107,20,35,15,ST77XX_BLACK);
    tft.setCursor(107,20);
    tft.println(float(map(controllerbatteryvoltage, 0, 2323, 0, 4200))/1000,1); //(controllerbatteryvoltage)/4095)*6.6)
    tft.fillRect(80,40,60,15,ST77XX_BLACK);
    tft.setCursor(80,40);
    tft.println(incomingFrontLeftRealSpeed);
    tft.fillRect(80,60,60,15,ST77XX_BLACK);
    tft.setCursor(80,60);
    tft.println(incomingFrontRightRealSpeed);
  }
}
