/*************************************************** 
TOAZ v0.1a

TOAZ-LEGS_Config:
  Servo 0 (In-A) :  Servo 2 (Ou-A)
  Servo 3 (In-B) :  Servo 1 (Ou-B)
  Servo 4 (In-C) :  Servo 6 (Ou-C)
  Servo 7 (In-D) :  Servo 5 (Ou-D)

****************************************************/

#include <Wire.h>
#include <Keyboard.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

//Name the robot TOAZ
String BROADCAST_NAME = "TOAZ";
String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);
 
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
 
 
// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
 
// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
 
// the packet buffer
extern uint8_t packetbuffer[];
 
char buf[60];

// uses the default address 0x40 for the Servo Wing
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (100)
// define BNO055 sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// define battery object
#define VBATPIN A7
float measuredvbat = analogRead(VBATPIN);

// Define the MIN and MAX rang of the Servo
#define SERVOMIN 900 // Absolute Minimum PWM 860 @ 250 Hz
#define SERVOMAX 2100 // Absolute Maximum PWM 2150 @ 250Hz
#define SERVOMID 1505 // Servo Centering in theory

// Servo tirm 
int t_00 = 0;     //Center
int t_01 = -30;   //Leg
int t_02 = -80;   //Leg
int t_03 = -20;   //Center
int t_04 = 12;     //Center
int t_05 = -30;   //Leg
int t_06 = -50;     //Leg
int t_07 = 10;    //Center

// Servo # Counter
uint8_t servo = 0;
uint16_t pulselen = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("TOAZ Initializing...");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Serial.println("Orientation Sensor Ready"); Serial.println("");

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);

  pwm.begin();
  pwm.setPWMFreq(250);  // Servos run at ~60 Hz updates

  yield();

  /* Initialize BLE module */
  BLEsetup();

  delay(1000);

  init_Pos();
  delay(1000);

  TOAZ_Pos();
  delay(2000);

  init_Pos();
  delay(2000);
}

void loop() {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();
  
  measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print(" VBat: " ); Serial.println(measuredvbat);
  Serial.println("");

  
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;
  
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
    switch (buttnum) {
      case 1:
        Serial.println("[Initital Pos]");
        init_Pos();
        delay(250);
        break;
      case 2:
        Serial.println("[TOAZ Dance]");
        TOAZ_DAN();
        delay(250);
        break;
      case 3:
        Serial.println("[TOAZ Mode]");
        TOAZ_Pos();
        delay(250);
        break;
      case 4:
        Serial.println("[Rotate Mode]");
        break;
      case 5:
        Serial.println("[Forward]");
        Walk_ctl(0);
        break;
      case 6:
        Serial.println("[Backward]");
        Walk_ctl(3);
        break;
      case 7:
        Serial.println("[Left]");
        Walk_ctl(1);
        break;
      case 8:
        Serial.println("[Right]");
        Walk_ctl(2);
        break;
      default:
        init_Pos();
        delay(250);
      break;
    }
  }
}

// Walking with control
void Walk_ctl(uint8_t direction){

  uint16_t MOV_I = 100;
  uint16_t MOV_II = 120;
  uint16_t MOV_III = 200;
  uint16_t MOV_IV = 10;

  switch (direction) {
    case 1: //Left  
    
        pwm.setPWM(servo+0, 0, SERVOMID+t_00+0);
        pwm.setPWM(servo+3, 0, SERVOMID+t_03-MOV_II);
        pwm.setPWM(servo+4, 0, SERVOMID+t_04-0);
        pwm.setPWM(servo+7, 0, SERVOMID+t_07+MOV_II); 
        
        pwm.setPWM(servo+2, 0, SERVOMID+t_02-200+MOV_III);
        pwm.setPWM(servo+1, 0, SERVOMID+t_01+200+MOV_IV);
        pwm.setPWM(servo+6, 0, SERVOMID+t_06+200+MOV_IV);
        pwm.setPWM(servo+5, 0, SERVOMID+t_05-200-MOV_I);
 
        delay(320);
      
        pwm.setPWM(servo+0, 0, SERVOMID+t_00-MOV_II);
        pwm.setPWM(servo+3, 0, SERVOMID+t_03-0);
        pwm.setPWM(servo+4, 0, SERVOMID+t_04+MOV_II);
        pwm.setPWM(servo+7, 0, SERVOMID+t_07+0); 
        
        pwm.setPWM(servo+2, 0, SERVOMID+t_02-200-MOV_IV);
        pwm.setPWM(servo+1, 0, SERVOMID+t_01+200+MOV_I);
        pwm.setPWM(servo+6, 0, SERVOMID+t_06+200-MOV_III);
        pwm.setPWM(servo+5, 0, SERVOMID+t_05-200-MOV_IV);

        delay(320); 
      break;
    case 2: //Right

        pwm.setPWM(servo+0, 0, SERVOMID+t_00+MOV_II);
        pwm.setPWM(servo+3, 0, SERVOMID+t_03-0);
        pwm.setPWM(servo+4, 0, SERVOMID+t_04-MOV_II);
        pwm.setPWM(servo+7, 0, SERVOMID+t_07+0); 

        pwm.setPWM(servo+2, 0, SERVOMID+t_02-200-MOV_IV);
        pwm.setPWM(servo+1, 0, SERVOMID+t_01+200-MOV_III);
        pwm.setPWM(servo+6, 0, SERVOMID+t_06+200+MOV_I);
        pwm.setPWM(servo+5, 0, SERVOMID+t_05-200-MOV_IV);
      
        delay(280);

        pwm.setPWM(servo+0, 0, SERVOMID+t_00+0);
        pwm.setPWM(servo+3, 0, SERVOMID+t_03+MOV_II);
        pwm.setPWM(servo+4, 0, SERVOMID+t_04-0);
        pwm.setPWM(servo+7, 0, SERVOMID+t_07-MOV_II); 

        pwm.setPWM(servo+2, 0, SERVOMID+t_02-200-MOV_I);
        pwm.setPWM(servo+1, 0, SERVOMID+t_01+200+MOV_IV);
        pwm.setPWM(servo+6, 0, SERVOMID+t_06+200+MOV_IV);
        pwm.setPWM(servo+5, 0, SERVOMID+t_05-200+MOV_III);  
      
        delay(280);
      break;
    case 3: //Backward
        pwm.setPWM(servo+0, 0, SERVOMID+t_00+MOV_II);
        pwm.setPWM(servo+3, 0, SERVOMID+t_03-0);
        pwm.setPWM(servo+4, 0, SERVOMID+t_04-MOV_II);
        pwm.setPWM(servo+7, 0, SERVOMID+t_07+0); 

        pwm.setPWM(servo+2, 0, SERVOMID+t_02-200-MOV_IV);
        pwm.setPWM(servo+1, 0, SERVOMID+t_01+200-MOV_III);
        pwm.setPWM(servo+6, 0, SERVOMID+t_06+200+MOV_I);
        pwm.setPWM(servo+5, 0, SERVOMID+t_05-200-MOV_IV);
      
        delay(280);
    
        pwm.setPWM(servo+0, 0, SERVOMID+t_00+0);
        pwm.setPWM(servo+3, 0, SERVOMID+t_03-MOV_II);
        pwm.setPWM(servo+4, 0, SERVOMID+t_04-0);
        pwm.setPWM(servo+7, 0, SERVOMID+t_07+MOV_II); 

        pwm.setPWM(servo+2, 0, SERVOMID+t_02-200+MOV_III);
        pwm.setPWM(servo+1, 0, SERVOMID+t_01+200-MOV_IV);
        pwm.setPWM(servo+6, 0, SERVOMID+t_06+200-MOV_IV);
        pwm.setPWM(servo+5, 0, SERVOMID+t_05-200-MOV_I);  
      
        delay(280);
      break;
    default: //Foreward
        pwm.setPWM(servo+0, 0, SERVOMID+t_00+0);
        pwm.setPWM(servo+3, 0, SERVOMID+t_03+MOV_II);
        pwm.setPWM(servo+4, 0, SERVOMID+t_04-0);
        pwm.setPWM(servo+7, 0, SERVOMID+t_07-MOV_II); 
        
        pwm.setPWM(servo+2, 0, SERVOMID+t_02-200-MOV_I);
        pwm.setPWM(servo+1, 0, SERVOMID+t_01+200-MOV_IV);
        pwm.setPWM(servo+6, 0, SERVOMID+t_06+200-MOV_IV);
        pwm.setPWM(servo+5, 0, SERVOMID+t_05-200+MOV_III);
      
        delay(320);
      
        pwm.setPWM(servo+0, 0, SERVOMID+t_00-MOV_II);
        pwm.setPWM(servo+3, 0, SERVOMID+t_03-0);
        pwm.setPWM(servo+4, 0, SERVOMID+t_04+MOV_II);
        pwm.setPWM(servo+7, 0, SERVOMID+t_07+0); 
        
        pwm.setPWM(servo+2, 0, SERVOMID+t_02-200+MOV_IV);
        pwm.setPWM(servo+1, 0, SERVOMID+t_01+200+MOV_I);
        pwm.setPWM(servo+6, 0, SERVOMID+t_06+200-MOV_III);
        pwm.setPWM(servo+5, 0, SERVOMID+t_05-200+MOV_IV);
      
        delay(320);
      break;
  }
}

// Walking Style I
void Walk_I(){

  uint16_t MOV_I = 140;
  uint16_t MOV_II = 100;
  uint16_t MOV_III = 200;
  uint16_t MOV_IV = 60;

  pwm.setPWM(servo+0, 0, SERVOMID+t_00+0);
  pwm.setPWM(servo+3, 0, SERVOMID+t_03+MOV_II);
  pwm.setPWM(servo+4, 0, SERVOMID+t_04-0);
  pwm.setPWM(servo+7, 0, SERVOMID+t_07-MOV_II); 
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-200-MOV_I);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+200-MOV_IV);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+200+MOV_IV);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-200+MOV_III);

  delay(280);

  pwm.setPWM(servo+0, 0, SERVOMID+t_00-MOV_II);
  pwm.setPWM(servo+3, 0, SERVOMID+t_03-0);
  pwm.setPWM(servo+4, 0, SERVOMID+t_04+MOV_II);
  pwm.setPWM(servo+7, 0, SERVOMID+t_07+0); 
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-200+MOV_IV);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+200+MOV_I);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+200-MOV_III);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-200-MOV_IV);

  delay(280);

}

// Walking Style II
void Walk_II(){

  uint16_t MOV_I = 140;
  uint16_t MOV_II = 100;
  uint16_t MOV_III = 200;

  pwm.setPWM(servo+0, 0, SERVOMID+t_00+0);
  pwm.setPWM(servo+3, 0, SERVOMID+t_03+MOV_II);
  pwm.setPWM(servo+4, 0, SERVOMID+t_04-0);
  pwm.setPWM(servo+7, 0, SERVOMID+t_07+MOV_II); 
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-200-MOV_I);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+200+0);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+200-0);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-200+MOV_III);

  delay(300);

  pwm.setPWM(servo+0, 0, SERVOMID+t_00-MOV_II);
  pwm.setPWM(servo+3, 0, SERVOMID+t_03-0);
  pwm.setPWM(servo+4, 0, SERVOMID+t_04-MOV_II);
  pwm.setPWM(servo+7, 0, SERVOMID+t_07+0); 
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-200-0);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+200+MOV_I);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+200-MOV_III);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-200+0);

  delay(300);

}

// Do a TOAZY DANCE
void TOAZ_DAN(){
  
  uint16_t DANpp = 100;
  uint16_t DBNpp = 60;
  uint16_t LEGpp = 120;
  uint16_t LFGpp = 160;
  
  init_Pos();
  delay(10);

  pwm.setPWM(servo+0, 0, SERVOMID+t_00+LEGpp);
  pwm.setPWM(servo+3, 0, SERVOMID+t_03-LEGpp);
  pwm.setPWM(servo+4, 0, SERVOMID+t_04-LFGpp);
  pwm.setPWM(servo+7, 0, SERVOMID+t_07+LFGpp); 
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-200-DBNpp);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+200+DBNpp);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+200-DANpp);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-200+DANpp);

  delay(500);

  init_Pos();
  delay(10);

  pwm.setPWM(servo+0, 0, SERVOMID+t_00-LEGpp);
  pwm.setPWM(servo+3, 0, SERVOMID+t_03-LFGpp);
  pwm.setPWM(servo+4, 0, SERVOMID+t_04+LFGpp);
  pwm.setPWM(servo+7, 0, SERVOMID+t_07+LEGpp); 
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-200-DANpp);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+200-DBNpp);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+200+DBNpp);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-200+DANpp);

  delay(500);
  
  init_Pos();
  delay(10);

  pwm.setPWM(servo+0, 0, SERVOMID+t_00-LFGpp);
  pwm.setPWM(servo+3, 0, SERVOMID+t_03+LFGpp);
  pwm.setPWM(servo+4, 0, SERVOMID+t_04+LEGpp);
  pwm.setPWM(servo+7, 0, SERVOMID+t_07-LEGpp); 
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-200+DANpp);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+200-DANpp);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+200+DBNpp);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-200-DBNpp);

  delay(500);

  init_Pos();
  delay(10);

  pwm.setPWM(servo+0, 0, SERVOMID+t_00+LFGpp);
  pwm.setPWM(servo+3, 0, SERVOMID+t_03+LEGpp);
  pwm.setPWM(servo+4, 0, SERVOMID+t_04-LEGpp);
  pwm.setPWM(servo+7, 0, SERVOMID+t_07-LFGpp); 
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-200+DBNpp);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+200+DANpp);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+200-DANpp);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-200-DBNpp);

  delay(500);

}

// Set LEGS to initial position
void init_Pos(){

  pwm.setPWM(servo+0, 0, SERVOMID+t_00);
  pwm.setPWM(servo+3, 0, SERVOMID+t_03);
  pwm.setPWM(servo+4, 0, SERVOMID+t_04);
  pwm.setPWM(servo+7, 0, SERVOMID+t_07); 
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-200);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+200);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+200);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-200);
}

// Set LEGS to TOAZ positon
void TOAZ_Pos(){

  pwm.setPWM(servo+2, 0, SERVOMID+t_02-410);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+410);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+410);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-410);

  for (uint16_t Zpulselen = SERVOMID; Zpulselen < 2014; Zpulselen+=5){
    pwm.setPWM(servo+0, 0, Zpulselen+t_00);
    pwm.setPWM(servo+3, 0, Zpulselen+t_03);
    pwm.setPWM(servo+4, 0, Zpulselen+t_04);
    pwm.setPWM(servo+7, 0, Zpulselen+t_07); 
    delay(2);
  }
}

// Set LEGS to Arch Position
void Arch_Pos(){

  uint16_t Apulselen = SERVOMID;
  uint16_t Bpulselen = SERVOMID;

  boolean D1 = false;

  while(D1 == false) {
    if (Apulselen == SERVOMIN && Bpulselen == SERVOMAX){
      D1 = true;  
    }

    pwm.setPWM(servo+0, 0, Apulselen+t_00+10);
    pwm.setPWM(servo+3, 0, Bpulselen+t_03+10);
    pwm.setPWM(servo+4, 0, Apulselen+t_04+25);
    pwm.setPWM(servo+7, 0, Bpulselen+t_07); 

    delay(2);

    if(Apulselen > SERVOMIN){
      Apulselen-=5;
    }
    if(Bpulselen < SERVOMAX){
      Bpulselen+=5;
    }
  }
 // Instant move
 // pwm.setPWM(servo+0, 0, SERVOMIN+t_00+10);
 // pwm.setPWM(servo+3, 0, SERVOMAX+t_03+10);
 // pwm.setPWM(servo+4, 0, SERVOMIN+t_04+25);
 // pwm.setPWM(servo+7, 0, SERVOMAX+t_07); 

  uint8_t Arch_pulse = 305;
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-304);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+306);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-304);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+309);

  
}

// Set LEGS to positions adjust for Arch Position
void postArch_Pos(){

  pwm.setPWM(servo+0, 0, SERVOMID+t_00);
  pwm.setPWM(servo+3, 0, SERVOMID+t_03);
  pwm.setPWM(servo+4, 0, SERVOMID+t_04);
  pwm.setPWM(servo+7, 0, SERVOMID+t_07); 
  
  pwm.setPWM(servo+2, 0, SERVOMID+t_02-304);
  pwm.setPWM(servo+1, 0, SERVOMID+t_01+306);
  pwm.setPWM(servo+6, 0, SERVOMID+t_06+304);
  pwm.setPWM(servo+5, 0, SERVOMID+t_05-309);
}

//Displays some basic information on this sensor from the unified
//sensor API sensor_t type (see Adafruit_Sensor for more information)

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

//Display some basic info about the sensor status

void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

//Display sensor calibration status
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

//Setup BLE
void BLEsetup(){
  Serial.print(F("Initialising the Bluefruit LE module: "));
 
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
 
  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }
 
  //Convert the name change command to a char array
  BROADCAST_CMD.toCharArray(buf, 60);
 
  //Change the broadcast device name here!
  if(ble.sendCommandCheckOK(buf)){
    Serial.println("name changed");
  }
  delay(250);
 
  //reset to take effect
  if(ble.sendCommandCheckOK("ATZ")){
    Serial.println("resetting");
  }
  delay(250);
 
  //Confirm name change
  ble.sendCommandCheckOK("AT+GAPDEVNAME");
 
  /* Disable command echo from Bluefruit */
  ble.echo(false);
 
  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
 
  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();
 
  ble.verbose(false);  // debug info is a little annoying after this point!
 
  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }
 
  Serial.println(F("*****************"));
 
  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
 
  Serial.println(F("*****************"));
}
