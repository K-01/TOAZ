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
#include <Adafruit_PWMServoDriver.h>

// uses the default address 0x40 for the Servo Wing
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define the MIN and MAX rang of the Servo

// MIN 206
// MAX 494
#define SERVOMIN 900 // Absolute Minimum PWM 860 @ 250 Hz
#define SERVOMAX 2100 // Absolute Maximum PWM 2150 @ 250Hz

#define SERVOMID 1505


// Servo tirm 
int t_00 = 0;     //Center
int t_01 = -30;   //Leg
int t_02 = -80;   //Leg
int t_03 = -20;   //Center
int t_04 = 5;     //Center
int t_05 = -30;   //Leg
int t_06 = -50;     //Leg
int t_07 = 10;    //Center

// Servo # Counter
uint8_t servo = 0;
uint16_t pulselen = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("TOAZ test Begin!");

  pwm.begin();
  pwm.setPWMFreq(250);  // Servos run at ~60 Hz updates

  yield();

  delay(5000);

  init_Pos();
  delay(3000);

  TOAZ_Pos();
  delay(4000);

  init_Pos();
  delay(3000);

  TOAZ_DAN();
  TOAZ_DAN();
  TOAZ_DAN();
  TOAZ_DAN();
  delay(200);
}

void loop() {

  Walk_I();
  
//  TOAZ_DAN();
//  delay(150);

 // TOAZ_Pos();
 // delay(2500);

 // init_Pos();
 // delay(2500);

 // postArch_Pos();
 // delay(150);

 // Arch_Pos();
 // delay(2500);

 // postArch_Pos();
 // delay(150);

}
// Walking That I
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

// Walking That I
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

