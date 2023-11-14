#include <Arduino.h>
#include <Wire.h>
#include <Ultrasonic.h>
#include <XBOXRECV.h>
//#include <XBOXUSB.h>
#include <SPI.h>
#include <EEPROM.h>

USB Usb;
XBOXRECV Xbox(&Usb);
//XBOXUSB Xbox(&Usb);



//relay Pneumatic 
#define Pneumatic_Load_Ring     PG0  // relay 1 โหลดห่วง 
#define Pneumatic_shoot_Ring    PE1  // relay 2 ดันห่วงเพื่อยิง
//#define Pneumatic_3             PD5  // relay 3 
//#define Pneumatic_4             PG2  // relay 4


//Motor DriveTrain Pin
#define PWM_FL   PE11  //OK
#define PWM_FR   PE13  //OK
#define PWM_BL   PE9   //OK
#define PWM_BR   PE14  //OK

#define DIR_FL   PF15  //OK
#define DIR_FR   PF12  //OK
#define DIR_BL   PF11   //OK
#define DIR_BR   PF13  //OK
//Motor ShootUp and Down
#define DIRA_SH   PD0  //OK
#define DIRB_SH   PG1  //OK
//Motor Lift
#define PWM_LIFT_L  PF8   //OK  
#define DIR_LIFT_LA PF1   //OK
#define DIR_LIFT_LB PF2   //OK

#define DIR_LIFT_RA PF0   //OK
#define DIR_LIFT_RB PD1   //OK
#define PWM_LIFT_R  PF9   //OK


#define LM_DOWN_LIFT_LEFT       PG7    //  
#define LM_UP_LIFT_LEFT         PD10    //
#define LM_DOWN_LIFT_RIGHT      PG5    //
#define LM_UP_LIFT_RIGHT        PG8    //

#define PWM_SPEED       PE6

#define RAMP_SET   6500 //เมือ PWM น้อยกว่า 50 ไม่ให้มอเตอร์หมุน
#define RAMP_SET_Z 4500 //เมือ PWM น้อยกว่า 50 ไม่ให้มอเตอร์หมุน
#define SLOW  9500      //สำหรับควบคุมจอยโดยใช้ลูกศร

#define MAX_PWM    32767

void MotorPinBegin();
void RelayPinBegin();

void motor_fl(int pwm);
void motor_fr(int pwm);
void motor_bl(int pwm);
void motor_br(int pwm);
void motorStop();

void forWard(int pwm);
void backWard(int pwm);
void left_F(int pwm);
void right_F(int pwm);
void left_B(int pwm);
void right_B(int pwm);
void left(int pwm);
void right(int pwm);

void ModeSet();

uint64_t pretime_ = 0;
uint64_t time_ = 0;


int speed1 = 0;
int speed2 = 0;
int speed3 = 0;
int speed4 = 0;

int16_t setSpeed1_ =  0;
int16_t setSpeed2_ =  0;
int16_t setSpeed3_ =  0;
int16_t setSpeed4_ =  0;

int setSpeed  = 0;

int eeAddressSpeed1 = 0;   
int eeAddressSpeed2 = 10;   
int eeAddressSpeed3 = 20;   
int eeAddressSpeed4 = 30;   



void setup() {
  Serial.begin (115200);
  SPI.begin();                                                                // Begins the SPI commnuication
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.usingInterrupt(1);  
  pinMode(D10,OUTPUT);        //SS
  digitalWrite(D10,HIGH);     //SS
  pinMode(LM_DOWN_LIFT_LEFT,INPUT_PULLUP);
  pinMode(LM_UP_LIFT_LEFT,INPUT_PULLUP);
  pinMode(LM_DOWN_LIFT_RIGHT,INPUT_PULLUP);
  pinMode(LM_UP_LIFT_RIGHT,INPUT_PULLUP);
 // analogWriteResolution(10);
  //analogWriteFrequency(20000);
  pinMode(A0,INPUT_ANALOG);
  pinMode(A1,INPUT_ANALOG);
  pinMode(A2,INPUT_ANALOG);
  pinMode(A3,INPUT_ANALOG);

  MotorPinBegin();
  RelayPinBegin();
  //ImuBegin();
  //calculate_IMU_error();
  pinMode(USER_BTN,INPUT);
  pinMode(LED_BLUE,OUTPUT);
  pinMode(LED_RED,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  pinMode(PWM_SPEED,OUTPUT);
  analogWrite(PWM_SPEED,0);


  Serial.println("Start Begin");
  if (Usb.Init() == -1) {
    Serial.println(F("\r\nOSC did not start"));
    //hile (1); //halt
  }
  else {Serial.println(F("\r\nXBOX USB Library Started"));}

while(millis() < 1000){
  if(( millis() - pretime_ )>30){
    pretime_ = millis();
    digitalToggle(LED_BUILTIN);
    digitalToggle(LED_BLUE);
    digitalToggle(LED_RED);
  }
}

ModeSet();

EEPROM.get(eeAddressSpeed1,setSpeed1_);
EEPROM.get(eeAddressSpeed2,setSpeed2_);
EEPROM.get(eeAddressSpeed3,setSpeed3_);
EEPROM.get(eeAddressSpeed4,setSpeed4_);

speed1 = map(setSpeed1_,0,1000,10000,32767);
speed2 = map(setSpeed2_,0,1000,10000,32767);
speed3 = map(setSpeed3_,0,1000,10000,32767);
speed4 = map(setSpeed4_,0,1000,10000,32767);

}

int mode_speed = 0;
void loop() {

  if(( millis() - pretime_ )>250){
  pretime_ = millis();
  digitalToggle(LED_BUILTIN);
  digitalToggle(LED_BLUE);
  digitalToggle(LED_RED);
}

  Usb.Task();
    if (Xbox.XboxReceiverConnected) {
      // Relay For control Pneumatic
      if (Xbox.getButtonPress(A)){
        digitalWrite(Pneumatic_Load_Ring,LOW);
      }
      else if(Xbox.getButtonPress(Y)){
        digitalWrite(Pneumatic_shoot_Ring,LOW);
      }
      else{
        digitalWrite(Pneumatic_shoot_Ring,HIGH);
        digitalWrite(Pneumatic_Load_Ring,HIGH);
      }

      if(mode_speed == 0){
        analogWrite(PWM_SPEED,0);
      }
      else if (mode_speed == 1){
        analogWrite(PWM_SPEED,speed1);
      }  
      else if (mode_speed == 2) {
        analogWrite(PWM_SPEED,speed2);
      }
      else if (mode_speed == 3) {
        analogWrite(PWM_SPEED,speed3);
      }
      else if (mode_speed == 4){
        analogWrite(PWM_SPEED,speed4);
      }

      if(Xbox.getButtonClick(X)){
        mode_speed = 0;
      }
      else if (Xbox.getButtonClick(B)){
         mode_speed = 1;
      }  
      else if (Xbox.getButtonClick(START)) {
         mode_speed = 2;
      }
      else if (Xbox.getButtonClick(BACK)) {
         mode_speed = 3;
      }
      else if (Xbox.getButtonClick(L3)){
         mode_speed = 4;
      }
     
      // Control Motor ShootUp or ShootDown
      if (Xbox.getButtonPress(L1)){
        digitalWrite(DIRA_SH,LOW);
        digitalWrite(DIRB_SH,HIGH);
      }
      else if (Xbox.getButtonPress(R1)){
        digitalWrite(DIRA_SH,HIGH);
        digitalWrite(DIRB_SH,LOW);
      }else{
        digitalWrite(DIRA_SH,LOW);
        digitalWrite(DIRB_SH,LOW);
      }


    if(Xbox.getButtonPress(R2)){

        int pwm_r2 = map(Xbox.getButtonPress(R2),0,255,0,32676);
        
        analogWrite(PWM_LIFT_R,pwm_r2);
        analogWrite(PWM_LIFT_L,pwm_r2);
        if(digitalRead(LM_UP_LIFT_LEFT) ==0){
          digitalWrite(DIR_LIFT_LA,LOW);
        }else{
          digitalWrite(DIR_LIFT_LA,HIGH);
        }
        if(digitalRead(LM_UP_LIFT_RIGHT)==0){
          digitalWrite(DIR_LIFT_RA,LOW);
        }else{
          digitalWrite(DIR_LIFT_RA,HIGH);
        }
        digitalWrite(DIR_LIFT_LB,LOW);
        digitalWrite(DIR_LIFT_RB,LOW);
      }
      else if(Xbox.getButtonPress(L2)){
        int pwm_l2 = map(Xbox.getButtonPress(L2),0,255,0,32676);
        analogWrite(PWM_LIFT_R,pwm_l2);
        analogWrite(PWM_LIFT_L,pwm_l2);
        if(digitalRead(LM_DOWN_LIFT_LEFT)==0){
          digitalWrite(DIR_LIFT_LB,LOW);
        }else{
          digitalWrite(DIR_LIFT_LB,HIGH);
        }

        if(digitalRead(LM_DOWN_LIFT_RIGHT)==0){
          digitalWrite(DIR_LIFT_RB,LOW);
        }else{
          digitalWrite(DIR_LIFT_RB,HIGH);
        }
        digitalWrite(DIR_LIFT_LA,LOW);
        digitalWrite(DIR_LIFT_RA,LOW);
      }
      else{
        digitalWrite(DIR_LIFT_LA,LOW);
        digitalWrite(DIR_LIFT_LB,LOW);
        digitalWrite(DIR_LIFT_RA,LOW);
        digitalWrite(DIR_LIFT_RB,LOW);
        analogWrite(PWM_LIFT_R,0);
        analogWrite(PWM_LIFT_L,0);

      }
    // Drive Train
    if(Xbox.getButtonPress(UP)||Xbox.getButtonPress(DOWN)||Xbox.getButtonPress(LEFT)||Xbox.getButtonPress(RIGHT)){
      int pwmy = Xbox.getAnalogHat(RightHatY);
      int pwmx = Xbox.getAnalogHat(RightHatX);
      //เดินหน้า -32768 to +32767

      if (Xbox.getButtonPress(UP)) {
        if(abs(pwmy)>SLOW){
          forWard(abs(pwmy));
        }else{forWard(SLOW);}
        //Serial.print("UP");
      }
      else if(Xbox.getButtonPress(RIGHT)){
        if(abs(pwmx)>SLOW){
          right(abs(pwmx));
        }else{right(SLOW);
        }
        //Serial.print("RIGHT");
      }
      else if(Xbox.getButtonPress(DOWN)){
        if(abs(pwmy)>SLOW){
          backWard(abs(pwmy));
        }else{backWard(SLOW);}
        //Serial.print("DOWN");
      }
      else if(Xbox.getButtonPress(LEFT)){
        if(abs(pwmx)>SLOW){
          left(abs(pwmx));
        }else{left(SLOW);
        }
        //Serial.print("LEFT");
      }
    }
    else if (abs(Xbox.getAnalogHat(LeftHatX))>RAMP_SET ||abs(Xbox.getAnalogHat(LeftHatY))>RAMP_SET ||abs(Xbox.getAnalogHat(RightHatX))>RAMP_SET_Z  ){
      //-32768 to +327
      
      double x =  Xbox.getAnalogHat(LeftHatX);
      double y =  Xbox.getAnalogHat(LeftHatY);
      double rx = Xbox.getAnalogHat(RightHatX)*0.45;
      if(abs(x)<RAMP_SET)  x = 0;
      if(abs(y)<RAMP_SET)  y = 0;
      if(abs(rx)<RAMP_SET_Z) rx = 0;
      double denominator = abs(y) + abs(x) + abs(rx);
      if(denominator < 32767.0){
        denominator = 32767.0;
      }
      double frontLeftPWM  = (y + x + rx)/denominator;
      double backLeftPWM   = (y - x + rx)/denominator;
      double frontRightPWM = (y -x - rx)/denominator;
      double backRightPWM  = (y + x - rx)/denominator;

      frontLeftPWM  = frontLeftPWM*MAX_PWM;
      backLeftPWM   = backLeftPWM*MAX_PWM;
      frontRightPWM = frontRightPWM*MAX_PWM*0.85;
      backRightPWM  = backRightPWM*MAX_PWM*0.9;


      //FL
      if(abs(frontLeftPWM)<RAMP_SET){
        motor_fl(0);
      }else motor_fl(frontLeftPWM);
      //FR
      if(abs(frontRightPWM)<RAMP_SET){
        motor_fr(0);
      }else motor_fr(frontRightPWM);
      //BL
      if(abs(backLeftPWM)<RAMP_SET){
        motor_bl(0);
      }else motor_bl(backLeftPWM);
      //BR
      if(abs(backRightPWM)<RAMP_SET){
        motor_br(0);
      }else motor_br(backRightPWM);
    }
    else {
      motorStop();    
    }
    
    //////////////////////////////////////////////////////////////////
    if (Xbox.getButtonClick(L3))
      Serial.println(F("L3"));
    if (Xbox.getButtonClick(R3))
      Serial.println(F("R3"));
    if (Xbox.getButtonClick(XBOX)) {
      //Xbox.setLedMode(ROTATING);
      Serial.print(F("Xbox (Battery: "));
      Serial.print(Xbox.getBatteryLevel()); // The battery level in the range 0-3
      Serial.println(F(")"));
    }

  }
  else{
     motorStop();
     Serial.println(" ERR ");
  }
}


void MotorPinBegin(){
  analogWriteResolution(15);    
  analogWriteFrequency(20000);
  pinMode(PWM_FL,OUTPUT);
  pinMode(PWM_FR,OUTPUT);
  pinMode(PWM_BL,OUTPUT);
  pinMode(PWM_BR,OUTPUT);
  analogWrite(PWM_FL,0);
  analogWrite(PWM_FR,0);
  analogWrite(PWM_BL,0);
  analogWrite(PWM_BR,0);

  pinMode(DIR_FL,OUTPUT);
  pinMode(DIR_FR,OUTPUT);
  pinMode(DIR_BL,OUTPUT);
  pinMode(DIR_BR,OUTPUT);
  digitalWrite(DIR_FL,LOW);
  digitalWrite(DIR_FR,LOW);
  digitalWrite(DIR_BL,LOW);
  digitalWrite(DIR_BR,LOW);

  pinMode(DIRA_SH,OUTPUT);
  pinMode(DIRB_SH,OUTPUT);
  digitalWrite(DIRA_SH,LOW);
  digitalWrite(DIRB_SH,LOW);

  pinMode(PWM_LIFT_L,OUTPUT);
  pinMode(DIR_LIFT_LA,OUTPUT);
  pinMode(DIR_LIFT_LB,OUTPUT);
  pinMode(DIR_LIFT_RA,OUTPUT);
  pinMode(DIR_LIFT_RB,OUTPUT);
  pinMode(PWM_LIFT_R,OUTPUT);
  
  analogWrite(PWM_LIFT_L,0);
  digitalWrite(DIR_LIFT_LA,LOW);
  digitalWrite(DIR_LIFT_LB,LOW);
  digitalWrite(DIR_LIFT_RA,LOW);
  digitalWrite(DIR_LIFT_RB,LOW);
  analogWrite(PWM_LIFT_R,0);
}

void RelayPinBegin(){
    pinMode(Pneumatic_Load_Ring,OUTPUT_OPEN_DRAIN);
    pinMode(Pneumatic_shoot_Ring,OUTPUT_OPEN_DRAIN);

    digitalWrite(Pneumatic_shoot_Ring,HIGH);
    digitalWrite(Pneumatic_Load_Ring,HIGH);

}

void motor_fl(int pwm){
  if(pwm>=0){
    digitalWrite(DIR_FL,LOW);
    analogWrite(PWM_FL,pwm);
  }
  else{
    digitalWrite(DIR_FL,HIGH);
    analogWrite(PWM_FL,-pwm);
  }
}

void motor_fr(int pwm){
  if(pwm>=0){
    digitalWrite(DIR_FR,HIGH);
    analogWrite(PWM_FR,pwm);
  }
  else{
    digitalWrite(DIR_FR,LOW);
    analogWrite(PWM_FR,-pwm);
  }
}

void motor_bl(int pwm){
  if(pwm>0){
    digitalWrite(DIR_BL,LOW);
    analogWrite(PWM_BL,pwm);
  }
  else{
    digitalWrite(DIR_BL,HIGH);
    analogWrite(PWM_BL,-pwm);
  }
}

void motor_br(int pwm){
  if(pwm>0){
    digitalWrite(DIR_BR,HIGH);
    analogWrite(PWM_BR,pwm);
  }
  else{
    digitalWrite(DIR_BR,LOW);
    analogWrite(PWM_BR,-pwm);
  }
}

//    =A-----B=  
//     |  =  |
//     |  =  |
//    =C-----D=
void motorStop(){
  analogWrite(PWM_FL,0);
  analogWrite(PWM_BL,0);
  analogWrite(PWM_FR,0);
  analogWrite(PWM_BR,0);
}

//    ↑A-----B↑   
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void forWard(int pwm){
  motor_fl(pwm);
  motor_fr(pwm);
  motor_bl(pwm);
  motor_br(pwm);
}
//    ↑A-----B↑   
//     |  |  |
//     |  ↓  |
//    ↑C-----D↑
void backWard(int pwm){
  motor_fl(-pwm);
  motor_fr(-pwm);
  motor_bl(-pwm);
  motor_br(-pwm);
}

//    ↓A-----B↑   
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void left(int pwm){
  motor_fl(-pwm);
  motor_fr(pwm);
  motor_bl(pwm);
  motor_br(-pwm);
}
//    ↑A-----B↓   
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void right(int pwm){
  motor_fl(pwm);
  motor_fr(-pwm);
  motor_bl(-pwm);
  motor_br(pwm);
}

//    ↑A-----B=   
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void left_F(int pwm){
  motor_fl(pwm);
  motor_fr(0);
  motor_bl(0);
  motor_br(pwm);
}
//    =A-----B↑   
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void right_F(int pwm){
  motor_fl(0);
  motor_fr(pwm);
  motor_bl(pwm);
  motor_br(0);
}
//    ↓A-----B=   
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void left_B(int pwm){
  motor_fl(-pwm);
  motor_fr(0);
  motor_bl(0);
  motor_br(-pwm);
}
//    =A-----B↓   
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void right_B(int pwm){
  motor_fl(0);
  motor_fr(-pwm);
  motor_bl(-pwm);
  motor_br(0);
}



void ModeSet(){
  if(digitalRead(USER_BTN)){
    digitalWrite(LED_BUILTIN,HIGH);
    digitalWrite(LED_RED,HIGH);
    digitalWrite(LED_BLUE,HIGH);
    while(1){
    delay(10);
    Usb.Task();
    if (Xbox.XboxReceiverConnected) {
      
      if(Xbox.getButtonPress(R2)){

        int pwm_r2 = map(Xbox.getButtonPress(R2),0,255,0,32676);
        
        analogWrite(PWM_LIFT_R,pwm_r2);
        analogWrite(PWM_LIFT_L,pwm_r2);
        if(digitalRead(LM_UP_LIFT_LEFT) ==0){
          digitalWrite(DIR_LIFT_LA,LOW);
        }else{
          digitalWrite(DIR_LIFT_LA,HIGH);
        }
        if(digitalRead(LM_UP_LIFT_RIGHT)==0){
          digitalWrite(DIR_LIFT_RA,LOW);
        }else{
          digitalWrite(DIR_LIFT_RA,HIGH);
        }
        digitalWrite(DIR_LIFT_LB,LOW);
        digitalWrite(DIR_LIFT_RB,LOW);
      }
      else if(Xbox.getButtonPress(L2)){
        int pwm_l2 = map(Xbox.getButtonPress(L2),0,255,0,32676);
        analogWrite(PWM_LIFT_R,pwm_l2);
        analogWrite(PWM_LIFT_L,pwm_l2);
        if(digitalRead(LM_DOWN_LIFT_LEFT)==0){
          digitalWrite(DIR_LIFT_LB,LOW);
        }else{
          digitalWrite(DIR_LIFT_LB,HIGH);
        }

        if(digitalRead(LM_DOWN_LIFT_RIGHT)==0){
          digitalWrite(DIR_LIFT_RB,LOW);
        }else{
          digitalWrite(DIR_LIFT_RB,HIGH);
        }
        digitalWrite(DIR_LIFT_LA,LOW);
        digitalWrite(DIR_LIFT_RA,LOW);
      }
      else{
        digitalWrite(DIR_LIFT_LA,LOW);
        digitalWrite(DIR_LIFT_LB,LOW);
        digitalWrite(DIR_LIFT_RA,LOW);
        digitalWrite(DIR_LIFT_RB,LOW);
        analogWrite(PWM_LIFT_R,0);
        analogWrite(PWM_LIFT_L,0);

      }
    // Drive Train
    if(Xbox.getButtonPress(UP)||Xbox.getButtonPress(DOWN)||Xbox.getButtonPress(LEFT)||Xbox.getButtonPress(RIGHT)){
      int pwmy = Xbox.getAnalogHat(RightHatY);
      int pwmx = Xbox.getAnalogHat(RightHatX);
      //เดินหน้า -32768 to +32767

      if (Xbox.getButtonPress(UP)) {
        if(abs(pwmy)>SLOW){
          forWard(abs(pwmy));
        }else{forWard(SLOW);}
        //Serial.print("UP");
      }
      else if(Xbox.getButtonPress(RIGHT)){
        if(abs(pwmx)>SLOW){
          right(abs(pwmx));
        }else{right(SLOW);
        }
        //Serial.print("RIGHT");
      }
      else if(Xbox.getButtonPress(DOWN)){
        if(abs(pwmy)>SLOW){
          backWard(abs(pwmy));
        }else{backWard(SLOW);}
        //Serial.print("DOWN");
      }
      else if(Xbox.getButtonPress(LEFT)){
        if(abs(pwmx)>SLOW){
          left(abs(pwmx));
        }else{left(SLOW);
        }
        //Serial.print("LEFT");
      }
    }
    else if (abs(Xbox.getAnalogHat(LeftHatX))>RAMP_SET ||abs(Xbox.getAnalogHat(LeftHatY))>RAMP_SET ||abs(Xbox.getAnalogHat(RightHatX))>RAMP_SET_Z  ){
      //-32768 to +327
      
      double x =  Xbox.getAnalogHat(LeftHatX);
      double y =  Xbox.getAnalogHat(LeftHatY);
      double rx = Xbox.getAnalogHat(RightHatX)*0.45;
      if(abs(x)<RAMP_SET)  x = 0;
      if(abs(y)<RAMP_SET)  y = 0;
      if(abs(rx)<RAMP_SET_Z) rx = 0;
      double denominator = abs(y) + abs(x) + abs(rx);
      if(denominator < 32767.0){
        denominator = 32767.0;
      }
      double frontLeftPWM  = (y + x + rx)/denominator;
      double backLeftPWM   = (y - x + rx)/denominator;
      double frontRightPWM = (y -x - rx)/denominator;
      double backRightPWM  = (y + x - rx)/denominator;

      frontLeftPWM  = frontLeftPWM*MAX_PWM;
      backLeftPWM   = backLeftPWM*MAX_PWM;
      frontRightPWM = frontRightPWM*MAX_PWM*0.85;
      backRightPWM  = backRightPWM*MAX_PWM*0.9;


      //FL
      if(abs(frontLeftPWM)<RAMP_SET){
        motor_fl(0);
      }else motor_fl(frontLeftPWM);
      //FR
      if(abs(frontRightPWM)<RAMP_SET){
        motor_fr(0);
      }else motor_fr(frontRightPWM);
      //BL
      if(abs(backLeftPWM)<RAMP_SET){
        motor_bl(0);
      }else motor_bl(backLeftPWM);
      //BR
      if(abs(backRightPWM)<RAMP_SET){
        motor_br(0);
      }else motor_br(backRightPWM);
    }
    
    else {
      motorStop();    
    }  

      if(Xbox.getButtonClick(X)) {
        setSpeed           = 0;
        digitalWrite(LED_BUILTIN,HIGH);
        digitalWrite(LED_RED,HIGH);
        digitalWrite(LED_BLUE,HIGH);
      }
      else if (Xbox.getButtonClick(B)){
        setSpeed     = 1;
        digitalWrite(LED_BUILTIN,HIGH);
        digitalWrite(LED_RED,LOW);
        digitalWrite(LED_BLUE,LOW);
      }
      else if(Xbox.getButtonClick(START)){
        setSpeed  = 2;
        digitalWrite(LED_BUILTIN,LOW);
        digitalWrite(LED_RED,HIGH);
        digitalWrite(LED_BLUE,LOW);
      } 
      else if(Xbox.getButtonClick(BACK)){
        setSpeed   = 3;
        digitalWrite(LED_BUILTIN,LOW);
        digitalWrite(LED_RED,LOW);
        digitalWrite(LED_BLUE,HIGH);
      } 
      else if(Xbox.getButtonClick(L3)){
        setSpeed     = 4;
        digitalWrite(LED_BUILTIN,HIGH);
        digitalWrite(LED_RED,LOW);
        digitalWrite(LED_BLUE,HIGH);
      } 
      if (Xbox.getButtonPress(A)){
        digitalWrite(Pneumatic_Load_Ring,LOW);
      }
      else if(Xbox.getButtonPress(Y)){
        digitalWrite(Pneumatic_shoot_Ring,LOW);
      }
      else{
        digitalWrite(Pneumatic_shoot_Ring,HIGH);
        digitalWrite(Pneumatic_Load_Ring,HIGH);
      }
     
    speed1 = map(setSpeed1_,0,1000,10000,32767);
    speed2 = map(setSpeed2_,0,1000,10000,32767);
    speed3 = map(setSpeed3_,0,1000,10000,32767);
    speed4 = map(setSpeed4_,0,1000,10000,32767);
    
    if(setSpeed == 0){
        analogWrite(PWM_SPEED,0);
    }
    if(setSpeed==1){
        analogWrite(PWM_SPEED,speed1);
        if(Xbox.getButtonPress(R1)){
          setSpeed1_ = setSpeed1_+1;
          if(setSpeed1_ >=1000 )setSpeed1_ = 1000;  
        }
        else if(Xbox.getButtonPress(L1)){
          setSpeed1_ = setSpeed1_-1;
          if(setSpeed1_ <= 1 )setSpeed1_ = 1;
        }
          if (Xbox.getButtonClick(R3)) {
            motorStop(); 
            EEPROM.put(eeAddressSpeed1,setSpeed1_);
        }
    }
    else if(setSpeed == 2){
        analogWrite(PWM_SPEED,speed2);        
        if(Xbox.getButtonPress(R1)){
          setSpeed2_ = setSpeed2_+1;
          if(setSpeed2_ >=1000 )setSpeed2_ = 1000;
        }
        else if(Xbox.getButtonPress(L1)){
          setSpeed2_ = setSpeed2_-1;
          if(setSpeed2_ <= 1 )setSpeed2_ = 1;
        }
        if (Xbox.getButtonClick(R3)) {
          motorStop(); 
          EEPROM.put(eeAddressSpeed2,setSpeed2_);
        }
      }
      else if(setSpeed == 3){
        analogWrite(PWM_SPEED,speed3);
        if(Xbox.getButtonPress(R1)){
          setSpeed3_ = setSpeed3_+1;
          if(setSpeed3_ >=1000 )setSpeed3_ = 1000;
        }
        else if(Xbox.getButtonPress(L1)){
          setSpeed3_ = setSpeed3_-1;
          if(setSpeed3_ <= 1 )setSpeed3_ = 1;
        }
        if (Xbox.getButtonClick(R3)) {
          motorStop(); 
          EEPROM.put(eeAddressSpeed3,setSpeed3_);
        }
      }
      else if(setSpeed == 4){
        analogWrite(PWM_SPEED,speed4);
        if(Xbox.getButtonPress(R1)){
          setSpeed4_ = setSpeed4_+1;
          if(setSpeed4_ >=1000 )setSpeed4_ = 1000;
        }
        else if(Xbox.getButtonPress(L1)){
          setSpeed4_ = setSpeed4_-1;
          if(setSpeed4_ <= 1 )setSpeed4_ = 1;
        }
        if (Xbox.getButtonClick(R3)) {
          motorStop(); 
          EEPROM.put(eeAddressSpeed4,setSpeed4_);
        }
      }
    }
  }
}
}