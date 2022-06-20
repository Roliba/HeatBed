//             ---Have in mind---
// *This code is a first version for the reflow hot plate project.
// *Tutorial here: http://electronoobs.com/eng_arduino_tut161.php
// *Please be careful working with "High Voltage" and double check everything. 
// *Only use this project with supervision, never leave it plugged.
// *You can use this code at your own risk. I don't offer any guarantee that you 
// *will get the same results as I did and you might have to adjsut some PID values */


#include <thermistor.h>             //Downlaod it here: http://electronoobs.com/eng_arduino_thermistor.php
thermistor therm1(A0,0);            //The 3950 Thermistor conencted on A0 
//LCD config
#include <Wire.h>                   //Included by Arduino IDE
#include <LiquidCrystal.h>
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Inputs and Outputs
int EncoderA = 7;
int EncoderB = 8;
int but_1 = 9;
int SSR = 10;
int buzzer = 6;
int Thermistor_PIN = A0;

//Variables
unsigned int millis_before, millis_before_2;    //We use these to create the loop refresh rate
unsigned int millis_now = 0;
float refresh_rate = 300;                       //LCD refresh rate. You can change this if you want
float pid_refresh_rate  = 50;                   //PID Refresh rate
float seconds = 0;                              //Variable used to store the elapsed time                   
int running_mode = 0;                           //We store the running selected mode here
int selected_mode = 0;                          //Selected mode for the menu
int max_modes = 3;                              //For now, we only work with 1 mode...
float temperature = 0;                          //Store the temperature value here
float preheat_setoint = 140;                    //Mode 1 preheat ramp value is 140-150ºC
float soak_setoint = 150;                       //Mode 1 soak is 150ºC for a few seconds
float reflow_setpoint = 200;                    //Mode 1 reflow peak is 200ºC
float temp_setpoint = 0;                        //Used for PID control
float MAX_temp_setpoint = 450;                        //Used for PID control
float pwm_value = 255;                          //The SSR is OFF with HIGH, so 255 PWM would turn OFF the SSR
float MIN_PID_VALUE = 0;
float MAX_PID_VALUE = 180;                      //Max PID value. You can change this. 
float cooldown_temp = 40;                       //When is ok to touch the plate
bool aState;
bool aLastState;  
/////////////////////PID VARIABLES///////////////////////
/////////////////////////////////////////////////////////
float Kp = 2;               //Mine was 2
float Ki = 0.0025;          //Mine was 0.0025
float Kd = 9;               //Mine was 9
float PID_Output = 0;
float PID_P, PID_I, PID_D;
float PID_ERROR, PREV_ERROR;
/////////////////////////////////////////////////////////

void setup() {
  //Define the pins as outputs or inputs
  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, HIGH);        //Make sure we start with the SSR OFF (is off with HIGH)
  pinMode(buzzer, OUTPUT); 
  digitalWrite(buzzer, LOW);  
  pinMode(EncoderA, INPUT_PULLUP);
  pinMode(EncoderB, INPUT_PULLUP);
  pinMode(but_1, INPUT_PULLUP);
  pinMode(Thermistor_PIN, INPUT);

  lcd.begin(20, 4);
  lcd.print("Hello!");

  Serial.begin(9600);
  //tone(buzzer, 1800, 200);     
  millis_before = millis();
  millis_now = millis();
  aLastState = digitalRead(EncoderA);   
}

void loop() {
  ////////input/////////////
  if(digitalRead(but_1)==0){    //PB
    delay(500);
    if(running_mode==0) running_mode=1;
    else running_mode=0;
  }
  if(running_mode == 1){
    aState = digitalRead(EncoderA); // RotaryEncoder
    if (aState != aLastState){     
      if (digitalRead(EncoderB) != aState) { 
        temp_setpoint -=0.5;
        if(temp_setpoint<=0){
          temp_setpoint=0;
        }
      } else {
        temp_setpoint +=0.5;
        if(temp_setpoint>=MAX_temp_setpoint){
          temp_setpoint=MAX_temp_setpoint;
        }
      }
    } 
    aLastState = aState; 
  }
  ///////running////////////
  millis_now = millis();
  if(millis_now - millis_before_2 > pid_refresh_rate){    //Refresh rate of the PID
    millis_before_2 = millis(); 
    temperature = therm1.analog2temp();
    if(running_mode==0){                   // OFF mode
      digitalWrite(SSR, LOW);            //With HIGH the SSR is OFF
      temp_setpoint = 0;
    }
    if(running_mode == 1){                                //ON Mode
      // if(temp_setpoint < preheat_setoint){
      //   temp_setpoint = seconds*1.666;                    //Reach 150ºC till 90s (150/90=1.666)
      // }  
      
      //Calculate PID
      PID_ERROR = temp_setpoint - temperature;
      PID_P = Kp*PID_ERROR;
      PID_I = PID_I+(Ki*PID_ERROR);      
      PID_D = Kd * (PID_ERROR-PREV_ERROR);
      PID_Output = PID_P + PID_I + PID_D;
      //Define maximun PID values
      if(PID_Output > MAX_PID_VALUE){
        PID_Output = MAX_PID_VALUE;
      }
      else if (PID_Output < MIN_PID_VALUE){
        PID_Output = MIN_PID_VALUE;
      }
      //Since the SSR is ON with LOW, we invert the pwm singal
      pwm_value = PID_Output;//255 - PID_Output;
      
      analogWrite(SSR,pwm_value);           //We change the Duty Cycle applied to the SSR
      
      PREV_ERROR = PID_ERROR;
    }
  }
  /////////////display///////////////////
  millis_now = millis();
  if(millis_now - millis_before > refresh_rate){          //Refresh rate of prntiong on the LCD
    millis_before = millis();   
    seconds = seconds + (refresh_rate/1000);              //We count time in seconds
    if(running_mode == 0){             // OFF mode
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("T: ");
      lcd.print(temperature);
      lcd.print(" / ");
      lcd.print(temp_setpoint);
      lcd.setCursor(0,1);      
      lcd.print("OFF MODE"); 
    }
    else if(running_mode == 1){       // ON mode
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("T: ");
      lcd.print(temperature);
      lcd.print(" / ");
      lcd.print(temp_setpoint);
      lcd.setCursor(0,1);      
      lcd.print("ON MODE"); 
    }//End of running_mode == 1
  }
}