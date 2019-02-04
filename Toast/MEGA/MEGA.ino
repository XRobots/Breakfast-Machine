#include <Wire.h>                     // for LCD
#include <LiquidCrystal_I2C.h>        // for LCD

#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

#include <Servo.h>

Servo servoTimer;
Servo servoDoor;
Servo servoFork1;
Servo servoFork2;

int servoTimerPos;
int servoDoorPos;
int servoFork1Pos;
int servoFork2Pos;

int motorPos;
int motorPos1;

int servoTimerPos_filtered;
int servoDoorPos_filtered;
int servoFork1Pos_filtered;
int servoFork2Pos_filtered;

int motorPos_filtered;

double Pk1 = 15;  //speed it gets there
double Ik1 = 0;
double Dk1 = 0;
double Setpoint1, Input1, Output1, Output1a;    // PID variables wheel

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

int pot;

// Set the pins on the I2C chip used for LCD connections (Some LCD use Address 0x27 and others use 0x3F):
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address (addr, en, rw, rs, d4, d5, d6, d7, backlight, polarity) 

unsigned long previousMillis = 0;
const long interval = 20;

unsigned long previousOvenMillis = 0;

int red;        // buttons
int white;

int state = 0;        // stage of the automation



void setup() {

    servoTimer.attach(10);
    servoDoor.attach(11);
    servoFork1.attach(44);
    servoFork2.attach(46);

    pinMode(22, INPUT_PULLUP);    // red
    pinMode(24, INPUT_PULLUP);    // white

    pinMode(A0, INPUT);           // analog pot

    pinMode(2, OUTPUT);           // pwm
    pinMode(3, OUTPUT);           // pwm

    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-255, 255);
    PID1.SetSampleTime(20);
    
    Serial.begin(115200); // start serial port   
    
    lcd.begin(20,4);   // Initialize the lcd for 20 chars 4 lines, turn on backlight
  
    // NOTE: Cursor Position: (CHAR, LINE) starts at 0  
    lcd.setCursor(0,0);
    lcd.print("BreakFast Machine   ");
    lcd.setCursor(0,1);
    lcd.print("Ready for TOAST ?!  ");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,3);
    lcd.print("Press red to start  ");

    servoTimerPos = 0;             // 0 off - 180 on           
    servoDoorPos = 180;               // 83 open - 160 closed
    servoFork1Pos = 151;               // left fork. Lower number toward oven
    servoFork2Pos = 20;              // right fork. Higher number toward oven
    Setpoint1 = 125;          // 125 - 250. high number is closer to oven

    Serial.println("Ready!");
    Serial.println("'a' to open door");
    Serial.println("'b' to close door");
    Serial.println("'c' to move tray back");
    Serial.println("'d' to move tray forward");
    Serial.println("'e' to move forks down");
    Serial.println("'f' to move forks mid");
    Serial.println("'g' to move forks tip");
    Serial.println("'h' to set timer");
    Serial.println("'i' to release timer");
} 


void loop() { 

   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {  // start timed event for read and send
      previousMillis = currentMillis; 

         if (Serial.available()) {
                char c = Serial.read();

                if (c == 'a') {
                    servoDoorPos = 87;
                }
                else if (c == 'b') {
                    servoDoorPos = 180;
                }
                else if (c == 'f') {
                    servoFork1Pos = 36;
                    servoFork2Pos = 134;
                }
                else if (c == 'e') {
                    servoFork1Pos = 19;
                    servoFork2Pos = 150;
                }
                else if (c == 'g') {
                    servoFork1Pos = 151;
                    servoFork2Pos = 20;
                }
                else if (c == 'h') {
                    servoTimerPos = 180;                    
                }
                else if (c == 'i') {
                    servoTimerPos = 0;                    
                }
                else if (c == 'c') {
                    Setpoint1 = 125;          // 125 - 250. high number is closer to oven                   
                }
                else if (c == 'd') {
                    Setpoint1 = 250;          // 125 - 250. high number is closer to oven                 
                }

         }

         red = digitalRead(22);
         white = digitalRead(24);

         // state machine stuff in here

         lcd.setCursor(0,2);
         lcd.print((currentMillis - previousOvenMillis)/1000);

         if (state == 0 && red == 0) {
             servoTimerPos = 180;
             Setpoint1 = 125;          // 125 - 250. high number is closer to oven 
             state = 1;
         }

         else if (state == 1 && currentMillis - previousOvenMillis > 1000) {
             servoFork1Pos = 151;
             servoFork2Pos = 20;             
             lcd.setCursor(0,3);
             lcd.print("Preheating oven     ");
             previousOvenMillis = currentMillis;
             state = 2;
         }

         else if (state == 2 && currentMillis - previousOvenMillis > 30000){     // preheat timer 1 min
              servoDoorPos = 87;
              previousOvenMillis = currentMillis;
              state = 3;
         }

         else if (state == 3 && currentMillis - previousOvenMillis > 1000){
              servoFork1Pos = 36;               // forks ready for food
              servoFork2Pos = 134;
              lcd.setCursor(0,3);
              lcd.print("Insert food         ");
              state = 4;
         }
         
         else if (state == 4 && red == 0) {
              Setpoint1 = 250;          // move tray into oven
              state = 5;
              previousOvenMillis = currentMillis;
         }

         else if (state == 5 && currentMillis - previousOvenMillis > 2000) {
              servoFork1Pos = 19;      // lower tray
              servoFork2Pos = 150;
              state = 6;
              previousOvenMillis = currentMillis;
         }

         else if (state == 6 && currentMillis - previousOvenMillis > 2000) {
              Setpoint1 = 125;          // move carriage away from oven
              state = 7;
              previousOvenMillis = currentMillis;
         }

         else if (state == 7 && currentMillis - previousOvenMillis > 2000) {
              servoFork1Pos = 151;      // raise forks away from door.
              servoFork2Pos = 20;
              state = 8;
              previousOvenMillis = currentMillis;
         }

         else if (state == 8 && currentMillis - previousOvenMillis > 1000) {
              servoDoorPos = 180;       // close door
              state = 9;
              previousOvenMillis = currentMillis;
         }

         else if (state == 9 && currentMillis - previousOvenMillis > 1000) {
              servoTimerPos = 0;        // release timer
              state = 10;
              previousOvenMillis = currentMillis;
         }

         else if (state == 10 && currentMillis - previousOvenMillis > 30000) {
              servoDoorPos = 87;        // open door
              state = 11;
              previousOvenMillis = currentMillis;
         }

         else if (state == 11 && currentMillis - previousOvenMillis > 2000) {
              servoFork1Pos = 19;       // forks ready to pick up tray
              servoFork2Pos = 150;
              state = 12;
              previousOvenMillis = currentMillis;
         }

         else if (state == 12 && currentMillis - previousOvenMillis > 2000) {
              Setpoint1 = 250;          // move carrieage into oven
              state = 13;
              previousOvenMillis = currentMillis;
         }

         else if (state == 13 && currentMillis - previousOvenMillis > 2000) {
              servoFork1Pos = 36;               // raise forks
              servoFork2Pos = 134;
              state = 14;
              previousOvenMillis = currentMillis;
         }

         else if (state == 14 && currentMillis - previousOvenMillis > 3000) {
              Setpoint1 = 125;          // move carriage away from oven
              state = 15;
              previousOvenMillis = currentMillis;
         }

         else if (state == 15 && currentMillis - previousOvenMillis > 7000) {
              servoFork1Pos = 151;      // throw out food
              servoFork2Pos = 20;
              state = 16;
              previousOvenMillis = currentMillis;
         }

         else if (state == 16 && currentMillis - previousOvenMillis > 3000) {
              servoDoorPos = 180;       // close door
              state = 0;
              lcd.setCursor(0,3);
              lcd.print("Press red to start  ");
              previousOvenMillis = currentMillis;
         }

         












         // end  of state machine

          pot = analogRead(A0);

          servoTimerPos_filtered = filter(servoTimerPos, servoTimerPos_filtered, 10);
          servoDoorPos_filtered = filter(servoDoorPos, servoDoorPos_filtered, 20);
          servoFork1Pos_filtered = filter(servoFork1Pos, servoFork1Pos_filtered, 10);
          servoFork2Pos_filtered = filter(servoFork2Pos, servoFork2Pos_filtered, 10);

          servoTimer.write(servoTimerPos_filtered);
          servoDoor.write(servoDoorPos_filtered);
          servoFork1.write(servoFork1Pos_filtered);
          servoFork2.write(servoFork2Pos_filtered);

          Input1 = pot;
          
          PID1.Compute();

          motorPos_filtered = filter(Output1, motorPos_filtered, 15);

          // drive motor
          
          if (motorPos_filtered < -100)                                       // decide which way to turn the motor
          {
            motorPos1 = abs(motorPos_filtered);
            analogWrite(2, motorPos1);                           // set PWM pins 
            analogWrite(3, 0);
          }
          else if (motorPos_filtered > 100)                                  // decide which way to turn the motor
          { 
            motorPos1 = abs(motorPos_filtered);
            analogWrite(3, motorPos1);  
            analogWrite(2, 0);
          } 
          else
          {
            analogWrite(2, 0);  
            analogWrite(3, 0);
          }  

          

          

   }  // end of timed event
   
}   // end of main loop



//***************filter  motions*****************

double filter(double lengthOrig, double currentValue, int filter) {
  double lengthFiltered =  (lengthOrig + (currentValue * filter)) / (filter + 1);
  return lengthFiltered;  
}
