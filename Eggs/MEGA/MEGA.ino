
#include <OneWire.h>                  // for temp densor
#include <DallasTemperature.h>        // for temp densor

#include <Wire.h>                     // for LCD
#include <LiquidCrystal_I2C.h>        // for LCD

#include <Adafruit_TiCoServo.h>       // Adafruit servo library

#define SERVO_MIN 800 // 1 ms pulse
#define SERVO_MAX 2300 // 2 ms pulse

Adafruit_TiCoServo servo1;
Adafruit_TiCoServo servo2;

// Set the pins on the I2C chip used for LCD connections (Some LCD use Address 0x27 and others use 0x3F):
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address (addr, en, rw, rs, d4, d5, d6, d7, backlight, polarity)

// Data wire of temp sensor is plugged into pin 2 on the Arduino 
#define ONE_WIRE_BUS 8 

// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

unsigned long previousMillis = 0;
const long interval = 20;

int yellow;           // buttons
int red;
int green1;
int green2;

int state = 0;        // stage of the automation

long bookmarktime;          // the time at the start
long timeleft;              // the time left as the couinter counts down

void setup() {

    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);

    pinMode(37, OUTPUT);
    pinMode(39, OUTPUT);

    digitalWrite(37, LOW);        // low for up  
    digitalWrite(39, HIGH);       // low for down

    servo1.attach(44, SERVO_MIN, SERVO_MAX);
    servo2.attach(45, SERVO_MIN, SERVO_MAX);

    servo1.write(8);   // sive
    servo2.write(180);    // base
    
    Serial.begin(9600); // start serial port     
    
    sensors.begin(); // Start up the temp sensor library 

    lcd.begin(20,4);   // Initialize the lcd for 20 chars 4 lines, turn on backlight
  
    // NOTE: Cursor Position: (CHAR, LINE) starts at 0  
    lcd.setCursor(0,0);
    lcd.print("BreakFast Machine   ");
    lcd.setCursor(0,1);
    lcd.print("Ready for EGGS ??!! ");
    lcd.setCursor(0,2);
    lcd.print("Water temp:");
    lcd.setCursor(0,3);
    lcd.print("Yellow to start     ");
} 


void loop() { 

   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {  // start timed event for read and send
      previousMillis = currentMillis;
    
          // call sensors.requestTemperatures() to issue a global temperature 
          // request to all devices on the bus         
          //Serial.print(" Requesting temperatures..."); 
          sensors.requestTemperatures(); // Send the command to get temperature readings 
          //Serial.println("DONE"); 
          //Serial.print("Temperature is: "); 
          //Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"?  
          // You can have more than one DS18B20 on the same bus.  
          // 0 refers to the first IC on the wire 
          lcd.setCursor(12,2);
          lcd.print(sensors.getTempCByIndex(0)); 

          yellow = digitalRead(2);
          red = digitalRead(3);
          green1 = digitalRead(4);
          green2 = digitalRead(5);

          if (state == 0 && yellow == 0) {
              servo1.write(8);   // sive
              digitalWrite(37, LOW);        // low for up  
              digitalWrite(39, HIGH);       // low for down
              lcd.setCursor(0,3);
              lcd.print("Waiting for water   ");
              state = 1;
          }

          else if (state == 1 && sensors.getTempCByIndex(0) > 90) {
              lcd.setCursor(0,3);
              lcd.print("Water at temperature");
              //move robot arm stuff here
              servo1.write(8);   // sive
              servo2.write(0);    // base
              delay(2000);
              digitalWrite(37, HIGH);        // low for up  
              digitalWrite(39, LOW);       // low for down
              delay(2000);
              state = 2;
          }

          else if (state == 2) {
            bookmarktime = millis()  +  360000;                     // set the clock for 5 minutes in the future in millisseconds (1000 x 60 seconds x 6 minutes)
            state = 3;
          }

          else if (state == 3) {
              timeleft = bookmarktime - millis();                   // work out the time left on every cycle based on the actual time that's expired according to the Arduino internal clock
              lcd.setCursor(0,3);
              lcd.print("Time left:          ");
              lcd.setCursor(11,3);
              lcd.print(timeleft);
                if (timeleft <= 0) {
                    state = 4;
              }
          }

          else if (state == 4) {
              lcd.setCursor(0,3);
              lcd.print("Eggs are ready!!    ");
              // move robot arm stuff
              digitalWrite(37, LOW);        // low for up  
              digitalWrite(39, HIGH);       // low for down
              delay(2000);
              servo1.write(8);   // sive
              servo2.write(180);    // base
              delay(1000);
              servo1.write(120);   // sive
              state = 5;             
          }

          else if (state == 5) {
              lcd.setCursor(0,3);
              lcd.print("Press red for reset");
              if (red == 0) {
                state = 6;
                servo1.write(8);   // sive
              } 
          }

          else if (state == 6) {
              lcd.setCursor(0,3);
              lcd.print("Yellow to start     ");
              state = 0;
          }         

          

   }  // end of timed event
   
}   // end of main loop
