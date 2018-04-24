/*  Author: Jan Schöfer
    Changelog:

    Version 0.00a: 24.04.2018
    derrived from EnvironmentSensorBME280 Version 0.20a: 12.02.2018
      
    Setup:
    Arduino UNO
    LCD Keypad Shield and 
    DS18B20 connected to A1(Yellow), 5V (red), GND (black)
*/


#define VERSION "0.00a"
#define SENSOR_TYPE "DS18B20"

#define SERIAL_INTERVAL 1000 //interval for sending serial [ms]
#define LCD_INTERVAL 1000 //interval for refreshing LCD [ms] 
#define BME_INTERVAL 2000 //interval for reading BME280 (may collide with sensor standby)
#define BLINK_INTERVAL 1000 //interval for blinking (LED, LCD heart)

//I2C library
#include <Wire.h>

//for DS18B20
#define SENSOR_PIN A1
/*TODO
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(SENSOR_PIN);
DallasTemperature myDS18B20(&oneWire);
*/

//for LCD
#include <Wire.h>
#include <LiquidCrystal.h>

// pins for LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

long lastDisplay = 0;
long lastSerial = 0;
long lastSensorRead = 0;
long lastBlink = 0;

float temperature = 0.0;

bool blinkState = false;

// stuff for LCD Keypad Shield
int btnID     = 0;
int btnAnalogValue  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//additional LCD characters
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0}; //heart symbol
uint8_t degree[8] = {0x2, 0x5, 0x2, 0x0, 0x0, 0x0, 0x0}; //degree symbol
uint8_t arrowup[8] = {0x4, 0xe, 0x15, 0x4, 0x4, 0x4, 0x4}; //arrow-up symbol

// read buttons
int read_buttons()
{
  btnAnalogValue = analogRead(0);      // read the value from analog button pin
  // analog button values (right:0; up:132; down:306; left:480; select:721; none:1023)

  if (btnAnalogValue > 1000) return btnNONE;
  if (btnAnalogValue < 50)   return btnRIGHT;
  if (btnAnalogValue < 250)  return btnUP;
  if (btnAnalogValue < 400)  return btnDOWN;
  if (btnAnalogValue < 650)  return btnLEFT;
  if (btnAnalogValue < 850)  return btnSELECT;

  return btnNONE;  // when all others fail, return this...
}

void displayHandler()
{
  if (lastDisplay < (millis() - LCD_INTERVAL)) {
    lastDisplay = millis();
    btnID = read_buttons();         // read the buttons
    switch (btnID) {               // depending on which button was pushed, we perform an action
        
        case btnUP:
          {
            //show up time
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Up-Time:");
            lcd.setCursor(0, 1);
            lcd.print(String(millis()/1000) + "s ~ ");
            lcd.print(String(float(millis())/3600000.0, 3));
            lcd.print("h");
            break;
          }
        case btnRIGHT:
          {
            //break;
          }
        case btnLEFT:
          {
            //break;
          }
        case btnDOWN:
          {
            //break;
          }
        case btnSELECT:
          {
            //break;
          }
        case btnNONE:
          {
            // standard state
            lcd.clear();
            lcd.setCursor(0, 0);
            if (!blinkState) {
              lcd.write(byte(0)); //heart
              //lcd.print("X ");
            } else {
              lcd.print("  "); //delete heart
            }

            lcd.setCursor(1, 0);
            lcd.print(" " + String(temperature));
            lcd.write(byte(1)); //degree symbol
            lcd.print("C");
          }
          break;
    }
  }
}

void serialWriteHandler()
{
  if (lastSerial < (millis() - SERIAL_INTERVAL)) {
    lastSerial = millis();
    Serial.println(String(millis()/1000) + "; " + String(temperature) + "; ");
  }
}

void serialReadHandler()
{
  if (Serial.available() > 0) {
    String tmp = Serial.readString();
  }
}

void setup()
{
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  //load lcd
  //lcd.init();
  //lcd.backlight();
  lcd.clear();
  lcd.createChar(0, heart);
  lcd.createChar(1, degree);
  lcd.createChar(2, arrowup);
  lcd.home();

  pinMode(LED_BUILTIN, OUTPUT);

  lcd.begin(16, 2);              // initialise LCD
  lcd.setCursor(0, 0);           // set cursor to beginning
  lcd.print("Sensor: ");      // print sensor
  lcd.print(SENSOR_TYPE);
  lcd.setCursor(0, 1);           // set cursor to second line
  lcd.print("EnviroSens");         // print a simple message
  lcd.print(VERSION);

  delay(2000); // wait so someone can read the display
}

void loop()
{
  //blink LED and Display
  if (lastBlink < (millis() - BLINK_INTERVAL)) {
    lastBlink = millis();
    if (!blinkState) {
      digitalWrite(LED_BUILTIN, HIGH);
      blinkState = true;
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      blinkState = false;
    }
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // read Sensor
  if (lastSensorRead < (millis() - BME_INTERVAL)) {
    lastSensorRead = millis();
    temperature = analogRead(SENSOR_PIN);
  }

  // serialReadHandler();
  serialWriteHandler();
  displayHandler();
  


/*
    switch (lcd_key)               // depending on which button was pushed, we perform an action
    {
      case btnRIGHT:
        {
          lcd.print("RIGHT ");
          break;
        }
      case btnLEFT:
        {
          lcd.print("LEFT   ");
          break;
        }
      case btnUP:
        {
          lcd.print("UP    ");
          break;
        }
      case btnDOWN:
        {
          lcd.print("DOWN  ");
          break;
        }
      case btnSELECT:
        {
          lcd.print("SELECT");
          break;
        }
      case btnNONE:
        {
          lcd.print("NONE  ");
          break;
        }
    }
    */
}
