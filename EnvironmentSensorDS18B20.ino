/*  Author: Jan Schöfer
    Changelog:

    Version 0.00a: 26.04.2018
    -tested and working
    -added support for multiple sensors

    Version 0.00a: 26.04.2018
    -tested and working
    -derrived from EnvironmentSensorBME280 Version 0.20a: 12.02.2018
      
    Setup:
    Arduino UNO
    LCD Keypad Shield and 
    DS18B20 connected to A1(Yellow), 5V (red), GND (black)
    4k7 pull-up resistor bewtween A1 and 5V
*/


#define VERSION "0.01a"
#define SENSOR_TYPE "DS18B20"

#define SERIAL_INTERVAL 1000 //interval for sending serial [ms]
#define LCD_INTERVAL 1000 //interval for refreshing LCD [ms] 
#define BME_INTERVAL 2000 //interval for reading BME280 (may collide with sensor standby)
#define BLINK_INTERVAL 1000 //interval for blinking (LED, LCD heart)
#define NO_VALUE_FLOAT 0.00f //dummy value for error data
#define SENSOR_ERROR_STRING "/SENSOR ERROR/" //string to print on error

//I2C library
#include <Wire.h>

//for DS18B20
#include <OneWire.h> 
#include <Dalt_lastemperature.h> 
#define ONE_WIRE_SENSOR_PIN A1
#define DS18B20_resolution 12
DeviceAddress DS18B20_adress;
OneWire oneWireSensor(ONE_WIRE_SENSOR_PIN); 
Dalt_lastemperature myDS18B20(&oneWireSensor); 
#define MAX_SENSORS_DS18B20  5
float temperature[MAX_SENSORS_DS18B20];

//for LCD
#include <Wire.h>
#include <LiquidCrystal.h>

// pins for LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

long t_lastDisplay = 0;
long t_lastSerial = 0;
long t_lastSensorRead = 0;
long t_lastBlink = 0;

bool blinkState = false;
bool sensorError = 0;

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
uint8_t heart[8] = {0x00, 0x0A, 0x1F, 0x1F, 0x0E, 0x04, 0x00}; //heart symbol
uint8_t degree[8] = {0x02, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00}; //degree symbol
uint8_t arrowup[8] = {0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x04}; //arrow-up symbol
//uint8_t skull[8] = {0x0E,0x1F,0x15,0x04,0x0E,0x0E,0x15,0x00}; //skull symbol

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
  if (t_lastDisplay < (millis() - LCD_INTERVAL)) {
    t_lastDisplay = millis();
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
            } else {
              lcd.print("  "); //delete heart
            }

            lcd.setCursor(1, 0);
            lcd.print(" " + String(temperature[0]));
            lcd.print(" " + String(temperature[1]));
            lcd.setCursor(0, 1);
            lcd.print(String(temperature[2]));
            lcd.print(" " + String(temperature[3]));
            lcd.print(" " + String(temperature[4]));
            // lcd.write(byte(1)); //degree symbol
            // lcd.print("C ");
            if (sensorError) {
              lcd.print("ERROR"); //heart
            } else {
              lcd.print(" ");
            }
          }
          break;
    }
  }
}

void serialWriteHandler()
{
  if (t_lastSerial < (millis() - SERIAL_INTERVAL)) {
    t_lastSerial = millis();
    Serial.print(String(millis()/1000)+"; ");
    for(byte i=0 ;i < MAX_SENSORS_DS18B20; i++) {	
      Serial.print(String(temperature[i]) + "; ");
      if (sensorError) {
        Serial.print(SENSOR_ERROR_STRING);
      }
		} 
    Serial.println();
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
  // initialize serial communication at 115200 bits per second
  Serial.begin(115200);

  //load lcd
  //lcd.init();
  //lcd.backlight();
  lcd.clear();
  lcd.createChar(0, heart);
  lcd.createChar(1, degree);
  lcd.createChar(2, arrowup);
  //lcd.createChar(3, skull);
  lcd.home();

  pinMode(LED_BUILTIN, OUTPUT);

  // initialize sensor communication
  myDS18B20.begin();
  for(byte i=0 ;i < myDS18B20.getDeviceCount(); i++) {
    if(myDS18B20.getAddress(DS18B20_adress, i)) {
      myDS18B20.setResolution(DS18B20_adress, DS18B20_resolution);
    }
  }

  lcd.begin(16, 2);              // initialise LCD
  lcd.setCursor(0, 0);           // set cursor to beginning
  lcd.print("Sensor: ");
  lcd.print(SENSOR_TYPE);
  lcd.print(" " + String(myDS18B20.getDeviceCount()));      // print sensor info
  lcd.setCursor(0, 1);           // set cursor to second line
  lcd.print("EnviroSens");
  lcd.print(VERSION);         // print Version

  delay(2000); // wait so someone can read the display
}

void loop()
{
  //blink LED and Display
  if (t_lastBlink < (millis() - BLINK_INTERVAL)) {
    t_lastBlink = millis();
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
  if (t_lastSensorRead < (millis() - BME_INTERVAL)) {
    sensorError = 0;
    t_lastSensorRead = millis();
    myDS18B20.requestTemperatures();
    for(byte i=0 ;i < MAX_SENSORS_DS18B20; i++) {
       if (i < myDS18B20.getDeviceCount()) {	
        temperature[i] = myDS18B20.getTempCByIndex(i);
        if (temperature[i] == DEVICE_DISCONNECTED_C) {
	  	    temperature[i] = NO_VALUE_FLOAT;
		      sensorError = 1;
        }
      }
	}
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
