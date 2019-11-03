 /* tinylab testing code
 * coded by Yasin Kaya (selengalp)
 * 18/05/2016    
*/

// definitions
#define S1_PIN 9
#define S2_PIN 8
#define S3_PIN A5 // LOW
#define S4_PIN A5 // HIGH
#define L1_PIN 13
#define L2_PIN 12
#define L3_PIN 11 
#define L4_PIN 10 
//#define MOTOR_PIN 5
#define POT_PIN A0
#define LDR_PIN A2
#define LM35_PIN A3
//#define BUZZER_PIN A1
#define SD_CS_PIN 4
#define RELAY_PIN A4
//#define ROTARY_BUTTON A5 

// libraries
//#include "Wire.h"
#include <Time.h>
#include <TimeLib.h>
#include <LiquidTWI2.h>
//#include <RotaryEncoder.h>
#include <DS1307RTC.h>
#include <SdFat.h>
#include "LedControl.h"
//#include <extEEPROM.h>
// Radio
#include <SPI.h>
//#include <RF24.h>

// instances
LiquidTWI2 lcd(0x20);
//RotaryEncoder encoder(6, 7);
tmElements_t tm;
SdFat SD;
LedControl lc=LedControl(10,12,11,1);
//RF24 radio(8,9);

//variables
uint8_t test_selector = 0; // begin test statement
boolean c_leds = 0;
boolean c_buzz = 0;
uint16_t c_7seg = 0;
boolean c_sd = 0;
boolean c_eeprom = 0;
boolean c_esp = 0;
boolean c_xbee = 0;
uint8_t c_nrf = 0;
boolean c_nrf_setup = 0;
boolean c_relay = 0;
uint16_t timer = 0;
uint16_t tap_lenght = 0;
uint8_t data_xbee = 0;

float tempc; //variable to store temperature in degree Celsius
float tempf; //variable to store temperature in Fahreinheit
float vout; //temporary variable to hold sensor reading

boolean S1_clicked = false;
boolean S2_clicked = false;
boolean S3_clicked = false;
boolean S4_clicked = false;
int8_t rotary_pos = 0;
uint16_t pot_value = 0;
uint16_t ldr_value = 0;
float lm35_value = 0;
//uint16_t notes[] = {262,294,330,349,392,440,494,523};
uint16_t delay_7segment = 250;

// NRF
/***      Set this radio as radio number 0 or 1         ***/
//bool radioNumber = 1;
/**********************************************************/
//byte addresses[][6] = {"1Node","2Node"};
// Used to control whether this node is sending or receiving
//bool role = 1;

double V2, R1, R2, Vi, temp, To, B, Ro;

// SD chip select pin
const uint8_t chipSelect = SS;

// file system object
SdFat sd;

// define a serial output stream
ArduinoOutStream cout(Serial);
//------------------------------------------------------------------------------
/*
 * Append a line to logfile.txt
 */
void logEvent(const char *msg) {
  // create dir if needed
  sd.mkdir("logs/2014/Jan");

  // create or open a file for append
  ofstream sdlog("logs/2014/Jan/logfile.txt", ios::out | ios::app);

  // append a line to the file
  sdlog << msg << endl;

  // check for errors
  if (!sdlog) {
    sd.errorHalt("append failed");
  }

  sdlog.close();
}
//------------------------------------------------------------------------------
 

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  // LCD setup
  lcd.setMCPType(LTI_TYPE_MCP23008);
  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
  // LED
  pinMode(L1_PIN, OUTPUT);
  pinMode(L2_PIN, OUTPUT);
  pinMode(L3_PIN, OUTPUT);
  pinMode(L4_PIN, OUTPUT);
  // BUTTON
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(S3_PIN, INPUT);
  // RELAY
  pinMode(RELAY_PIN,OUTPUT);
  // SENSORS
  pinMode(POT_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(LM35_PIN, INPUT);
  // BUZZER
//  pinMode(BUZZER_PIN, OUTPUT);
  // SD
  pinMode(SD_CS_PIN, OUTPUT);

  // RTC
  bool parse=false;
  bool config=false;

  // get the date and time the compiler was run
    if (getTime(__TIME__)) {
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
      }
    }

    // 7 Segment
    /*
    The MAX72XX is in power-saving mode on startup,
    we have to do a wakeup call
    */
    lc.shutdown(0,false);
    /* Set the brightness to a medium values */
    lc.setIntensity(0,8);
    /* and clear the display */
    lc.clearDisplay(0);


  V2 = analogRead(A3);
  R1 = 10000;
  Vi = 3.3;
  To = 25;
  B = 4400;
  Ro = 100000;
  
}

void loop()
{
  S1_clicked = !digitalRead(S1_PIN);  
        // LM35 Test
        //lm35_value = (5.0 * analogRead(LM35_PIN) * 100.0) / 1024;
  vout=analogRead(LM35_PIN); //Reading the value from sensor
  delay(10); // The trick when using multiple analog sensors is to read them twice, 
  // with a small delay after each read (10ms is good), then discard the first reading. 
  // This is because the ADC multiplexer needs switching time and the voltage needs time 
  // to stabilize after switching..  Basically the first analogRead call causes 
  // the multiplexer to switch, the delay gives the voltage time to stabilize, 
  // then your second read should be much more accurate with less jitter. 
  // 
  vout=analogRead(LM35_PIN);
  lm35_value=(vout*500)/1023;
  delay(10);

  //lm35_value=vout; // Storing value in Degree Celsius
  tempf=(vout*1.8)+32; // Converting to Fahrenheit

        // LDR Test
        //ldr_value = ((2500.0 / (analogRead(LDR_PIN) * (5.0 / 1024.0))) - 500) / 10.0;;
        
      // log
      lcd.setCursor(0, 0);
      lcd.print(" (C)|Clock|Lum ");
      lcd.setCursor(0, 1);
      //lcd.print(lm35_value);
      lcd.print(temp);
      lcd.print("|");
        lcd.print(tm.Hour);
        lcd.print(":");
        lcd.print(tm.Minute);
      lcd.print("|");  
      delay(10);
              // LDR Test
        ldr_value = ((2500.0 / (analogRead(LDR_PIN) * (5.0 / 1024.0))) - 500) / 10.0;;
          delay(10);
                  ldr_value = ((2500.0 / (analogRead(LDR_PIN) * (5.0 / 1024.0))) - 500) / 10.0;;
      lcd.print(ldr_value);

          delay(10);
        pot_value = analogRead(POT_PIN);
                  delay(10);
                          pot_value = analogRead(POT_PIN);

          c_7seg=abs(pot_value/100)+20;
lc.setDigit(0,3,abs(c_7seg-abs(c_7seg/10)*10),false);
lc.setDigit(0,2,abs(abs(c_7seg/10)),false);
//lc.setDigit(0,1,abs(pot_value/10),false);
//lc.setDigit(0,0,abs(pot_value),false);
      Serial.println(vout);
  delay(10);
  
  // put your main code here, to run repeatedly:
  V2 = analogRead(A2);
  V2 = V2 / 1023 * Vi;
  R2 = (V2 * R1) / (Vi - V2);
//  temp =  (B / log(R2 / (100000 * pow(M_E, (-B / 298.15)))));
//  temp = temp - 273.15;
  temp = Serial.read();

  
String dataString = String(temp) +", " +String(lm35_value) + ", " + String(tm.Hour) + ":" + String(tm.Minute) + ", " + String(ldr_value) + ", " + String(c_7seg);

  delay(400);  // catch Due reset problem
  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(chipSelect, SD_SCK_MHZ(10))) {
    sd.initErrorHalt();
  }

  // append a line to the logfile
  logEvent(dataString.c_str());

 // cout << F("Done - check /logs/2014/Jan/logfile.txt on the SD") << endl;

  
}

int selktor(){
  if(S1_clicked == false){

    if(tap_lenght < 1000 && tap_lenght > 0){
      Serial.println("click");
      test_selector++;
      tap_lenght = 0;
    }else if (tap_lenght > 1000){
      Serial.println("long click");
      test_selector = 100;
      tap_lenght =0;  
    }else{

    }

    timer = millis();
  }

  if(S1_clicked == true){
    tap_lenght = millis() - timer;    
  }

  


}


// FUNCTIONS 
bool getTime(const char *str){
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}
