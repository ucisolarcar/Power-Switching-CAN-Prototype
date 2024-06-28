#include <canFloat.h>
#include <sensorFunctions.h>

#include <SPI.h>              //Library for using SPI Communication 
#include <mcp2515.h>          //Library for using CAN Communication (https://github.com/autowp/arduino-mcp2515/)
#include <Adafruit_GPS.h>
#include "sensorFunctions.h"

#define GPSECHO false

//timer to regulate frequency at which data is transmitted
uint32_t timer = millis();
uint32_t timer2 = millis();

//Using hardware serial 2
Adafruit_GPS GPS;

//CAN related Variables
struct can_frame canMsg;
struct can_frame recMsg;
MCP2515 mcp2515(53);  

//can id constant:
//for the board, not sure if this is necessary
const canid_t CANID = 0x90;
//for each component
const canid_t ACCEL1ID = 0x1010;
const canid_t ACCEL2ID = 0x1011;
const canid_t GPSID = 0x1012;
const canid_t CURR1ID = 0x1013;
const canid_t CURR2ID = 0x1014;

//Defining pin connections:
const int xpin1 = A0;       // x-axis of the accelerometer
const int ypin1 = A1;       // y-axis
const int zpin1 = A2;       // z-axis (only on 3-axis models)
//CHECK THESE PINS~~
const int currPin1 = A4;    // port for current sensor 1 readings
const int currPin2 = A5;    // port for current sensor 2 readings

SensorFunctions Mega = SensorFunctions();
float currOffset = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //GPS setup:
  Adafruit_GPS GPS = Mega.gpsSetup(&Serial2);

  //CAN related setup:
  SPI.begin();                       //Begins SPI communication
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();                  //Sets CAN at normal mode

  //Calibrate Current Sensor(s):
  currOffset = Mega.calibrateCurr(currPin1);
}

// This function should run if the Arduino is expected to recieve any data
bool receive() {
  if (mcp2515.readMessage(&recMsg) == MCP2515::ERROR_OK) // To receive data (Poll Read)
  {
    Serial.print("ID: ");
    Serial.print(recMsg.can_id, HEX);
    if (recMsg.can_id == 0x200) { //if arduino 2 sending data
    Serial.print("   Received: ");
      Serial.print(recMsg.data[0]);
      Serial.print("\t");
      Serial.println(recMsg.data[1]);
      return true;
      
    }
    
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned nextInterval = 500;
  static unsigned fastInterval = 50;
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  // Essentially, if data can be parsed from the GPS send it
  

  //transmit readings from other sensors
  
  if (millis() - timer > nextInterval)
  {
    Mega.transmitAccel(mcp2515, canMsg, ACCEL1ID, xpin1, ypin1, zpin1, true);
    Mega.transmitCurr(mcp2515, canMsg, CURR1ID, currPin1, currOffset, true);
    
    //When GPS isn't plugged in this statement always evals to false, so transmitGPS is not reached
    if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Mega.transmitGPS(mcp2515, GPS, canMsg, GPSID, false);
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  
    timer = millis(); // reset the timer
    nextInterval = 1500 + random(1000);
  }

  //setup another timer to send current at a higher frequency
  else if(millis() - timer2 > fastInterval)
  {
    Mega.transmitCurr(mcp2515, canMsg, CURR1ID, currPin1, currOffset, true);
    timer2 = millis();
    fastInterval = 100 + random(50);
  }
  receive();
}
