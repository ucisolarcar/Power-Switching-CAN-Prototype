#include <SPI.h>              //Library for using SPI Communication 
#include <mcp2515.h>          //Library for using CAN Communication (https://github.com/autowp/arduino-mcp2515/)
#include <stdlib.h>


//including custom library to convert CAN to 2 float numbers:
#include <canFloat.h>
#include <sensorFunctions.h>

struct can_frame canMsg;
struct can_frame transMsg;
uint32_t timer = millis();

//pins used for indicating current range
const int greenLED = 25;
const int orangeLED = 26;
const int redLED = 27;

//pins to control power switching:
const int ARDUINO_FAULT = 14;

const canid_t MSGID = 0x1200;

SensorFunctions Uno = SensorFunctions();

//FAULT ID:
const canid_t errorID = 0x0009; //ID number 9 will be the can id that all errors will have
//these data numbers will be used in the can message that is sent out for faults
//For 8 bits I can only have 256 unique errors. Should ask to see if that is ok. If not I might need to up to 9 or 16bits. Need to find a way to do that if that's the case
const uint8_t overCurrID= 0x0001;
const uint8_t underCurrID = 0x0002;

struct recieveData
{
  int8_t arr[8];
};
 
MCP2515 mcp2515(5);                 // SPI CS Pin 5
 
void setup()
{
  Serial.begin(9600);                //Begins Serial Communication at 9600 baudrate
  SPI.begin();                       //Begins SPI communication
  delay(3000);
 
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();                  //Sets CAN at normal mode
  Serial.println("Recieving initialized!\n");

  //setting up GPIO pins for LED output:
  pinMode(greenLED, OUTPUT);
  pinMode(orangeLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(ARDUINO_FAULT, OUTPUT);


  //this is also where we would have a function to control precharge:
  //Going to be monitoring current constantly:

  //1. precharge 1 is CLOSED and precharge 2 is OPEN
  //Loop: continuously monitor the current, make sure that it is increasing at a specified rate
  //(there should be a precharge curve we are following)
  //2. Once current is at desired threshold for x amount of ms then we OPEN precharge 1 and CLOSE precharge 2

  //Do we need to open both precharge circuits if there is a BMS issue?
}

bool receive(void)
{
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) // To receive data (Poll Read)
  {
    if(canMsg.can_id == 0x10)
    {
        Uno.parseAccel(canMsg, true);
    }
    else if(canMsg.can_id == 0x11)
    {
        Uno.parseAccel(canMsg, true);
    }
    else if(canMsg.can_id == 0x12)
    {
        Uno.parseGPS(canMsg, true);
    }
    else if(canMsg.can_id == 0x13)
    {
        float current = Uno.parseCurrent(canMsg, true);
        lightLogic(current);
    }
    else if(canMsg.can_id == 0x14)
    {
        Uno.parseCurrent(canMsg, true);
    }
    else
    {
      Serial.print("Got invalid ID: ");
      Serial.println(canMsg.can_id, HEX);
    }
    return true;
  }
  else
    return false;
}

void lightLogic(float currentData)
{
  float goodVal = 0;
  if(currentData > goodVal + 0.05)
  {
    Serial.println("OVER CURRENT!");
    //turn on RED LED to indicate fault
    digitalWrite(redLED, HIGH);
    digitalWrite(orangeLED, LOW);
    digitalWrite(greenLED, LOW);
    //and also trip the ARDUINO FAULT for power switching:
    digitalWrite(ARDUINO_FAULT, HIGH);
    sendFault(overCurrID);
  }
  else if(currentData < goodVal - 0.05)
  {
    Serial.println("UNDER CURRENT!");
    digitalWrite(redLED, LOW);
    digitalWrite(orangeLED, HIGH);
    digitalWrite(greenLED, LOW);
    digitalWrite(ARDUINO_FAULT, HIGH);
    sendFault(underCurrID);
  }
  //otherwise current is the value we want it to be:
  else
  {
    digitalWrite(ARDUINO_FAULT, LOW);
    digitalWrite(redLED, LOW);
    digitalWrite(orangeLED, LOW);
    digitalWrite(greenLED, HIGH);
    sendFault(0);
  }
}

void sendFault(canid_t canId)
{
  transMsg.can_id = canId;
  transMsg.can_dlc = 0;
  mcp2515.sendMessage(&transMsg);
  Serial.print("Fault ID: ");
  Serial.print(canId);
  Serial.println(" was sent!");
}


void transmitMsg()
{
  transMsg.can_id = MSGID;
  transMsg.can_dlc = 2;

  transMsg.data[0] = 12;
  transMsg.data[1] = 24;

  mcp2515.sendMessage((&transMsg));
  Serial.println("Sent 12 & 24");
}


 
 
void loop()
{
  static unsigned nextInterval = 500;
  receive();
  if (millis() - timer > nextInterval)
  {
    timer = millis(); // reset the timer
    nextInterval = 1500 + random(1000);
    //transmitMsg();
  }
}