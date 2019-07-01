#define REMOTE_MAC {0x36, 0x33, 0x33, 0x33, 0x33, 0x33}

#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "config.h"
#include "main.h"
#include "protocol.h"
#include "HoverboardAPI.h"
#include <ArduinoJson.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define NUMSLAVES 20
#define COMMANDLENGTH 130
esp_now_peer_info_t slaves[NUMSLAVES] = {};
int SlaveCnt = 0;
bool rftimeout = true;

typedef struct commandStruct
{
  byte macAdr[6];
  int speed;
  int angle;
};

commandStruct command;

typedef struct telemetryStruct
{
  float batteryVoltage;
  float current1;
  float current2;
  float realSpeed;
  float realSteer;
  int isCharging;
  float tempInC;
};

telemetryStruct telemetry;
#define WIFI_CHANNEL 1
esp_now_peer_info_t master;
const esp_now_peer_info_t *masterNode = &master;
uint8_t masterDeviceMac[] = REMOTE_MAC; // Remote Control
const byte maxDataFrameSize = 200;
byte cnt = 0;
int steer1 = 0;
int speed1 = 0;

int last, rate, owncount;
int successcount, failcount;
int roboSpeed = 0;
int roboAngle = 0;

uint8_t dataToSend[maxDataFrameSize];

HardwareSerial *COM[3] = {&Serial, &Serial1, &Serial2};

volatile motorControl motor = {{0.0, 0.0}, {0.0, 0.0}};
volatile motorControl motor1 = {{0.0, 0.0}, {0.0, 0.0}};

#define DEBUG_COM 0
  int serialWrapper(unsigned char *data, int len) {
      for(int i = 0; i< len; i++) {
        switch (i) {
        case 0:
          COM[DEBUG_COM]->printf("SOM:%01i ", data[i]);
          break;

        case 1:
          COM[DEBUG_COM]->printf("CI:%03i ", data[i]);
          break;

        case 2:
          COM[DEBUG_COM]->printf("len:%03i ", data[i]);
          break;

        case 3:
          COM[DEBUG_COM]->printf("CMD:%c ", data[i]);
          break;

        case 4:
          if(i==len-1) {
            COM[DEBUG_COM]->printf("CS:0x%02X ", data[i]);
          } else if(data[i] == HoverboardAPI::Codes::setPointPWM) {
            COM[DEBUG_COM]->print("PWM       ");
          } else if(data[i] == HoverboardAPI::Codes::setPointPWMData) {
            COM[DEBUG_COM]->print("PWM Data  ");
          } else if(data[i] == HoverboardAPI::Codes::protocolSubscriptions) {
            COM[DEBUG_COM]->print("Subscribe ");
          } else if(data[i] == HoverboardAPI::Codes::sensHall) {
            COM[DEBUG_COM]->print("Hall      ");
          } else if(data[i] == HoverboardAPI::Codes::protocolCountSum) {
            COM[DEBUG_COM]->print("CounterS  ");
          } else if(data[i] == HoverboardAPI::Codes::setBuzzer) {
            COM[DEBUG_COM]->print("Buzzer    ");
          } else if(data[i] == HoverboardAPI::Codes::enableMotors) {
            COM[DEBUG_COM]->print("Enable    ");
          } else if(data[i] == HoverboardAPI::Codes::disablePoweroff) {
            COM[DEBUG_COM]->print("PowerOff  ");
          } else if(data[i] == HoverboardAPI::Codes::sensElectrical) {
            COM[DEBUG_COM]->print("El. Meas  ");
          } else {
            COM[DEBUG_COM]->printf("Code:0x%02X ", data[i]);
          }
          break;

        default:
          if(i==len-1) {
            COM[DEBUG_COM]->printf("CS:0x%02X ", data[i]);
          } else {
            COM[DEBUG_COM]->printf("%02X ", data[i]);
          }
          break;
        }
      }
      COM[DEBUG_COM]->println();
      return (int) COM[2]->write(data,len);
  }
HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);
int serialWrapper1(unsigned char *data, int len) {
  return (int) COM[1]->write(data, len);
}
HoverboardAPI hoverboard1 = HoverboardAPI(serialWrapper1);

void processHalldata ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, int len ) {
  switch (fn_type) {
    case FN_TYPE_POST_READRESPONSE:
    case FN_TYPE_POST_WRITE:
      motor.measured.actualSpeed_kmh = hoverboard.getSpeed_kmh();
      motor.measured.actualSteer_kmh = hoverboard.getSteer_kmh();
      break;
  }
}
void processHalldata1 ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, int len ) {
  switch (fn_type) {
    case FN_TYPE_POST_READRESPONSE:
    case FN_TYPE_POST_WRITE:
      motor1.measured.actualSpeed_kmh = hoverboard1.getSpeed_kmh();
      motor1.measured.actualSteer_kmh = hoverboard1.getSteer_kmh();
      break;
  }
}

void setup()
{

  COM[0]->begin(115200, SERIAL_8N1, 03, 01);
  COM[1]->begin(115200, SERIAL_8N1, 27, 26);
  COM[2]->begin(115200, SERIAL_8N1, 16, 17);

  WiFi.mode(WIFI_AP_STA);
  uint8_t slaveCustomMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x34};
  esp_wifi_set_mac(ESP_IF_WIFI_AP, &slaveCustomMac[0]);
  Serial.println( WiFi.softAPmacAddress() );
  WiFi.disconnect();
  esp_now_init();
  memcpy( &master.peer_addr, &masterDeviceMac, 6 );
  master.channel = WIFI_CHANNEL;
  master.encrypt = 0;
  master.ifidx = ESP_IF_WIFI_AP;
  esp_now_add_peer(masterNode);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  delay(200);
  hoverboard.sendPWMData(0, 0, 1000, -1000, 1, PROTOCOL_SOM_ACK);
  hoverboard1.sendPWMData(0, 0, 1000, -1000, 1, PROTOCOL_SOM_ACK);
  hoverboard.setParamHandler(hoverboard.Codes::sensHall, processHalldata);
  hoverboard1.setParamHandler(hoverboard1.Codes::sensHall, processHalldata1);
  hoverboard.sendBuzzer( 5, 0, 30);
  hoverboard1.sendBuzzer( 5, 0, 30);
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  ReceiveMessage(data, data_len);
  SendMessage();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)++successcount; else ++failcount;
}

void loop()
{
  //hoverboard.printStats(*COM[0]);
  hoverboard.requestRead(hoverboard.Codes::protocolCountSum);
  hoverboard.requestRead(hoverboard.Codes::sensElectrical);
  hoverboard.requestRead(hoverboard.Codes::sensHall);
  COM[0]->print("V: ");
  COM[0]->print(hoverboard.getBatteryVoltage());
  COM[0]->print(" Current0: ");
  COM[0]->print(hoverboard.getMotorAmpsAvg(0));
  COM[0]->print(" Current1: ");
  COM[0]->print(hoverboard.getMotorAmpsAvg(1));
  COM[0]->print(" Speed: ");
  COM[0]->print(hoverboard.getSpeed_kmh());
  COM[0]->print(" Steer: ");
  COM[0]->print(hoverboard.getSteer_kmh());
  COM[0]->print(" Current Limit: ");
  COM[0]->print(hoverboard.getCurrentLimit());
  COM[0]->print(" Is Charging: ");
  COM[0]->print(hoverboard.getIsCharging());
  COM[0]->print(" Temperature: ");
  COM[0]->print(hoverboard.getTempInC());
  COM[0]->print(" Success: ");
  COM[0]->print(successcount);
  COM[0]->print(" Fail: ");
  COM[0]->print(failcount);
  COM[0]->println();

  int i = 0;
  while (COM[2]->available() && i++ < 1024) { // read maximum 1024 byte at once.
    unsigned char readChar = COM[2]->read();
    hoverboard.protocolPush( readChar );
  }
  hoverboard.protocolTick();

  int er = 0;
  while (COM[1]->available() && er++ < 1024) { // read maximum 1024 byte at once.
    unsigned char readChar = COM[1]->read();
    hoverboard1.protocolPush( readChar );
  }
  hoverboard1.protocolTick();
  telemetry.batteryVoltage = hoverboard.getBatteryVoltage();
  telemetry.current1 = hoverboard.getMotorAmpsAvg(0);
  telemetry.current2 = hoverboard.getMotorAmpsAvg(1);
  telemetry.realSpeed = hoverboard.getSpeed_kmh();
  telemetry.realSteer = hoverboard.getSteer_kmh();
  telemetry.isCharging = hoverboard.getIsCharging();
  telemetry.tempInC = hoverboard.getTempInC();

  if (millis() - last > 100) {
    speed1 = 0;
    steer1 = 0;
    if (!rftimeout) {
      rftimeout = true;
      hoverboard.sendBuzzer( 5, 0, 100);
      hoverboard1.sendBuzzer( 5, 0, 100);
    }
  } else {
    rftimeout = false;
    speed1 = roboSpeed;
    steer1 = roboAngle;
  }
  hoverboard.sendPWM(speed1, steer1);
  hoverboard1.sendPWM(speed1, steer1);
 //hoverboard.sendPowerOff('r');
 //hoverboard1.sendPowerOff('r');
  delay(40);
}

void ReceiveMessage(const uint8_t *data, int data_len)
{
  rate = millis() - last;
  last = millis();
  char jsonChar[COMMANDLENGTH];
  jsonChar[COMMANDLENGTH - 1] = 0;
  strncpy( jsonChar, (char*)data, COMMANDLENGTH - 1 );
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(jsonChar);
  if (!root.success()) {
    Serial.println("failed!!!");
  } else {
    roboSpeed = root["speed"];
    roboAngle = root["angle"];
    Serial.print(roboSpeed);
    Serial.print("  ");
    Serial.println(roboAngle);
  }
}

void SendMessage()
{
  char jsonChar[COMMANDLENGTH];
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["battery"] = telemetry.batteryVoltage;
  root["current1"] = telemetry.current1;
  root["current2"] = telemetry.current2;
  root["realSpeed"] = telemetry.realSpeed;
  root["realSteer"] = telemetry.realSteer;
  root["isCharging"] = telemetry.isCharging;
  root["tempInC"] = telemetry.tempInC;
  root.printTo(jsonChar, COMMANDLENGTH);

  int JSONlen = 0;
  while (jsonChar[JSONlen] != '}' && JSONlen < COMMANDLENGTH - 2) JSONlen++;
  JSONlen++;
  jsonChar[JSONlen] = 0;

  uint8_t commandJSON[COMMANDLENGTH];
  memcpy(commandJSON, jsonChar, JSONlen + 1);
  esp_err_t sendResult = esp_now_send(master.peer_addr, commandJSON, JSONlen + 1);
}
