#include <Arduino.h>


// Main variables:
// #define DEBUG_ESP_CORE

// Enable wifi diagnostic using platformio build_glag: -D ENABLE_SERIAL_DEBUG:
// #define ENABLE_SERIAL_DEBUG true


// Device configurations
unsigned long currentLoopMillis = 0;
unsigned long previousMainLoopMillis = 0;
size_t totalBytes;
size_t usedBytes;
size_t freeBytes;
unsigned long previousSPIFFSLoopMillis = 0;
#define SPIFFS_CHECK_SPACE_TIME 5000


// WebConfigServer Configuration
#include "WebConfigServer.h"
WebConfigServer config;   // <- global configuration object

#ifdef ARDUINO_IOTPOSTBOX_V1
#include "PowerManagement.h"
PowerManagement power;
#endif

#include <MQTTClient.h>
MQTTClient * mqttClient;

#include <WeatherStation.h>
WeatherStation wStation;

#define GPS_DATA_PUBLISH_TIME 10000



String topic = "";
StaticJsonDocument<256> doc;


unsigned long lastGPSPublish = 0UL;


//Lora and TTN
//------------
#include "loraFunctions.h"


// Websocket functions to publish:
String getLoopTime(){ return String(currentLoopMillis - previousMainLoopMillis);}
String getRSSI(){ return String(WiFi.RSSI());}
String getHeapFree(){ return String((float)GET_FREE_HEAP/1000);}
String getMemoryUsageString(){ 
  String r = String("\"Used: " + config.formatBytes(usedBytes) + " Free: " +  config.formatBytes(freeBytes) + "\"");
  // Serial.println(r);
  return r;}
String getMemoryFree(){  return String(freeBytes);};
#ifdef ARDUINO_IOTPOSTBOX_V1
String getVBat(){ return String((float)power.vBatSense.mV/1000,4);}
String getVBus(){ return String((float)power.vBusSense.mV/1000,3);}
  #endif

void setup() {
  Serial.begin(115200);
  
  // TODO: this is messing up with ESP32S2 Native USB CDC serial output
  #if defined(ENABLE_SERIAL_DEBUG)
    Serial.setDebugOutput(true);
  #endif

  #ifdef ARDUINO_IOTPOSTBOX_V1
  while(!Serial) {}
  pinMode(LDO2_EN_PIN, OUTPUT);
  digitalWrite(LDO2_EN_PIN, HIGH);
  power.setup();
  #endif

  config.begin();

  // loraSetup();
 
  config.addDashboardObject("heap_free", getHeapFree);
  config.addDashboardObject("loop", getLoopTime);
  config.addDashboardObject("RSSI", getRSSI);
  config.addDashboardObject("SPIFFS_Usage", getMemoryUsageString);
  config.addDashboardObject("SPIFFS_Free", getMemoryFree);
  #ifdef ARDUINO_IOTPOSTBOX_V1
  config.addDashboardObject("VBat", getVBat);
  config.addDashboardObject("VBus", getVBus);
  #endif
  
  mqttClient = config.getMQTTClient();
  wStation.setMQTTClient(mqttClient);

  #ifdef ARDUINO_IOTPOSTBOX_V1
  power.update();
  #endif

  topic = config.getDeviceTopic() + "data";

  wStation.setup();
  
  Serial.println("###  Looping time\n");

}

void loop() {

  currentLoopMillis = millis();

   if (currentLoopMillis - previousSPIFFSLoopMillis > SPIFFS_CHECK_SPACE_TIME){
    previousSPIFFSLoopMillis = currentLoopMillis;
    totalBytes = LittleFS.totalBytes();
    usedBytes = LittleFS.usedBytes();
    freeBytes  = totalBytes - usedBytes;
   }

  config.loop();


  wStation.loop();

  // loraLoop();


  #ifdef ARDUINO_IOTPOSTBOX_V1
  power.update();
  #endif


  if (currentLoopMillis - lastGPSPublish > GPS_DATA_PUBLISH_TIME){
    lastGPSPublish = currentLoopMillis;

    // if (gps.location.isUpdated()){
    //   Serial.printf("---> NEW GPS location: %lf - %lf\n", gps.location.lat(), gps.location.lng());
    // } 
    // if (gps.speed.isUpdated()){
    //   Serial.printf("---> NEW GPS speed: %lf\n", gps.speed.kmph());
    // }

    // publish2TTN();


    if(mqttClient->connected()) {

      String msg_pub;
      StaticJsonDocument<256> doc;

      // doc["lat"] = lat;
      // doc["lng"] = lng;
      // doc["date"] = gpsDate;
      // doc["time"] = gpsTime;
      // doc["speed"] = gpsSpeed;
      // doc["satellites"] = gpsSat;
      // doc["altitude"] = gpsAltitude;
      // doc["hdop"] = gpsHdop;
      // doc["course"] = gpsCourse;
      // doc["rssi_STA"] = WiFi.RSSI();
      #ifdef ARDUINO_IOTPOSTBOX_V1
      doc["vBat"] = (float)power.vBatSense.mV/1000;
      doc["vBus"] = (float)power.vBusSense.mV/1000;
      doc["PowerStatus"] = (int)power.getPowerStatus();
      doc["ChargingStatus"] = (int)power.getChargingStatus();
      #endif

      serializeJson(doc, msg_pub);
      mqttClient->publish(topic.c_str(), msg_pub.c_str());
      // Serial.println(msg_pub);
    }

  }

  previousMainLoopMillis = currentLoopMillis;
}
