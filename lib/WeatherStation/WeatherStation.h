#ifndef WeatherStation_H
#define WeatherStation_H
#pragma once

#include <Arduino.h>
#include "IWebConfig.h"
#include <MQTTClient.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "WindSensor.h"
#include "WindDirSensor.h"
#include "RainSensor.h"

class WeatherStation: 
  public IWebConfig,
  public MQTTClientCallback {

private:

  WindSensor * windSensor;
  WindDirSensor * windDirSensor;
  RainSensor * rainSensor;

  MQTTClient * WeatherStation_mqttClient;  
  String mqttBaseTopic = "/";

  unsigned long currentMillis = millis();
  unsigned long lastPublishMillis = currentMillis;

  bool enabled = false;
  std::string serverName;
  int port;
  std::string arrayString[4];



  // MQTTClient Observer callback functions:
  virtual void onConnected(MQTTClient* client) { 
    Serial.println("WeatherStation receives MQTTClient onConnected callback. The MQTT client is connected to the broker now.");
  };
	virtual void onDataReceived(MQTTClient* client, const mqtt_client_event_data *data) { 
    Serial.printf("[%lu] +++ WeatherStation receives MQTT message of size %d on topic %s: %.*s\r\n", millis(), data->data_len, data->topic.c_str(), data->data_len, data->data);
  };
  virtual void onSubscribed(MQTTClient* thisClient, const mqtt_client_topic_data *topic) { 
    Serial.printf("WeatherStation receives new topic subscription status -> Topic[%d]: %s status %d\n", topic->subs_msg_id, topic->topic.c_str(), topic->subs_status);
  };

  // IWebConfig callback functions:
  void parseWebConfig(JsonObjectConst configObject);

public:

  WeatherStation(void);
  WeatherStation(String name);

  void setup(void);
  void loop(void);
  void setMQTTBaseTopic(String topic) {
    this->mqttBaseTopic = topic;
    // Subscribe to topic example to receive published messages
    this->WeatherStation_mqttClient->addTopicSub(this->mqttBaseTopic.c_str());
    }

  // MQTTClient Observer callback functions:
  void setMQTTClient(MQTTClient *client) {
    // Set WeatherStation MQTT client and its observer callbacks:
    this->WeatherStation_mqttClient = client;
    this->WeatherStation_mqttClient->addCallback(this);
  }


  float getWDirADC(void) { return this->windDirSensor->getWDirADC(); }
  float getWDirDeg(void) { return this->windDirSensor->getWDirDeg(); }
};
#endif
