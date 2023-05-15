#include "WeatherStation.h"


WeatherStation::WeatherStation(void) {
  this->nameConfigObject = "WeatherStation";
};


WeatherStation::WeatherStation(String name) {
  this->nameConfigObject = name;
};

void WeatherStation::parseWebConfig(JsonObjectConst configObject){
  // WebCOnfigServer pass using this callback the updated 
  // json configuration for this class WeatherStation.
  // This configObject is the "WeatherStation" json object from 
  // the data/config/config.json file

  // JsonObject received:
  serializeJsonPretty(configObject, Serial);

  this->enabled = configObject["enabled"] | false;
  this->serverName = configObject["serverName"] | "servenamedefault.com";
  this->port = configObject["port"] | 8888;

  if (configObject["arrayString"].size() > 0)
      for (unsigned int i = 0; i < configObject["arrayString"].size(); i++)
          this->arrayString[i] = configObject["arrayString"][i].as<std::string>();
  else
      this->arrayString[0] = configObject["arrayString"].as<std::string>();

};

void WeatherStation::setup(void){
  windSensor = new WindSensor(PIN_WSPEED);
  windDirSensor = new WindDirSensor(PIN_WDIR);
  rainSensor = new RainSensor(PIN_RAIN);

  windSensor->begin();
  windDirSensor->begin();
  rainSensor->begin();
};

void WeatherStation::loop(void){
  currentMillis = millis();
  if( WeatherStation_mqttClient->connected() && (currentMillis - lastPublishMillis > 1200)) {
    lastPublishMillis = currentMillis;
    
    String topic = this->mqttBaseTopic + "WeatherStation";
    // WeatherStation_mqttClient->publish(topic.c_str(),"Message from WeatherStation loop");
    // Serial.printf("Message from WeatherStation loop published to topic %s\n", topic.c_str());

  }

};

