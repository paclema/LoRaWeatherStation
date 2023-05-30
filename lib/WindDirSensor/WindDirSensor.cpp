#include "WindDirSensor.h"


WindDirSensor::WindDirSensor(int gpioPin): ADCpin(gpioPin){
  // this->gpioPin = (gpio_num_t) gpioPin;
  pinMode(ADCpin, INPUT);

};

void WindDirSensor::begin(void){

  adc1_config_width(width_bit);
  adc1_config_channel_atten((adc1_channel_t)gpioPin, ADC_ATTEN_DB_11);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, width_bit, ESP_ADC_CAL_VAL_DEFAULT_VREF, &adcCalCharacteristics);


  // Task sensor readings configuration
  xTaskCreate(WindDirSensor::calculateValue, "WDirCalculate", 2048, this, 10, &xHandle);
    
};

int WindDirSensor::getWindDirection() {
  // int rawValue = adc1_get_raw(this->ADCchannel);
  // uint32_t voltage = esp_adc_cal_raw_to_voltage(rawValue, &this->adcCalCharacteristics);

  // Serial.printf("VBAT sense reads %dmV --> CALIBRATED: %dmV \n", rawValue, voltage);

  // // Escalar el valor a voltaje
  // // float scaledVoltage = (float)voltage / 1000.0;
  // float scaledVoltage = (float)voltage;

  uint32_t adc_reading = 0;
  // Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc_reading += adc1_get_raw((adc1_channel_t)this->ADCchannel);
  }
  adc_reading /= NO_OF_SAMPLES;
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &this->adcCalCharacteristics);
  Serial.printf("WDIR sense reads %dmV --> CALIBRATED: %dmV \n", adc_reading, voltage);

  // int rawValue = adc1_get_raw(this->ADCchannel);
  // uint32_t voltage = esp_adc_cal_raw_to_voltage(rawValue, &this->adcCalCharacteristics);
  // Serial.printf("WDIR sense reads %dmV --> CALIBRATED: %dmV \n", rawValue, voltage);


  // Escalar el valor a voltaje
  float scaledVoltage = (float)adc_reading;
  currentWDirmV = adc_reading;

  // currentWDirmV = rawValue;
  // float scaledVoltage = (float)rawValue;

  // Buscar el valor de grados más cercano en la tabla
  int closestIndex = 0;
  float closestDiff = abs(voltajes[0] - scaledVoltage);
  for (int i = 1; i < 16; i++) {
    float diff = abs(voltajes[i] - scaledVoltage);
    if (diff < closestDiff) {
      closestIndex = i;
      closestDiff = diff;
    }
  }

  // Devolver el índice correspondiente al grado de dirección del viento
  return closestIndex;
}


void WindDirSensor::calculateValue(void* arg) {
  WindDirSensor* sensor = (WindDirSensor*) arg;

  float currentSpeed;
  unsigned long lastWspeedCheck = millis();

  while(1) {
    float deltaTime  = (millis() - lastWspeedCheck) / 1000.0 ;

    int index = sensor->getWindDirection();
    Serial.printf("WindDir:%d  Deg %.1f-- deltaTime: %.4f\n", index, grados[index], (float)deltaTime);

    vTaskDelay(WDIR_PRINT_TIME / portTICK_PERIOD_MS);
  }
};