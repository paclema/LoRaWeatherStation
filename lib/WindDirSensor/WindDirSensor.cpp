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

uint16_t WindDirSensor::updateWDirmV() {
  uint32_t adc_reading = 0;
  // Without multisampling, only read one ADC value:
  // adc_reading = adc1_get_raw((adc1_channel_t)this->ADCchannel);
  // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &this->adcCalCharacteristics);
  // Serial.printf("WDIR sense reads %dmV --> CALIBRATED: %dmV \n", adc_reading, voltage);

  // Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc_reading += adc1_get_raw((adc1_channel_t)this->ADCchannel);
  }
  adc_reading /= NO_OF_SAMPLES;
  _currentWDirmV = esp_adc_cal_raw_to_voltage(adc_reading, &this->adcCalCharacteristics);

  Serial.printf("WDIR sense reads %d ADC --> CALIBRATED: %dmV \n", adc_reading, _currentWDirmV);
  return _currentWDirmV;
}

float WindDirSensor::mVToDir(uint16_t mV) {
  // Search the closest degree associated with the ADC mV value
  uint8_t closestIndex = 0;
  uint16_t dir = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (mV >= _windvane_table[15][1]) {
        // Prevent index overflow
        dir = _windvane_table[15][0];
        closestIndex = i;
        break;
    } else if (mV <= (_windvane_table[i][1] + ((_windvane_table[i + 1][1] - _windvane_table[i][1]) >> 1))) {
        // The mV value can be up to half the difference to next
        dir = _windvane_table[i][0];
        closestIndex = i;
        break;
    }
  }

  _currentWDirDeg = static_cast<float>(dir) / 10;
  Serial.printf(" \t closestIndex: %d \t\tValue:%d  dir Deg %.1f \t ",closestIndex, mV, _currentWDirDeg);
  return _currentWDirDeg;
}

int WindDirSensor::getWindDirection() {
  // // int rawValue = adc1_get_raw(this->ADCchannel);
  // // uint32_t voltage = esp_adc_cal_raw_to_voltage(rawValue, &this->adcCalCharacteristics);

  // // Serial.printf("VBAT sense reads %dmV --> CALIBRATED: %dmV \n", rawValue, voltage);

  // // // Escalar el valor a voltaje
  // // // float scaledVoltage = (float)voltage / 1000.0;
  // // float scaledVoltage = (float)voltage;

  // uint32_t adc_reading = 0;
  // // Multisampling
  // for (int i = 0; i < NO_OF_SAMPLES; i++) {
  //   adc_reading += adc1_get_raw((adc1_channel_t)this->ADCchannel);
  // }
  // adc_reading /= NO_OF_SAMPLES;
  // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &this->adcCalCharacteristics);
  // Serial.printf("WDIR sense reads %dmV --> CALIBRATED: %dmV \n", adc_reading, voltage);

  // // int rawValue = adc1_get_raw(this->ADCchannel);
  // // uint32_t voltage = esp_adc_cal_raw_to_voltage(rawValue, &this->adcCalCharacteristics);
  // // Serial.printf("WDIR sense reads %dmV --> CALIBRATED: %dmV \n", rawValue, voltage);


  // // Escalar el valor a voltaje
  // float scaledVoltage = (float)adc_reading;
  // currentWDirmV = adc_reading;

  // // currentWDirmV = rawValue;
  // // float scaledVoltage = (float)rawValue;


  // Serial.printf("WindDir:%d  Deg %.1f ----> NEW calc: %f\n", closestIndex, grados[closestIndex], mVToDir(voltage));


  // // Devolver el índice correspondiente al grado de dirección del viento
  // return closestIndex;
}


void WindDirSensor::calculateValue(void* arg) {
  WindDirSensor* sensor = (WindDirSensor*) arg;

  float currentSpeed;
  // unsigned long lastWspeedCheck = millis();

  while(1) {
    // float deltaTime  = (millis() - lastWspeedCheck) / 1000.0 ;

    // int index = sensor->getWindDirection();
    uint16_t mV = sensor->updateWDirmV();
    float deg = sensor->mVToDir(mV);

    // Serial.printf("WindDir:%d  Deg %.1f-- deltaTime: %.4f  ----> NEW calc: %f\n", index, grados[index], (float)deltaTime, sensor->adcToDir(sensor->currentWDirmV));
    vTaskDelay(WDIR_PRINT_TIME / portTICK_PERIOD_MS);
  }
};