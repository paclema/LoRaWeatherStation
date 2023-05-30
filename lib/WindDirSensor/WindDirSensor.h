#ifndef WindDirSensor_H
#define WindDirSensor_H

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>

#include "driver/adc.h"
#include <esp_adc_cal.h>


const float grados[16] = {
  0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5,
  180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5
};

const float voltajes[16] = { 
  1426, 518, 599, 195, 202, 182, 276, 228,
  375, 332, 928, 856, 2396, 1616, 1955, 1128
};

#define ESP_INTR_FLAG_DEFAULT 0

#define WDIR_PRINT_TIME 5000

#define NO_OF_SAMPLES   64          // Multisampling

class WindDirSensor {
private:

  gpio_num_t gpioPin;
  const uint8_t ADCpin;
  int currentWDirmV;
  #if CONFIG_IDF_TARGET_ESP32
	  static const adc1_channel_t ADCchannel = ADC1_CHANNEL_4;
    static const adc_bits_width_t width_bit = ADC_WIDTH_BIT_12;
  #elif CONFIG_IDF_TARGET_ESP32S2
	  static const adc1_channel_t ADCchannel = ADC1_CHANNEL_4; // GPIO5 for ESP32S2
    static const adc_bits_width_t width_bit = ADC_WIDTH_BIT_13;
  #endif

  esp_adc_cal_characteristics_t adcCalCharacteristics;

  volatile unsigned long wspeedTime, wspeedLast;

  TaskHandle_t xHandle = NULL;
  SemaphoreHandle_t wspeedSemaphore = NULL;

  static void calculateValue(void* arg);

public:
  WindDirSensor(int gpioPin);
  void begin(void);
  int getWindDirection(void);

  float getWDirADC(void) { 
    this->getWindDirection(); 
    return (float) currentWDirmV;
  }

  float getWDirDeg(void) { 
    int index = this->getWindDirection(); 
    return (float) grados[index];
  }

};
#endif
