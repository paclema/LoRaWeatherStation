#ifndef WindSensor_H
#define WindSensor_H

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>

#define RAIN_KMH_PER_INTERRUPT 2.4
#define RAIN_PRINT_TIME 5000

#define ESP_INTR_FLAG_DEFAULT 0


class WindSensor {
private:

  gpio_num_t gpioPin;
  uint64_t gpioPinBitMask;
  volatile int rainISRConuter = 0;

  TaskHandle_t xHandle = NULL;
  SemaphoreHandle_t rainSemaphore = NULL;

  static void IRAM_ATTR ISRCounter(void* arg);
  static void calculateValue(void* arg);

public:
  WindSensor(int gpioPin);
  void begin(void);
};
#endif
