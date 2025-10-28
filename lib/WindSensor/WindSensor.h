#ifndef WindSensor_H
#define WindSensor_H
#pragma once

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>

#define WSPEED_KMH_PER_INTERRUPT 2.4
#define WSPEED_PRINT_TIME 5000

#define ESP_INTR_FLAG_DEFAULT 0


class WindSensor {
private:

  gpio_num_t gpioPin;
  uint64_t gpioPinBitMask;
  volatile int wspeedISRConuter = 0;
  volatile unsigned long wspeedTime, wspeedLast;

  TaskHandle_t xHandle = NULL;
  SemaphoreHandle_t wspeedSemaphore = NULL;

  static void IRAM_ATTR ISRCounter(void* arg);
  static void calculateValue(void* arg);

public:
  WindSensor(int gpioPin);
  void begin(void);
};
#endif
