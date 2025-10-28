#ifndef RainSensor_H
#define RainSensor_H
#pragma once

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>

#define RAIN_MM_PER_INTERRUPT 0.2794
#define RAIN_PRINT_TIME 30000

#define ESP_INTR_FLAG_DEFAULT 0


class RainSensor {
private:

  gpio_num_t gpioPin;
  uint64_t gpioPinBitMask;
  volatile int rainISRConuter = 0;
  volatile unsigned long rainTime, rainLast;

  TaskHandle_t xHandle = NULL;
  SemaphoreHandle_t rainSemaphore = NULL;

  static void IRAM_ATTR ISRCounter(void* arg);
  static void calculateValue(void* arg);

public:
  RainSensor(int gpioPin);
  void begin(void);
};
#endif
