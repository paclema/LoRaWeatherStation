#include "RainSensor.h"


RainSensor::RainSensor(int gpioPin) {
  this->gpioPin = (gpio_num_t) gpioPin;
  this->gpioPinBitMask = (1ULL<<gpioPin);
};

void RainSensor::begin(void){

  // GPIO pin configuration
  gpio_config_t io_conf = {};

  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.pin_bit_mask = this->gpioPinBitMask;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);

  // Interrupt configuration
  rainSemaphore = xSemaphoreCreateBinary();
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(this->gpioPin, RainSensor::ISRCounter, (void*) this);

  // Task sensor readings configuration
  xTaskCreate(RainSensor::calculateValue, "RainCalculate", 2048, this, 10, &xHandle);
    
};

void RainSensor::ISRCounter(void* arg) {
  RainSensor* sensor = (RainSensor*) arg;
  sensor->rainTime = millis();
  // Debounce of 10ms
  if (sensor->rainTime - sensor->rainLast > 10) {
    xSemaphoreTakeFromISR(sensor->rainSemaphore, NULL);
    sensor->rainISRConuter++;
    xSemaphoreGiveFromISR(sensor->rainSemaphore, NULL);
    sensor->rainLast = sensor->rainTime;
  }
};

void RainSensor::calculateValue(void* arg) {
  RainSensor* sensor = (RainSensor*) arg;

  float currentRain = 0;
  unsigned long lastWspeedCheck = millis();

   while(1) {

    float deltaTime  = (millis() - lastWspeedCheck) / 1000.0 ;

    xSemaphoreTake(sensor->rainSemaphore, portMAX_DELAY);
    int rainISRConuterTmp = sensor->rainISRConuter;
    sensor->rainISRConuter = 0;
    lastWspeedCheck = millis();
    xSemaphoreGive(sensor->rainSemaphore);

    currentRain = rainISRConuterTmp * RAIN_MM_PER_INTERRUPT;

    Serial.printf("Rainfall: %.4f mm  -- Ticks:  %d -- deltaTime: %.4f\n", (float)currentRain, rainISRConuterTmp, (float)deltaTime);

    vTaskDelay(RAIN_PRINT_TIME / portTICK_PERIOD_MS);

  }
};