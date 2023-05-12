#include "WindSensor.h"


WindSensor::WindSensor(int gpioPin) {
  this->gpioPin = (gpio_num_t) gpioPin;
  this->gpioPinBitMask = (1ULL<<gpioPin);
};

void WindSensor::begin(void){

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
  gpio_isr_handler_add(this->gpioPin, WindSensor::ISRCounter, (void*) this);

  // Task sensor readings configuration
  xTaskCreate(WindSensor::calculateValue, "WSpeedCalculate", 2048, this, 10, &xHandle);
    
};

void WindSensor::ISRCounter(void* arg) {
  WindSensor* sensor = (WindSensor*) arg;
  xSemaphoreTakeFromISR(sensor->rainSemaphore, NULL);
  sensor->rainISRConuter++;
  xSemaphoreGiveFromISR(sensor->rainSemaphore, NULL);
};

void WindSensor::calculateValue(void* arg) {
  WindSensor* sensor = (WindSensor*) arg;

  float currentSpeed;
  unsigned long lastWspeedCheck = millis();

   while(1) {

    float deltaTime  = (millis() - lastWspeedCheck) / 1000.0 ;

    xSemaphoreTake(sensor->rainSemaphore, portMAX_DELAY);
    int rainISRConuterTmp = sensor->rainISRConuter;
    sensor->rainISRConuter = 0;
    lastWspeedCheck = millis();
    xSemaphoreGive(sensor->rainSemaphore);

    currentSpeed = (float) rainISRConuterTmp / deltaTime ;
    currentSpeed *= RAIN_KMH_PER_INTERRUPT;

    Serial.printf("WindSpeed: %.4f km/h  -- Ticks:  %d -- deltaTime: %.4f\n", (float)currentSpeed, rainISRConuterTmp, (float)deltaTime);

    vTaskDelay(RAIN_PRINT_TIME / portTICK_PERIOD_MS);

  }
};