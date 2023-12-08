#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "TCS230.h"

// SENSOR DE COR
#define TCS230_S0_PIN  GPIO_NUM_25 // Output frequency scaling S0
#define TCS230_S1_PIN  GPIO_NUM_26 // Output frequency scaling S1
#define TCS230_S2_PIN  GPIO_NUM_27 // Filter selection S2
#define TCS230_S3_PIN  GPIO_NUM_14 // Filter selection S3
#define TCS230_OE_PIN  GPIO_NUM_12 // Output Enable Pin
#define TCS230_OUT_PIN GPIO_NUM_13 // Output Sensor
#define TCS230_RGB_SIZE 3

TCS230 tcs(
    TCS230_OUT_PIN, 
    TCS230_S2_PIN, 
    TCS230_S3_PIN, 
    TCS230_S0_PIN, 
    TCS230_S1_PIN, 
    TCS230_OE_PIN
);

void setup(){
    Serial.begin(115200);
    tcs.begin();
    
}

void loop(){
    tcs.read();
    printf("Leitura RAW{%d, %d, %d}\n", tcs.getRawRed(), tcs.getRawGreen(), tcs.getRawBlue());
    printf("Leitura RGB{%d, %d, %d}\n", tcs.getRed(), tcs.getGreen(), tcs.getBlue());
    delay(3000);
}