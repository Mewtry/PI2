#include <Arduino.h>
#line 1 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
/**************************************************************************/
/**
 * @file    MYT_600.ino
 * @author  Theo Pires
 * @date    26/08/2023
 * @see     www.linkedin.com/in/theo-pires-a34b33183/
*/
/**************************************************************************/

#include "Wire.h"
#include <AccelStepper.h>
#include "LiquidCrystal_I2C.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

/********************* DEFINES *********************/

// MOTOR CC DA ESTEIRA
#define CC_MOTOR_PIN GPIO_NUM_4
#define CC_MOTOR_FREQ 5000
#define CC_MOTOR_FADE_TIME 3000
#define CC_MOTOR_RESOLUTION LEDC_TIMER_12_BIT
#define TOP 2^LEDC_TIMER_12_BIT

// MOTOR DE PASSO DO MAGAZINE
#define MAGAZINE_PUL_PIN GPIO_NUM_16
#define MAGAZINE_DIR_PIN GPIO_NUM_17
#define MAGAZINE_STEPS_PER_REV 48
#define MAGAZINE_ACCEL 240
#define MAGAZINE_SPEED  96

// SENSOR DE COR
#define TCS230_S0_PIN  GPIO_NUM_25
#define TCS230_S1_PIN  GPIO_NUM_26
#define TCS230_S2_PIN  GPIO_NUM_27
#define TCS230_S3_PIN  GPIO_NUM_14
#define TCS230_OUT_PIN GPIO_NUM_13

// DISPLAY LCD I2C
#define SDA_LCD_PIN GPIO_NUM_21
#define SCL_LCD_PIN GPIO_NUM_22
#define LCD_I2C_ADDR 0x27

// COMUNICAÇÂO SERIAL UART
#define UART_NUM UART_NUM_0

/******************** INSTANCES ********************/

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2);

AccelStepper magazine(AccelStepper::DRIVER, MAGAZINE_PUL_PIN, MAGAZINE_DIR_PIN);

/**************** GLOBAL VARIABLES *****************/ 

static const char * LCD_TAG = "LCD";
static const char * UART_TAG = "UART";
static const char * TCS230_TAG = "TCS230";
static const char * CC_MOTOR_TAG = "CC_MOTOR";
static const char * MAGAZINE_TAG = "MAGAZINE";

static const char * versao = "1.0.0";

static QueueHandle_t uart_queue;
static QueueHandle_t gpio_queue = NULL;

/******************** FUNCTIONS ********************/
    // CONVEYOR MOTOR CONTROL FUNCS
    // MAGAZINE MOTOR CONTROL FUNCS
    // COLOR SENSOR READ FUNC
    // DISPLAY FUNCS
    // UART READ FUNCS
    // UART WRITE FUNCS
    // INIT SETUP FUNC

/******************** INTERRUPTS ********************/
    // GPIO ISR HANDLER (IHM BUTTONS, SENSORS)

/********************** TASKS ***********************/
    // PRINCIPAL TASK AND IHM MONITOR (VARIABLES AND DISPLAY UPDATE, MOTOR CONTROL AND READ SENSORS)
    // SERIAL PROTOCOL CONTROL

/********************** SETUP **********************/
#line 89 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void setup(void);
#line 100 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void loop(void);
#line 89 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void setup(void){
    // SETUP AND TASK CREATE
    lcd.init();
    lcd.backlight();
    lcd.setCursor(4,0);
    lcd.print("MYT-D600");
    lcd.setCursor(4,1);
    lcd.print("v: ");
    lcd.print(versao);
}
/********************** LOOP **********************/
void loop(void){
    // loop principal
}
