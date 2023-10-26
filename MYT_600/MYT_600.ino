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
#include "TCS230.h"

/********************* DEFINES *********************/

// MOTOR CC DA ESTEIRA
#define CC_MOTOR_IN1  GPIO_NUM_18
#define CC_MOTOR_IN2  GPIO_NUM_19
#define CC_MOTOR_ENA  GPIO_NUM_5
#define CC_MOTOR_FADE_TIME 5000
#define CC_MOTOR_FADE_STEP 2
#define CC_MOTOR_FREQ 1000
#define CC_MOTOR_RESOLUTION LEDC_TIMER_12_BIT
#define TOP 4095

// MOTOR DE PASSO DO MAGAZINE
#define MAGAZINE_PUL_PIN GPIO_NUM_16
#define MAGAZINE_DIR_PIN GPIO_NUM_17
#define MAGAZINE_STEPS_PER_REV 48
#define MAGAZINE_ACCEL 240
#define MAGAZINE_SPEED  96

// SENSOR DE COR
#define TCS230_S0_PIN  GPIO_NUM_25 // Output frequency scaling S0
#define TCS230_S1_PIN  GPIO_NUM_26 // Output frequency scaling S1
#define TCS230_S2_PIN  GPIO_NUM_27 // Filter selection S2
#define TCS230_S3_PIN  GPIO_NUM_14 // Filter selection S3
#define TCS230_OE_PIN  GPIO_NUM_12 // Output Enable Pin
#define TCS230_OUT_PIN GPIO_NUM_13 // Output Sensor
#define TCS230_RGB_SIZE 3

// DISPLAY LCD I2C
#define SDA_LCD_PIN GPIO_NUM_21
#define SCL_LCD_PIN GPIO_NUM_22
#define LCD_I2C_ADDR 0x27

// COMUNICAÇÂO SERIAL UART
#define UART_NUM UART_NUM_0

/******************** INSTANCES ********************/

// volatile uint32_t TCS230::_pulseCounter;

TCS230 tcs(
    TCS230_OUT_PIN, 
    TCS230_S2_PIN, 
    TCS230_S3_PIN, 
    TCS230_S0_PIN, 
    TCS230_S1_PIN, 
    TCS230_OE_PIN
);

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 20, 4);

AccelStepper magazine(AccelStepper::DRIVER, MAGAZINE_PUL_PIN, MAGAZINE_DIR_PIN);

/**************** GLOBAL VARIABLES *****************/ 

static const char * LCD_TAG = "LCD";
static const char * UART_TAG = "UART";
static const char * TCS230_TAG = "TCS230";
static const char * CC_MOTOR_TAG = "CC_MOTOR";
static const char * MAGAZINE_TAG = "MAGAZINE";

static const char * versao = "1.0.0";

static QueueHandle_t uart_queue;
static QueueHandle_t gpio_event_queue = NULL;

uint32_t ccMotorDuty = 500;

sensorData raw;
colorData rgb;

// create a block caracter for lcd
uint8_t block[8] = {
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111
};

ledc_timer_config_t timer = {                   // Confuguração do timer

    .speed_mode      = LEDC_LOW_SPEED_MODE,     // Modo de Velocidade
    .duty_resolution = CC_MOTOR_RESOLUTION,     // Resolução do ciclo de trabalho (2^13 = 8192 valores | 0 ~ 8191)
    .timer_num       = LEDC_TIMER_0,            // Utilizado o TIMER 0
    .freq_hz         = CC_MOTOR_FREQ,           // Frequência de opperação do sinal PWM
    .clk_cfg         = LEDC_AUTO_CLK            // Seleção automática da fonte geradora do clock (interna ou externa)

};
ledc_channel_config_t channel_0 = {             // Configuração do canal de PWM

    .gpio_num   = CC_MOTOR_ENA,                 // Pino de saído do PWM
    .speed_mode = LEDC_LOW_SPEED_MODE,          // Modo de velocidade
    .channel    = LEDC_CHANNEL_0,               // Canal a vincular ao GPIO
    .duty       = ccMotorDuty,                  // Duty cicle do PWM
    .hpoint     = 0

};

/******************** INTERRUPTS ********************/
    // GPIO ISR HANDLER (IHM BUTTONS, SENSORS)

static void IRAM_ATTR gpio_isr_handler(void *arg){
    if(xQueueIsQueueFullFromISR(gpio_event_queue) == pdFALSE){

        uint32_t gpio_num = (uint32_t) arg;
        xQueueSendFromISR(gpio_event_queue, &gpio_num, NULL);

    }else{
        xQueueReset(gpio_event_queue);
    }
}

/******************** FUNCTIONS ********************/
    // CONVEYOR MOTOR CONTROL FUNCS
    // MAGAZINE MOTOR CONTROL FUNCS
    // COLOR SENSOR READ FUNC
    // DISPLAY FUNCS
    // UART READ FUNCS
    // UART WRITE FUNCS
    // INIT SETUP FUNC

/******************************************************************/
void motorByFadeTime(){
    digitalWrite(CC_MOTOR_IN1, HIGH);
    digitalWrite(CC_MOTOR_IN2, LOW);

    ledc_set_fade_time_and_start(
        channel_0.speed_mode, 
        channel_0.channel,
        TOP, 
        CC_MOTOR_FADE_TIME, 
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    ledc_set_fade_time_and_start(
        channel_0.speed_mode, 
        channel_0.channel, 
        0, 
        CC_MOTOR_FADE_TIME, 
        LEDC_FADE_WAIT_DONE
    );

    digitalWrite(CC_MOTOR_IN1, LOW);
    digitalWrite(CC_MOTOR_IN2, HIGH);
    
    ledc_set_fade_time_and_start(
        channel_0.speed_mode, 
        channel_0.channel,
        TOP, 
        CC_MOTOR_FADE_TIME, 
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    ledc_set_fade_time_and_start(
        channel_0.speed_mode, 
        channel_0.channel, 
        0, 
        CC_MOTOR_FADE_TIME, 
        LEDC_FADE_WAIT_DONE
    );

    digitalWrite(CC_MOTOR_IN1, LOW);
    digitalWrite(CC_MOTOR_IN2, LOW);

    delay(1000);
}

void motorByFadeStep(){

    ledc_set_duty_and_update(channel_0.speed_mode, channel_0.channel, 0, 0);

    delay(300);

    digitalWrite(CC_MOTOR_IN1, HIGH);
    digitalWrite(CC_MOTOR_IN2, LOW);

    ledc_set_fade_step_and_start(
        channel_0.speed_mode, 
        channel_0.channel,
        TOP, 
        CC_MOTOR_FADE_STEP,
        1, 
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    ledc_set_fade_step_and_start(
        channel_0.speed_mode, 
        channel_0.channel, 
        900, 
        CC_MOTOR_FADE_STEP,
        1, 
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    digitalWrite(CC_MOTOR_IN1, LOW);
    digitalWrite(CC_MOTOR_IN2, HIGH);
    
    ledc_set_fade_step_and_start(
        channel_0.speed_mode, 
        channel_0.channel,
        TOP, 
        CC_MOTOR_FADE_STEP,
        1,
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    digitalWrite(CC_MOTOR_IN1, LOW);
    digitalWrite(CC_MOTOR_IN2, LOW);
    
    ledc_set_fade_step_and_start(
        channel_0.speed_mode, 
        channel_0.channel, 
        0, 
        4095,
        1, 
        LEDC_FADE_WAIT_DONE
    );    

    delay(1000);
}

/********************** TASKS ***********************/
    // PRINCIPAL TASK AND IHM MONITOR (VARIABLES AND DISPLAY UPDATE, MOTOR CONTROL AND READ SENSORS)
    // SERIAL PROTOCOL CONTROL

/********************** SETUP **********************/
void setup(void){
    // SETUP AND TASK CREATE
    // Serial.begin(115200);
    // tcs.begin();
    // tcs.setSampling(500);

    // sensorData darkCal = {
    //     .value = {18091, 14092, 19023}
    // };
    // sensorData whiteCal = {
    //     .value = {219059, 210688, 285216}
    // };

    // tcs.setDarkCal(&darkCal);
    // tcs.setWhiteCal(&whiteCal);

    lcd.init();

    lcd.createChar(0, block);

    lcd.backlight();
    lcd.setCursor(6,0);
    lcd.print("MYT-D600");
    lcd.setCursor(6,1);
    lcd.print("v: ");
    lcd.print(versao);

    // create a loading bar to show the user that the system is starting
    lcd.setCursor(4,2);
    lcd.print("Iniciando...");
    for(int i = 0; i < 20; i++){
        lcd.setCursor(i,3);
        lcd.write(255);
        delay(100);
    }
    delay(1000);

    lcd.clear();
    
    lcd.setCursor(0,0);
    lcd.print("MONITOR|SENSOR| MAG");
    lcd.setCursor(7,1);
    lcd.print("|R: 255| ~VR");
    lcd.setCursor(7,2);
    lcd.print("|G: 255|  VD");
    lcd.setCursor(7,3);
    lcd.print("|B: 255|  AZ");

    // ledc_timer_config(&timer);
    // ledc_channel_config(&channel_0);
    // ledc_fade_func_install(0);

    // pinMode(CC_MOTOR_IN1, OUTPUT);
    // pinMode(CC_MOTOR_IN2, OUTPUT);
}
/********************** LOOP **********************/
void loop(void){
    // tcs.read();
    // tcs.getRaw(&raw);
    // tcs.getRGB(&rgb);
    
    // printf("TCS230: RAW(%d, %d, %d)\n", raw.value[RED], raw.value[GREEN], raw.value[BLUE]);
    // printf("TCS230: RGB(%d, %d, %d)\n", rgb.value[RED], rgb.value[GREEN], rgb.value[BLUE]);

    // printf("TCS230: Color Read -> %s\n\n", tcs.getColorToString());
    // vTaskDelay(250 / portTICK_PERIOD_MS);
    // motorByFadeStep();

    // Tela 1
    




}
