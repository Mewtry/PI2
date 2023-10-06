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
typedef struct
{
  int32_t value[TCS230_RGB_SIZE];  ///< One element each for R, G and B raw data readings
} sensorData;

typedef struct
{
  uint8_t value[TCS230_RGB_SIZE];  ///< One element each for RGB evaluated color data (RGB value 0-255 or other)
} colorData;

volatile unsigned long pulseCounter = 0;
int timeReadPulses = 200; // Tempo em ms que será contato pulsos da saída do sensor para execução de um leitura de frequência

sensorData rawData;
sensorData rawDark;
sensorData rawWhite;

colorData rgb;

const char* colorsPrintable[TCS230_RGB_SIZE] = {"RED  ", "GREEN", "BLUE "};
enum colors {TCS230_RGB_R, TCS230_RGB_G, TCS230_RGB_B, TCS230_RGB_X};
enum freq {TCS230_FREQ_HI, TCS230_FREQ_MID, TCS230_FREQ_LO, TCS230_FREQ_OFF};



uint32_t ccMotorDuty = 500;

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

#line 148 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void beginTCS230();
#line 170 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void setEnableTCS230(bool b);
#line 174 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void setFrequencyTCS230(uint8_t f);
#line 185 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void setFilterTCS230(uint8_t c);
#line 196 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void RGBTransformation();
#line 217 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void readRGB();
#line 232 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void motorByFadeTime();
#line 281 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void motorByFadeStep();
#line 346 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void setup(void);
#line 368 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void loop(void);
#line 125 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void IRAM_ATTR pulseColorSensor(){
    if(pulseCounter < 4000000000){
        pulseCounter++;
    }
} // end pulseColorSensor

/******************** FUNCTIONS ********************/
    // CONVEYOR MOTOR CONTROL FUNCS
    // MAGAZINE MOTOR CONTROL FUNCS
    // COLOR SENSOR READ FUNC
    // DISPLAY FUNCS
    // UART READ FUNCS
    // UART WRITE FUNCS
    // INIT SETUP FUNC

/********************* TCS230 Configurações **********************/    
// S0  |S1 |Output Freq Scaling | |S2  |S3 |Photodiode Selection  |
// :--:|--:|:-------------------| |:--:|--:|:---------------------|
// L   |L  |Power Down          | |L   |L  |Red                   |
// L   |H  |2%                  | |L   |H  |Blue                  |
// H   |L  |20%                 | |H   |L  |Clear (no filter)     |
// H   |H  |100%                | |H   |H  |Green                 |
/*****************************************************************/
void beginTCS230(){
    pinMode(TCS230_S0_PIN, OUTPUT);
    pinMode(TCS230_S1_PIN, OUTPUT);
    pinMode(TCS230_S2_PIN, OUTPUT);
    pinMode(TCS230_S3_PIN, OUTPUT);
    pinMode(TCS230_OE_PIN, OUTPUT);
    pinMode(TCS230_OUT_PIN, INPUT);

    setEnableTCS230(false);
    setFrequencyTCS230(TCS230_FREQ_HI);

    rawDark.value[TCS230_RGB_R] = 18091;
    rawDark.value[TCS230_RGB_G] = 14092;
    rawDark.value[TCS230_RGB_B] = 19023;

    rawWhite.value[TCS230_RGB_R] = 219059;
    rawWhite.value[TCS230_RGB_G] = 210688;
    rawWhite.value[TCS230_RGB_B] = 285216;

    pulseCounter = 0;
}

void setEnableTCS230(bool b){
    digitalWrite(TCS230_OUT_PIN, (b) ? LOW : HIGH);
}

void setFrequencyTCS230(uint8_t f){
    switch (f)
    {
    case TCS230_FREQ_HI:  digitalWrite(TCS230_S0_PIN, HIGH); digitalWrite(TCS230_S1_PIN, HIGH); break;
    case TCS230_FREQ_MID: digitalWrite(TCS230_S0_PIN, HIGH); digitalWrite(TCS230_S1_PIN, LOW);  break;
    case TCS230_FREQ_LO:  digitalWrite(TCS230_S0_PIN, LOW);  digitalWrite(TCS230_S1_PIN, HIGH); break;
    case TCS230_FREQ_OFF: digitalWrite(TCS230_S0_PIN, LOW);  digitalWrite(TCS230_S1_PIN, LOW);  break;
    default: break;
    }
}

void setFilterTCS230(uint8_t c){
    switch (c)
    {
    case TCS230_RGB_R: digitalWrite(TCS230_S2_PIN, LOW);  digitalWrite(TCS230_S3_PIN, LOW);  break;
    case TCS230_RGB_G: digitalWrite(TCS230_S2_PIN, HIGH); digitalWrite(TCS230_S3_PIN, HIGH); break;
    case TCS230_RGB_B: digitalWrite(TCS230_S2_PIN, LOW);  digitalWrite(TCS230_S3_PIN, HIGH); break;
    case TCS230_RGB_X: digitalWrite(TCS230_S2_PIN, HIGH); digitalWrite(TCS230_S3_PIN, LOW);  break; // no filter
    default: break;
    }
}

void RGBTransformation(){
    int32_t x;

    for (uint8_t i=0; i<TCS230_RGB_SIZE; i++)
    {
        // Famosa regra de 3
        x = (rawData.value[i] - rawDark.value[i]) * 255;
        x /= (rawWhite.value[i] - rawDark.value[i]);

        // copia o resultado para as estruturas globais
        if (x < 0) rgb.value[i] = 0; 
        else if (x > 255) rgb.value[i] = 255;
        else rgb.value[i] = x;
    }
    printf("%s: RGB(%d, %d, %d)\n", TCS230_TAG, 
                                    rgb.value[TCS230_RGB_R], 
                                    rgb.value[TCS230_RGB_G], 
                                    rgb.value[TCS230_RGB_B]
    );
}

void readRGB(){
    for(uint8_t i=0; i<TCS230_RGB_SIZE; i++){
        pulseCounter = 0;
        setFilterTCS230(i);
        setEnableTCS230(true);
        attachInterrupt(TCS230_OUT_PIN, pulseColorSensor, RISING); // Habilita interrupção por borda de subida no pino de saída do sensor de cor
        vTaskDelay(timeReadPulses / portTICK_PERIOD_MS);
        setEnableTCS230(false);
        detachInterrupt(TCS230_OUT_PIN);
        rawData.value[i] = 1000 * pulseCounter / timeReadPulses;
        printf("%s: Raw %s %d\n", TCS230_TAG, colorsPrintable[i], rawData.value[i]);
    }
        RGBTransformation();
}
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
    Serial.begin(115200);
    beginTCS230();


    // lcd.init();
    // lcd.backlight();
    // lcd.setCursor(4,0);
    // lcd.print("MYT-D600");
    // lcd.setCursor(4,1);
    // lcd.print("v: ");
    // lcd.print(versao);

    // ledc_timer_config(&timer);
    // ledc_channel_config(&channel_0);
    // ledc_fade_func_install(0);

    // pinMode(CC_MOTOR_IN1, OUTPUT);
    // pinMode(CC_MOTOR_IN2, OUTPUT);
}
/********************** LOOP **********************/
void loop(void){
    readRGB();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // motorByFadeStep();
}
