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

#line 136 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void menuPrincipal();
#line 149 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void calibracao();
#line 160 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void acionamentos();
#line 173 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void creditos();
#line 186 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void monitoramento();
#line 210 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void motorByFadeTime();
#line 259 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void motorByFadeStep();
#line 324 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void setup(void);
#line 370 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
void loop(void);
#line 116 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
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

void menuPrincipal(){
    lcd.clear();
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("~ 1.ACIONAMENTOS");
    lcd.setCursor(0,1);
    lcd.print("  2.PROG ALUNO");
    lcd.setCursor(0,2);
    lcd.print("  3.CALIBRACAO");
    lcd.setCursor(0,3);
    lcd.print("  4.CREDITOS");
}

void calibracao(){
    lcd.clear();
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("~ 1.ESTEIRA");
    lcd.setCursor(0,1);
    lcd.print("  2.MAGAZINE");
    lcd.setCursor(0,2);
    lcd.print("  3.SENSOR");
}

void acionamentos(){
    lcd.clear();
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("~ 1.MODO PADRAO");
    lcd.setCursor(0,1);
    lcd.print("  2.CONTROLE ESTEIRA");
    lcd.setCursor(0,2);
    lcd.print("  3.CONTROLE MAG.");
    lcd.setCursor(0,3);
    lcd.print("  4.DETEC. CORES");
}

void creditos(){
    lcd.clear();
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("Por:Matheus Beirao");
    lcd.setCursor(0,1);
    lcd.print("    Yasmin Georgetti");
    lcd.setCursor(0,2);
    lcd.print("    Theo V. Pires");
    lcd.setCursor(0,3);
    lcd.print("    Denise Costa");
}

void monitoramento(){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("QTD|MODO OP: PROG.");
    lcd.setCursor(0,1);
    lcd.print("R~3|STATUS : ERRO12");
    lcd.setCursor(0,2);
    lcd.print("G~4|VEL(m/min):10");
    lcd.setCursor(0,3);
    lcd.print("B~1|PECAS/MIN : 2");
    // lcd.setCursor(3,0);
    // lcd.write(255);
    // lcd.setCursor(3,1);
    // lcd.write(255);
    // lcd.setCursor(3,2);
    // lcd.write(255);
    // lcd.setCursor(3,3);
    // lcd.write(255);
    lcd.setCursor(0,1);
    lcd.blink();
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

    lcd.backlight();
    lcd.setCursor(6,0);
    lcd.print("MYT-D600");
    lcd.setCursor(6,1);
    lcd.print("v: ");
    lcd.print(versao);

    lcd.setCursor(4,2);
    lcd.print("Iniciando...");
    for(int i = 0; i < 20; i++){
        lcd.setCursor(i,3);
        lcd.write(255);
        delay(100);
    }
    delay(1000);

    monitoramento();
    


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
    

    // menuPrincipal();
    // delay(1000);
    // calibracao();
    // delay(1000);
    // acionamentos();
    // delay(1000);
    // creditos();
    // delay(1000);
    // monitoramento();
    // delay(1000);

}

