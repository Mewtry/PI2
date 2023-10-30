#include <Arduino.h>
#line 1 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
/**************************************************************************/
/**
 * @file    MYT_600.ino
 * @author  Theo Pires
 * @date    26/08/2023
 * @see     www.linkedin.com/in/theo-pires-a34b33183/
*/
/**************************************************************************/

#include <Wire.h>
#include <ArduinoJson.h>
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
#define BUF_SIZE (1024)

// BOTÕES DA IHM
#define KEY_LEFT_PIN  GPIO_NUM_36
#define KEY_RIGHT_PIN GPIO_NUM_39
#define KEY_UP_PIN    GPIO_NUM_34
#define KEY_DOWN_PIN  GPIO_NUM_35
#define KEY_ENTER_PIN GPIO_NUM_32
#define PIN_MASK (1ULL << KEY_LEFT_PIN) | (1ULL << KEY_RIGHT_PIN) | (1ULL << KEY_UP_PIN) | (1ULL << KEY_DOWN_PIN) | (1ULL << KEY_ENTER_PIN)

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
gpio_num_t buttonPins[] = {KEY_LEFT_PIN, KEY_RIGHT_PIN, KEY_UP_PIN, KEY_DOWN_PIN, KEY_ENTER_PIN};
bool modoPadrao = true;

enum telasIHM {
    INICIALIZACAO     = 0,
    MENU_PRINCIPAL    = 1,
    MONITORAMENTO     = 2,

    // Menu anterior * 10 + linhaAtual
    MENU_ACIONAMENTOS = 10, 
    MENU_PROG_ALUNO   = 11,
    MENU_CALIBRACAO   = 12,
    MENU_CREDITOS     = 13,

    MENU_ESTEIRA      = 100,
    MENU_MAGAZINE     = 101,
    MENU_SENSOR       = 102,

    MENU_CAL_ESTEIRA  = 120,
    MENU_CAL_MAGAZINE = 121,
    MENU_CAL_SENSOR   = 122
};

enum keys {
    KEY_NONE,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_UP,
    KEY_DOWN,
    KEY_ENTER
};

enum caracters {
    ARROW_UP,
    ARROW_DOWN,
    ARROW_CW,
    ARROW_CCW,
    ENTER
};

uint8_t arrowUp[8] = {
    0b00000,
    0b00100,
    0b01110,
    0b10101,
    0b00100,
    0b00100,
    0b00000,
    0b00000
};

uint8_t arrowDown[8] = {
    0b00000,
    0b00100,
    0b00100,
    0b10101,
    0b01110,
    0b00100,
    0b00000,
    0b00000
};

uint8_t arrowCW[8] = {
    0b00000,
    0b01101,
    0b10011,
    0b10111,
    0b10000,
    0b10000,
    0b01110
};

uint8_t arrowCCW[8] = {
    0b00000,
    0b10110,
    0b11001,
    0b11101,
    0b00001,
    0b00001,
    0b01110
};

uint8_t enter[8] = {
    0b00001,
    0b00001,
    0b00101,
    0b01101,
    0b11111,
    0b01100,
    0b00100
};

uint32_t keyPressed = KEY_NONE;
uint8_t  telaAtual = INICIALIZACAO;
uint8_t  telaAnterior = INICIALIZACAO;
uint8_t  subMenuAtual = 0;
uint8_t  linhaAtual = 0;

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

#line 247 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void keyLeft();
#line 256 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void keyRight();
#line 262 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void keyUp();
#line 266 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void keyDown();
#line 270 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void keyEnter();
#line 297 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void atualizaTela();
#line 335 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void inicializacao();
#line 355 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void menuPrincipal();
#line 369 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void monitoramento();
#line 398 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void menuAcionamentos();
#line 413 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void menuCalibracao();
#line 428 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void menuCreditos();
#line 440 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void acionamentoEsteira();
#line 460 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void acionamentoMagazine();
#line 481 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void uartBegin();
#line 509 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void responseOK();
#line 519 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void responseError( uint8_t code, const char * message);
#line 532 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void trataComandoRecebido(uint8_t * dt);
#line 574 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void gpioBegin();
#line 599 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void motorByFadeTime();
#line 648 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void motorByFadeStep();
#line 713 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
static void uart_event_task(void *pvParameters);
#line 761 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
static void principal_task(void *pvParameters);
#line 792 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void setup(void);
#line 829 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
void loop(void);
#line 225 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
static void IRAM_ATTR gpio_isr_handler(void *arg){
    if(xQueueIsQueueFullFromISR(gpio_event_queue) == pdFALSE){

        uint32_t gpio_num = (uint32_t) arg;
        xQueueSendFromISR(gpio_event_queue, &gpio_num, NULL);

    }else{
        // xQueueReset(gpio_event_queue);
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

void keyLeft(){
    if(subMenuAtual == 0)
        telaAtual > 1 ? telaAtual-- : telaAtual = 2;
    else if (telaAtual != MENU_ESTEIRA){
        telaAtual = telaAtual / 10;
        subMenuAtual--;
    }
}

void keyRight(){
    if(subMenuAtual == 0){
        telaAtual < 2 ? telaAtual++ : telaAtual = 1;
    }
}

void keyUp(){
    linhaAtual > 0 ? linhaAtual-- : linhaAtual = 3;
}

void keyDown(){
    linhaAtual < 3 ? linhaAtual++ : linhaAtual = 0;
}

void keyEnter(){
    if(subMenuAtual == 0){
        telaAtual = telaAtual * 10 + linhaAtual;
        subMenuAtual++;
    } else if (telaAtual == MENU_ACIONAMENTOS){
        if(linhaAtual == 0)
            modoPadrao = !modoPadrao;
        else if(linhaAtual == 1){
            telaAtual = MENU_ESTEIRA;
            subMenuAtual++;
        }
        else if(linhaAtual == 2){
            telaAtual = MENU_MAGAZINE;
            subMenuAtual++;
        }
        else if(linhaAtual == 3){
            telaAtual = MENU_SENSOR;
            subMenuAtual++;
        }
    }
    else if(telaAtual == MENU_ESTEIRA){
        telaAtual = telaAtual / 10;
        subMenuAtual--;
    }
}


void atualizaTela(){
    if(telaAtual != telaAnterior){
        lcd.clear();
        telaAnterior = telaAtual;
    }
    switch (telaAtual)
    {
    case INICIALIZACAO:
        inicializacao();
        break;
    case MENU_PRINCIPAL:
        menuPrincipal();
        break;
    case MONITORAMENTO:
        monitoramento();
        break;
    case MENU_ACIONAMENTOS:
        menuAcionamentos();
        break;
    case MENU_PROG_ALUNO:
        break;
    case MENU_CALIBRACAO:
        menuCalibracao();
        break;
    case MENU_CREDITOS:
        menuCreditos();
        break;
    case MENU_ESTEIRA:
        acionamentoEsteira();
        break;
    case MENU_MAGAZINE:
        acionamentoMagazine();
        break;
    default:
        break;
    }
}

void inicializacao(){
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
    lcd.clear();
    telaAtual = MONITORAMENTO;
    atualizaTela();
}

void menuPrincipal(){
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("  1.ACIONAMENTOS");
    lcd.setCursor(0,1);
    lcd.print("  2.PROG ALUNO");
    lcd.setCursor(0,2);
    lcd.print("  3.CALIBRACAO");
    lcd.setCursor(0,3);
    lcd.print("  4.CREDITOS");
    lcd.setCursor(0,linhaAtual);
    lcd.print("~");
}

void monitoramento(){
    lcd.setCursor(0,0);
    lcd.print("QTD|MODO OP: PROG.");
    lcd.setCursor(0,1);
    lcd.print("R~3|STATUS : ERRO12");
    lcd.setCursor(0,2);
    lcd.print("G~4|VEL(m/min):10");
    lcd.setCursor(0,3);
    lcd.print("B~1|PECAS/MIN : 2");
    switch (tcs.getColor())
    {
    case RED:
        lcd.setCursor(0,1);
        lcd.blink();
        break;
    case GREEN:
        lcd.setCursor(0,2);
        lcd.blink();
        break;
    case BLUE:
        lcd.setCursor(0,3);
        lcd.blink();
        break;
    default:
        lcd.noBlink();
        break;
    }
}

void menuAcionamentos(){
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("  1.MODO: ");
    modoPadrao ? lcd.print("PADRAO") : lcd.print("PROG.");
    lcd.setCursor(0,1);
    lcd.print("  2.CONTROLE ESTEIRA");
    lcd.setCursor(0,2);
    lcd.print("  3.CONTROLE MAG.");
    lcd.setCursor(0,3);
    lcd.print("  4.DETEC. CORES");
    lcd.setCursor(0,linhaAtual);
    lcd.print("~");
}

void menuCalibracao(){
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("  1.ESTEIRA");
    lcd.setCursor(0,1);
    lcd.print("  2.MAGAZINE");
    lcd.setCursor(0,2);
    lcd.print("  3.SENSOR");
    if(linhaAtual > 2){
        linhaAtual = 0;
    }
    lcd.setCursor(0,linhaAtual);
    lcd.print("~");
}

void menuCreditos(){
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

void acionamentoEsteira(){
    lcd.setCursor(0,0);
    lcd.print("**CONTROLE ESTEIRA**");
    lcd.setCursor(0,1);
    lcd.print(" VOLTAR: ");
    lcd.write(ENTER);
    lcd.setCursor(0,2);
    lcd.print(" MOVER ");
    lcd.write(ARROW_CW);
    lcd.print(": ~  VEL+: ");
    lcd.write(ARROW_UP);
    lcd.setCursor(0,3);
    lcd.print(" MOVER ");
    lcd.write(ARROW_CCW);
    lcd.print(": ");
    lcd.write(127);
    lcd.print("  VEL-: ");
    lcd.write(ARROW_DOWN);
}

void acionamentoMagazine() {
    lcd.setCursor(0,0);
    lcd.print("*CONTROLE  MAGAZINE*");
    lcd.setCursor(0,1);
    lcd.print(" VOLTAR: ");
    lcd.write(ENTER);
    lcd.setCursor(0,2);
    lcd.print(" MOVER ");
    lcd.write(ARROW_CW);
    lcd.print(": ~  VEL+: ");
    lcd.write(ARROW_UP);
    lcd.setCursor(0,3);
    lcd.print(" MOVER ");
    lcd.write(ARROW_CCW);
    lcd.print(": ");
    lcd.write(127);
    lcd.print("  VEL-: ");
    lcd.write(ARROW_DOWN);
}


void uartBegin(){
    // Cria a estrutura com dados de configuração da UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Configura UART com as informações setadas acima
    uart_param_config(UART_NUM_0, &uart_config);

    // Configura os pinos como padrão para a UART0
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Configura a instalação do driver para UART0
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart_queue, 0);

    // Configura interrupção por padrão de caracter. Padrão '\n'(ASCII) '0x0a'(HEX) '10'(DEC)
    //uart_enable_pattern_det_intr(EX_UART_NUM, 0x0a, 3, 10000, 10, 10); // Função desatualizada
    uart_enable_pattern_det_baud_intr(UART_NUM_1, 0x0a, 1, 9, 0, 0); 

    // Cria a task no nucleo 0 com prioridade 1
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 2, NULL);

} // end uart_init

void responseOK(){
    char * output = (char *) malloc((sizeof(char) * 50));
    StaticJsonDocument<50> json_OUT;
    json_OUT["type"] = "response";
    json_OUT["status"] = "ok";
    serializeJson(json_OUT, output, 50);
    printf("%s\r\n", output);
    free(output);
}

void responseError( uint8_t code, const char * message){
    char * output = (char *) malloc((sizeof(char) * 200));
    StaticJsonDocument<200> json_OUT;
    json_OUT["type"] = "response";
    json_OUT["status"] = "error";
    JsonObject error = json_OUT.createNestedObject("error");
    error["code"] = code;
    error["message"] = message;
    serializeJson(json_OUT, output, 200);
    printf("%s\r\n", output);
    free(output);
}

void trataComandoRecebido(uint8_t * dt){
    // printf("Dado em tratamento: %s\r\n", dt);
    if(dt[0] == 'v'){
        printf("Versao: %s\r\n", versao);
        return;
    }
    if(dt[0] == '{'){
        // printf("JSON: %s\r\n", dt);
        StaticJsonDocument<200> json_IN;
        DeserializationError err = deserializeJson(json_IN, dt);
        if(err){
            responseError(err.code(), err.c_str());
            return;
        }

        const char * jsonType = json_IN["type"];
        if(jsonType){
            if( ! strcmp(jsonType, "teste")){
                responseOK();
                return;
            }
            if( ! strcmp(jsonType, "menu")){
                telaAtual = json_IN["menu"];
                subMenuAtual = json_IN["submenu"];
                responseOK();
                return;
            }
            if( ! strcmp(jsonType, "emulate")){
                uint32_t key = json_IN["key"];
                xQueueSend(gpio_event_queue, &key, 0);
                responseOK();
                return;
            }
        }
        else{
            responseError(99, "Tipo de comando não reconhecido");
            return;
        }
    }

}

void gpioBegin(){
    gpio_config_t io_config = {                 // Configuração do pino de interrupção
        .pin_bit_mask = PIN_MASK,               // Máscara de seleção dos pinos
        .mode         = GPIO_MODE_INPUT,        // Modo de operação do pino
        .pull_up_en   = GPIO_PULLUP_ENABLE,     // Habilita resistor de pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Desabilita resistor de pull-down
        .intr_type    = GPIO_INTR_ANYEDGE       // Tipo de interrupção
    };

    gpio_config(&io_config);                    // Chama a função para configurar o GPIO

    // Cria um fila de eventos para lidar com as interrupções do GPIO
    gpio_event_queue = xQueueCreate(50, sizeof(uint32_t));

    // Instala o manipulador de interrupção GPIO
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

    // Configura o manipulador de interrupção GPIO
    for(int i = 0; i < 5; i++){
        gpio_isr_handler_add(buttonPins[i], gpio_isr_handler, (void *) buttonPins[i]);
    }

} // end gpioBegin

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

// Task que monitora os eventos UART e trata cada um deles
static void uart_event_task(void *pvParameters){
    // Cria um manipulador de evento
    uart_event_t event;

    // Aloca o buffer de memória, do tamanho epecificado em BUF_SIZE
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE+1);
    int len = 0;

    while(1){
        // Primeiro aguardamos pela ocorrência de um evento e depois analisamos seu tipo
        if (xQueueReceive(uart_queue, (void *) &event, (portTickType) portMAX_DELAY)){
            // Ocorreu um evento, então devemos analisar seu tipo e então finalizar o loop
            switch (event.type)
            {
            case UART_DATA:
                len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 200 / portTICK_RATE_MS);
                if(len > 0){
                    data[len] = '\0';  // Trunca o buffer para trabalhar como uma string                   
                    // printf("Dado recebido: %s\r\n", data);
                    if(data[len-1] == '\n' || data[len-1] == '\r' || data[len-1] == ' '){
                        data[len-1] = 0;
                        trataComandoRecebido(data);
                    }
                }
                break;
            case UART_FIFO_OVF:
                ESP_LOGE(UART_TAG, "Evento: hw overflow");
                uart_flush(UART_NUM_0);
                break;
            case UART_BUFFER_FULL:
                // Neste caso o dado provavelmente não estará completo, devemos tratá-lo para não perder info
                ESP_LOGW(UART_TAG, "Evento: Dado > buffer");
                uart_flush(UART_NUM_0);
                break;
            default:
                // Evento desconhecido
                ESP_LOGE(UART_TAG, "Evento: Erro desconhecido");
                break;
            }
        }
    }
    // Desacola a memória dinâmica criada na task
    free(data);
    data = NULL;
    // Deleta a task após a sua conclusão
    vTaskDelete(NULL);
} // end uart_event_task

static void principal_task(void *pvParameters){

    while(true){
        if(xQueueReceive(gpio_event_queue, &keyPressed, pdMS_TO_TICKS(1000))){
            switch (keyPressed)
            {
            case KEY_LEFT:
                keyLeft();
                break;
            case KEY_RIGHT:
                keyRight();
                break;
            case KEY_UP:
                keyUp();
                break;
            case KEY_DOWN:
                keyDown();
                break;
            case KEY_ENTER:
                keyEnter();
                break;
            default:
                break;
            }
        }

        atualizaTela();
    }
}

/********************** SETUP **********************/
void setup(void){
    // SETUP AND TASK CREATE
    // Serial.begin(115200);
    uartBegin();

    tcs.begin();
    tcs.setSampling(500);
    sensorData darkCal = {
        .value = {4162, 3764, 5166}
    };
    sensorData whiteCal = {
        .value = {50551, 46568, 60065}
    };
    tcs.setDarkCal(&darkCal);
    tcs.setWhiteCal(&whiteCal);

    gpioBegin();

    lcd.init();
    lcd.createChar(ARROW_UP,   arrowUp  );
    lcd.createChar(ARROW_DOWN, arrowDown);
    lcd.createChar(ARROW_CW,   arrowCW  );
    lcd.createChar(ARROW_CCW,  arrowCCW );
    lcd.createChar(ENTER,      enter    );
    lcd.backlight();
    atualizaTela();

    xTaskCreate(principal_task, "principal_task", 4096, NULL, 3, NULL); // Cria a task com prioridade 3

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

    // tela(telaAtual);
    // delay(1000);

}

