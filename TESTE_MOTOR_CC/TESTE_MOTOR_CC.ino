#include "driver/uart.h"
#include "driver/ledc.h"

#define BUF_SIZE (1024)
#define EX_UART_NUM UART_NUM_0
#define MOTOR_CC GPIO_NUM_25
#define FADE_TIME 5000 
#define FREQ 5000
#define TOP  8191

static QueueHandle_t uart0_queue;
static const char * PWM_TAG = "PWM";
static const char * UART_TAG = "UART";

uint32_t duty = TOP;
bool motorON = false;

static void gpio_task_PWM(void *arg){

    ledc_timer_config_t timer = {                   // Confuguração do timer

        .speed_mode      = LEDC_LOW_SPEED_MODE,     // Modo de Velocidade
        .duty_resolution = LEDC_TIMER_13_BIT,       // Resolução do ciclo de trabalho (2^13 = 8192 valores | 0 ~ 8191)
        .timer_num       = LEDC_TIMER_0,            // Utilizado o TIMER 0
        .freq_hz         = FREQ,                    // Frequência de opperação do sinal PWM
        .clk_cfg         = LEDC_AUTO_CLK            // Seleção automática da fonte geradora do clock (interna ou externa)
    
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t channel_1 = {             // Configuração do canal de PWM

        .gpio_num   = MOTOR_CC,                     // Pino de saído do PWM
        .speed_mode = LEDC_LOW_SPEED_MODE,          // Modo de velocidade
        .channel    = LEDC_CHANNEL_0,               // Canal a vincular ao GPIO
        .duty       = duty,                         // Duty cicle do PWM
        .hpoint     = 0

    };

    uint32_t lastDuty = duty;
    bool motorState = false;

    ledc_channel_config(&channel_1);
    ledc_fade_func_install(0);

    while(true){
        ledc_set_fade_time_and_start(channel_1.speed_mode, channel_1.channel,   0, FADE_TIME, LEDC_FADE_WAIT_DONE);
        ledc_set_fade_time_and_start(channel_1.speed_mode, channel_1.channel, TOP, FADE_TIME, LEDC_FADE_WAIT_DONE);
        if(motorState != motorON){
            if(motorON){
                printf("%s | Ligando motor...\n", PWM_TAG, duty);
                ledc_channel_config(&channel_1);
            }
            else{
                ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1);
            }
            motorState = motorON;
        }
        if(lastDuty != duty){
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            printf("%s | Duty atualizado : %d\n", PWM_TAG, duty);
            lastDuty = duty;
        }
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

int convertCstrToInt( char * dt, int pos){
    size_t len = strlen(dt);
    if(len > abs(pos)){
        if(pos > 0)
            dt = dt + pos;
        else
            dt[len + pos] = '\0';            
    }
    return atoi((const char *) dt);
}

// Executa funções a partir do comando recebido
void trataComandoRecebido(uint8_t * dt){
    printf("%s | Dado em tratamento: %s\r\n", UART_TAG, dt);
    if(dt[0] == '-'){
        duty = (duty + 1000) < TOP ? (duty + 1000) : TOP;
        printf("%s | Novo duty lido: %d\n", UART_TAG, duty);
        motorON = true;
    }
    else if(dt[0] == '+'){
        duty = duty > 1000 ? (duty - 1000) : 0;
        printf("%s | Novo duty lido: %d\n", UART_TAG, duty);
        motorON = true;
    }
    else if(dt[0] == '0'){
        motorON = false;
        printf("%s | Comando de parar recebido\n", UART_TAG);
    }
} // end tratarDadosRecebidos

// Task que monitora os eventos UART e trata cada um deles
static void uart_event_task(void *pvParameters){
    // Cria um manipulador de evento
    uart_event_t event;

    // Aloca o buffer de memória, do tamanho epecificado em BUF_SIZE
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE+1);
    int len = 0;

    while(1){
        // Primeiro aguardamos pela ocorrência de um evento e depois analisamos seu tipo
        if (xQueueReceive(uart0_queue, (void *) &event, (portTickType) portMAX_DELAY)){
            // Ocorreu um evento, então devemos analisar seu tipo e então finalizar o loop
            switch (event.type)
            {
            case UART_DATA:
                len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 200 / portTICK_RATE_MS);
                if(len > 0){
                    data[len] = '\0';  // Trunca o buffer para trabalhar como uma string                   
                    printf("%s | Dado recebido: %s\r\n", UART_TAG, data);
                    if(data[len-1] == '\n' || data[len-1] == '\r' || data[len-1] == ' ' || data[len-1] == '+' || data[len-1] == '-' || data[len-1] == '0'){
                        // data[len-1] = 0;
                        trataComandoRecebido(data);
                    }
                }
                break;
            case UART_FIFO_OVF:
                ESP_LOGE(UART_TAG, "Evento: hw overflow");
                uart_flush(EX_UART_NUM);
                break;
            case UART_BUFFER_FULL:
                // Neste caso o dado provavelmente não estará completo, devemos tratá-lo para não perder info
                ESP_LOGW(UART_TAG, "Evento: Dado > buffer");
                uart_flush(EX_UART_NUM);
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
}

void IniciaUart(){
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
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart0_queue, 0);

    // Configura interrupção por padrão de caracter. Padrão '\n'(ASCII) '0x0a'(HEX) '10'(DEC)
    //uart_enable_pattern_det_intr(EX_UART_NUM, 0x0a, 3, 10000, 10, 10); // Função desatualizada
    uart_enable_pattern_det_baud_intr(UART_NUM_1, 0x0a, 1, 9, 0, 0); 

    printf("UART configurada\n");

    // Cria a task no nucleo 0 com prioridade 1
    xTaskCreate(uart_event_task, "uart_event_task", 10000, NULL, 1, NULL);

} // end uart_init

void setup(){

    IniciaUart();

    // Cria tarefa de controle do PWM
    xTaskCreate(gpio_task_PWM, "gpio_task_PWM", 4096, NULL, 8, NULL);
}

void loop(){

}