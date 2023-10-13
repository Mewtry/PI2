# 1 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
/**************************************************************************/
/**

 * @file    MYT_600.ino

 * @author  Theo Pires

 * @date    26/08/2023

 * @see     www.linkedin.com/in/theo-pires-a34b33183/

*/
# 8 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
/**************************************************************************/

# 11 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 2
# 12 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 2
# 13 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 2
# 14 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 2
# 15 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 2
# 16 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 2

# 18 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 2
# 19 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 2

# 21 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 2
# 22 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 2

/********************* DEFINES *********************/

// MOTOR CC DA ESTEIRA
# 35 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
// MOTOR DE PASSO DO MAGAZINE






// SENSOR DE COR
# 51 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
// DISPLAY LCD I2C




// COMUNICAÇÂO SERIAL UART


/******************** INSTANCES ********************/

volatile uint32_t TCS230::_pulseCounter;

TCS230 tcs(
    GPIO_NUM_13 /* Output Sensor*/,
    GPIO_NUM_27 /* Filter selection S2*/,
    GPIO_NUM_14 /* Filter selection S3*/,
    GPIO_NUM_25 /* Output frequency scaling S0*/,
    GPIO_NUM_26 /* Output frequency scaling S1*/,
    GPIO_NUM_12 /* Output Enable Pin*/
);

LiquidCrystal_I2C lcd(0x27, 16, 2);

AccelStepper magazine(AccelStepper::DRIVER, GPIO_NUM_16, GPIO_NUM_17);

/**************** GLOBAL VARIABLES *****************/

static const char * LCD_TAG = "LCD";
static const char * UART_TAG = "UART";
static const char * TCS230_TAG = "TCS230";
static const char * CC_MOTOR_TAG = "CC_MOTOR";
static const char * MAGAZINE_TAG = "MAGAZINE";

static const char * versao = "1.0.0";

static QueueHandle_t uart_queue;
static QueueHandle_t gpio_event_queue = 
# 87 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 3 4
                                       __null
# 87 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
                                           ;

// const char* colorsPrintable[TCS230_RGB_SIZE] = {"RED  ", "GREEN", "BLUE "};

uint32_t ccMotorDuty = 500;

ledc_timer_config_t timer = { // Confuguração do timer

    .speed_mode = LEDC_LOW_SPEED_MODE, // Modo de Velocidade
    .duty_resolution = LEDC_TIMER_12_BIT, // Resolução do ciclo de trabalho (2^13 = 8192 valores | 0 ~ 8191)
    .timer_num = LEDC_TIMER_0, // Utilizado o TIMER 0
    .freq_hz = 1000, // Frequência de opperação do sinal PWM
    .clk_cfg = LEDC_AUTO_CLK // Seleção automática da fonte geradora do clock (interna ou externa)

};
ledc_channel_config_t channel_0 = { // Configuração do canal de PWM

    .gpio_num = GPIO_NUM_5, // Pino de saído do PWM
    .speed_mode = LEDC_LOW_SPEED_MODE, // Modo de velocidade
    .channel = LEDC_CHANNEL_0, // Canal a vincular ao GPIO
    .duty = ccMotorDuty, // Duty cicle do PWM
    .hpoint = 0

};

/******************** INTERRUPTS ********************/
    // GPIO ISR HANDLER (IHM BUTTONS, SENSORS)

static void __attribute__((section(".iram1" "." "28"))) gpio_isr_handler(void *arg){
    if(xQueueIsQueueFullFromISR(gpio_event_queue) == ( ( BaseType_t ) 0 )){

        uint32_t gpio_num = (uint32_t) arg;
        xQueueGenericSendFromISR( ( gpio_event_queue ), ( &gpio_num ), ( 
# 119 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 3 4
       __null 
# 119 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
       ), ( ( BaseType_t ) 0 ) );

    }else{
        xQueueGenericReset( gpio_event_queue, ( ( BaseType_t ) 0 ) );
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
    digitalWrite(GPIO_NUM_18, 0x1);
    digitalWrite(GPIO_NUM_19, 0x0);

    ledc_set_fade_time_and_start(
        channel_0.speed_mode,
        channel_0.channel,
        4095,
        5000,
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    ledc_set_fade_time_and_start(
        channel_0.speed_mode,
        channel_0.channel,
        0,
        5000,
        LEDC_FADE_WAIT_DONE
    );

    digitalWrite(GPIO_NUM_18, 0x0);
    digitalWrite(GPIO_NUM_19, 0x1);

    ledc_set_fade_time_and_start(
        channel_0.speed_mode,
        channel_0.channel,
        4095,
        5000,
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    ledc_set_fade_time_and_start(
        channel_0.speed_mode,
        channel_0.channel,
        0,
        5000,
        LEDC_FADE_WAIT_DONE
    );

    digitalWrite(GPIO_NUM_18, 0x0);
    digitalWrite(GPIO_NUM_19, 0x0);

    delay(1000);
}

void motorByFadeStep(){

    ledc_set_duty_and_update(channel_0.speed_mode, channel_0.channel, 0, 0);

    delay(300);

    digitalWrite(GPIO_NUM_18, 0x1);
    digitalWrite(GPIO_NUM_19, 0x0);

    ledc_set_fade_step_and_start(
        channel_0.speed_mode,
        channel_0.channel,
        4095,
        2,
        1,
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    ledc_set_fade_step_and_start(
        channel_0.speed_mode,
        channel_0.channel,
        900,
        2,
        1,
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    digitalWrite(GPIO_NUM_18, 0x0);
    digitalWrite(GPIO_NUM_19, 0x1);

    ledc_set_fade_step_and_start(
        channel_0.speed_mode,
        channel_0.channel,
        4095,
        2,
        1,
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    digitalWrite(GPIO_NUM_18, 0x0);
    digitalWrite(GPIO_NUM_19, 0x0);

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
    tcs.begin();
    tcs.setSampling(500);

    sensorData darkCal = {
        .value = {18091, 14092, 19023}
    };
    sensorData whiteCal = {
        .value = {219059, 210688, 285216}
    };

    tcs.setDarkCal(&darkCal);
    tcs.setWhiteCal(&whiteCal);
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
    tcs.read();
    printf("TCS230: Color Read -> %s\n", tcs.getColorToString());
    vTaskDelay(1000 / ( ( TickType_t ) 1000 / ( 
# 284 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 3 4
                     1000 
# 284 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
                     ) ));
    // motorByFadeStep();
}
