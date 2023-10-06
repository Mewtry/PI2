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

/********************* DEFINES *********************/

// MOTOR CC DA ESTEIRA
# 34 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
// MOTOR DE PASSO DO MAGAZINE






// SENSOR DE COR
# 50 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
// DISPLAY LCD I2C




// COMUNICAÇÂO SERIAL UART


/******************** INSTANCES ********************/

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
static QueueHandle_t gpio_queue = 
# 75 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 3 4
                                 __null
# 75 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
                                     ;
typedef struct
{
  int32_t value[3]; ///< One element each for R, G and B raw data readings
} sensorData;

typedef struct
{
  uint8_t value[3]; ///< One element each for RGB evaluated color data (RGB value 0-255 or other)
} colorData;

volatile unsigned long pulseCounter = 0;
int timeReadPulses = 200; // Tempo em ms que será contato pulsos da saída do sensor para execução de um leitura de frequência

sensorData rawData;
sensorData rawDark;
sensorData rawWhite;

colorData rgb;

const char* colorsPrintable[3] = {"RED  ", "GREEN", "BLUE "};
enum colors {TCS230_RGB_R, TCS230_RGB_G, TCS230_RGB_B, TCS230_RGB_X};
enum freq {TCS230_FREQ_HI, TCS230_FREQ_MID, TCS230_FREQ_LO, TCS230_FREQ_OFF};



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

void __attribute__((section(".iram1" "." "28"))) pulseColorSensor(){
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
    pinMode(GPIO_NUM_25 /* Output frequency scaling S0*/, 0x03);
    pinMode(GPIO_NUM_26 /* Output frequency scaling S1*/, 0x03);
    pinMode(GPIO_NUM_27 /* Filter selection S2*/, 0x03);
    pinMode(GPIO_NUM_14 /* Filter selection S3*/, 0x03);
    pinMode(GPIO_NUM_12 /* Output Enable Pin*/, 0x03);
    pinMode(GPIO_NUM_13 /* Output Sensor*/, 0x01);

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
    digitalWrite(GPIO_NUM_13 /* Output Sensor*/, (b) ? 0x0 : 0x1);
}

void setFrequencyTCS230(uint8_t f){
    switch (f)
    {
    case TCS230_FREQ_HI: digitalWrite(GPIO_NUM_25 /* Output frequency scaling S0*/, 0x1); digitalWrite(GPIO_NUM_26 /* Output frequency scaling S1*/, 0x1); break;
    case TCS230_FREQ_MID: digitalWrite(GPIO_NUM_25 /* Output frequency scaling S0*/, 0x1); digitalWrite(GPIO_NUM_26 /* Output frequency scaling S1*/, 0x0); break;
    case TCS230_FREQ_LO: digitalWrite(GPIO_NUM_25 /* Output frequency scaling S0*/, 0x0); digitalWrite(GPIO_NUM_26 /* Output frequency scaling S1*/, 0x1); break;
    case TCS230_FREQ_OFF: digitalWrite(GPIO_NUM_25 /* Output frequency scaling S0*/, 0x0); digitalWrite(GPIO_NUM_26 /* Output frequency scaling S1*/, 0x0); break;
    default: break;
    }
}

void setFilterTCS230(uint8_t c){
    switch (c)
    {
    case TCS230_RGB_R: digitalWrite(GPIO_NUM_27 /* Filter selection S2*/, 0x0); digitalWrite(GPIO_NUM_14 /* Filter selection S3*/, 0x0); break;
    case TCS230_RGB_G: digitalWrite(GPIO_NUM_27 /* Filter selection S2*/, 0x1); digitalWrite(GPIO_NUM_14 /* Filter selection S3*/, 0x1); break;
    case TCS230_RGB_B: digitalWrite(GPIO_NUM_27 /* Filter selection S2*/, 0x0); digitalWrite(GPIO_NUM_14 /* Filter selection S3*/, 0x1); break;
    case TCS230_RGB_X: digitalWrite(GPIO_NUM_27 /* Filter selection S2*/, 0x1); digitalWrite(GPIO_NUM_14 /* Filter selection S3*/, 0x0); break; // no filter
    default: break;
    }
}

void RGBTransformation(){
    int32_t x;

    for (uint8_t i=0; i<3; i++)
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
    for(uint8_t i=0; i<3; i++){
        pulseCounter = 0;
        setFilterTCS230(i);
        setEnableTCS230(true);
        attachInterrupt(GPIO_NUM_13 /* Output Sensor*/, pulseColorSensor, 0x01); // Habilita interrupção por borda de subida no pino de saída do sensor de cor
        vTaskDelay(timeReadPulses / ( ( TickType_t ) 1000 / ( 
# 223 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 3 4
                                   1000 
# 223 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
                                   ) ));
        setEnableTCS230(false);
        detachInterrupt(GPIO_NUM_13 /* Output Sensor*/);
        rawData.value[i] = 1000 * pulseCounter / timeReadPulses;
        //printf("%s: Raw %s %d\n", TCS230_TAG, colorsPrintable[i], rawData.value[i]);
    }
        RGBTransformation();
}
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
    vTaskDelay(1000 / ( ( TickType_t ) 1000 / ( 
# 370 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 3 4
                     1000 
# 370 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
                     ) ));
    // motorByFadeStep();
}
