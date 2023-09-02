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






// MOTOR DE PASSO DO MAGAZINE






// SENSOR DE COR






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
# 70 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino" 3 4
                                 __null
# 70 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\MYT_600.ino"
                                     ;

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
