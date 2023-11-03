# 1 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
/**************************************************************************/
/**

 * @file    MYT_600.ino

 * @author  Theo Pires

 * @date    26/08/2023

 * @see     www.linkedin.com/in/theo-pires-a34b33183/

*/
# 8 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
/**************************************************************************/

# 11 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2
# 12 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2
# 13 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2
# 14 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2
# 15 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2
# 16 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2
# 17 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2

# 19 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2
# 20 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2

# 22 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2
# 23 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 2

/********************* DEFINES *********************/

// MOTOR CC DA ESTEIRA
# 35 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
// MOTOR DE PASSO DO MAGAZINE






// SENSOR DE COR
# 51 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
// DISPLAY LCD I2C




// COMUNICAÇÂO SERIAL UART



// BOTÕES DA IHM







/******************** ESTRUTURAS *******************/

enum telas_ihm {
    INICIALIZACAO = 0,
    MENU_PRINCIPAL = 1,
        MENU_ACIONAMENTOS = 10,
         // MODO OP   
            MENU_ESTEIRA = 101,
            MENU_MAGAZINE = 102,
            MENU_SENSOR = 103,
        MENU_PROG_ALUNO = 11,
        MENU_CONFIGURACAO = 12,
            MENU_CAL_ESTEIRA = 120,
            MENU_CAL_MAGAZINE = 121,
            MENU_CAL_SENSOR = 122,
         // RESTAURAR CONFIG
        MENU_CREDITOS = 13,
    MONITORAMENTO = 2
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
    ENTER,
    POW_2
};
enum sentido_rotacao {
    CW,
    CCW
};
enum status {
    STATE_OK,
    RUNNING,
    ERROR_1,
    ERROR_2,
    ERROR_3,
    ERROR_4,
    ERROR_5,
    ERROR_6,
    ERROR_7,
    ERROR_8,
    ERROR_9,
    ERROR_10,
};

typedef struct {
    ledc_timer_config_t timer;
    ledc_channel_config_t channel;
    uint32_t duty;
    uint32_t duty_acionamento;
    uint32_t duty_max;
    uint32_t velocidade;
    uint32_t rampa_acel;
    uint32_t rampa_acel_max;
    uint32_t rampa_acel_min;
    uint8_t pecas_per_min;
    bool sentido;
    bool is_running;
} esteira_config_t;

typedef struct {
    uint32_t velocidade;
    uint32_t velocidade_acionamento;
    uint32_t velocidade_max;
    uint32_t aceleracao;
    uint32_t aceleracao_max;
    uint32_t steps_per_rev;
    uint8_t position;
} magazine_config_t;

typedef struct {
    sensorData fd;
    sensorData fw;
    sensorData raw;
    colorData rgb;
    uint16_t read_time;
} tcs_config_t;

typedef struct {
    uint8_t tela_atual;
    uint8_t tela_anterior;
    uint8_t linha_atual;
    uint8_t linha_min;
    uint8_t linha_max;
    bool linha_selecionada;
    uint32_t key_pressed; // mudar para uint8_t ?
    gpio_num_t button_pins[8];
} ihm_config_t;

typedef struct {
    esteira_config_t esteira;
    magazine_config_t magazine;
    tcs_config_t tcs;
    ihm_config_t ihm;
    bool operation_mode;
    uint8_t status;
    char status_printable[12][8];
    uint8_t qtd_pecas[3];
} app_config_t;

typedef struct {
    esteira_config_t esteira;
    magazine_config_t magazine;
    tcs_config_t tcs;
} aluno_config_t;


/******************** INSTANCES ********************/

TCS230 tcs(
    GPIO_NUM_13 /* Output Sensor*/,
    GPIO_NUM_27 /* Filter selection S2*/,
    GPIO_NUM_14 /* Filter selection S3*/,
    GPIO_NUM_25 /* Output frequency scaling S0*/,
    GPIO_NUM_26 /* Output frequency scaling S1*/,
    GPIO_NUM_12 /* Output Enable Pin*/
);

LiquidCrystal_I2C lcd(0x27, 20, 4);

AccelStepper magazine(AccelStepper::DRIVER, GPIO_NUM_16, GPIO_NUM_17);

/**************** GLOBAL VARIABLES *****************/

static const char * LCD_TAG = "LCD";
static const char * UART_TAG = "UART";
static const char * TCS230_TAG = "TCS230";
static const char * ESTEIRA_TAG = "ESTEIRA";
static const char * MAGAZINE_TAG = "MAGAZINE";

static const char * versao = "1.0.0";

static QueueHandle_t uart_queue;
static QueueHandle_t gpio_event_queue = 
# 210 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 3 4
                                       __null
# 210 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
                                           ;

// declaração das estruturas de app e aluno
app_config_t app = {
    .esteira = {
        .timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_12_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 1000,
            .clk_cfg = LEDC_AUTO_CLK
        },
        .channel = {
            .gpio_num = GPIO_NUM_5,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .duty = 0,
            .hpoint = 0
        },
        .duty = 0,
        .duty_acionamento = 0,
        .duty_max = 4095,
        .velocidade = 0,
        .rampa_acel = 5000,
        .rampa_acel_max = 9999,
        .rampa_acel_min = 1000,
        .pecas_per_min = 0,
        .sentido = CW,
        .is_running = false
    },
    .magazine = {
        .velocidade = 96,
        .velocidade_max = 240,
        .aceleracao = 240,
        .aceleracao_max = 600,
        .steps_per_rev = 48,
        .position = 0
    },
    .tcs = {
        .fd = {4162, 3764, 5166},
        .fw = {50551, 46568, 60065},
        .read_time = 100
    },
    .ihm = {
        .tela_atual = INICIALIZACAO,
        .tela_anterior = INICIALIZACAO,
        .linha_atual = 0,
        .linha_min = 0,
        .linha_max = 3,
        .linha_selecionada = false,
        .key_pressed = KEY_NONE,
        .button_pins = {GPIO_NUM_36, GPIO_NUM_39, GPIO_NUM_34, GPIO_NUM_35, GPIO_NUM_32}
    },
    .operation_mode = true,
    .status = STATE_OK,
    .status_printable = {
        "OK     ",
        "RUNNING",
        "ERROR 1",
        "ERROR 2",
        "ERROR 3",
        "ERROR 4",
        "ERROR 5",
        "ERROR 6",
        "ERROR 7",
        "ERROR 8",
        "ERROR 9",
        "ERROR10"
    },
    .qtd_pecas = {0, 0, 0}
};
aluno_config_t aluno = {
    .esteira = {
        .timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_12_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 1000,
            .clk_cfg = LEDC_AUTO_CLK
        },
        .channel = {
            .gpio_num = GPIO_NUM_5,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .duty = 0,
            .hpoint = 0
        },
        .duty = 0,
        .duty_max = 4095,
        .velocidade = 0,
        .rampa_acel = 5000,
        .rampa_acel_max = 9999,
        .rampa_acel_min = 1000,
        .pecas_per_min = 0,
        .sentido = CW,
        .is_running = false
    },
    .magazine = {
        .velocidade = 96,
        .velocidade_max = 240,
        .aceleracao = 240,
        .aceleracao_max = 600,
        .steps_per_rev = 48,
        .position = 0
    },
    .tcs = {
        .fd = {4162, 3764, 5166},
        .fw = {50551, 46568, 60065},
        .read_time = 100
    }
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
uint8_t pow_2[8] = {
    0b01110,
    0b00001,
    0b00110,
    0b01000,
    0b01111,
    0b00000,
    0b00000
};


/******************** INTERRUPTS ********************/

static void __attribute__((section(".iram1" "." "28"))) gpio_isr_handler(void *arg){
    if(xQueueIsQueueFullFromISR(gpio_event_queue) == ( ( BaseType_t ) 0 )) {

        uint32_t gpio_num = (uint32_t) arg;
        xQueueGenericSendFromISR( ( gpio_event_queue ), ( &gpio_num ), ( 
# 386 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 3 4
       __null 
# 386 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
       ), ( ( BaseType_t ) 0 ) );
    }
}

/******************** FUNCTIONS ********************/

void keyLeft(){
    if(app.ihm.linha_selecionada)
        app.ihm.linha_selecionada = false;
    else if(app.ihm.tela_atual < 10)
        app.ihm.tela_atual > 1 ? app.ihm.tela_atual-- : app.ihm.tela_atual = 2;

    else if(app.ihm.tela_atual != MENU_ESTEIRA && app.ihm.tela_atual != MENU_MAGAZINE)
        app.ihm.tela_atual = app.ihm.tela_atual / 10;

    else if(app.ihm.tela_atual == MENU_ESTEIRA)
        printf("Mover esteira sentido Anti-Horario\n");

    else if(app.ihm.tela_atual == MENU_MAGAZINE)
        printf("Mover magazine sentido Anti-Horario\n");
}
void keyRight(){
    if(app.ihm.tela_atual < 10)
        app.ihm.tela_atual < 2 ? app.ihm.tela_atual++ : app.ihm.tela_atual = 1;

    else if(app.ihm.tela_atual == MENU_ESTEIRA)
        printf("Mover esteira sentido Horario\n");

    else if(app.ihm.tela_atual == MENU_MAGAZINE)
        printf("Mover magazine sentido Horario\n");

}
void keyUp(){
    if(app.ihm.linha_max != 0 && app.ihm.linha_selecionada == false)
        app.ihm.linha_atual > app.ihm.linha_min ? app.ihm.linha_atual-- : app.ihm.linha_atual = app.ihm.linha_max;

    if(app.ihm.tela_atual == MENU_ESTEIRA)
        app.esteira.duty_acionamento < app.esteira.duty_max-100 ? app.esteira.duty_acionamento+=100 : app.esteira.duty_acionamento = app.esteira.duty_max;

    else if(app.ihm.tela_atual == MENU_MAGAZINE)
        app.magazine.velocidade_acionamento < app.magazine.velocidade_max-6 ? app.magazine.velocidade_acionamento+=6 : app.magazine.velocidade_acionamento = app.magazine.velocidade_max;

    else if(app.ihm.tela_atual == MENU_CAL_ESTEIRA && app.ihm.linha_selecionada) {
        if(app.ihm.linha_atual == 1)
            app.esteira.duty < app.esteira.duty_max-100 ? app.esteira.duty+=100 : app.esteira.duty = app.esteira.duty_max;

        else if(app.ihm.linha_atual == 2)
            app.esteira.rampa_acel < app.esteira.rampa_acel_max-100 ? app.esteira.rampa_acel+=100 : app.esteira.rampa_acel = app.esteira.rampa_acel_max;

        else if(app.ihm.linha_atual == 3)
            app.esteira.sentido = !app.esteira.sentido;
    }
    else if(app.ihm.tela_atual == MENU_CAL_MAGAZINE && app.ihm.linha_selecionada) {
        if(app.ihm.linha_atual == 1)
            app.magazine.velocidade < app.magazine.velocidade_max-6 ? app.magazine.velocidade+=6 : app.magazine.velocidade = app.magazine.velocidade_max;

        else if(app.ihm.linha_atual == 2)
            app.magazine.aceleracao < app.magazine.aceleracao_max-6 ? app.magazine.aceleracao+=6 : app.magazine.aceleracao = app.magazine.aceleracao_max;

        else if(app.ihm.linha_atual == 3)
            app.magazine.steps_per_rev < 60 ? app.magazine.steps_per_rev++ : app.magazine.steps_per_rev = 60;
    }
    else if(app.ihm.tela_atual == MENU_CAL_SENSOR && app.ihm.linha_selecionada) {
        if(app.ihm.linha_atual == 3) {
            app.tcs.read_time < 900 ? app.tcs.read_time+=100 : app.tcs.read_time = 1000;
            tcs.setSampling(app.tcs.read_time);
        }
    }
}
void keyDown(){
    if(app.ihm.linha_max != 0 && app.ihm.linha_selecionada == false)
        app.ihm.linha_atual < app.ihm.linha_max ? app.ihm.linha_atual++ : app.ihm.linha_atual = app.ihm.linha_min;

    if(app.ihm.tela_atual == MENU_ESTEIRA)
        app.esteira.duty_acionamento > 100 ? app.esteira.duty_acionamento-=100 : app.esteira.duty_acionamento = 0;

    else if(app.ihm.tela_atual == MENU_MAGAZINE)
        app.magazine.velocidade_acionamento > 6 ? app.magazine.velocidade_acionamento-=6 : app.magazine.velocidade_acionamento = 0;

    else if(app.ihm.tela_atual == MENU_CAL_ESTEIRA && app.ihm.linha_selecionada) {
        if(app.ihm.linha_atual == 1)
            app.esteira.duty > 100 ? app.esteira.duty-=100 : app.esteira.duty = 0;

        else if(app.ihm.linha_atual == 2)
            app.esteira.rampa_acel > 100 ? app.esteira.rampa_acel-=100 : app.esteira.rampa_acel = 0;

        else if(app.ihm.linha_atual == 3)
            app.esteira.sentido = !app.esteira.sentido;
    }
    else if(app.ihm.tela_atual == MENU_CAL_MAGAZINE && app.ihm.linha_selecionada) {
        if(app.ihm.linha_atual == 1)
            app.magazine.velocidade > 6 ? app.magazine.velocidade-=6 : app.magazine.velocidade = 0;

        else if(app.ihm.linha_atual == 2)
            app.magazine.aceleracao > 6 ? app.magazine.aceleracao-=6 : app.magazine.aceleracao = 0;

        else if(app.ihm.linha_atual == 3)
            app.magazine.steps_per_rev > 20 ? app.magazine.steps_per_rev-- : app.magazine.steps_per_rev = 20;
    }
    else if(app.ihm.tela_atual == MENU_CAL_SENSOR && app.ihm.linha_selecionada) {
        if(app.ihm.linha_atual == 3) {
            app.tcs.read_time > 200 ? app.tcs.read_time-=100 : app.tcs.read_time = 100;
            tcs.setSampling(app.tcs.read_time);
        }
    }
}
void keyEnter(){
    if(app.ihm.linha_selecionada){
        app.ihm.linha_selecionada = false;
        // passar o valor da linha para a variavel de controle
    }
    else if(app.ihm.tela_atual == MENU_PRINCIPAL || (app.ihm.tela_atual == MENU_ACIONAMENTOS && app.ihm.linha_atual != 0) || (app.ihm.tela_atual == MENU_CONFIGURACAO && app.ihm.linha_atual != 3))
        app.ihm.tela_atual = app.ihm.tela_atual * 10 + app.ihm.linha_atual;

    else if(app.ihm.tela_atual == MENU_ACIONAMENTOS && app.ihm.linha_atual == 0)
        app.operation_mode = !app.operation_mode;

    else if(app.ihm.tela_atual == MENU_ESTEIRA || app.ihm.tela_atual == MENU_MAGAZINE)
        app.ihm.tela_atual = app.ihm.tela_atual / 10;

    else if(app.ihm.tela_atual == MENU_CONFIGURACAO && app.ihm.linha_atual == 3)
        esp_restart();

    else if(app.ihm.tela_atual == MENU_CAL_ESTEIRA && app.ihm.linha_atual != 0)
        app.ihm.linha_selecionada = true;

    else if(app.ihm.tela_atual == MENU_CAL_MAGAZINE && app.ihm.linha_atual != 0)
        app.ihm.linha_selecionada = true;

    else if(app.ihm.tela_atual == MENU_CAL_SENSOR){
        if(app.ihm.linha_atual == 1)
            tcs.whiteCalibration();

        else if(app.ihm.linha_atual == 2)
            tcs.darkCalibration();

        else if(app.ihm.linha_atual == 3)
            app.ihm.linha_selecionada = true;

    }

}

void atualizaTela() {
    if(app.ihm.tela_atual != app.ihm.tela_anterior){
        lcd.clear();
        app.ihm.linha_atual = 0;
        app.ihm.tela_anterior = app.ihm.tela_atual;
        if(app.ihm.tela_atual == MENU_ESTEIRA || app.ihm.tela_atual == MENU_MAGAZINE){
            app.esteira.duty_acionamento = app.esteira.duty;
            app.magazine.velocidade_acionamento = app.magazine.velocidade;
        }
    }
    switch (app.ihm.tela_atual)
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
        menuProgAluno();
        break;
    case MENU_CONFIGURACAO:
        menuConfiguracao();
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
    case MENU_SENSOR:
        detecSensor();
        break;
    case MENU_CAL_ESTEIRA:
        configEsteira();
        break;
    case MENU_CAL_MAGAZINE:
        configMagazine();
        break;
    case MENU_CAL_SENSOR:
        configSensor();
        break;
    default:
        menuPrincipal();
        break;
    }
}
void inicializacao() {
    app.ihm.linha_max = 0; // Desabilita a seleção de linha
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
    app.ihm.tela_atual = MONITORAMENTO;
    atualizaTela();
}
void menuPrincipal() {
    app.ihm.linha_min = 0;
    app.ihm.linha_max = 3;
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("  1.ACIONAMENTOS");
    lcd.setCursor(0,1);
    lcd.print("  2.PROG ALUNO");
    lcd.setCursor(0,2);
    lcd.print("  3.CONFIGURACAO");
    lcd.setCursor(0,3);
    lcd.print("  4.CREDITOS");
    lcd.setCursor(0,app.ihm.linha_atual);
    lcd.print("~");
}
void monitoramento() {
    app.ihm.linha_max = 0; // Desabilita a seleção de linha
    lcd.setCursor(0,0);
    lcd.print("QTD|MODO OP: ");
    app.operation_mode ? lcd.print("PADRAO ") : lcd.print("PROG.  ");
    lcd.setCursor(0,1);
    lcd.print("R~");
    lcd.print(app.qtd_pecas[RED]);
    lcd.print("|STATUS : ");
    lcd.print(app.status_printable[app.status]);
    lcd.setCursor(0,2);
    lcd.print("G~");
    lcd.print(app.qtd_pecas[GREEN]);
    lcd.print("|VEL(duty): ");
    lcd.print(app.esteira.duty);
    lcd.setCursor(0,3);
    lcd.print("B~");
    lcd.print(app.qtd_pecas[BLUE]);
    lcd.print("|PECAS/MIN: 2");
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
void menuAcionamentos() {
    app.ihm.linha_min = 0;
    app.ihm.linha_max = 3;
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("  1.MODO:       ");
    lcd.setCursor(10,0);
    app.operation_mode ? lcd.print("PADRAO") : lcd.print("PROG.");
    lcd.setCursor(0,1);
    lcd.print("  2.CONTROLE ESTEIRA");
    lcd.setCursor(0,2);
    lcd.print("  3.CONTROLE MAG.");
    lcd.setCursor(0,3);
    lcd.print("  4.DETEC. CORES");
    lcd.setCursor(0,app.ihm.linha_atual);
    lcd.print("~");
}
void menuProgAluno() {
    app.ihm.linha_max = 0; // Desabilita a seleção de linha
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("  Utilizar a porta  ");
    lcd.setCursor(0,1);
    lcd.print("serial p/ o controle");
    lcd.setCursor(0,2);
    lcd.print("Projeto: https://git");
    lcd.setCursor(0,3);
    lcd.print("hub.com/Mewtry/PI2  ");
}
void menuConfiguracao() {
    app.ihm.linha_min = 0;
    app.ihm.linha_max = 3;
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("  1.ESTEIRA         ");
    lcd.setCursor(0,1);
    lcd.print("  2.MAGAZINE        ");
    lcd.setCursor(0,2);
    lcd.print("  3.SENSOR TCS230   ");
    lcd.setCursor(0,3);
    lcd.print("  4.RESTAURAR CONFIG");
    lcd.setCursor(0,app.ihm.linha_atual);
    lcd.print("~");
}
void menuCreditos() {
    app.ihm.linha_max = 0; // Desabilita a seleção de linha
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("Por:Matheus Beirao  ");
    lcd.setCursor(0,1);
    lcd.print("    Yasmin Georgetti");
    lcd.setCursor(0,2);
    lcd.print("    Theo V. Pires   ");
    lcd.setCursor(0,3);
    lcd.print("    Denise Costa    ");
}
void acionamentoEsteira() {
    app.ihm.linha_max = 0; // Desabilita a seleção de linha
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("**CONTROLE*ESTEIRA**");
    lcd.setCursor(0,1);
    lcd.print("VOLTAR: ");
    lcd.write(ENTER);
    lcd.setCursor(0,2);
    lcd.print("MOVER ");
    lcd.write(ARROW_CW);
    lcd.print(": ~   VEL+: ");
    lcd.write(ARROW_UP);
    lcd.setCursor(0,3);
    lcd.print("MOVER ");
    lcd.write(ARROW_CCW);
    lcd.print(": ");
    lcd.write(127);
    lcd.print("   VEL-: ");
    lcd.write(ARROW_DOWN);
}
void acionamentoMagazine() {
    app.ihm.linha_max = 0; // Desabilita a seleção de linha
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("*CONTROLE**MAGAZINE*");
    lcd.setCursor(0,1);
    lcd.print("VOLTAR: ");
    lcd.write(ENTER);
    lcd.setCursor(0,2);
    lcd.print("MOVER ");
    lcd.write(ARROW_CW);
    lcd.print(": ~   VEL+: ");
    lcd.write(ARROW_UP);
    lcd.setCursor(0,3);
    lcd.print("MOVER ");
    lcd.write(ARROW_CCW);
    lcd.print(": ");
    lcd.write(127);
    lcd.print("   VEL-: ");
    lcd.write(ARROW_DOWN);
}
void detecSensor() {
    /* tcs.getRaw(main.tcs.raw)*/
    tcs.getRaw(&app.tcs.raw);
    tcs.getRGB(&app.tcs.rgb);

    app.ihm.linha_max = 0; // Desabilita a seleção de linha

    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("   LEITURA TCS230   ");
    lcd.setCursor(0,1);
    lcd.print("Raw:R      ");
    lcd.setCursor(5,1);
    lcd.print(app.tcs.raw.value[RED]);
    lcd.setCursor(11,1);
    lcd.print("RGB={   ");
    lcd.setCursor(16,1);
    if(app.tcs.rgb.value[RED] <= 9)
        lcd.setCursor(18,1);
    else if(app.tcs.rgb.value[RED] <= 99)
        lcd.setCursor(17,1);
    lcd.print(app.tcs.rgb.value[RED]);
    lcd.setCursor(19,1);
    lcd.print(",");
    lcd.setCursor(0,2);
    lcd.print("    G              ");
    lcd.setCursor(5,2);
    lcd.print(app.tcs.raw.value[GREEN]);
    if(app.tcs.rgb.value[GREEN] <= 9)
        lcd.setCursor(18,2);
    else if(app.tcs.rgb.value[GREEN] <= 99)
        lcd.setCursor(17,2);
    else
        lcd.setCursor(16,2);
    lcd.print(app.tcs.rgb.value[GREEN]);
    lcd.setCursor(19,2);
    lcd.print(",");
    lcd.setCursor(0,3);
    lcd.print("    B              ");
    lcd.setCursor(5,3);
    lcd.print(app.tcs.raw.value[BLUE]);
    if(app.tcs.rgb.value[BLUE] <= 9)
        lcd.setCursor(18,3);
    else if(app.tcs.rgb.value[BLUE] <= 99)
        lcd.setCursor(17,3);
    else
        lcd.setCursor(16,3);
    lcd.print(app.tcs.rgb.value[BLUE]);
    lcd.setCursor(19,3);
    lcd.print("}");
}
void configEsteira() {
    if(app.ihm.linha_atual == 0)
        app.ihm.linha_atual = 1;
    app.ihm.linha_min = 1;
    app.ihm.linha_max = 3;
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("   CONFIG ESTEIRA   ");
    lcd.setCursor(0,1);
    lcd.print("  Vel(duty):        ");
    lcd.setCursor(13,1);
    lcd.print(app.esteira.duty); // 0 a 4095
    lcd.setCursor(0,2);
    lcd.print("  Rampa(ms): ");
    lcd.print(app.esteira.rampa_acel); // 1000 a 9999
    lcd.setCursor(0,3);
    lcd.print("  Sentido  : ");
    app.esteira.sentido ? lcd.print("CW ") : lcd.print("CCW");
    lcd.setCursor(0,app.ihm.linha_atual);
    lcd.print("~");
    if(app.ihm.linha_selecionada){
        lcd.setCursor(12, app.ihm.linha_atual);
        lcd.print("#");
    }
}
void configMagazine() {
    if(app.ihm.linha_atual == 0)
        app.ihm.linha_atual = 1;
    app.ihm.linha_min = 1;
    app.ihm.linha_max = 3;
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("  CONFIG  MAGAZINE  ");
    lcd.setCursor(0,1);
    lcd.print("  Vel(step/s) :     ");
    lcd.setCursor(16,1);
    lcd.print(app.magazine.velocidade);
    lcd.setCursor(0,2);
    lcd.print("  Acc(step/s");
    lcd.write(POW_2);
    lcd.print("): ");
    lcd.print(app.magazine.aceleracao);
    lcd.setCursor(0,3);
    lcd.print("  Steps/rev   :     ");
    lcd.setCursor(16,3);
    lcd.print(app.magazine.steps_per_rev);
    lcd.setCursor(0,app.ihm.linha_atual);
    lcd.print("~");
    if(app.ihm.linha_selecionada){
        lcd.setCursor(15, app.ihm.linha_atual);
        lcd.print("#");
    }
}
void configSensor() {
    if(app.ihm.linha_atual == 0)
        app.ihm.linha_atual = 1;
    app.ihm.linha_min = 1;
    app.ihm.linha_max = 3;
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print("   CONFIG  TCS230   ");
    lcd.setCursor(0,1);
    lcd.print("  Calibrar_branco() ");
    lcd.setCursor(0,2);
    lcd.print("  Calibrar_preto()  ");
    lcd.setCursor(0,3);
    lcd.print("  Amostragem:     ms");
    lcd.setCursor(14,3);
    lcd.print(app.tcs.read_time);
    lcd.setCursor(0,app.ihm.linha_atual);
    lcd.print("~");
    if(app.ihm.linha_selecionada){
        lcd.setCursor(13, app.ihm.linha_atual);
        lcd.print("#");
    }
}


void uartBegin(){
    // Cria a estrutura com dados de configuração da UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Configura UART com as informações setadas acima
    uart_param_config((0) /*!< UART port 0 */, &uart_config);

    // Configura os pinos como padrão para a UART0
    uart_set_pin((0) /*!< UART port 0 */, (-1), (-1), (-1), (-1));

    // Configura a instalação do driver para UART0
    uart_driver_install((0) /*!< UART port 0 */, (1024) * 2, (1024) * 2, 10, &uart_queue, 0);

    // Configura interrupção por padrão de caracter. Padrão '\n'(ASCII) '0x0a'(HEX) '10'(DEC)
    //uart_enable_pattern_det_intr(EX_UART_NUM, 0x0a, 3, 10000, 10, 10); // Função desatualizada
    uart_enable_pattern_det_baud_intr((1) /*!< UART port 1 */, 0x0a, 1, 9, 0, 0);

    // Cria a task no nucleo 0 com prioridade 1
    xTaskCreate(uart_event_task, "uart_event_task", 4096, 
# 909 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 3 4
                                                         __null
# 909 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
                                                             , 2, 
# 909 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 3 4
                                                                  __null
# 909 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
                                                                      );

} // end uart_init
void gpioBegin(){
    gpio_config_t io_config = { // Configuração do pino de interrupção
        .pin_bit_mask = (1ULL << GPIO_NUM_36) | (1ULL << GPIO_NUM_39) | (1ULL << GPIO_NUM_34) | (1ULL << GPIO_NUM_35) | (1ULL << GPIO_NUM_32), // Máscara de seleção dos pinos
        .mode = GPIO_MODE_INPUT, // Modo de operação do pino
        .pull_up_en = GPIO_PULLUP_ENABLE, // Habilita resistor de pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Desabilita resistor de pull-down
        .intr_type = GPIO_INTR_ANYEDGE // Tipo de interrupção
    };

    gpio_config(&io_config); // Chama a função para configurar o GPIO

    // Cria um fila de eventos para lidar com as interrupções do GPIO
    gpio_event_queue = xQueueGenericCreate( ( 50 ), ( sizeof(uint32_t) ), ( ( ( uint8_t ) 0U ) ) );

    // Instala o manipulador de interrupção GPIO
    gpio_install_isr_service((1<<9) /*|< Edge-triggered interrupt*/);

    // Configura o manipulador de interrupção GPIO
    for(int i = 0; i < 5; i++){
        gpio_isr_handler_add(app.ihm.button_pins[i], gpio_isr_handler, (void *) app.ihm.button_pins[i]);
    }

} // end gpioBegin

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
                app.ihm.tela_atual = json_IN["menu"];
                responseOK();
                return;
            }
            if( ! strcmp(jsonType, "emulate")){
                uint32_t key = json_IN["key"];
                xQueueGenericSend( ( gpio_event_queue ), ( &key ), ( 0 ), ( ( BaseType_t ) 0 ) );
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




void motorByFadeTime(){
    digitalWrite(GPIO_NUM_18, 0x1);
    digitalWrite(GPIO_NUM_19, 0x0);

    ledc_set_fade_time_and_start(
        app.esteira.channel.speed_mode,
        app.esteira.channel.channel,
        4095,
        5000,
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    ledc_set_fade_time_and_start(
        app.esteira.channel.speed_mode,
        app.esteira.channel.channel,
        0,
        5000,
        LEDC_FADE_WAIT_DONE
    );

    digitalWrite(GPIO_NUM_18, 0x0);
    digitalWrite(GPIO_NUM_19, 0x1);

    ledc_set_fade_time_and_start(
        app.esteira.channel.speed_mode,
        app.esteira.channel.channel,
        4095,
        5000,
        LEDC_FADE_WAIT_DONE
    );

    delay(2000);

    ledc_set_fade_time_and_start(
        app.esteira.channel.speed_mode,
        app.esteira.channel.channel,
        0,
        5000,
        LEDC_FADE_WAIT_DONE
    );

    digitalWrite(GPIO_NUM_18, 0x0);
    digitalWrite(GPIO_NUM_19, 0x0);

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
    uint8_t *data = (uint8_t *) malloc((1024)+1);
    int len = 0;

    while(1){
        // Primeiro aguardamos pela ocorrência de um evento e depois analisamos seu tipo
        if (xQueueReceive(uart_queue, (void *) &event, (TickType_t) ( TickType_t ) 0xffffffffUL)){
            // Ocorreu um evento, então devemos analisar seu tipo e então finalizar o loop
            switch (event.type)
            {
            case UART_DATA:
                len = uart_read_bytes((0) /*!< UART port 0 */, data, (1024), 200 / ( ( TickType_t ) 1000 / ( 
# 1070 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 3 4
                                                                       1000 
# 1070 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
                                                                       ) ));
                if(len > 0){
                    data[len] = '\0'; // Trunca o buffer para trabalhar como uma string                   
                    // printf("Dado recebido: %s\r\n", data);
                    if(data[len-1] == '\n' || data[len-1] == '\r' || data[len-1] == ' '){
                        data[len-1] = 0;
                        trataComandoRecebido(data);
                    }
                }
                break;
            case UART_FIFO_OVF:
                do {} while(0);
                uart_flush((0) /*!< UART port 0 */);
                break;
            case UART_BUFFER_FULL:
                // Neste caso o dado provavelmente não estará completo, devemos tratá-lo para não perder info
                do {} while(0);
                uart_flush((0) /*!< UART port 0 */);
                break;
            default:
                // Evento desconhecido
                do {} while(0);
                break;
            }
        }
    }
    // Desacola a memória dinâmica criada na task
    free(data);
    data = 
# 1098 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 3 4
          __null
# 1098 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
              ;
    // Deleta a task após a sua conclusão
    vTaskDelete(
# 1100 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 3 4
               __null
# 1100 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
                   );
} // end uart_event_task

static void principal_task(void *pvParameters){

    while(true){
        if(xQueueReceive(gpio_event_queue, &app.ihm.key_pressed, ( ( TickType_t ) ( ( ( TickType_t ) ( 1000 ) * ( TickType_t ) ( 
# 1106 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 3 4
                                                                1000 
# 1106 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
                                                                ) ) / ( TickType_t ) 1000U ) ))){
            switch (app.ihm.key_pressed)
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
        tcs.read();
        atualizaTela();
    }
}

/********************** SETUP **********************/
void setup(void){
    // SETUP AND TASK CREATE
    uartBegin();
    gpioBegin();

    tcs.begin();
    tcs.setSampling(app.tcs.read_time);
    tcs.setDarkCal(&app.tcs.fd);
    tcs.setWhiteCal(&app.tcs.fw);

    lcd.init();
    lcd.createChar(ARROW_UP, arrowUp );
    lcd.createChar(ARROW_DOWN, arrowDown);
    lcd.createChar(ARROW_CW, arrowCW );
    lcd.createChar(ARROW_CCW, arrowCCW );
    lcd.createChar(ENTER, enter );
    lcd.createChar(POW_2, pow_2 );
    lcd.backlight();
    atualizaTela();

    xTaskCreate(principal_task, "principal_task", 4096, 
# 1154 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 3 4
                                                       __null
# 1154 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
                                                           , 3, 
# 1154 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino" 3 4
                                                                __null
# 1154 "D:\\workspace\\IFSC\\PI2\\MYT_600\\MYT_600.ino"
                                                                    ); // Cria a task com prioridade 3

    // ledc_timer_config(&timer);
    // ledc_channel_config(&app.esteira.channel);
    // ledc_fade_func_install(0);

    // pinMode(ESTEIRA_IN1, OUTPUT);
    // pinMode(ESTEIRA_IN2, OUTPUT);
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

    // tela(app.ihm.tela_atual);
    // delay(1000);

}
