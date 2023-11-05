#line 1 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\TCS230.cpp"
/**************************************************************************/
/**
 * @file    TCS230.cpp
 * @author  Theo Pires
 * @date    08/10/2023
 * @see     www.linkedin.com/in/theo-pires-a34b33183/
 * @brief   Arquivo da classe da biblioteca TCS230
*/
/**************************************************************************/

#include "TCS230.h"

#define DEBUG_TCS230 0

#if  DEBUG_TCS230
#define DUMP(s, v)  { Serial.print(F("TCS230: ")); Serial.print(F(s)); Serial.print(v); }  // s - info | v - value
#define DUMPS(s)    { Serial.print(F("TCS230: ")); Serial.print(F(s)); }           
#else
#define DUMP(s, v)  
#define DUMPS(s)    
#endif

void IRAM_ATTR TCS230::pulseCounterIntr(void * data){
    TCS230* dt = (TCS230*)data;
    if(dt->_pulseCounter < 4000000000){
        dt->_pulseCounter += 1;
    }
}

void TCS230::initialize(void) {
    _OE  = NO_PIN;
    _S0  = NO_PIN;
    _S1  = NO_PIN;
    _S2  = NO_PIN;
    _S3  = NO_PIN;
    _OUT = GPIO_NUM_NC;
    _readTime = 100;
    _pulseCounter = 0;
    _freqSet = TCS230_FREQ_HI;
    _readState = TCS230_READY;

    for (uint8_t i=0; i<RGB_SIZE; i++) {
        _fd.value[i] = 4000L;  // just typical values
        _fw.value[i] = 50000L; // just typical values
    }
}

TCS230::TCS230(gpio_num_t out, uint8_t s2, uint8_t s3) {
    initialize();
    _OUT = out;
    _S2  = s2;
    _S3  = s3;
}

TCS230::TCS230(gpio_num_t out, uint8_t s2, uint8_t s3, uint8_t oe) {
    initialize();
    _OUT = out;
    _S2  = s2;
    _S3  = s3;
    _OE  = oe;
}

TCS230::TCS230(gpio_num_t out, uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1) {
    initialize();
    _OUT = out;
    _S0  = s0;
    _S1  = s1;
    _S2  = s2;
    _S3  = s3;
}

TCS230::TCS230(gpio_num_t out, uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1, uint8_t oe) {
    initialize();
    _OUT = out;
    _S0  = s0;
    _S1  = s1;
    _S2  = s2;
    _S3  = s3;
    _OE  = oe;
}

TCS230::~TCS230(void) {}

void TCS230::begin() {
    if (_S0 != NO_PIN) pinMode(_S0, OUTPUT);
    if (_S1 != NO_PIN) pinMode(_S1, OUTPUT);
    if (_S2 != NO_PIN) pinMode(_S2, OUTPUT);
    if (_S3 != NO_PIN) pinMode(_S3, OUTPUT);
    if (_OE != NO_PIN) pinMode(_OE, OUTPUT);
    if (_OUT != GPIO_NUM_NC) {
        gpio_config_t io_conf {
            .pin_bit_mask  = 1ULL<<_OUT,
            .mode          = GPIO_MODE_INPUT,
            .pull_up_en    = GPIO_PULLUP_DISABLE,
            .pull_down_en  = GPIO_PULLDOWN_DISABLE,
            .intr_type     = GPIO_INTR_POSEDGE
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        (void)gpio_install_isr_service(0); // ignore errors as it could be already installed
        ESP_ERROR_CHECK(gpio_isr_handler_add(_OUT, pulseCounterIntr, this));
        gpio_intr_disable(_OUT);
    }

    setEnable(false);
    setFrequencyInternal(_freqSet);


    DUMPS("\nLibrary begin initialised");
}

void TCS230::setFilter(uint8_t f) {
    if ((_S2 == NO_PIN) || (_S3 == NO_PIN))
        return;

    DUMPS("\nsetFilter ");
    switch (f)
    {
        case TCS230_RGB_R:  DUMPS("R"); digitalWrite(_S2, LOW);   digitalWrite(_S3, LOW);   break;
        case TCS230_RGB_G:  DUMPS("G"); digitalWrite(_S2, HIGH);  digitalWrite(_S3, HIGH);  break;
        case TCS230_RGB_B:  DUMPS("B"); digitalWrite(_S2, LOW);   digitalWrite(_S3, HIGH);  break;
        case TCS230_RGB_X:  DUMPS("X"); digitalWrite(_S2, HIGH);  digitalWrite(_S3, LOW);   break;
        default:  DUMP("Unknown filter option", f); break;
    }
}

void TCS230::setFrequencyInternal(uint8_t f) {
    if ((_S0 == NO_PIN) || (_S1 == NO_PIN)) {
        DUMPS("Pins not set properly for this function");
        return;
    }

    DUMPS("\nsetFrequency ");
    switch (f)
    {
        case TCS230_FREQ_HI:  DUMPS("HI");  digitalWrite(_S0, HIGH);  digitalWrite(_S1, HIGH);  break;
        case TCS230_FREQ_MID: DUMPS("MID"); digitalWrite(_S0, HIGH);  digitalWrite(_S1, LOW);   break;
        case TCS230_FREQ_LO:  DUMPS("LO");  digitalWrite(_S0, LOW);   digitalWrite(_S1, HIGH);  break;
        case TCS230_FREQ_OFF: DUMPS("OFF"); digitalWrite(_S0, LOW);   digitalWrite(_S1, LOW);   break;
        default:  DUMP("Unknown freq option", f);	break;
    }
}

void TCS230::setFrequency(uint8_t f) {
    _freqSet = f;
    setFrequencyInternal(f);
}

void TCS230::setSampling(uint16_t t) {
    if (_readTime > 0 && _readTime <= 1000)
        _readTime = t;
}

void TCS230::setEnable(bool b) {
    if(b)
        DUMPS("\nEnabling using");
    else
        DUMPS("\nDisabling using");
    
    if (_OE != NO_PIN) {
        DUMPS("OE");
        digitalWrite(_OE, (b) ? LOW : HIGH);	// reverse logic
    }
    else {
        DUMPS("FREQ");
        setFrequencyInternal((b) ? _freqSet : TCS230_FREQ_OFF);
    }
}

void TCS230::setDarkCal(sensorData *d) {
    if (d == NULL)
        return;

    DUMPS("\nsetDarkCal");
    for (uint8_t i=0; i<RGB_SIZE; i++) {
        DUMP(" ", d->value[i]);
        _fd.value[i] = d->value[i];
    }
}

void TCS230::darkCalibration(void) {
    DUMPS("\ndarkCalibration");
    read();
    setDarkCal(&_fo);
}

void TCS230::setWhiteCal(sensorData *d) {
    if (d == NULL)
        return;

    DUMPS("\nsetWhiteCal");
        for (uint8_t i=0; i<RGB_SIZE; i++) {
        DUMP(" ", d->value[i]);
        _fw.value[i] = d->value[i];
    }
}

void TCS230::whiteCalibration(void) {
    DUMPS("\nwhiteCalibration");
    read();
    setWhiteCal(&_fo);
}

// get the rgb value of the current reading
void TCS230::getRGB(colorData *rgb) {
    if (rgb == NULL)
        return;

    DUMPS("\ngetRGB");
    for (uint8_t i=0; i<RGB_SIZE; i++) {
        rgb->value[i] = _rgb.value[i];
        DUMP(" ", rgb->value[i]);
    }
}

void TCS230::getRaw(sensorData *d) {
// get the raw data of the current reading
// useful to set dark and white calibration data
    if (d == NULL)
        return;

    DUMPS("\ngetRAW");
    for (uint8_t i=0; i<RGB_SIZE; i++) {
        d->value[i] = _fo.value[i];
        DUMP(" ", d->value[i]);
    }
}

uint8_t TCS230::getRed(void) {
    return _rgb.value[RED];
}

uint8_t TCS230::getGreen(void) {
    return _rgb.value[GREEN];
}

uint8_t TCS230::getBlue(void) {
    return _rgb.value[BLUE];
}

uint32_t TCS230::getRawRed(void) {
    return _fo.value[RED];
}

uint32_t TCS230::getRawGreen(void) {
    return _fo.value[GREEN];
}

uint32_t TCS230::getRawBlue(void) {
    return _fo.value[BLUE];
}

uint8_t TCS230::getColor(void) {
    return _color;
}

char * TCS230::getColorToString(void) {
    switch (_color) {
        case BLACK: return "BLACK";
        case WHITE: return "WHITE";
        case RED:   return "RED";
        case GREEN: return "GREEN";
        case BLUE:  return "BLUE";
        case GRAY:  return "GRAY";
        default:    return "UNKNOWN";
    }
}

void TCS230::setDarkSensitive(uint8_t d) {
    if(d > 0 && d < 255)
        _ds = d;
}

void TCS230::setWhiteSensitive(uint16_t d) {
    if(d > 0 && d < 765)
        _ws = d;
}

// blocking read of a single sensor value (ie, not rgb)
uint32_t TCS230::readSingle(void) {
    DUMPS("\nreadSingle");
    _readState = TCS230_READING;
    _pulseCounter = 0;
    setEnable(true);
    // attachInterrupt(_OUT, pulseCounterIntr, RISING); // Habilita interrupção por borda de subida no pino de saída do sensor de cor
    gpio_intr_enable(_OUT);
    vTaskDelay(_readTime / portTICK_PERIOD_MS);
    setEnable(false);
    // detachInterrupt(_OUT);
    gpio_intr_disable(_OUT);
    _readState = TCS230_READY;
    return ( 1000 * _pulseCounter / _readTime);
}

void TCS230::read(void) {
    _readState = TCS230_READING;
    for(uint8_t i=0; i<RGB_SIZE; i++) {
        _pulseCounter = 0;
        setFilter(i);
        setEnable(true);
        // attachInterrupt(_OUT, pulseCounterIntr, RISING); // Habilita interrupção por borda de subida no pino de saída do sensor de cor
        gpio_intr_enable(_OUT);
        vTaskDelay(_readTime / portTICK_PERIOD_MS);
        setEnable(false);
        // detachInterrupt(_OUT);
        gpio_intr_disable(_OUT);
        _fo.value[i] = 1000 * _pulseCounter / _readTime;
    }
    _readState = TCS230_READY;
    RGBTransformation();
    _color = ColorID();
}

bool TCS230::available(void) {
    return (_readState == 0);
}

void TCS230::RGBTransformation(void) {
    int32_t x;

    for (uint8_t i=0; i<RGB_SIZE; i++) {
        // Famosa regra de 3
        x = (_fo.value[i] - _fd.value[i]) * 255;
        x /= (_fw.value[i] - _fd.value[i]);

        // copia o resultado para as estruturas globais
        if (x < 0) _rgb.value[i] = 0; 
        else if (x > 255) _rgb.value[i] = 255;
        else _rgb.value[i] = x;
    }
}

uint8_t TCS230::ColorID(void) {
    int total = _rgb.value[RED]+_rgb.value[GREEN]+_rgb.value[BLUE];

    if (total < _ds)
        return BLACK;
    else if (total > _ws)
        return WHITE;
    else if (_rgb.value[RED] > _rgb.value[GREEN] && _rgb.value[RED] > _rgb.value[BLUE])
        return RED;
    else if (_rgb.value[GREEN] > _rgb.value[RED] && _rgb.value[GREEN] > _rgb.value[BLUE])
        return GREEN;
    else if (_rgb.value[BLUE] > _rgb.value[RED] && _rgb.value[BLUE] > _rgb.value[GREEN])
        return BLUE;
    else
        return GRAY;
}



