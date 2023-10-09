/**************************************************************************/
/**
 * @file    TCS230.cpp
 * @author  Theo Pires
 * @date    08/10/2023
 * @see     www.linkedin.com/in/theo-pires-a34b33183/
 * @brief   Arquivo da classe da biblioteca TCS230
*/
/**************************************************************************/

#include <TCS230.h>

#define DEBUG_TCS230 0

#if  DEBUG_TCS230
#define DUMP(s, v)  { Serial.print(F("TCS230: ")); Serial.print(F(s)); Serial.print(v); }  // s - info | v - value
#define DUMPS(s)    { Serial.print(F("TCS230: ")); Serial.print(F(s)); }           
#else
#define DUMP(s, v)  
#define DUMPS(s)    
#endif

void IRAM_ATTR pulseCounterIntr(){
    if(_pulseCounter < 4000000000){
        _pulseCounter++;
    }
}

void TCS230::initialize(void) {
    _OE  = NO_PIN;
    _S0  = NO_PIN;
    _S1  = NO_PIN;
    _S2  = NO_PIN;
    _S3  = NO_PIN;
    _OUT = NO_PIN;
    _readDiv = 10;
    _pulseCounter = 0;
    _freqSet = TCS230_FREQ_HI;
    _readState = TCS230_READY;

    for (uint8_t i=0; i<RGB_SIZE; i++) {
        _Fd.value[i] = 6000L;  // just typical values
        _Fw.value[i] = 55000L; // just typical values
    }
}

TCS230::TCS230(uint8_t out, uint8_t s2, uint8_t s3) {
    initialise();
    _OUT = out;
    _S2  = s2;
    _S3  = s3;
}

TCS230::TCS230(uint8_t out, uint8_t s2, uint8_t s3, uint8_t oe) {
    initialise();
    _OUT = out;
    _S2  = s2;
    _S3  = s3;
    _OE  = oe;
}

TCS230::TCS230(uint8_t out, uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1) {
    initialise();
    _OUT = out;
    _S0  = s0;
    _S1  = s1;
    _S2  = s2;
    _S3  = s3;
}

TCS230::TCS230(uint8_t out, uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1, uint8_t oe) {
    initialise();
    _OUT = out;
    _S0  = s0;
    _S1  = s1;
    _S2  = s2;
    _S3  = s3;
    _OE  = oe;
}

TCS230::~TCS230(void) {}

void TCS230::begin() {
    if (_OUT != NO_PIN) pinMode(_OUT, OUTPUT);
    if (_S0 != NO_PIN) pinMode(_S0, OUTPUT);
    if (_S1 != NO_PIN) pinMode(_S1, OUTPUT);
    if (_S2 != NO_PIN) pinMode(_S2, OUTPUT);
    if (_S3 != NO_PIN) pinMode(_S3, OUTPUT);
    if (_OE != NO_PIN) pinMode(_OE, OUTPUT);

    setEnable(false);
    setFrequencyInternal(_freqSet);

    DUMPS("\nLibrary begin initialised", );
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

void TCS230::setSampling(uint8_t t) {
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
        setFrequency2((b) ? _freqSet : TCS230_FREQ_OFF);
    }
}

void TCS230::setDarkCal(sensorData *d) {
    if (d == NULL)
        return;

    DUMPS("\nsetDarkCal");
    for (uint8_t i=0; i<RGB_SIZE; i++) {
        DUMP(" ", d->value[i]);
        _Fd.value[i] = d->value[i];
    }
}

void TCS230::setWhiteCal(sensorData *d) {
    if (d == NULL)
        return;

    DUMPS("\nsetWhiteCal");
        for (uint8_t i=0; i<RGB_SIZE; i++) {
        DUMP(" ", d->value[i]);
        _Fw.value[i] = d->value[i];
    }
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
        d->value[i] = _Fo.value[i];
        DUMP(" ", d->value[i]);
    }
}

// blocking read of a single sensor value (ie, not rgb)
uint32_t TCS230::readSingle(void) {
    DUMPS("\nreadSingle");
    _readState = TCS230_READING;
    _pulseCounter = 0;
    setEnable(true);
    attachInterrupt(_OUT, pulseCounterIntr, RISING); // Habilita interrupção por borda de subida no pino de saída do sensor de cor
    vTaskDelay(_readTime / portTICK_PERIOD_MS);
    setEnable(false);
    detachInterrupt(_OUT);
    _readState = TCS230_READY;
    return = 1000 * pulseCounter / _readTime;
}

void TCS230::read(void) {
    _readState = TCS230_READING;
    for(uint8_t i=0; i<RGB_SIZE; i++) {
        _pulseCounter = 0;
        setEnable(true);
        attachInterrupt(_OUT, pulseCounterIntr, RISING); // Habilita interrupção por borda de subida no pino de saída do sensor de cor
        vTaskDelay(_readTime / portTICK_PERIOD_MS);
        setEnable(false);
        detachInterrupt(_OUT);
        _fo.value[i] = 1000 * pulseCounter / timeReadPulses;
    }
    _readState = TCS230_READY;
    RGBTransformation();
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