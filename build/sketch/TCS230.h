#line 1 "C:\\Users\\theo-\\Área de Trabalho\\Arquivos Theo\\Projeto Integrador II\\Firmware\\MYT_600\\TCS230.h"
/**************************************************************************/
/**
 * @file    TCS230.h
 * @author  Theo Pires
 * @date    08/10/2023
 * @see     www.linkedin.com/in/theo-pires-a34b33183/
 * @brief   Arquivo de cabeçalho da biblioteca TCS230
*/
/**************************************************************************/
#ifndef _TCS230_H
#define _TCS230_H

#include <Arduino.h>

/********************* TCS230 Configurações **********************/    
// S0  |S1 |Output Freq Scaling | |S2  |S3 |Photodiode Selection  |
// :--:|--:|:-------------------| |:--:|--:|:---------------------|
// L   |L  |Power Down          | |L   |L  |Red                   |
// L   |H  |2%                  | |L   |H  |Blue                  |
// H   |L  |20%                 | |H   |L  |Clear (no filter)     |
// H   |H  |100%                | |H   |H  |Green                 |
/*****************************************************************/

// Frequency setting defines
enum id_freq {TCS230_FREQ_HI, TCS230_FREQ_MID, TCS230_FREQ_LO, TCS230_FREQ_OFF};
// Indices for RGB data and filter selection
enum id_rgb_colors {TCS230_RGB_R, TCS230_RGB_G, TCS230_RGB_B, TCS230_RGB_X};
// Indices for reading colors response
enum id_colors {RED, GREEN, BLUE, BLACK, WHITE, GRAY};
// States of class
enum id_states {TCS230_READY, TCS230_READING};

const uint8_t RGB_SIZE = 3;  // Limit index of RGB components
const uint8_t NO_PIN = 0xff; // Default value for pins that not defined
const uint8_t DARK_SENSETIVE = 120; // Limite para entender como preto em comparação ao total entre os valores RGB
const uint8_t WHITE_SENSETIVE = 650; // Limite para entender como branco em comparação ao total entre os valores RGB

// Sensor data structure type. Contains the RGB raw data from sensor.
typedef struct {
    int32_t value[RGB_SIZE];  
} sensorData;

// RGB data structure type. Contains de RGB data from the sensor
typedef struct {
    uint8_t value[RGB_SIZE];
} colorData;

class TCS230 {
    private:
    uint8_t _S0;            // frequency scaler S0 pin
    uint8_t _S1;            // frequency scaler S1 pin
    uint8_t _S2;            // photodiode filter selection S2 pin
    uint8_t _S3;            // photodiode filter selection S3 pin
    uint8_t _OE;            // output enable pin
    uint8_t _OUT;           // output frequency pin
    uint8_t _readTime;      // time of sampling in ms
    uint8_t _freqSet;       // current frequency setting
    uint8_t _readState;     // state of class reader
    static volatile uint32_t _pulseCounter; // pulse counter of frequency output from sensor

    sensorData _fd;         // dark calibration parameters raw data
    sensorData _fw;         // white calibration parameters raw data

    sensorData _fo;         // current raw data from sensor reading
    colorData _rgb;         // current rgb data from sensor reading

    void initialize(void);                  // initialize variables 
    void RGBTransformation(void);           // convert the raw data structure to rgb data structure
    void setFrequencyInternal(uint8_t f);   // internal function for frequency prescaler
    static void pulseCounterIntr(void);

    public:

    TCS230(uint8_t out, uint8_t s2, uint8_t s3);
    TCS230(uint8_t out, uint8_t s2, uint8_t s3, uint8_t oe);
    TCS230(uint8_t out, uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1);
    TCS230(uint8_t out, uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1, uint8_t oe);

    void begin(void);
    ~TCS230(void);

    // Methods for hardware and object control

    void setFilter(uint8_t f);

    void setFrequency(uint8_t f);

    void setEnable(bool b);

    void setSampling(uint8_t t);

    void setDarkCal(sensorData *d);
    
    void setWhiteCal(sensorData *d);

    // Methods for reading sensor data

    void getRGB(colorData *rgb);

    void getRaw(sensorData *d);

    uint32_t readSingle(void);

    void read(void);

    bool available(void);

    char * getColorToString(void);
};

#endif