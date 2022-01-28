#include "Arduino.h"

#include "CayenneLpp.h"

#include "CCS811.h"

/* Redefine GPIOs */
#define GPIO_1 49
#define GPIO_2 50
#define GPIO_3 52
#define GPIO_4 7
#define GPIO_5 8

enum SensorType
{
    _NONE,
    _BMP180,
    _BMP280,
    _CCS811,
    _DHT11,
    _230V_INPUT
};

class Sensors
{
    public:
        Sensors(CayenneLPP* lpp);

        void resetLppChannel();

        void readBMP180();
        void readBMP180(float& temperature, float& pressure, float& altitude);

        void readBME280();
        void readBME280(float& temperature, float& pressure, float& altitude, float& humidity);

        void readBH1750();
        void readBH1750(float& luminosity);

        void initCCS811();
        void readCCS811();
        void readCCS811(uint16_t& tvoc, uint16_t& eco2);

        void readDHT11(int gpio);
        void readDHT11(int gpio, int& temperature, int& humidity);

        void init230VInputs(uint8_t inputs[]);
        void read230VInputs(uint8_t inputs[]);

    private:

        uint8_t appDataIndex;
        CayenneLPP* lpp;

        CCS811 ccs;

        const float SEALEVELPRESSURE_HPA  = 1026.25;

};