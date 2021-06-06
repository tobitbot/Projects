

#include "Sensors.h"
#include "Wire.h"


// Sensor libraries
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <BMP180.h>
#include <DHT11.h>


Sensors::Sensors(CayenneLPP* lpp)
{
    this->lpp = lpp;
}


void Sensors::readBMP180()
{
    float temperature, pressure, altitude;
    readBMP180(temperature, pressure, altitude);
}

void Sensors::resetLppChannel()
{
    this->appDataIndex = 0;
    this->lpp->reset();
}

void Sensors::readBMP180(float& temperature, float& pressure, float& altitude)
{
    BMP180 bmp;
    Wire.begin();

    if (bmp.begin(0x76))
    {
        bmp.readTemperature();
        bmp.readPressure();

        delay(50);

        temperature = bmp.readTemperature();
        pressure    = bmp.readPressure() / 100.0F;
        altitude    = bmp.readAltitude(SEALEVELPRESSURE_HPA);

        lpp->addTemperature(appDataIndex++, temperature);
        lpp->addBarometricPressure(appDataIndex++, pressure);
        lpp->addAltitude(appDataIndex++, altitude);

        char buf[256];
        sprintf(buf, "BMP 180 results:\n Temperature: %d °C Pressure: %d mBar Altitude: %d m \n",
            static_cast<int>(temperature), static_cast<int>(pressure), static_cast<int>(altitude));
        Serial.println(buf);
    }
    else
    {
        Serial.println("Error initialising BMP180!");
    }
    Wire.end();
}


void Sensors::readBME280()
{
    float temperature, pressure, altitude, humidity;
    readBME280(temperature, pressure, altitude, humidity);
}

void Sensors::readBME280(float& temperature, float& pressure, float& altitude, float& humidity)
{
    Adafruit_BME280 bme;
    Wire.begin();

    if (bme.begin(0x76))
    {
        bme.readTemperature();
        bme.readPressure();
        bme.readHumidity();

        delay(50);

        temperature = bme.readTemperature();
        pressure    = bme.readPressure() / 100.0F;
        altitude    = bme.readAltitude(SEALEVELPRESSURE_HPA);
        humidity    = bme.readHumidity();

        lpp->addTemperature(appDataIndex++, temperature);
        lpp->addBarometricPressure(appDataIndex++, pressure);
        lpp->addAltitude(appDataIndex++, altitude);
        lpp->addRelativeHumidity(appDataIndex++, humidity);

        char buf[256];
        sprintf(buf, "BME 280 results:\n Temperature: %d °C Pressure: %d mBar Altitude: %d m Humdidity: %d \n",
            static_cast<int>(temperature), static_cast<int>(pressure), static_cast<int>(altitude), static_cast<int>(humidity));
        Serial.println(buf);
    }
    else
    {
        Serial.println("Error initialising BME280!");
    }
    Wire.end();
}


void Sensors::readBH1750()
{
    float luminosity;
    readBH1750(luminosity);
}

void Sensors::readBH1750(float& luminosity)
{
    BH1750 lightMeter(0x23);
    Wire.begin();

    if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE))
    {
        luminosity = lightMeter.readLightLevel();

        lpp->addLuminosity(appDataIndex++, luminosity);

        char buf[100];
        sprintf(buf, "BH1750 results:\n Luminosity: %d LUX \n", static_cast<int>(luminosity));
        Serial.println(buf);
    }
    else
    {
        Serial.println("Error initialising BH1750!");
    }
    Wire.end();
}

/**
 * This function needs to be called in setup();
 */
void Sensors::initCCS811()
{
    Wire.begin();

    ccs.begin();

    uint16_t eco2, etvoc, baseline, baselinetemp;
    uint8_t baselineflash[2];

    //read flash at addr to data2
    FLASH_read_at(CCS811_ADDRESS, baselineflash, sizeof(baselineflash));

    baselinetemp = (baselineflash[0] << 8) | baselineflash[1];

    if (baselinetemp > 0)
    {
        Serial.print("Set CCS811 baseline: ");
        Serial.println(baselinetemp);
        ccs.setBaseline(baselinetemp);
    }
    else
    {
        Serial.println("Error while initilazation of CCS811.");
    }
}

void Sensors::readCCS811()
{
    uint16_t tvoc, eco2;
    readCCS811(tvoc, eco2);
}

void Sensors::readCCS811(uint16_t& tvoc, uint16_t& eco2)
{
    if (true)
    {
        if(ccs.available())
        {
            if(!ccs.readData())
            {
                tvoc = ccs.getTVOC();
                eco2 = ccs.geteCO2();

                lpp->addTVOC(appDataIndex++, tvoc);
                lpp->addEco2(appDataIndex++, eco2);

                char buf[100];
                sprintf(buf, "CCS811 results:\n TVOC: %d ECO02: %d \n", static_cast<int>(tvoc), static_cast<int>(eco2));
                Serial.println(buf);
            }
            else
            {
                Serial.println("Could not read CCS811 data.");
            }
        }
        else
        {
            Serial.println("Error initialising CCS811!");
        }
    }
    else
    {
        Serial.println("CCS811 not initialized. Call initCCS811() in setup().");
    }
}


void Sensors::readDHT11(int gpio)
{
    int temperature, humidity;
    readDHT11(gpio, temperature, humidity);
}

void Sensors::readDHT11(int gpio, int& temperature, int& humidity)
{
    DHT11 dht;

    dht.read(gpio);

    humidity    = dht.readHumidity();
    temperature = dht.readTemperature();

    lpp->addTemperature(appDataIndex++, temperature);
    lpp->addRelativeHumidity(appDataIndex++, humidity);

    char buf[100];
    sprintf(buf, "DHT11 results:\n Temperature: %d Humdidity: %d \n", temperature, humidity);
    Serial.println(buf);
}

