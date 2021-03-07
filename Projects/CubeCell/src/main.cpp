#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "config.h"
#include "cayenne_lpp.h"
#include "Wire.h"

#include <Adafruit_BME280.h>
#include <BH1750.h>

#define SEALEVELPRESSURE_HPA (1026.25) // this should be set according to the weather forecast

#define timetosleep 20000
#define timetowake 20000
static TimerEvent_t sleep;
static TimerEvent_t wakeup;
uint8_t lowpower = 1;

// Generate LPP Object with size
cayenne_lpp_t lpp = { 0 };
uint8_t appDataIndex = 0;

enum SensorType
{
    _NONE    = 0,
    _BME280  = 1,
    _BMP280  = 2,
    _GY521   = 3,
    _BH1750  = 4
};

void OnSleep()
{
    Serial.printf("into lowpower mode, %d ms later wake up.\r\n", timetowake);
    lowpower = 1;
    //timetosleep ms later wake up;
    TimerSetValue(&wakeup, timetowake);
    TimerStart(&wakeup);
}

void OnWakeup()
{
    Serial.printf("wake up, %d ms later into lowpower mode.\r\n", timetosleep);
    lowpower = 0;
    //timetosleep ms later into lowpower mode;
    TimerSetValue(&sleep, timetosleep);
    TimerStart(&sleep);
}

void setDemoValues()
{
    Serial.println("Set demo values");
    cayenne_lpp_add_digital_input(&lpp, appDataIndex++, 1);
    cayenne_lpp_add_digital_input(&lpp, appDataIndex++, 0);
    cayenne_lpp_add_digital_input(&lpp, appDataIndex++, 1);
}

/**
 * Detect sensor type and set the given place in the array of
 * sensors.
 */
int getSensorType(uint8_t address, SensorType* sensors, int index)
{
    Adafruit_BME280 bme;
    BH1750 lightMeter;

    Serial.println("");

    if (bme.begin(address))
    {
        Serial.print(" BME280 found on address 0x");
        Serial.println(address, HEX);
        return sensors[index] = SensorType::_BME280;
    }
    else if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE, address, &Wire))
    {
        Serial.print(" BH1750 found on address 0x");
        Serial.println(address, HEX);
        return sensors[index] = SensorType::_BH1750;
    }
    else
    {
        Serial.print(" No valid sensor type found on address 0x ");
        Serial.println(address, HEX);
        return sensors[index] = SensorType::_NONE;
    }

    return sensors[index] = SensorType::_NONE;
}

/**
 * Read BME 280 values
 */
void readBME280()
{
    Adafruit_BME280 bme;
    Wire.begin();

    if (bme.begin(0x76))
    {
        bme.readTemperature();
        bme.readPressure();
        bme.readHumidity();

        delay(50);

        float temperature = bme.readTemperature();
        float pressure    = bme.readPressure() / 100.0F;
        float altitude    = bme.readAltitude(SEALEVELPRESSURE_HPA);
        float humidity    = bme.readHumidity();

        cayenne_lpp_add_temperature(&lpp, appDataIndex++, temperature);
        cayenne_lpp_add_barometric_pressure(&lpp, appDataIndex++, pressure);
        cayenne_lpp_add_relative_humidity(&lpp, appDataIndex++, humidity);

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

void readBH1750()
{
    BH1750 lightMeter(0x23);
    Wire.begin();

    if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
        float luminosity = lightMeter.readLightLevel();

        cayenne_lpp_add_luminosity(&lpp, appDataIndex++, luminosity);

        char buf[100];
        sprintf(buf, "BH1750 results:\n Luminosity: %d LUX \n", static_cast<int>(luminosity));
        Serial.println(buf);
    }
    else {
        Serial.println("Error initialising BH1750!");
    }
    Wire.end();
}

/**
 * Scan I2C devices
 *
 * @return the number of sensors found
 */
int scanI2C(SensorType* sensors)
{
    Wire.begin(); // Wire communication begin

    byte error, address; //variable for error and I2C address
    int nDevices;

    Serial.println("Scanning I2C bus...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            getSensorType(address, sensors, nDevices);
            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices <= 0)
    {
        Serial.println("No I2C devices found.");
    }
    else
    {
        Serial.print("Found ");
        Serial.print(nDevices);
        Serial.println(" devices on I2C bus.\n");
    }
    Wire.end();
    return nDevices;
}

/**
 * Print payload buffer
 */
static void _print_buffer(cayenne_lpp_t *lpp)
{
    Serial.print("appData: ");
    for (uint8_t i = 0; i < lpp->cursor; ++i)
    {
        appData[i] = lpp->buffer[i];

        Serial.print(appData[i], HEX);
        Serial.print(" ");
    }

    appDataSize = lpp->cursor;
    Serial.println(appDataSize);
}

void getSensorValues()
{
    // switch on external voltage, inverse logic
    digitalWrite(Vext, LOW);
    delay(20);

    SensorType sensors[2] = {SensorType::_NONE, SensorType::_NONE};

    int nDevices = scanI2C(sensors);

    cayenne_lpp_reset(&lpp);

    for (int i = 0; i < sizeof(sensors); i++)
    {
        switch (sensors[i])
        {
            case SensorType::_NONE:
                //Serial.print("Empty sensor slot: ");
                //Serial.println(i);
                break;
            case SensorType::_BME280:
                readBME280();
                break;
            case SensorType::_BH1750:
                readBH1750();
                break;
            default:
                setDemoValues();
                break;
        }
    }
    // switch off external voltage
    digitalWrite(Vext, HIGH);
    _print_buffer(&lpp);
}

/* Prepares the payload of the frame */
static void PrepareTxFrame(uint8_t port)
{
    cayenne_lpp_reset(&lpp);
    appDataIndex = 0;
    getSensorValues();
}

void setup()
{
    boardInitMcu();
    Serial.begin(115200);

    pinMode(Vext, OUTPUT);
    // switch vext off, inverse logic
    digitalWrite(Vext, HIGH);

    Radio.Sleep();
    TimerInit(&sleep, OnSleep);
    TimerInit(&wakeup, OnWakeup);
    OnSleep();

#if (AT_SUPPORT)
    enableAt();
#endif

    deviceState = DEVICE_STATE_INIT;
    LoRaWAN.ifskipjoin();
}

void loop()
{
    if (lowpower)
    {
        //note that LowPower_Handler() run six times the mcu into lowpower mode;
        lowPowerHandler();
    }

    switch (deviceState)
    {
    case DEVICE_STATE_INIT:
    {
#if (AT_SUPPORT)
        getDevParam();
#endif
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
    }
    case DEVICE_STATE_JOIN:
    {
        LoRaWAN.join();
        break;
    }
    case DEVICE_STATE_SEND:
    {
        PrepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
    }
    case DEVICE_STATE_CYCLE:
    {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
    }
    case DEVICE_STATE_SLEEP:
    {
        LoRaWAN.sleep();
        break;
    }
    default:
    {
        deviceState = DEVICE_STATE_INIT;
        break;
    }
    }
}