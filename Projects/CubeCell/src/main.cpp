#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "config.h"

#include <Wire.h>
#include <string.h>
#include "cayenne_lpp.h"


// Supported sensors libraries
#include <BMP280.h>


// Generate LPP Object with size
cayenne_lpp_t lpp = { 0 };
uint8_t appDataIndex = 0;

enum sensorType
{
    _NONE    = 0,
    _BME280  = 1,
    _BMP280  = 2,
    _GY521   = 3
};

sensorType sensor = sensorType::_NONE;

#define SEALEVELPRESSURE_HPA (1026.25)

// BME280 / BMP280 data are saved here: Temperature, Humidity, Pressure, Altitude calculated from atmospheric pressure
float tmp, hum, pressure, alt_barometric;

BMP280 bmp;

/**
 * Detect sensor type BME or BMP 280
 */
sensorType getSensorType()
{

    //bool isBME = bme.begin(BMP280_ADDRESS);
    bool isBMP = bmp.begin(BMP280_ADDRESS);

    if (isBMP)
    {
        if (isBMP)
        {
            Serial.println("BMP280 found!");
            return sensorType::_BMP280;
        }
        else
        {
            Serial.println("Error: 1, getSensorType()");
            return sensorType::_NONE;
        }
    }
    else if (!isBMP)
    {
        Serial.println("Could not find sensor");
        return sensorType::_NONE;
    }
    else
    {
        Serial.println("Error: 2, getSensorType()");
        return sensorType::_NONE;
    }
}

/**
 * Scan I2C devices
 */
void scanI2C(bool &deviceFound)
{

    Wire.begin(); // Wire communication begin
    Serial.println("\nI2C Scanner");

    byte error, address; //variable for error and I2C address
    int nDevices;

    Serial.println("Scanning...");
    deviceFound = false;

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
            Serial.println("  !");
            nDevices++;
            deviceFound = true;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
            deviceFound = false;
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }
}

void readBMP280()
{
    bmp.readTemperature();
    bmp.readPressure();
    bmp.readAltitude(SEALEVELPRESSURE_HPA);
    delay(50);

    tmp = bmp.readTemperature();
    pressure = bmp.readPressure() / 100.0F;
    alt_barometric = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    cayenne_lpp_add_temperature(&lpp, appDataIndex++, tmp);
    cayenne_lpp_add_barometric_pressure(&lpp, appDataIndex++ ,pressure);
}

void getSensorValues()
{
    bool devFound;
    scanI2C(devFound);
    sensor = getSensorType();

    cayenne_lpp_reset(&lpp);

    switch (sensor)
    {
        case sensorType::_BMP280 :
            readBMP280();
            break;
        case sensorType::_BME280 :
            //readBME280();
            break;
        default:
            break;
    }

    Serial.print("Temperature = ");
    Serial.print(tmp);
    Serial.print("C, ");
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.print("hPa, ");
    Serial.print("Approx. Altitude = ");
    Serial.print(alt_barometric);
    Serial.print("m, ");
    Serial.print("Humidity = ");
    Serial.print(hum);
    Serial.println("%");

}

static void _print_buffer(cayenne_lpp_t *lpp)
{
    Serial.println("Print buffer: ");
    Serial.println(sizeof(lpp->buffer));
    Serial.println("Print cursor: ");
    Serial.println(lpp->cursor);

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

/* Prepares the payload of the frame */
static bool prepareTxFrame(uint8_t port)
{
    cayenne_lpp_reset(&lpp);
    appDataIndex = 0;

    getSensorValues();

    switch (port)
    {
        case APPPORT: // woke up from interrupt
            Serial.println("Sending data packet");
            appDataSize = 1;   //AppDataSize max value is 64
            //appData[0] = 0xFF; // set to something useful
            break;
        case DEVPORT: // daily wake up
            Serial.println("Sending dev status packet");
            appDataSize = 1;   //AppDataSize max value is 64
            appData[0] = 0xA0; // set to something else useful
            break;
    }

    _print_buffer(&lpp);

    return true;
}

void accelWakeup()
{
    delay(10);
    if (digitalRead(INT_PIN) == HIGH)
    {
        accelWoke = true;
    }
}

void setup()
{
    boardInitMcu();
    Serial.begin(115200);
#if (AT_SUPPORT)
    enableAt();
#endif

    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
    delay(500);

    Serial.begin(115200);

    deviceState = DEVICE_STATE_INIT;
    LoRaWAN.ifskipjoin();

    accelWoke = false;
    pinMode(INT_PIN, INPUT);
    attachInterrupt(INT_PIN, accelWakeup, RISING);
    Serial.println("Interrupts attached");
}

void loop()
{
    if (accelWoke)
    {
        uint32_t now = TimerGetCurrentTime();
        Serial.print(now);
        Serial.println("accel woke");
    }

    switch (deviceState)
    {
    case DEVICE_STATE_INIT:
    {
#if (AT_SUPPORT)
        getDevParam();
#endif
        //printDevParam();
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
        prepareTxFrame(DEVPORT);
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
        if (accelWoke)
        {
            if (IsLoRaMacNetworkJoined)
            {
                if (prepareTxFrame(APPPORT))
                {
                    LoRaWAN.send();
                }
            }
            accelWoke = false;
        }
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