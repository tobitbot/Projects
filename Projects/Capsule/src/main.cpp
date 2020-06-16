#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>

#include "config.h"

#include "HDC1080.h"
#include <BMP180.h>

#include "cayenne_lpp.h"


enum sensorType
{
    _NONE    = 0x00,
    _BME180  = 0x01,
    _BME280  = 0x02,
    _BMP280  = 0x03,
    _GY521   = 0x04,
    _HDC1080 = 0x05
};

// Define your sensor here
sensorType sensor = _HDC1080;


/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 600 * 1000;
uint8_t appDataIndex = 0;

cayenne_lpp_t lpp = { 0 };

HDC1080 hdc1080;
BMP085 bmp;

/**
 *  Prepares the payload of the frame
 *
 *  size of double is 8 bytes on this platform
 */
void getBMP180Data()
{
    Serial.println("BMP Data ###############");
    bmp.begin();

    Serial.print("Temperature: ");
    Serial.println(bmp.readTemperature());
    cayenne_lpp_add_temperature(&lpp, appDataIndex++, bmp.readTemperature());

    Serial.print("Pressure: ");
    Serial.println(bmp.readPressure()/100.0);
    float pressure = bmp.readPressure()/100.0;
    cayenne_lpp_add_barometric_pressure(&lpp, appDataIndex++, pressure);
}

void getHDC1080Data()
{
    Serial.println("HDC Data ###############");
    hdc1080.begin(0x40);

    Serial.print("Humidity: ");
    Serial.println(hdc1080.readHumidity());
    float humidity = hdc1080.readHumidity()/1.0;
    cayenne_lpp_add_relative_humidity(&lpp, ++appDataIndex, humidity);

    Serial.print("Temperature: ");
    Serial.println(hdc1080.readTemperature());
    float temperature = hdc1080.readTemperature()/1.0;
    cayenne_lpp_add_temperature(&lpp, ++appDataIndex, temperature);
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

/**
 *
 */
void prepareTxFrame(uint8_t appPort)
{
    Serial.println("");
    Serial.println("Prepare tx frame");

    cayenne_lpp_reset(&lpp);
    appDataIndex = 0;

    switch (sensor) {
        case _BME180:
            getBMP180Data();
            break;
        case _HDC1080:
            getHDC1080Data();
            break;
        default:
            Serial.println("No sensor data available");
            break;
    }

    _print_buffer(&lpp);
}

/**
 * Scan I2C devices
 */
void scanI2C(bool &deviceFound)
{
    /*
    Wire.begin(); // Wire communication begin
    Serial.println("\nI2C Scanner");

    byte error, address; //variable for error and I2C address
    int nDevices;

    Serial.println("Scanning...");
    deviceFound = false;

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
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
    */
}





void setup()
{
    boardInitMcu();
    Serial.begin(115200);

    // Power to Sensor
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);

    //bool deviceFound;
    //scanI2C(deviceFound);

    deviceState = DEVICE_STATE_INIT;
    LoRaWAN.ifskipjoin();

}

void loop()
{
    switch (deviceState)
    {
        case DEVICE_STATE_INIT:
        {
            printDevParam();
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
            prepareTxFrame(appPort);
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
