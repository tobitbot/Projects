/**
 * This sketch is designed for Adafruit Feather M0 LoRa with external BME280 or BMP280.
 * Sending data of the sensor via LoRa in LPP format.
 *
 * The sensor is powerd by GPIO 12  (3.3V) and is talked to via I2C on SCL and SCA.
 * It is only powered, when we need data.
 *
 * We provide deep sleep mode between lora messages for low power cosumption.
 * Should be down to 180uA while sleeping.
 *
 * On Feather M0 LoRa you need to connect:
 * GPIO 11  |   IO2  of RFM95
 * GPI0 6   |   IO1  of RFM95
 *
 * GPIO 12  |   VIN  of BME280
 * GND      |   GND  of BME280
 * SCL      |   SCL  of BME280
 * SDA      |   SDA  of BME280
 */

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include "Arduino.h"

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <string.h>
#include <CayenneLPP.h>

#include "config.h"


#if defined(FEATHERWING)
    //#include <Adafruit_SleepyDog.h>
#elif defined (TTGOLORA)
    #include "WiFi.h"
    RTC_DATA_ATTR int bootCount = 0;
#endif


#define SEALEVELPRESSURE_HPA (1026.25) // this should be set according to the weather forecast
#define SENSOR_ADDRESS 0x76            // was originally 76

Adafruit_BME280 bme;
Adafruit_BMP280 bmp;

enum sensorType
{
    NONE    = 0,
    BME280  = 1,
    BMP280  = 2,
    GY521   = 3
};

sensorType sensor = NONE;

// BME280 / BMP280 data are saved here: Temperature, Humidity, Pressure, Altitude calculated from atmospheric pressure
float tmp, hum, pressure, alt_barometric;

// Generate LPP Object with size
CayenneLPP lpp(40);

// Get lora keys from config.h
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;

// Deep sleep time
const uint64_t SLEEP_TIME = 16; //for now, this must be a scaler of 16
#define uS_TO_S_FACTOR 1000000



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function declaration starts here.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Detect sensor type BME or BMP 280
 */
sensorType getSensorType()
{

    bool isBME = bme.begin(SENSOR_ADDRESS);
    bool isBMP = bmp.begin(SENSOR_ADDRESS);

    if (isBME || isBMP)
    {
        if (isBME)
        {
            Serial.println("BME280 found!");
            return BME280;
        }
        else if (isBMP)
        {
            Serial.println("BMP280 found!");
            return BMP280;
        }
        else
        {
            Serial.println("Error: 1, getSensorType()");
            return NONE;
        }
    }
    else if (!isBME && !isBMP)
    {
        Serial.println("Could not find sensor");
        return NONE;
    }
    else
    {
        Serial.println("Error: 2, getSensorType()");
        return NONE;
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
    delay(1000);
}

/**
 * Powers the bme280 for the time it is needed.
 */
void getSensorValues()
{
    // Power on BME
    digitalWrite(12, HIGH);
    delay(100);

    bool deviceFound;
    scanI2C(deviceFound);
    if (deviceFound){

        sensor = getSensorType();
        if (sensor != NONE) {

            if (sensor == BME280) {
                // Read once for initialization
                bme.readTemperature();
                bme.readPressure();
                bme.readAltitude(SEALEVELPRESSURE_HPA);
                bme.readHumidity();
                delay(50);

                tmp = bme.readTemperature();
                pressure = bme.readPressure() / 100.0F;
                alt_barometric = bme.readAltitude(SEALEVELPRESSURE_HPA);
                hum = bme.readHumidity();
            }
            else if (sensor == BMP280 ) {
                bmp.begin();
                bmp.readTemperature();
                bmp.readPressure();
                bmp.readAltitude(SEALEVELPRESSURE_HPA);
                delay(50);

                tmp = bmp.readTemperature();
                pressure = bme.readPressure() / 100.0F;
                alt_barometric = bme.readAltitude(SEALEVELPRESSURE_HPA);
            }
            else
            {
                Serial.println("Invalid sensor type!");
            }
        }
    }
    else {
        Serial.println("No sensor found");
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

    delay(100);

    // Power of BME280
    Serial.println("Power down Sensor");
    digitalWrite(12, LOW);
}

/**
 * Activate deepsleep mode
 */
void goToSleep()
{
    Serial.println("Going to sleep now...");
    delay(100);
    digitalWrite(LED_BUILTIN, LOW); // Show we're asleep

    for (uint i = 0; i < SLEEP_TIME / 16; i++)
    {
        #if defined(FEATHERWING)
            //Watchdog.sleep(16000);
        #elif defined(TTGOLORA)
            esp_light_sleep_start();
        #endif
        delay(5000); //change to debug
    }

    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Woke up!");
}

/**
 *
 */
void do_send(osjob_t *j)
{
    getSensorValues();

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        lpp.reset();

        // Add BME280 values to payload
        lpp.addTemperature(2, tmp);
        lpp.addRelativeHumidity(3, hum);
        lpp.addBarometricPressure(4, pressure);

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.print(lpp.getSize());
        Serial.println(F(" bytes long LPP packet queued."));
        digitalWrite(LED_BUILTIN, HIGH);
    }
    Serial.println("do_send");
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");

    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.dataLen)
        {
            Serial.print("Yeey! Received ");
            Serial.print(LMIC.dataLen);
            Serial.println(" bytes of payload!");
            Serial.println("RSSI \t SNR \t");
            Serial.println(LMIC.rssi, LMIC.snr);
            Serial.println("--------------------");
        }
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.println(F("Received "));
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Go to deep sleep mode
        goToSleep();

        // Workaround to skip waiting time of underlaying lmic time management.
        // LMIC time is not running while deep sleep mode is active.
        // So we would wait a long time to respect the duty cycle.
        LMIC.bands[BAND_MILLI].avail =
            LMIC.bands[BAND_CENTI].avail =
                LMIC.bands[BAND_DECI].avail = os_getTime();
        os_setCallback(&sendjob, do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
}

/**
 * //////////////////////////////////////////////////////////////////
 * Setup
 * //////////////////////////////////////////////////////////////////
 */
void setup()
{
    // Set led to output
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    #if defined(FEATHERWING)
        Serial.begin(9600);
    #elif defined(TTGOLORA)
        Serial.begin(115200);

        // Turn off all wifis
        btStop();
        WiFi.mode(WIFI_OFF);

        esp_sleep_enable_timer_wakeup(SLEEP_TIME * uS_TO_S_FACTOR);
    #endif

    // Set GPIO high to power the sensor
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);

    bool deviceFound;
    scanI2C(deviceFound);
    if (deviceFound)
    {
        sensor = getSensorType();
    }
    else {
        Serial.println("No Sensor found!");
        while(!deviceFound)
        {
            scanI2C(deviceFound);
            delay(1000);
        };
        Serial.println("Sensor found! Going on...");
    }

    //increase boot counter after successfull boot
    bootCount++;
    Serial.println("Boot number: " + String(bootCount));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

/**
 * //////////////////////////////////////////////////////////////////
 * Main process
 * //////////////////////////////////////////////////////////////////
 */
void loop()
{
    // Process lora
    os_runloop_once();
}