#include <Arduino.h>

#include "LoRaWan_APP.h"
#include "Wire.h"

#include "config.h"
#include "cayenne_lpp.h"

#include <Adafruit_BME280.h>
#include <GPS_Air530.h>
#include "cubecell_SSD1306Wire.h"


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
    _GY521   = 3
};

SensorType sensor = SensorType::_NONE;
float temperature, humidity, pressure, altitude;

Adafruit_BME280 bme;

SSD1306Wire oled(0x3c, 500000, I2C_NUM_0, GEOMETRY_128_64, GPIO10); // addr , freq , i2c group , ratio , rst


void printGpsInfoSerial()
{
    Serial.print("Date/Time: ");
    if (Air530.date.isValid())
    {
        Serial.printf("%d/%02d/%02d", Air530.date.year(), Air530.date.day(), Air530.date.month());
    }
    else
    {
        Serial.print("INVALID");
    }

    if (Air530.time.isValid())
    {
        Serial.printf(" %02d:%02d:%02d.%02d", Air530.time.hour(), Air530.time.minute(), Air530.time.second(), Air530.time.centisecond());
    }
    else
    {
        Serial.print(" INVALID");
    }
    Serial.println();

    Serial.print("LAT: ");
    Serial.print(Air530.location.lat(), 6);
    Serial.print(", LON: ");
    Serial.print(Air530.location.lng(), 6);
    Serial.print(", ALT: ");
    Serial.print(Air530.altitude.meters());

    Serial.println();

    Serial.print("SATS: ");
    Serial.print(Air530.satellites.value());
    Serial.print(", HDOP: ");
    Serial.print(Air530.hdop.hdop());
    Serial.print(", AGE: ");
    Serial.print(Air530.location.age());
    Serial.print(", COURSE: ");
    Serial.print(Air530.course.deg());
    Serial.print(", SPEED: ");
    Serial.println(Air530.speed.kmph());
    Serial.println();
}

int32_t fracPart(double val, int n)
{
    return (int32_t)((val - (int32_t)(val)) * pow(10, n));
}

void printGpsInfoOled()
{
    Serial.println("print Oled Info");

    char str[30];
    oled.clear();
    oled.setFont(ArialMT_Plain_10);
    int index = sprintf(str, "%02d-%02d-%02d", Air530.date.year(), Air530.date.day(), Air530.date.month());
    str[index] = 0;
    oled.setTextAlignment(TEXT_ALIGN_LEFT);
    oled.drawString(0, 0, str);

    index = sprintf(str, "%02d:%02d:%02d", Air530.time.hour(), Air530.time.minute(), Air530.time.second(), Air530.time.centisecond());
    str[index] = 0;
    oled.drawString(60, 0, str);

    if (Air530.location.age() < 1000)
    {
        oled.drawString(120, 0, "A");
    }
    else
    {
        oled.drawString(120, 0, "V");
    }

    index = sprintf(str, "alt: %d.%d", (int)Air530.altitude.meters(), fracPart(Air530.altitude.meters(), 2));
    str[index] = 0;
    oled.drawString(0, 16, str);

    index = sprintf(str, "hdop: %d.%d", (int)Air530.hdop.hdop(), fracPart(Air530.hdop.hdop(), 2));
    str[index] = 0;
    oled.drawString(0, 32, str);

    index = sprintf(str, "lat :  %d.%d", (int)Air530.location.lat(), fracPart(Air530.location.lat(), 4));
    str[index] = 0;
    oled.drawString(60, 16, str);

    index = sprintf(str, "lon:%d.%d", (int)Air530.location.lng(), fracPart(Air530.location.lng(), 4));
    str[index] = 0;
    oled.drawString(60, 32, str);

    index = sprintf(str, "speed: %d.%d km/h", (int)Air530.speed.kmph(), fracPart(Air530.speed.kmph(), 3));
    str[index] = 0;
    oled.drawString(0, 48, str);
    oled.display();
}

/**
 * Read GPS
 */
void readGPS()
{
    uint32_t starttime = millis();
    while ((millis() - starttime) < 1000)
    {
        while (Air530.available() > 0)
        {
            Air530.encode(Air530.read());
        }
    }
    printGpsInfoSerial();
    //printGpsInfoOled();

    if (millis() > 5000 && Air530.charsProcessed() < 10)
    {
        Serial.println("No GPS detected: check wiring.");
        while (true)
            ;
    }
    cayenne_lpp_add_gps(&lpp, appDataIndex++, Air530.location.lat(), Air530.location.lng(), Air530.altitude.meters());
}

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

/**
 * Read BME 280 values
 */
void readBME280()
{
    bme.readTemperature();
    bme.readPressure();
    bme.readHumidity();

    delay(50);

    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();

    cayenne_lpp_add_temperature(&lpp, appDataIndex++, temperature);
    cayenne_lpp_add_barometric_pressure(&lpp, appDataIndex++, pressure);
    cayenne_lpp_add_relative_humidity(&lpp, appDataIndex++, humidity);
    Wire.end();
}

void setDemoValues()
{
    Serial.println("Set demo values");
    cayenne_lpp_add_digital_input(&lpp, appDataIndex++, 1);
    cayenne_lpp_add_luminosity(&lpp, appDataIndex++, 250);
}

/**
 * Detect sensor type BME or BMP 280
 */
SensorType getSensorType()
{
    bool isBME = bme.begin(0x76);

    if (isBME)
    {
        if (isBME)
        {
            Serial.println("BME280 found!");
            return SensorType::_BME280;
        }
        else
        {
            Serial.println("Error: 2, getSensorType()");
            return SensorType::_NONE;
        }
    }
    else if ((!isBME))
    {
        Serial.println("Could not find sensor");
        return SensorType::_NONE;
    }
    else
    {
        Serial.println("Error: 2, getSensorType()");
        return SensorType::_NONE;
    }

    return SensorType::_NONE;
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

    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.print("C, ");
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.print("hPa, ");
    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println("%");
}

void getSensorValues()
{
    // switch on external voltage
    digitalWrite(Vext, LOW);
    delay(20);

    cayenne_lpp_reset(&lpp);

    sensor = getSensorType();
    Serial.println(sensor);

    switch (sensor)
    {
        case SensorType::_BME280 :
            readBME280();
            break;
        case SensorType::_NONE:
            setDemoValues();
            break;
        default:
            setDemoValues();
            break;
    }
    // switch off external voltage
    digitalWrite(Vext, HIGH);
}

/* Prepares the payload of the frame */
static void PrepareTxFrame(uint8_t port)
{
    cayenne_lpp_reset(&lpp);
    appDataIndex = 0;
    //getSensorValues();
    readGPS();

    _print_buffer(&lpp);
}

void setup()
{
    boardInitMcu();
    Serial.begin(115200);
    Air530.begin();

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

    printGpsInfoSerial();
    printGpsInfoOled();

    oled.init();
    oled.clear();

    char str[30];

    oled.setFont(ArialMT_Plain_10);
    int index = sprintf(str, "Starting");
    str[index] = 0;
    oled.setTextAlignment(TEXT_ALIGN_LEFT);
    oled.drawString(0, 0, str);
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
