#include <Arduino.h>
#include "LoRaWan_APP.h"
#include "Wire.h"
#include "config.h"
#include "cayenne_lpp.h"
#include <GPS_Air530.h>
#include "cubecell_SSD1306Wire.h"

#define SEALEVELPRESSURE_HPA (1026.25) // this should be set according to the weather forecast

#define timetosleep 20000
#define timetowake 20000
static TimerEvent_t sleep;
static TimerEvent_t wakeup;
uint8_t lowpower = 1;

// Generate LPP Object with size
cayenne_lpp_t lpp = {0};
uint8_t appDataIndex = 0;

uint8_t dataRate = DR_5;



void printGpsInfo()
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

    LoRaWAN.displayGPSInfo(String(Air530.location.lat()), String(Air530.location.lng()), String(Air530.altitude.meters()));

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
    printGpsInfo();

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

/* Prepares the payload of the frame */
static bool PrepareTxFrame(uint8_t port)
{
    cayenne_lpp_reset(&lpp);
    appDataIndex = 0;

    switch (port)
    {
        case APPPORT:
            readGPS();
            break;
        case DEVPORT:
            readGPS();
            break;
        default:
            readGPS();
            break;
    }

    _print_buffer(&lpp);
    return true;
}

/**
 * Callback function for interrupt pin
 */
void accelWakeup()
{
    delay(10);
    Serial.println("accelWakeup");

    LoRaWAN.setDataRateForNoADR(dataRate);
    if (digitalRead(INT_PIN) == HIGH)
    {
        accelWoke = true;
    }
}

void switchDataRate()
{
    Serial.println("Change datarate");
    Serial.print("DR_PIN: "); Serial.println(digitalRead(DR_PIN));
    dataRate++;

    if (dataRate > DR_5)
    {
        dataRate = DR_0;
    }
    String newDr("Using DR_");
    newDr += dataRate;
    LoRaWAN.displayText("", newDr);

    Serial.print("Using DR_");
    Serial.print(dataRate);
    Serial.println(" now");
    delay(1000);
}



void setup()
{
    boardInitMcu();
    Serial.begin(115200);
    Air530.begin();

    pinMode(Vext, OUTPUT);
    // switch vext off, inverse logic
    digitalWrite(Vext, HIGH);

    /**
     * Attach DR_PIN and INT_PIN to interrupts.
     *
     * Somehow this was the only way to use interrupts with
     * extern buttons to work reliable.
     */
    accelWoke = false;
    pinMode(INT_PIN, OUTPUT);
    digitalWrite(INT_PIN, LOW);
    attachInterrupt(INT_PIN, accelWakeup, RISING);

    pinMode(DR_PIN, OUTPUT);
    digitalWrite(DR_PIN, LOW);
    attachInterrupt(DR_PIN, switchDataRate, RISING);

    LoRaWAN.displayMcuInit();

    Radio.Sleep();
    TimerInit(&sleep, OnSleep);
    TimerInit(&wakeup, OnWakeup);
    OnSleep();

#if (AT_SUPPORT)
    enableAt();
#endif

    deviceState = DEVICE_STATE_INIT;
    LoRaWAN.ifskipjoin();

    printGpsInfo();
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
            LoRaWAN.displayJoining();
            LoRaWAN.join();
            break;
        }
        case DEVICE_STATE_SEND:
        {
            LoRaWAN.displaySending();
            PrepareTxFrame(DEVPORT);
            LoRaWAN.send();
            deviceState = DEVICE_STATE_CYCLE;
            break;
        }
        case DEVICE_STATE_CYCLE:
        {
            LoRaWAN.displayText("State:", "CYCLE");
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
                LoRaWAN.displayText("Woke up","Try to send");
                if (IsLoRaMacNetworkJoined)
                {
                    if (PrepareTxFrame(APPPORT))
                    {
                        LoRaWAN.displaySending();
                        LoRaWAN.send();
                    }
                }
                accelWoke = false;
            }
            else
            {
                LoRaWAN.displayText("Sleeping...", "");
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