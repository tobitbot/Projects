#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "Wire.h"

#include "loraConfig.h"
#include "sensorConfig.h"

#include "CayenneLpp.h"

#define SEALEVELPRESSURE_HPA (1026.25) // this should be set according to the weather forecast

#define timetosleep 20000
#define timetowake 20000
static TimerEvent_t sleep;
static TimerEvent_t wakeup;
uint8_t lowpower = 1;

// Generate LPP Object with size
CayenneLPP lpp(51);
uint8_t appDataIndex = 1;

Sensors sensor(&lpp);


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
 * Sets some demo values on lpp
 */
void setDemoValues()
{
    Serial.println("Set demo values");
    lpp.addDigitalInput(appDataIndex++, 1);
    lpp.addDigitalInput(appDataIndex++, 0);
    lpp.addDigitalInput(appDataIndex++, 1);
}



/**
 * Copys the lpp buffer into appData that will
 * be send via LoRa.
 */
static void setPayloadData()
{
    Serial.print("appData: ");
    lpp.getBuffer();

    uint8_t* buffer = lpp.getBuffer();

    for (uint8_t i = 0; i < lpp.getSize(); ++i)
    {
        appData[i] = buffer[i];

        Serial.print(appData[i], HEX);
        Serial.print(" ");
    }

    appDataSize = lpp.getSize();
    Serial.print("Payload size: ");
    Serial.println(appDataSize);
}

void getSensorValues()
{
    Serial.println("\nGetSensorValues()");
    // switch on external voltage, inverse logic
    //! This may causes a bug on CCS811.
    //! Maybe it needs a new init, when power has gone since initCCS811();

    digitalWrite(Vext, LOW);
    delay(20);

    sensor.resetLppChannel();

    for (int i = 0; i < sizeof(sensors); i++)
    {
        switch (sensors[i])
        {
            case SensorType::_BMP180:
                sensor.readBMP180();
                break;
            case SensorType::_BME280:
                sensor.readBME280();
                break;
            case SensorType::_BH1750:
                sensor.readBH1750();
                break;
            case SensorType::_CCS811:
                sensor.readCCS811();
                break;
            case SensorType::_DHT11:
                sensor.readDHT11(DHT_PIN);
                break;
            case SensorType::_230V_INPUT:
                sensor.read230VInputs(inputs);
                break;
            default:
                setDemoValues();
                break;
        }
    }
    // switch off external voltage
    digitalWrite(Vext, HIGH);
    setPayloadData();
}

/* Prepares the payload of the frame */
static void PrepareTxFrame(uint8_t port)
{
    lpp.reset();
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

    //sensor.initCCS811();

    Serial.println(sizeof(inputs));

    sensor.init230VInputs(inputs);
    sensor.read230VInputs(inputs);

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