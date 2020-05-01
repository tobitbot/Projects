/**
 * This sketch is designed for Adafruit Feather M0 LoRa with external BME280.
 * Sending data of BME280 via LoRa.
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
#include <Adafruit_SleepyDog.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <string.h>
#include <CayenneLPP.h>

#define SEALEVELPRESSURE_HPA (1013.25) // this should be set according to the weather forecast
#define BME280_ADDRESS 0x76

Adafruit_BME280 bme;

// BME280 data are saved here: Temperature, Humidity, Pressure, Altitude calculated from atmospheric pressure
float tmp, hum, pressure, alt_barometric;

// Generate LPP Object with size
CayenneLPP lpp(40);

// This EUI is in little-endian format
static const u1_t PROGMEM APPEUI[8]={ 0xAC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This key is LITTLE ENDIAN. Start typing to your server with the last element of array DEVEUI
static const u1_t PROGMEM DEVEUI[8]={ 0xFA, 0xD3, 0xA4, 0xA4, 0x9B, 0x5F, 0xED, 0xDF };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key is BIG ENDIAN. Start typing to your server with the first element of array APPKEY
static const u1_t PROGMEM APPKEY[16] = { 0x7A, 0x80, 0xFB, 0x33, 0xD0, 0x47, 0xF1, 0xBF, 0x04, 0x5E, 0xD9, 0x1C, 0x94, 0x54, 0x90, 0x6D };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Deep sleep time
const unsigned SLEEP_TIME = 512; //for now, this must be a scaler of 16

// Pin mapping for LoRa-Radio
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, 11},
};



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function declaration starts here.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Powers the bme280 for the time it is needed.
 */
void getBME280Values()
{
    // Power on BME
    digitalWrite(12, HIGH);
    delay(100);

    if (!bme.begin(BME280_ADDRESS)) {
        Serial.println("Es konnte kein BME280 Sensor gefunden werden!");
        Serial.println("Bitte überprüfen Sie die Verkabelung!");
        while (!bme.begin()){
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(300);
        };
    }
    else
    {
        // Read once for initialization
        bme.readTemperature();
        bme.readHumidity();
        bme.readAltitude(SEALEVELPRESSURE_HPA);
        bme.readPressure();
        delay(50);
    }

    tmp = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    alt_barometric = bme.readAltitude(SEALEVELPRESSURE_HPA);
    hum = bme.readHumidity();

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
    Serial.println("Power down BME");
    digitalWrite(12,LOW);
}

/**
 * Activate deepsleep mode
 */
void goToSleep() {
    Serial.println("Going to sleep now...");
    delay(100);
    digitalWrite(LED_BUILTIN, LOW); // Show we're asleep

    for (uint i = 0; i < SLEEP_TIME /16; i++) {
        //Watchdog.sleep(16000);
        delay(5000); //change to debug
    }

    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Woke up!");
}

void do_send(osjob_t* j){

    getBME280Values();

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
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

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
                Serial.print( "Yeey! Received "); Serial.print(LMIC.dataLen); Serial.println(" bytes of payload!");
                Serial.println("RSSI \t SNR \t");
                Serial.println(LMIC.rssi, LMIC.snr);
                Serial.println("--------------------");
            }
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
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
void setup() {

    // Set led to output
    pinMode(LED_BUILTIN, OUTPUT);

    // Set output to power bme 280 on demand
    pinMode(12,OUTPUT);
    digitalWrite(12,LOW);

    // Starting routine
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);

    //while(!Serial);
    Serial.begin(9600);
    Serial.println("Setup done....");

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
void loop() {
    // Process lora
    os_runloop_once();
}