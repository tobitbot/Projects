#ifndef LORA_CONFIG_INCLUDED
#define LORA_CONFIG_INCLUDED

/**
 * Support diffrent types of boards
 */
//#define FEATHERWING
#define TTGOLORA

// This EUI is in LITTLE ENDIAN.
static const u1_t PROGMEM APPEUI[8] = { 0xAE, 0xEE, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

// This key is in LITTLE ENDIAN.
static const u1_t PROGMEM DEVEUI[8] = { 0xFB, 0xD3, 0xA4, 0xA4, 0x9B, 0x5F, 0xED, 0xDF };

// This key is in BIG ENDIAN. LSB -> MSB
static const u1_t PROGMEM APPKEY[16] = { 0x7b, 0x80, 0xfb, 0x33, 0xd0, 0x47, 0xf1, 0xbf, 0x04, 0x5e, 0xd9, 0x1c, 0x94, 0x54, 0x90, 0x6d };



// Pin mapping for LoRa-Radio
// Pin mapping for FeatherWing LoRa
#if defined(FEATHERWING)
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, 11},
};
#elif defined(TTGOLORA)
// Pin mapping for TTGO-LoRa
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},
};
#endif

#endif //LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED