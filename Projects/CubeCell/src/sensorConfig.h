#include "Arduino.h"
#include "Sensors.h"

#define DHT_PIN GPIO1

// Add the connected sensors here
SensorType sensors[] = {SensorType::_230V_INPUT};

uint8_t inputs[4] = { GPIO_1, GPIO_2, GPIO_3, GPIO_4 };