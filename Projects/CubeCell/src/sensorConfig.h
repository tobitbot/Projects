#include "Arduino.h"
#include "Sensors.h"

#define DHT_PIN GPIO1

// Add the connected sensors here
SensorType sensors[] = {SensorType::_CCS811, SensorType::_DHT11};