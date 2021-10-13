// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char THING_ID[] = "692375e6-27f7-418e-a7d0-f95139c86960";

const char SSID[] = SECRET_SSID; // Network SSID (name)
const char PASS[] = SECRET_PASS; // Network password (use for WPA, or use as key for WEP)

void onLedBuiltinChange();

float temperatureA;
float humidityA;
float temperatureB;
float pressureB;
float moistureC;
float watermarkC;

bool ledBuiltin;

void initProperties()
{
    ArduinoCloud.setThingId(THING_ID);
    ArduinoCloud.addProperty(temperatureA, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(humidityA, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(temperatureB, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(pressureB, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(moistureC, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(watermarkC, READ, ON_CHANGE, NULL);

    ArduinoCloud.addProperty(ledBuiltin, READWRITE, ON_CHANGE, onLedBuiltinChange);
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
