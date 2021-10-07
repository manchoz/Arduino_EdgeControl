#include <Arduino_EdgeControl.h>

#include <Arduino_DebugUtils.h>
#include <Arduino_HTS221.h>
#include <Arduino_JSON.h>
#include <Arduino_LPS22HB.h>

#include <openmvrpc.h>

// Configure the RPC controller
openmv::rpc_scratch_buffer<256> scratch_buffer; // static memory buffer
openmv::rpc_hardware_serial_uart_master rpc(115200);

constexpr uint32_t requestInterval { 1 * 100 };
uint32_t requestNow {};

constexpr uint32_t sensorsInterval { 5 * 1000 };
uint32_t sensorsNow {};

bool ledStatus { false };

#if !defined(EIMA_DEBUG_LEVEL)
#define EIMA_DEBUG_LEVEL DBG_INFO
#endif
constexpr auto debugLevel { EIMA_DEBUG_LEVEL };

float temperatureA {};
float humidityA {};
float pressureB {};
float temperatureB {};

void setup()
{
    Serial.begin(9600);

    auto startNow = millis() + 2500;
    while (!Serial && millis() < startNow)
        ;

    Debug.setDebugLevel(debugLevel);
    Debug.timestampOn();

    EdgeControl.begin();
    delay(1000);
    Debug.print(DBG_INFO, "%s", "Hello, EIMA!");

    Power.on(PWR_3V3);
    Power.on(PWR_VBAT);
    Power.on(PWR_MKR1);
    Power.on(PWR_MKR2);

    delay(5000);

    if (!HTS.begin()) {
        Debug.print(DBG_ERROR, "%s", "Failed to initialize HTS11 sensor");
        while (1)
            ;
    }
    if (!BARO.begin()) {
        Debug.print(DBG_ERROR, "%s", "Failed to initialize LPS22HB sensor");
        while (1)
            ;
    }

    // delay(10000);
    // Init the I2C bus
    Wire.begin();
    delay(500);

    // Init the I/O Expander
    Debug.print(DBG_ERROR, "I/O Expander initialisation ");
    if (!Expander.begin()) {
        Debug.print(DBG_ERROR, "failed");
        Debug.print(DBG_ERROR, "Please, be sure to enable gated 3V3 and 5V power rails");
        Debug.print(DBG_ERROR, "via Power.on(PWR_3V3) and Power.on(PWR_VBAT)");
    }
    Debug.print(DBG_ERROR, "succeeded.");

    // Configure the LED1 pin
    Expander.pinMode(EXP_LED1, OUTPUT);
    // LED1 is active low
    Expander.digitalWrite(EXP_LED1, HIGH);

    // Start the RPC controller
    rpc.begin();

    Debug.print(DBG_INFO, "Starting");
}

void loop()
{
    Expander.digitalWrite(EXP_LED1, ledStatus ? LOW : HIGH);
    if (millis() > requestNow) {
        Debug.print(DBG_DEBUG, "Doing RPC stuff...");

        rpcGetProperties();

        requestNow = millis() + requestInterval;
        Debug.print(DBG_DEBUG, "RPC stuff done.");
    }

    if (millis() > sensorsNow) {
        getSensors();
        rpcSetProperties();
        sensorsNow = millis() + sensorsInterval;
    }
}

void rpcGetProperties()
{
    Debug.print(DBG_DEBUG, "Getting Latest LED status via RPC");

    // buffer for return data from RPC client
    size_t bufferLen { scratch_buffer.buffer_size() };
    char buffer[bufferLen] {};

    // Call the "getProperties" callback on the MKR WiFi 1010
    // Remember to set large RPC timeouts: the Network is SLOW!
    auto ret = rpc.call("getProperties",
        // (void*)deviceID.c_str(), deviceID.length(), // arguments
        nullptr, 0, // arguments
        buffer, bufferLen, // returns
        true, 1000, 3000); // parameters

    if (ret == 0) {
        Debug.print(DBG_ERROR, "%s", "RPC Error");
        return;
    }

    // Data from props is in JSON format
    Debug.print(DBG_DEBUG, "%s", buffer);
    JSONVar props = JSON.parse(buffer);

    // Extract next LED status
    ledStatus = props["ledBuiltin"];

    // Pin LED is active low
    Debug.print(DBG_DEBUG, "LED Status = %d (%s)", ledStatus, ledStatus ? "Off" : "On");
}

void rpcSetProperties()
{
    JSONVar props;

    props["temperatureA"] = temperatureA;
    props["humidityA"] = humidityA;
    props["pressureB"] = pressureB;
    props["temperatureB"] = temperatureB;

    auto jsonStr = JSON.stringify(props);

    auto ret = rpc.call("setProperties",
        (void*)jsonStr.c_str(), jsonStr.length(), // arguments
        nullptr, 0, // returns
        false, 1000, 3000); // parameters
}

void getSensors()
{
    Debug.print(DBG_INFO, "**** **** **** ****");
    Debug.print(DBG_INFO, "Getting sensors...");

    temperatureA = HTS.readTemperature();
    Debug.print(DBG_INFO, "Temperature = %f °C", temperatureA);

    humidityA = HTS.readHumidity();
    Debug.print(DBG_INFO, "Humidity = %f %%", humidityA);

    pressureB = BARO.readPressure();
    Debug.print(DBG_INFO, "Pressure = %f kPa", pressureB);

    temperatureB = BARO.readTemperature();
    Debug.print(DBG_INFO, "Temperature (BARO) = %f °C", temperatureB);
    Debug.print(DBG_INFO, "");
}
