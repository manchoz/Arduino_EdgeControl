#include <Arduino_JSON.h>

// Configure the RPC client
#include <openmvrpc.h>
openmv::rpc_scratch_buffer<256> scratch_buffer; // static memory buffer
openmv::rpc_callback_buffer<2> callback_buffer; // max number of callbacks
openmv::rpc_hardware_serial_uart_slave remote(115200);

#include "arduino_secrets.h"
#include "thingProperties.h"

void setup()
{
    // Initialize serial and wait for port to open:
    Serial.begin(9600);
    // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
    delay(1500);

    pinMode(LED_BUILTIN, OUTPUT);
    for (auto i = 0u; i < 5; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(75);
    }

    initProperties();

    ArduinoCloud.begin(ArduinoIoTPreferredConnection);         

    setDebugMessageLevel(4);
    ArduinoCloud.printDebugInfo();

    // Register RPC callbacks and start the client
    remote.register_callback("getProperties", getProperties);
    remote.register_callback("setProperties", setProperties);
    remote.begin();
}

void loop()
{
    ArduinoCloud.update();
    remote.loop(1000, 3000);
}

void onLedBuiltinChange()
{
    // Add your code here to act upon LedBuiltin change
    digitalWrite(LED_BUILTIN, ledBuiltin);
}

void getProperties(void* in_data, size_t in_data_len, void** out_data, size_t* out_data_len)
{
    JSONVar props;

    props["ledBuiltin"] = ledBuiltin;
    // Set the return data
    auto jsonStr = JSON.stringify(props);

    memcpy(*out_data, jsonStr.c_str(), jsonStr.length());
    *out_data_len = jsonStr.length();

    return;
}

void setProperties(void* in_data, size_t in_data_len)
{
    // Serial.println(reinterpret_cast<char *>(in_data));

    JSONVar props = JSON.parse(reinterpret_cast<char *>(in_data));

    temperatureA = double { props["temperatureA"] };
    humidityA = double { props["humidityA"] };
    pressureB = double { props["pressureB"] };
    temperatureB = double { props["temperatureB"] };

    // Serial.print("temperatureA: "); Serial.println(temperatureA);
    // Serial.print("humidityA: "); Serial.println(humidityA);
    // Serial.print("pressureB: "); Serial.println(pressureB);
    // Serial.print("temperatureB: "); Serial.println(temperatureB);
}

