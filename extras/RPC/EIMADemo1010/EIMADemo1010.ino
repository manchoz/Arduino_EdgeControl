#include <Arduino_JSON.h>

// Configure the RPC client
#include <openmvrpc.h>
openmv::rpc_scratch_buffer<256> scratch_buffer; // static memory buffer
openmv::rpc_callback_buffer<2> callback_buffer; // max number of callbacks
openmv::rpc_hardware_serial_uart_slave remote(115200);

// TODO: shared SSID/PASSWD config
#include "arduino_secrets.h"
#include "thingProperties.h"

constexpr unsigned long keepAliveInterval { 5 * 1000 };
unsigned long keepAliveNow {};

void setup()
{
    // Initialize serial and wait for port to open:
    Serial.begin(9600);
    for (const auto timeout = millis() + 2500; !Serial && millis() < timeout; delay(250))
        ;

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
    remote.loop(1000, 3000);
    ArduinoCloud.update();
    if (millis() > keepAliveNow) {
        keepAlive();
        keepAliveNow = millis() + keepAliveInterval;
    }
}

void keepAlive()
{
    if (ArduinoCloud.connected()) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
    } else {
        for (auto i = 0u; i < 2; i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
            delay(75);
        }
    }
}

void onLedBuiltinChange()
{
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
    auto buffer = reinterpret_cast<char *>(in_data);

    // Null terminate the buffer for JSON operations
    buffer[in_data_len] = 0;

    Serial.println(buffer);
    JSONVar props = JSON.parse(buffer);

    temperatureA = double { props["tA"] };
    humidityA = double { props["hA"] };
    pressureB = double { props["pB"] };
    temperatureB = double { props["tB"] };
    moistureC = double { props["mC"] };
    watermarkC = double { props["wC"] };

    // Serial.print("temperatureA: "); Serial.println(temperatureA);
    // Serial.print("humidityA: "); Serial.println(humidityA);
    // Serial.print("pressureB: "); Serial.println(pressureB);
    // Serial.print("temperatureB: "); Serial.println(temperatureB);
}
