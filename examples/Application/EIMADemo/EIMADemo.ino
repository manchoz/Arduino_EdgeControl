/*
    NOTE: https://medium.com/fasal-engineering/a-low-cost-circuit-to-read-from-multiple-watermark-irrometer-200ss-sensors-a4c838da233a
*/

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
uint8_t moistureC {};
float watermarkC {};

constexpr auto adcResolution { 12 };

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

    // Wait for MKRs to power up
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

    analogReadResolution(adcResolution);

    Input.begin();
    Watermark.begin();

    // Start the RPC controller
    rpc.begin();

    Debug.print(DBG_INFO, "Starting");
}

void loop()
{
    Expander.digitalWrite(EXP_LED1, ledStatus ? LOW : HIGH);
    if (millis() > requestNow) {
        Debug.print(DBG_VERBOSE, "Doing RPC stuff...");

        rpcGetProperties();

        requestNow = millis() + requestInterval;
        Debug.print(DBG_VERBOSE, "RPC stuff done.");
    }

    if (millis() > sensorsNow) {
        getSensors();
        rpcSetProperties();
        sensorsNow = millis() + sensorsInterval;
    }
}

void rpcGetProperties()
{
    Debug.print(DBG_VERBOSE, "Getting Latest LED status via RPC");

    // buffer for return data from RPC client
    size_t bufferLen { scratch_buffer.buffer_size() };
    char buffer[bufferLen] {};

    // Call the "getProperties" callback on the MKR WiFi 1010
    // Remember to set large RPC timeouts: the Network is SLOW!
    auto ret = rpc.call("getProperties",
        // (void*)deviceID.c_str(), deviceID.length(), // arguments
        nullptr, 0, // arguments
        buffer, bufferLen, // returns
        true, 1000, 1000); // parameters

    if (ret == 0) {
        Debug.print(DBG_ERROR, "%s", "RPC Error");
        return;
    }

    // Data from props is in JSON format
    Debug.print(DBG_VERBOSE, "%s", buffer);
    JSONVar props = JSON.parse(buffer);

    // Extract next LED status
    ledStatus = props["ledBuiltin"];

    // Pin LED is active low
    Debug.print(DBG_VERBOSE, "LED Status = %d (%s)", ledStatus, ledStatus ? "Off" : "On");
}

void rpcSetProperties()
{
    JSONVar props;

    props["tA"] = temperatureA;
    props["hA"] = humidityA;
    props["pB"] = pressureB;
    props["tB"] = temperatureB;
    props["mC"] = moistureC;
    props["wC"] = watermarkC;

    auto jsonStr = JSON.stringify(props);
    Debug.print(DBG_INFO, "%s", jsonStr.c_str());

    auto ret = rpc.call("setProperties",
        (void*)jsonStr.c_str(), jsonStr.length(), // arguments
        nullptr, 0, // returns
        false, 1000, 1000); // parameters
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

    moistureC = getMoisturePerc();
    Debug.print(DBG_INFO, "Moisture = %d %%", moistureC);

    watermarkC = getWatermark();
    Debug.print(DBG_INFO, "Watermark = %f kPa", watermarkC);

    Debug.print(DBG_INFO, "");
}

uint16_t getAverageInputRead()
{
    constexpr int pin { INPUT_05V_CH01 };
    constexpr size_t loops { 10 };
    unsigned int tot { 0 };

    Input.enable();
    for (auto i = 0u; i < loops; i++)
        tot += Input.analogRead(pin);
    Input.disable();

    return tot / loops;
}

uint8_t getMoisturePerc()
{
    // Keep track ok dry/wet values. YMMV.
    static long dryValue { 2160 };
    static long wetValue { 975 };

    auto val = getAverageInputRead();

    // Self-update dry/wet values range.
    if (val > dryValue)
        dryValue = val;
    if (val < wetValue)
        wetValue = val;

    Debug.print(DBG_DEBUG, "Value = %d, Dry Value = %d – Wet Value = %d", val, dryValue, wetValue);

    auto perc = map(val, dryValue, wetValue, 0, 100);

    return static_cast<uint8_t>(perc);
}

int getAverageWatermarkRead(pin_size_t pin)
{
    constexpr size_t count { 20 };
    int sum { 0 };

    Watermark.calibrationMode(OUTPUT);
    Watermark.calibrationWrite(LOW);

    Watermark.commonMode(OUTPUT);

    Watermark.enable();

    for (auto i = 0u; i < count; i++) {
        Watermark.commonWrite(HIGH);
        delay(2);
        sum += Watermark.analogRead(pin);
        Watermark.commonWrite(LOW);
    }

    Watermark.disable();

    return sum / count;
}

float getWatermark()
{
    constexpr unsigned int calibResistor { 7870 };
    constexpr long openResistance { 35000 };
    constexpr long shortResistance { 200 };
    constexpr long shortkPa { 240 };
    constexpr long openkPa { 255 };

    constexpr auto maxValue { 1 << adcResolution };
    constexpr float toV { 3.3f / float { maxValue } };

    float kPa;
    
    auto val = getAverageWatermarkRead(WATERMARK_CH01);

    Debug.print(DBG_DEBUG, "S = %d, V = %f", val, float { val } * toV);

    if (val == 0)
        return openkPa;

    auto resistor = calibResistor * float { maxValue - val } / float { val };

    if (resistor > 550.f) {
        if (resistor > 8000.f) {
            kPa = -2.246f - 5.239f * (resistor / 1000.f) * (1.f + .018f * (temperatureB - 24.f)) - .06756f * (resistor / 1000.f) * (resistor / 1000.f) * ((1.f + 0.018f * (temperatureB - 24.f)) * (1.f + 0.018f * (temperatureB - 24.f)));
        } else if (resistor > 1000.f) {
            kPa = (-3.213f * (resistor / 1000.f) - 4.093f) / (1.f - 0.009733f * (resistor / 1000.f) - 0.01205f * (temperatureB));
        } else {
            kPa = ((resistor / 1000.f) * 23.156f - 12.736f) * (1.f + 0.018f * (temperatureB - 24.f));
        }
    } else {
        if (resistor > 300.f)
            kPa = 0.f;
        if (resistor < 300.f && resistor >= shortResistance)
            kPa = shortkPa; // 240 is a fault code for sensor terminal short
    }

    if (resistor >= openResistance) {
        kPa = openkPa; // 255 is a fault code for open circuit or sensor not present
    }

    Debug.print(DBG_DEBUG, "Watermark average analogRead value: %d - Calculated Resistor: %f", val, resistor);

    return abs(kPa);
}
