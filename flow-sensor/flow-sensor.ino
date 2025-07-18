#include <Arduino.h>
#include <SensirionI2cSf06Lf.h>
#include <Wire.h>

enum class SerialCode {
    close_valve = 0,
    open_valve = 1,
    read_flow = 2,
};

SensirionI2cSf06Lf sensor;
constexpr int valve_pin = 23;

void setup() {
    pinMode(valve_pin, OUTPUT);
    Serial.begin(9600);
    Wire.begin();
    sensor.begin(Wire, SLF3S_0600F_I2C_ADDR_08);
    sensor.stopContinuousMeasurement();
    int16_t error = sensor.startH2oContinuousMeasurement();
}

void loop() {
    if (Serial.available() > 0) {
        SerialCode code = static_cast<SerialCode>(Serial.read());
        if (code == SerialCode::open_valve) {
            digitalWrite(23, HIGH);
            Serial.write(0);
        } else if (code == SerialCode::close_valve) {
            digitalWrite(23, LOW);
            Serial.write(0);
        } else if (code == SerialCode::read_flow) {
            float flow = 0.0;
            float temperature = 0.0;
            uint16_t flags = 0;
            int16_t error = sensor.readMeasurementData(
                INV_FLOW_SCALE_FACTORS_SLF3S_0600F,
                flow,
                temperature,
                flags
            );
            if (error != NO_ERROR) {
                Serial.write(1);
                char message[64];
                Serial.print("Error trying to execute readMeasurementData(): ");
                errorToString(error, message, sizeof(message));
                Serial.println(message);
                Serial.write(0);
            } else {
                Serial.write(0);
                Serial.write((char *)&flow, sizeof(flow));
                Serial.write((char *)&temperature, sizeof(temperature));
                Serial.write((char *)&flags, sizeof(flags));
            }
        }
    }

    delay(10);
}
