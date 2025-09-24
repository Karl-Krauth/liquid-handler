#include <Wire.h>

#define I2C_HIGHDRIVER4_ADRESS  (0b1111011)

#define I2C_DEVICEID       0x00
#define I2C_POWERMODE      0x01
#define I2C_FREQUENCY      0x02
#define I2C_SHAPE          0x03
#define I2C_BOOST          0x04
#define I2C_PVOLTAGE       0x06
#define I2C_P1VOLTAGE      0x06
#define I2C_P2VOLTAGE      0x07
#define I2C_P3VOLTAGE      0x08
#define I2C_P4VOLTAGE      0x09
#define I2C_UPDATEVOLTAGE  0x0A
#define I2C_AUDIO          0x05

void setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.beginTransmission(I2C_HIGHDRIVER4_ADRESS);
    Wire.write(I2C_POWERMODE); // Start Register 0x01
    Wire.write(0x01); // Register 0x01 = 0x01 (enable)
    Wire.write(0x00); // Register 0x02 = 0x40 (100Hz)
    Wire.write(0x00); // Register 0x03 = 0x00 (sine wave)
    Wire.write(0x00); // Register 0x04 = 0x00 (800KHz)
    Wire.write(0x00); // Register 0x05 = 0x00 (audio off)
    Wire.write(0x00); // Register 0x06 = Amplitude1
    Wire.write(0x00); // Register 0x07 = Amplitude2
    Wire.write(0x00); // Register 0x08 = Amplitude3
    Wire.write(0x00); // Register 0x09 = Amplitude4
    Wire.write(0x01); // Register 0x0A = 0x01 (update)
    Wire.endTransmission();
}

void loop() {
    uint16_t frequency;
    uint8_t voltage;
    if (Serial.available() >= sizeof(frequency) + sizeof(voltage)) {
        Serial.readBytes((char *)&frequency, sizeof(frequency));
        Serial.readBytes((char *)&voltage, sizeof(voltage));

        // Set frequency.
        {
            uint8_t frequency_byte;
            if (frequency>=800) {
                frequency_byte = 0xFF;
            } else if (frequency >= 400) {
                frequency = 63 * (frequency - 400) / 400;
                frequency_byte = frequency | 0xC0;
            } else if (frequency >= 200) {
                frequency = 63 * (frequency - 200) / 200;
                frequency_byte = frequency | 0x80;
            } else if (frequency >= 100) {
                frequency = 63 * (frequency - 100) / 100;
                frequency_byte = frequency | 0x40;
            } else if (frequency >= 50) {
                frequency = 63 * (frequency - 50) / 50;
                frequency_byte = frequency | 0x00;
            } else {
                frequency_byte = 0x00;
            }
            Wire.beginTransmission(I2C_HIGHDRIVER4_ADRESS);
            Wire.write(I2C_FREQUENCY);
            Wire.write(frequency_byte);
            Wire.endTransmission();
        }


        // Set voltage for pump 3.
        {
            voltage *= 31.0 / 250.0;
            Wire.beginTransmission(I2C_HIGHDRIVER4_ADRESS);
            Wire.write(I2C_PVOLTAGE);
            // Pump 1
            Wire.write(constrain(voltage, 0, 31));
            // Pump 2
            Wire.write(constrain(voltage, 0, 31));
            // Pump 3
            Wire.write(constrain(voltage, 0, 31));
            // Pump 4
            Wire.write(constrain(voltage, 0, 31));
            Wire.write(0x01);
            Wire.endTransmission();
        }

        Serial.write(0);
    }

    delay(10);
}
