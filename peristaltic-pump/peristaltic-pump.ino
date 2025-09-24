#include <AccelStepper.h>

enum class Register {
    GlobalConfig = 0x00,
    GlobalStatus,
    TransmissionCount,
    NodeConfig,
    StartupConfigWrite,
    StartupConfigRead,
    PinStates,
    FactoryConfig,
    CurrentControl = 0x10,
    PowerDownDelay,
    StepTime,
    ChopperTypeThreshold,
    CoolStepThreshold,
    Velocity = 0x22,
    StallGuardThreshold = 0x40,
    MotorLoad,
    CoolStepConfig,
    MicrostepCount = 0x6a,
    MicrostepCurrent,
    ChopperConfig,
    DriverStatus = 0x6f,
    StealthChopConfig = 0x70,
    StealthChopScale,
    StealthChopAuto,
};

constexpr int rx_pin = 16;
constexpr int tx_pin = 17;
constexpr int step_pin = 18;
constexpr int dir_pin = 19;
constexpr int en_pin = 21;
HardwareSerial driver_serial = Serial2;
AccelStepper stepper(AccelStepper::DRIVER, step_pin, dir_pin);


uint8_t crc(uint8_t* msg, int length);
void declare(bool condition, char *error_msg);
uint32_t read(Register node_addr, uint8_t register_addr);
void write(uint8_t node_addr, Register register_addr, uint32_t data);
void clear_serial(int bytes_to_clear = -1);
uint32_t reverse(uint32_t data);


void setup() {
    Serial.begin(9600);
    pinMode(en_pin, OUTPUT);
    digitalWrite(en_pin, LOW);
    driver_serial.begin(115200, SERIAL_8N1, rx_pin, tx_pin);

    write(
        0,
        Register::GlobalConfig,
        0 << 0 | // Scale current based on internal reference voltage.
        0 << 1 | // Use external Rsense resistor to measure current.
        0 << 2 | // Enable stealthchop at low RPM.
        0 << 3 | // Don't invert motor direction.
        0 << 4 | // Index pin outputs first microstep position.
        0 << 5 | // Index pin doesn't output pulses from internal pulse generator.
        0 << 6 | // Don't use UART pin as powerdown pin.
        1 << 7 | // Microstep selected via UART.
        1 << 8 | // Filter step pulses.
        0 << 9   // Non-test mode.
    );
    write(
        0,
        Register::CurrentControl,
        12 << 0 | // Standstill current scaling.
        22 << 8 | // Running current scaling.
        5L << 16  // 5*2^18 cycles for motor power down.
    );
    write(
        0,
        Register::StealthChopConfig,
        12UL << 28 | // Limit for PWM_SCALE_AUTO
        8UL << 24 | // Maximum PWM amplitude change per half wave.
        0b00 << 20 | // Disable freewheel mode.
        1 << 19 | // Enable PWM automatic gradient tuning.
        1 << 18 | // Enable PWM automatic amplitude tuning.
        0b01 << 16 | // PWM frequency.
        14 << 8 | // Default PWM gradient.
        36 // Default PWM amplitude.
    );
    write(
        0,
        Register::ChopperConfig,
        1 << 31 | // Disable short protection low side
        0 << 30 | // Enable short to ground protection.
        0 << 29 | // Disable double edge step pulse.
        0 << 28 | // Don't extrapolate to 256 microsteps.
        0b1000 << 24 | // Enable PWM automatic gradient tuning.
        0 << 17 | // High sense resistor voltage.
        2 << 15 | // Set comparator blank time to 32 clocks.
        0 << 7 | // Hysteresis low value.
        4 << 4 | // Hysteresis start value added to end value.
        5 // Duraction of slow decay phase.
    );
    stepper.setMaxSpeed(500);
    stepper.setAcceleration(50);
    delay(1000);
}


void loop() {
    // Serial.println(read(0, Register::ChopperConfig), BIN);
    // Serial.println(read(0, Register::StealthChopConfig), BIN);
    // Spin continuously forward at max speed
    if (stepper.distanceToGo() == 0) {
        stepper.move(1L << 30);   // huge move makes it “infinite”
    }
    stepper.run();
}


uint8_t crc(uint8_t* msg, int length) {
    uint8_t crc = 0;
    for (int i = 0; i < length; ++i) {
        uint8_t curr_byte = msg[i];
        for (int j = 0; j < 8; ++j) {
            if ((crc >> 7) ^ (curr_byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            curr_byte = curr_byte >> 1;
        }
    }

    return crc;
}


uint32_t read(uint8_t node_addr, Register register_addr) {
    clear_serial();
    uint8_t request[4];
    request[0] = 0b00000101;
    request[1] = node_addr;
    request[2] = static_cast<uint8_t>(register_addr);
    request[3] = crc(request, 3);
    driver_serial.write(request, 4);

    clear_serial(4);
    uint8_t response[8];
    for (int i = 0; i < 8; ++i) {
        while (!driver_serial.available()) {
            delay(1);
        }
        response[i] = driver_serial.read();
    }
    declare((response[0] & 0b1111) == 0b0101, "Invalid sync bits in read response.");
    declare(response[1] == 0xff, "Invalid master address in read response.");
    declare(response[2] == static_cast<uint8_t>(register_addr), "Invalid register address in read response.");
    declare(response[7] == crc(response, 7), "Invalid CRC byte in read response.");
    return reverse(*reinterpret_cast<uint32_t *>(response + 3));
}


void write(uint8_t node_addr, Register register_addr, uint32_t data) {
    clear_serial();
    uint8_t request[8];
    request[0] = 0b00000101;
    request[1] = node_addr;
    request[2] = (1 << 7) | static_cast<uint8_t>(register_addr);
    *reinterpret_cast<uint32_t *>(request + 3) = reverse(data);
    request[7] = crc(request, 7);
    driver_serial.write(request, 8);
    clear_serial(8);
}

void clear_serial(int bytes_to_clear) {
    driver_serial.flush();
    if (bytes_to_clear == -1) {
        while (driver_serial.available()) {
            driver_serial.read();
        }
    } else {
        for (int i = 0; i < bytes_to_clear; ++i) {
            while (!driver_serial.available()) {
                delay(1);
            }
            driver_serial.read();
        }
    }
}

void declare(bool condition, char *error_msg) {
    if (!condition) {
        while (true) {
            Serial.println(error_msg);
            delay(1000);
        }
    }
}

uint32_t reverse(uint32_t data) {
    uint32_t reversed = 0;
    for (int i = 0; i < 4; ++i) {
        reversed |= ((data >> 8 * i) & 0xff) << 8 * (3 - i);
    }
    return reversed;
}
