enum class SerialCode {
    home = 0,
    position = 1,
};

const int delay_time = 1000;

const int x_switch = 10;
const int x_motor[4] = {4, 3, 2, A1};

const int y_switch = 11;
const int y_motor[4] = {A2, A3, A4, A5};

const int z_switch = 9;
const int z_motor[4] = {8, 7, 6, 5};

uint32_t x_pos = 0;
uint32_t y_pos = 0;
uint32_t z_pos = 0;

int x_step = 0;
int y_step = 0;
int z_step = 0;

void setup() {
    Serial.begin(9600);
    pinMode(x_switch, INPUT);
    for (int i = 0; i < 4; i++) {
        pinMode(x_motor[i], OUTPUT);
    }

    pinMode(y_switch, INPUT);
    for (int i = 0; i < 4; i++) {
        pinMode(y_motor[i], OUTPUT);
    }

    pinMode(z_switch, INPUT);
    for (int i = 0; i < 4; i++) {
        pinMode(z_motor[i], OUTPUT);
    }

    home();
}

void loop() {
    if (Serial.available() > 0) {
        SerialCode code = static_cast<SerialCode>(Serial.read());
        if (code == SerialCode::home) {
            home();
        } else if (code == SerialCode::position) {
            while (Serial.available() < 3 * sizeof(x_pos)) {
                delay(1);
            }

            uint32_t x_target, y_target, z_target;
            Serial.readBytes((char *)&x_target, sizeof(x_target));
            Serial.readBytes((char *)&y_target, sizeof(y_target));
            Serial.readBytes((char *)&z_target, sizeof(z_target));

            int x_direction = 1, y_direction = 1, z_direction = 1;
            while (x_pos != x_target || y_pos != y_target || z_pos != z_target) {
                if (x_pos < x_target) {
                    x_direction = 1;
                } else if (x_pos > x_target) {
                    x_direction = -1;
                } else {
                    x_direction = 0;
                }

                if (y_pos < y_target) {
                    y_direction = 1;
                } else if (y_pos > y_target) {
                    y_direction = -1;
                } else {
                    y_direction = 0;
                }

                if (z_pos < z_target) {
                    z_direction = 1;
                } else if (z_pos > z_target) {
                    z_direction = -1;
                } else {
                    z_direction = 0;
                }
                x_pos += x_direction;
                y_pos += y_direction;
                z_pos += z_direction;
                step_motors(x_direction, y_direction, z_direction);
            }
        }

        Serial.write(1);
    }

    // sleep_motors();
}

void home() {
    z_pos = 0;
    while (digitalRead(z_switch) == LOW) {
        step_motors(0, 0, -1);
    }

    x_pos = 0;
    while (digitalRead(x_switch) == LOW) {
        step_motors(-1, 0, 0);
    }

    y_pos = 0;
    while (digitalRead(y_switch) == LOW) {
        step_motors(0, -1, 0);
    }
}

void step_motors(int x_dir, int y_dir, int z_dir) {
    x_step = (x_step + 8 + x_dir) % 8;
    y_step = (y_step + 8 + y_dir) % 8;
    z_step = (z_step + 8 + z_dir) % 8;
    for (int i = 0; i < 4; i++) {
        if (i == x_step / 2 || (x_step % 2 == 1 && i == (x_step / 2 + 1) % 4)) {
            digitalWrite(x_motor[i], HIGH);
        } else {
            digitalWrite(x_motor[i], LOW);
        }
    }

    for (int i = 0; i < 4; i++) {
        if (i == y_step / 2 || (y_step % 2 == 1 && i == (y_step / 2 + 1) % 4)) {
            digitalWrite(y_motor[i], HIGH);
        } else {
            digitalWrite(y_motor[i], LOW);
        }
    }

    for (int i = 0; i < 4; i++) {
        if (i == z_step / 2 || (z_step % 2 == 1 && i == (z_step / 2 + 1) % 4)) {
            digitalWrite(z_motor[i], HIGH);
        } else {
            digitalWrite(z_motor[i], LOW);
        }
    }

    delayMicroseconds(delay_time);
}

void sleep_motors() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(x_motor[i], LOW);
        digitalWrite(y_motor[i], LOW);
        digitalWrite(z_motor[i], LOW);
    }
}
