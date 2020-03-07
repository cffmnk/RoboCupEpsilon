#include <Arduino.h>
#include "Robot.h"

void eps::Robot::calibrateGyro() {
    Serial1.write(0xA5);
    Serial1.write(0x55);
}

void eps::Robot::readLine() {
    for (int i = 0; i < 4; ++i) {
        digitalWrite(this->pin_line_multiplxr_1, i & 1);
        digitalWrite(this->pin_line_multiplxr_2, i >> 1);

        this->line_sensors[0][i] = analogRead(A0);
        this->line_sensors[1][i] = analogRead(A1);
        this->line_sensors[2][i] = analogRead(A2);
        this->line_sensors[3][i] = analogRead(A3);
    }
}

void eps::Robot::readGyro() {
    unsigned char Re_buf[8], counter = 0;
    Serial1.write(0XA5);
    Serial1.write(0X51);//send it for each read
    while (Serial1.available()) {
        Re_buf[counter] = (unsigned char)Serial1.read();
        if (counter == 0 && Re_buf[0] != 0xAA) this->gyro = {0, 0 ,0};
        counter++;
        if (counter == 8) {
            counter = 0;
            if (Re_buf[0] == 0xAA && Re_buf[7] == 0x55) {
                Serial.println("lol");
                this->gyro.yaw = DEG_TO_RAD * ((Re_buf[1] << 8 | Re_buf[2]) / 100.00);
                this->gyro.pitch = (int16_t)(Re_buf[3]<<8|Re_buf[4])/100.00;
                this->gyro.roll = (int16_t)(Re_buf[5]<<8|Re_buf[6])/100.00;
            }
        }
    }
    this->gyro = {0, 0 ,0};
}

void eps::Robot::setKicker(bool state) {
    digitalWrite(this->pin_kicker, state);
}

void eps::Robot::invertMotor(byte port) {
    byte tmp = this->pin_motors_in1[port];
    this->pin_motors_in1[port] = this->pin_motors_in2[port];
    this->pin_motors_in2[port] = tmp;
}

bool eps::Robot::readButton(byte btn) {
    if (btn == LEFT) {
        return digitalRead(this->pin_btn_left);
    }
    else if (btn == CENTER) {
        return digitalRead(this->pin_btn_center);
    }
    else if (btn == RIGHT) {
        return digitalRead(this->pin_btn_right);
    }
    else {
        return 0;
    }
}

void eps::Robot::setLED(byte led, bool state) {
    if (led == LEFT) {
        digitalWrite(this->pin_led_left, state);
    }
    else if (led == CENTER) {
        digitalWrite(this->pin_led_center, state);
    }
    else if (led == RIGHT) {
        digitalWrite(this->pin_led_right, state);
    }
}

void eps::Robot::setSpeed(byte port, double motor_speed) {
    if (port >= 0 && port < 4) {
        analogWrite(this->pin_motors_pwm[port], abs(constrain(motor_speed, -255, 255)));
        digitalWrite(this->pin_motors_in1[port], (motor_speed >= 0));
        digitalWrite(this->pin_motors_in2[port], (motor_speed < 0));
    }
}

bool eps::Robot::init() {
    Serial.begin(115200);
    Serial1.begin(115200);// SoftwareSerial can only support 9600 baud rate for GY 25 but Serial1 can support 115200 and 9600 both
    delay(4000);
    Serial1.write(0XA5);
    Serial1.write(0X54);//correction mode
    delay(4000);
    Serial1.write(0XA5);
    Serial1.write(0X51);//0X51:query mode, return directly to the angle value, to be sent each read, 0X52:Automatic mode,send a direct return angle, only initialization
    Serial1.write(0XA5);
    Serial1.write(0X55);

    this->pixy.init();
    this->pixy.setLED(128, 0, 128);

    pinMode(this->pin_led_left, OUTPUT);
    pinMode(this->pin_led_center, OUTPUT);
    pinMode(this->pin_led_right, OUTPUT);

    pinMode(this->pin_btn_left, INPUT_PULLUP);
    pinMode(this->pin_btn_center, INPUT_PULLUP);
    pinMode(this->pin_btn_right, INPUT_PULLUP);

    pinMode(this->pin_line_multiplxr_1, OUTPUT);
    pinMode(this->pin_line_multiplxr_2, OUTPUT);

    pinMode(this->pin_standby1, OUTPUT);
    digitalWrite(this->pin_standby1, HIGH);
    pinMode(this->pin_standby2, OUTPUT);
    digitalWrite(this->pin_standby2, HIGH);

    for (int i = 0; i < 4; ++i) {
        pinMode(this->pin_motors_pwm[i], OUTPUT);
        pinMode(this->pin_motors_in1[i], OUTPUT);
        pinMode(this->pin_motors_in2[i], OUTPUT);
    }

    pinMode(this->pin_ball_ir, OUTPUT);
    pinMode(this->pin_kicker, OUTPUT);

    Serial1.write(0XA5);
    Serial1.write(0X55);

    this->pixy.setLED(0, 0, 0);
}