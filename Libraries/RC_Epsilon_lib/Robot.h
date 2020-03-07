#ifndef EPS_ROBOT
#define EPS_ROBOT

#include <Arduino.h>
#include <Pixy2.h>

enum Location
{
    LEFT,
    CENTER,
    RIGHT
};

namespace eps {
    struct Gyro {
        double yaw;
        int16_t pitch;
        int16_t roll;
    };

    class Robot {
        private:
            const byte pin_kicker = 47;

            const byte pin_ball_ir = 43;

            const byte pin_led_left = 11;
            const byte pin_led_center = 7;
            const byte pin_led_right = 17;

            const byte pin_btn_left = 13;
            const byte pin_btn_center = 9;
            const byte pin_btn_right = 5;

            const byte pin_line_multiplxr_1 = 33;
            const byte pin_line_multiplxr_2 = 35;

            const byte pin_motors_pwm[4] = {2, 12, 8, 6};

            const byte pin_standby1 = 4;
            const byte pin_standby2 = 10;

        public:
            byte pin_motors_in1[4] = {36, 42, 44, 34};
            byte pin_motors_in2[4] = {38, 40, 46, 32};
            Pixy2 pixy;
            Gyro gyro;
            uint16_t line_sensors[4][4] = {0};

        public:
            bool init();

            bool readButton(byte btn);

            void setLED(byte led, bool state);

            void setSpeed(byte port, double motor_speed);
            void invertMotor(byte port);

            void setKicker(bool state);

            void readGyro();
            void calibrateGyro();

            void readLine();
    };
}

#endif