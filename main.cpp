// int main()
// {
//     while (1) {
//         for (float s = -1.0; s < 1.0 ; s += 0.01) {
//             left.speed(s);
//             right.speed(s);
//             wait(0.02);
//         }
//     }
// }

#include "LSM9DS1.h"
#include "Motor.h"
#include "mbed.h"
// #include "rtos.h"

Serial bluetooth(p9, p10);
Motor leftMotor(p23, p6, p5);
Motor rightMotor(p24, p8, p7);
LSM9DS1 imu(p9, p10, 0xD6, 0x3C);
Serial serial(USBTX, USBRX);
Ticker controller;

float imuBias = 0;
float accAngle = 0;
float pAngle = 0;
float desiredAngle = 0;

void calibrateIMU() {
    for (int i = 0; i < 200; ++i) {
        imu.readAccel();
        imuBias = atan2((double) imu.az, (double) -imu.ay); // https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html
    }
    imuBias /= 200;
    serial.printf("imuBias: %f\n", imuBias);
}

void measure() {
    imu.readAccel();
    accAngle = atan2((double)imu.az, (double)-imu.ay) - imuBias;
    pAngle = accAngle - desiredAngle;
    serial.printf("angle %f\n", pAngle);

    // TODO: RUN PID
    // leftMotor.speed(pAngle);
    // rightMotor.speed(pAngle);
}

int main() {
    imu.begin();
    calibrateIMU();
    controller.attach(&measure, 0.01); // 10ms, 100Hz
    
    while (1) {
        char bnum = 0;
        char bhit = 0;
        while (!bluetooth.readable()) {
            wait(0.1);
        }
        
        if (bluetooth.getc() == '!') {
            if (bluetooth.getc() == 'B') {
                // TODO
            }
        }
    }
}
