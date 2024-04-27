#include "LSM9DS1.h"
#include "Motor.h"
#include "mbed.h"

#define CALIBRATION_CYCLES 200
#define GYRO_CONFIDENCE 0.96
#define COMMAND_FILTER 1
#define PI 3.14159265359
#define RAD_TO_DEG 180/PI

#define TILT_ANGLE 0.05
#define ROTATION_SPEED 0.3
#define RESPONSE_CYCLES 6

float kP = -10.5;
float kD = 1.0;
float kF = 0;

Serial bluetooth(p13, p14);
Motor leftMotor(p25, p6, p5);
Motor rightMotor(p24, p8, p7);
LSM9DS1 imu(p28, p27, 0xD6, 0x3C);
Serial serial(USBTX, USBRX);

float accelBias = 0;
float gyroBias = 0;

float accelAngle = 0;
float gyroAngle = -PI/2;
float currentAngle = 0;
float lastAngle = 0;

int forwardTarget = 0;
int forwardCommand = 0;
float rotationCommand = 0;

float targetAngle = 0;
float lastCommand = 0;
bool enable = false;
int adjust = 0;

void calibrateIMU() {
    for (int i = 0; i < CALIBRATION_CYCLES; ++i) {
        imu.readAccel();
        imu.readGyro();
        accelBias += atan2((float) imu.az, (float) imu.ax); // https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html
        gyroBias += imu.gy;
    }

    accelBias /= CALIBRATION_CYCLES;
    gyroBias /= CALIBRATION_CYCLES;
    
    serial.printf("accelBias: %f\n", accelBias);
    serial.printf("gyroBias: %f\n", gyroBias);
}

void measure() {
    imu.readAccel();
    imu.readGyro();
    accelAngle = atan2((float) imu.az, (float) imu.ax) - accelBias - PI/2;
    if (accelAngle < -PI) {
        accelAngle += 2*PI;
    }

    float gyroVel = (gyroBias - imu.gy) * 0.00001 * 0.54;
    gyroAngle += gyroVel;

    currentAngle = GYRO_CONFIDENCE * (currentAngle + gyroVel) + (1 - GYRO_CONFIDENCE) * accelAngle;

    // serial.printf("%+3.2f %+3.2f %+3.2f\n", gyroAngle * RAD_TO_DEG, accelAngle * RAD_TO_DEG, currentAngle * RAD_TO_DEG);
    // if (bluetooth.writeable()) {
    //     bluetooth.printf("%+3.2f, %+3.2f, %+3.2f\n", gyroAngle * RAD_TO_DEG, accelAngle * RAD_TO_DEG, currentAngle * RAD_TO_DEG);
    // }
}

void controller() {
    float p = currentAngle - targetAngle - forwardCommand * TILT_ANGLE / RESPONSE_CYCLES;
    float d = currentAngle - lastAngle;
    float f = tan(currentAngle);
    lastAngle = currentAngle;

    float c = kP * p + kD * d + kF * f;
    c = COMMAND_FILTER * c + (1 - COMMAND_FILTER) * lastCommand;
    c = c < -1 ? -1 : c;
    c = c > 1 ? 1 : c;

    c *= enable;
    lastCommand = c;
    leftMotor.speed(c + rotationCommand);
    rightMotor.speed(-c + rotationCommand);

    serial.printf("%d, %+1.2f, %+1.3f, %+1.2f, %+1.2f, %+3.2f, %+3.2f, %+3.2f, %+3.2f, %d \n", adjust, kP, kD, kF, c, currentAngle * RAD_TO_DEG, targetAngle * RAD_TO_DEG, gyroAngle * RAD_TO_DEG, accelAngle * RAD_TO_DEG, forwardCommand);
    if (bluetooth.writeable()) {
        bluetooth.printf("%d, %+1.2f, %+1.3f, %+1.2f, %+1.2f, %+3.2f, %+3.2f, %+3.2f, %+3.2f, %d \n", adjust, kP, kD, kF, c, currentAngle * RAD_TO_DEG, targetAngle * RAD_TO_DEG, gyroAngle * RAD_TO_DEG, accelAngle * RAD_TO_DEG, forwardCommand);
    }
}

int main() {
    imu.begin();
    calibrateIMU();

    while (1) {
        if (!bluetooth.readable()) {
            measure();
            controller();
        } else {
            if (bluetooth.getc() == '!') {
                if (bluetooth.getc() == 'B') {
                    char bnum = bluetooth.getc();
                    switch (bnum) {
                        case '1':
                            if (bluetooth.getc() == '0') {
                                enable = !enable;
                            }
                            break;
                        case '2':
                            if (bluetooth.getc() == '1') {
                                adjust = (adjust + 1) % 4;
                            }
                            break;
                        case '3':
                            if (bluetooth.getc() == '1') {
                                targetAngle += 0.01;
                            }
                            break;
                        case '4':
                            if (bluetooth.getc() == '1') {
                                targetAngle -= 0.01;
                            }
                            break;
                        case '5':
                            if (bluetooth.getc() == '1') {
                                forwardTarget = RESPONSE_CYCLES;
                            } else {
                                forwardTarget = 0;
                            }
                            break;
                        case '6':
                            if (bluetooth.getc() == '1') {
                                forwardTarget = -RESPONSE_CYCLES;
                            } else {
                                forwardTarget = 0;
                            }
                            break;
                        case '7':
                            if (bluetooth.getc() == '1') {
                                if (adjust == 0) {
                                    rotationCommand = ROTATION_SPEED;
                                } else if (adjust == 1) {
                                    kP += 0.1;
                                } else if (adjust == 2) {
                                    kD += 0.1;
                                } else {
                                    kF += 0.1;
                                }
                            } else if (adjust == 0) {
                                rotationCommand = 0;
                            }
                            break;
                        case '8':
                            if (bluetooth.getc() == '1') {
                                if (adjust == 0) {
                                    rotationCommand = -ROTATION_SPEED;
                                } else if (adjust == 1) {
                                    kP -= 0.1;
                                } else if (adjust == 2) {
                                    kD -= 0.1;
                                } else {
                                    kF -= 0.1;
                                }
                            } else if (adjust == 0) {
                                rotationCommand = 0;
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        if (forwardTarget > forwardCommand) {
            forwardCommand += forwardCommand >= RESPONSE_CYCLES ? 0 : 1;
        } else if (forwardTarget < forwardCommand) {
            forwardCommand -= forwardCommand <= -RESPONSE_CYCLES ? 0 : 1;
        }
    }
}
