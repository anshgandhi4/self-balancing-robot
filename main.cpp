#include "LSM9DS1.h"
#include "Motor.h"
#include "mbed.h"

#define CALIBRATION_READS 200
#define GYRO_CONFIDENCE 0.98
#define PI 3.14159265359

float kP = -4.0;
float kD = 0;
float kF = 0;

#define IMU 1
#define MOTORS 1
#define BLUETOOTH 1

Serial bluetooth(p13, p14);
Motor leftMotor(p25, p6, p5);
Motor rightMotor(p24, p8, p7);
LSM9DS1 imu(p28, p27, 0xD6, 0x3C);
Serial serial(USBTX, USBRX);
Ticker loop;

BusOut leds(LED1, LED2, LED3, LED4);

float accelBias = 0;
float gyroBias = 0;
float accelAngle = 0;
float gyroAngle = 0;
float currentAngle = 0;
float lastAngle = 0;
float lastCommand = 0;
float targetAngle = PI/2;
float s = -1;
bool enable = false;

#if IMU
void calibrateIMU() {
    for (int i = 0; i < CALIBRATION_READS; ++i) {
        imu.readAccel();
        imu.readGyro();
        // accelBias += atan2((float) imu.az, (float) -imu.ay); // https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html
        // gyroBias += imu.gx;
        accelBias += atan2((float) imu.az, (float) imu.ax); // https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html
        gyroBias += imu.gy;
    }

    accelBias /= CALIBRATION_READS;
    gyroBias /= CALIBRATION_READS;
    
    serial.printf("accelBias: %f\n", accelBias);
    serial.printf("gyroBias: %f\n", gyroBias);
}

// void measure() {
//     float avgAccel = 0;
//     float avgGyro = 0;
//     int num_iters = 1;
//     for (int i = 0; i < num_iters; ++i) {
//         imu.readAccel();
//         // imu.readGyro();
//         // accelAngle = atan2((float) imu.az, (float) -imu.ay) - accelBias;
//         // gyroAngle = (imu.gx - gyroBias) * 0.01;
//         accelAngle = atan2((float) imu.az, (float) imu.ax) - accelBias;
//         gyroAngle = (imu.gy - gyroBias) * 0.01;

//         avgAccel += accelAngle;
//         avgGyro += gyroAngle;
//     }
//     avgAccel /= num_iters;
//     avgGyro /= num_iters;

//     // serial.printf("%3.2f, %3.2f\n", avgAccel * 180 / PI, avgGyro * 180 / PI);

//     currentAngle = GYRO_CONFIDENCE * (currentAngle + avgGyro) + (1 - GYRO_CONFIDENCE) * avgAccel + 3 * PI / 2;
//     if (currentAngle > PI) {
//         currentAngle -= 2 * PI;
//     }

//     if (fabs(currentAngle - lastAngle) < 40 * PI / 180) {
//         currentAngle = 0.99 * currentAngle + 0.01 * lastAngle;
//         lastAngle = currentAngle;
//     } else {
//         serial.printf("giga %3.2f %3.2f \n", fabs(currentAngle - lastAngle) * 180 / PI, currentAngle * 180 / PI);
//         currentAngle = lastAngle;
//     }

//     currentAngle = currentAngle < -PI/2 ? -PI/2 : currentAngle;
//     currentAngle = currentAngle > PI/2 ? PI/2 : currentAngle;
//     lastAngle = currentAngle;
//     // serial.printf("%3.2f\n", currentAngle * 180 / PI);
// }

void measure() {
    // float avgAccel = 0;
    int num_iters = 1;
    float avgAngle = 0;
    float tempAngle = 0;
    float gyroTemp = 0;
    float gyroVel = 0;

    for (int i = 0; i < num_iters; i++) {
        imu.readAccel();
        imu.readGyro();
        accelAngle = atan2((float) imu.az, (float) imu.ax) - accelBias;
        if (accelAngle < -PI) {
            accelAngle += 2*PI;
        }
        gyroTemp = (imu.gy - gyroBias) * -0.01 * 0.4 / 1000;
        gyroVel += gyroTemp;

        avgAngle += accelAngle;

        // serial.printf("%3.2f, %3.2f\n", avgAccel * 180 / PI, avgGyro * 180 / PI);

        // tempAngle = accelAngle + 3 * PI / 2;
        // if (tempAngle > PI) {
        //     tempAngle -= 2 * PI;
        // }

        // tempAngle = tempAngle < -PI/2 ? -PI/2 : tempAngle;
        // tempAngle = tempAngle > PI/2 ? PI/2 : tempAngle;
        // avgAngle += tempAngle;
        // serial.printf("%3.2f\n", tempAngle * 180 / PI);
    }
    
    gyroVel /= num_iters;
    avgAngle /= num_iters;
    gyroAngle += gyroVel;
    serial.printf("%+5.2f %+5.2f %+5.2f\n", gyroAngle * 180/PI, avgAngle * 180/PI, currentAngle * 180/PI);
    currentAngle = GYRO_CONFIDENCE * (currentAngle + gyroVel) + (1 - GYRO_CONFIDENCE) * avgAngle;

    // if (currentAngle - lastAngle < 40 * PI / 180 || lastAngle - currentAngle < 40 * PI / 180) {
    //     currentAngle = 0.99 * currentAngle + 0.01 * lastAngle;
    // } else {
    //     serial.printf("giga %3.2f %3.2f \n", fabs(currentAngle - lastAngle) * 180 / PI, currentAngle * 180 / PI);
    //     currentAngle = lastAngle;
    // }
    
    // serial.printf("%3.2f\n", currentAngle * 180 / PI);
}
#endif

#if MOTORS
void controller() {
    float p = currentAngle - targetAngle;
    float d = currentAngle - lastAngle;
    float f = tan(currentAngle);
    lastAngle = currentAngle; 

    float c = kP * p + kD * d + kF * f;
    c = 0.5 * c + 0.5 * lastCommand;
    c = c < -1 ? -1 : c;
    c = c > 1 ? 1 : c;

    c *= enable;
    lastCommand = c;
    leftMotor.speed(c);
    rightMotor.speed(-c);

    // serial.printf("%1.1f, %1.2f, %3.2f\n", kP, kD, currentAngle * 180 / PI);
    // serial.printf("%1.2f, %1.3f, %1.2f, %3.2f\n", kP, kD, c, currentAngle * 180 / PI);

    // leftMotor.speed(s);
    // rightMotor.speed(s);
    
    // if (s < 1) {
    //     s += 0.01;
    // } else {
    //     s = -1;
    // }
}
#endif

void update() {
    #if IMU && !MOTORS
    measure();
    #elif !IMU && MOTORS
    controller(0);
    #elif IMU && MOTORS
    measure();
    controller();
    #endif
}

int main() {
    #if IMU
    imu.begin();
    calibrateIMU();
    #endif
    // loop.attach(&update, 0.01); // 10ms, 100Hz

    while (1) {
        char bnum = 0;

        if (!bluetooth.readable()) {
            update();
        } else {
            #if BLUETOOTH
            if (bluetooth.getc() == '!') {
                if (bluetooth.getc() == 'B') {
                    bnum = bluetooth.getc();
                    if (bnum == '1') {
                        enable = true;
                    } else if (bnum == '2') {
                        enable = false;
                    } else if (bnum == '5') {
                        if (bluetooth.getc() == '1') {
                            kP += 0.1;
                        }
                    } else if (bnum == '6') {
                        if (bluetooth.getc() == '1') {
                            kP -= 0.1;
                        }
                    } else if (bnum == '7') {
                        if (bluetooth.getc() == '1') {
                            kD += 0.01;
                        }
                    } else if (bnum == '8') {
                        if (bluetooth.getc() == '1') {
                            kD -= 0.01;
                        }
                    }
                }
            }
            #endif
        }
    }
}


// #include "LSM9DS1.h"
// #include "mbed.h"

// #define CALIBRATION_READS 200
// #define GYRO_CONFIDENCE 0
// #define PI 3.14159265359

// Serial bluetooth(p28, p27);
// LSM9DS1 imu(p9, p10, 0xD6, 0x3C);
// Serial serial(USBTX, USBRX);

// float accelBias = 0;
// float gyroBias = 0;
// float accelAngle = 0;
// float gyroVel = 0;
// float gyroAngle = 0;
// float currentAngle = 0;

// void calibrateIMU() {
//     for (int i = 0; i < CALIBRATION_READS; ++i) {
//         imu.readAccel();
//         imu.readGyro();
//         // accelBias += atan2((float) imu.az, (float) -imu.ay); // https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html
//         // gyroBias += imu.gx;
//         accelBias += atan2((float) imu.az, (float) imu.ax); // https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html
//         gyroBias += imu.gy;
//     }

//     accelBias /= CALIBRATION_READS;
//     gyroBias /= CALIBRATION_READS;
    
//     serial.printf("accelBias: %f\n", accelBias);
//     serial.printf("gyroBias: %f\n", gyroBias);
// }

// void measure() {
//     imu.readAccel();
//     imu.readGyro();
//     accelAngle = atan2((float) imu.az, (float) imu.ax) - accelBias;
//     if (accelAngle < -PI) {
//         accelAngle += 2*PI;
//     }
//     gyroVel = (imu.gy - gyroBias) * -0.01 * 1.5 / 1000;
//     gyroAngle += gyroVel;
// }

// int main() {
//     imu.begin();
//     calibrateIMU();

//     serial.printf("test\n");
//     while (1) {
//         measure();
//         currentAngle = 0.98 * (currentAngle + gyroVel) + 0.02 * accelAngle;
//         serial.printf("%+5.2f %+5.2f %+5.2f\n", gyroAngle * 180/PI, accelAngle * 180/PI, currentAngle * 180/PI);
//         if (bluetooth.writeable()) {
//             bluetooth.printf("%5.2f, %5.2f, %5.2f\n", gyroAngle * 180/PI, accelAngle * 180/PI, currentAngle * 180/PI);
//         }
//         wait(0.1);
//     }

//     return 0;
// }
