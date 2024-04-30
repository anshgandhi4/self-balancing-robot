# Self-Balancing Robot
#### By: Ansh Gandhi, Ansh Bhatti, Niranjan Deepak
#### Georgia Institute of Technology

### Overview:
The purpose of this project is to design a robot with a two wheel base that can self balance when oriented vertically. Not only is this robot suppose to maintain balance it also should be able to move around.

The chasis of this robot was a custom design with hooks for attaching breadboard circuits and the motor. The chasis was created by water jetting polycarboante boards. 

Furthermore, the microcontroller used for this project is an mbed LPC 1768. Additional hardware that was utilized are an H-Bridge to control the dual motors, an IMU sensor to record gyroscopic & acceleration measurements, and a Bluetooth chip to manually control the robot.

This guide outlines the process of designing this robot including hardware and software walkthroughs along with a demo of the final robot.

### Materials:
1. [mbed LPC 1768](https://os.mbed.com/platforms/mbed-LPC1768/)
2. [IMU Sensor](https://os.mbed.com/components/LSM9DS1-IMU/)
3. [H-Bridge](https://os.mbed.com/cookbook/Motor)
4. [Adafruit Bluetooth](https://os.mbed.com/users/4180_1/notebook/adafruit-bluefruit-le-uart-friend---bluetooth-low-/)
5. [2 DC Motors](https://www.bananarobotics.com/shop/Yellow-Gear-Motor-with-48-to-1-reduction-ratio)
6. [Battery Pack](https://www.digikey.com/en/products/detail/sparkfun-electronics/PRT-09835/6161749?utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=PMax%20Shopping_Product_Low%20ROAS%20Categories&utm_term=&utm_content=&utm_id=go_cmp-20243063506_adg-_ad-__dev-c_ext-_prd-6161749_sig-Cj0KCQjwir2xBhC_ARIsAMTXk85yftyv_pHCKhm5V_gyFmn7Xrvi7-D7naQfL1gMGpO5F1d3YW9vyGEaAsYGEALw_wcB&gad_source=1&gclid=Cj0KCQjwir2xBhC_ARIsAMTXk85yftyv_pHCKhm5V_gyFmn7Xrvi7-D7naQfL1gMGpO5F1d3YW9vyGEaAsYGEALw_wcB)
7. [Barrel Jack Adapter](https://www.digikey.com/en/products/detail/cui-devices/PJ-102AH/408448?utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=PMax%20Supplier_Focus%20Supplier&utm_term=&utm_content=&utm_id=go_cmp-20243063242_adg-_ad-__dev-c_ext-_prd-408448_sig-Cj0KCQjwir2xBhC_ARIsAMTXk87SltCMliolDHcNn3urTq2zlLzNp1zLCkPbcD6KzQadYrdMgjCW7HcaAgr-EALw_wcB&gad_source=1&gclid=Cj0KCQjwir2xBhC_ARIsAMTXk87SltCMliolDHcNn3urTq2zlLzNp1zLCkPbcD6KzQadYrdMgjCW7HcaAgr-EALw_wcB)
8. [SparkFun Wheels](https://www.electromaker.io/shop/product/wheel-65mm-rubber-tire-pair?gad_source=1&gclid=Cj0KCQjwir2xBhC_ARIsAMTXk866VfV_39VTv8XwIU5cyQs2A4RIkEQxq-QjgyjtQOKmmzivmXojmj0aApZMEALw_wcB)
9. [Polycarbonate Frame](https://www.amazon.com/Polycarbonate-Plastic-Shatter-Resistant-Document/dp/B094F4D8CY/ref=asc_df_B094F4D8CY/?tag=hyprod-20&linkCode=df0&hvadid=647198461098&hvpos=&hvnetw=g&hvrand=4361963369218448029&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9060223&hvtargid=pla-1372682301516&psc=1&mcid=ff0c1e241f4d3c52b20e4a6d5afa5ee4) 

### Chassis Design:
Created a custom chasis design to fit our electronics and motors
![](https://github.com/anshgandhi4/self-balancing-robot/blob/master/Chasis.png)

### Component Schematic:
![](https://github.com/anshgandhi4/self-balancing-robot/blob/master/Component%20Schematic.png)
The IMU Sensor provides the mbed with accelerometer and gyroscopic measurements which is then utilized to compute appropriate values to supply to the H-Bridge. The H-birdge converts these values into appropriate values to drive the motors. The motors and mbed are each on their own power supply to prevent current spikes from affecting power to the mbed.

### Robot Design:
#### Acrylic Based Design w/ Horizontal Orientation of Breadboards
<img src="Iteration1.jpeg" alt="drawing" style="width:400px;"/></img>
#### PolyCarbonate Based Design w/ Vertical Orientation of Breadboards
<img src="FinalIteration.jpeg" alt="drawing" style="width:400px;"/></img>
### Pin Layouts:
#### H-Bridge:
<img src="H-Bridge.png" alt="drawing" style="width:200px;"/></img>
|H-Bridge Pins|mbed Pins|Motor Wires|
|:-----------:|:-------:|:---------:|
|VM|Battery Pack|-|
|VCC|VOUT|-|
|GND|GND|-|
|AO1|-| Right Motor Red|
|AO2|-|Right Motor Black|
|BO1|-|Left Motor Red|
|BO2|-|Left Motor Black|
|GND|GND|-|
|PWMA|p24|-|
|AI1|p8|-|
|AI2|p7|-|
|STBY|VOUT|-|
|PWMB|p25|-|
|BI1|p5|-|
|BI2|p6|-|
|GND|GND|-|

#### IMU:
<img src="IMU.png" alt="drawing" style="width:200px;"/></img>
|IMU Pins|mbed Pins|
|:-----------:|:-------:|
|SCL|p28|
|SDA|p27|
|VDD|VOUT|
|GND|GND|

#### Adafruit Bluetooth:
<img src="Bluetooth.png" alt="drawing" style="width:200px;"/></img>
|BLE Pins|mbed Pins|
|:-----------:|:-------:|
|TXO|p13|
|RXI|p14|
|CTS|GND|
|GND|GND|
|Vin| Battery Pack|
### Code:
The stabilization algorithm utilizes a combination of PID Steer to drive the motors with specific values and a Complementary Filter to reduce the noise from the IMU sensor. We take the first 200 cycles of data values from the IMU sensor to calibrate the bias in both acceleration and gyroscopic readings. To measure the angle the robot is tilted at, we utilize a combination of the gyroscopic readings along with acceleration data to help mitigate the gyroscope's drifting. After that we take the current angle and utilize the PID algorithm to calcualte motor speeds necessary to get the robot to the target tilt. With some fine tuning we have found that setting kP to -10.5 allows for fast enough response time to angle changes and setting kD reduces oscillation in the robots movement.
```cpp
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
```
### Iterations:
1. Our first approach involved using a gravity feed forward algorithm to assist in balancing. However, we ran into issues regarding motor power and absurd oscillatory movement.
2. Next we primarily used accleration data because the gyroscopic data kept drifting or giving absurd measurements. The issue with this approach was that sudden changes in acceleration resulted in sudden jerking of the robot which knocked it off its equilibrium.
3. Our third approach involved using a complementary filter to help remove the noise from the gyroscopic readings and utilize it to get an accurate angle reading for the robot.

### Demo:
1. Demo: https://youtu.be/zSYBFj76JRM
2. Oscillation Testing: https://youtu.be/4kjMF7BKbzk
3. Oscillation Testing Graphs: https://youtube.com/shorts/2Yzb6hkkgFo?feature=share
4. Problematic Oscillations: https://youtu.be/JfjvGOS7Too
5. Ideal Oscillations: https://youtu.be/NjUfE1U3GDE
6. Polycarbonate Cutting: https://youtu.be/hWCOUB6D1Eo

### Bloopers:
Doctor Engineers
![](https://github.com/anshgandhi4/self-balancing-robot/blob/master/Doctor%20Engineer.jpeg)

### Resources:
Past Projects:
- https://os.mbed.com/users/Samer/notebook/balancing-bot/
- https://os.mbed.com/users/fmmgramacho/notebook/self-balancing-robot-segway/
- https://os.mbed.com/users/Solomon_Martin/code/Stabilize/
- https://os.mbed.com/users/pandirimukund/code/segway-balancing/
Outside Resources:
- https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html
