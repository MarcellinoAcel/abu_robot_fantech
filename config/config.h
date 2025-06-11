// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

// base shape
#define LINO_BASE OMNI

// define motor driver
#define USE_BTS7960_MOTOR_DRIVER

// define the imu sensor
#define USE_BNO055_IMU

#define K_P 90
#define K_I 366.101694915
#define K_D 0

#define ESC_UP_KP 30
#define ESC_DOWN_KP 30
#define ESC_UP_KI 1 
#define ESC_DOWN_KI 1
#define ESC_UP_KD 0
#define ESC_DOWN_KD 0

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD1/MECANUM)
         BACK
*/

/*
ROBOT ORIENTATION OMNI
FRONT = X
LEFT  = Y
          FRONT
     MOTOR1    MOTOR4
     MOTOR2    MOTOR3
          BACK
*/

// define robot' specs here
#define MOTOR_MAX_RPS 8.4
#define MAX_RPS_RATIO 1.3
#define MOTOR_OPERATING_VOLTAGE 24
#define MOTOR_POWER_MAX_VOLTAGE 24
#define MOTOR_POWER_MEASURED_VOLTAGE 2
#define COUNTS_PER_REV1 3840
#define COUNTS_PER_REV2 3840
#define COUNTS_PER_REV3 3840
#define COUNTS_PER_REV4 3840
#define WHEEL_DIAMETER 0.0985
#define ROBOT_DIAMETER 0.46
#define ROBOT_RADIUS 0.23
#define PWM_BITS 8
#define PWM_FREQUENCY 25000

// declare encoder pin
#define MOTOR1_ENCODER_A 17
#define MOTOR1_ENCODER_B 16

#define MOTOR2_ENCODER_A 26
#define MOTOR2_ENCODER_B 27

#define MOTOR3_ENCODER_A 14
#define MOTOR3_ENCODER_B 15

#define MOTOR4_ENCODER_A 28
#define MOTOR4_ENCODER_B 29

#define LAUNCHER_UP_A 38
#define LAUNCHER_UP_B 39

#define LAUNCHER_DOWN_A 12
#define LAUNCHER_DOWN_B 11

// store encoder in array
const int enca[6] = {
    MOTOR1_ENCODER_A,
    MOTOR2_ENCODER_A,
    MOTOR3_ENCODER_A,
    MOTOR4_ENCODER_A,
    LAUNCHER_DOWN_A,
    LAUNCHER_UP_A};
const int encb[6] = {
    MOTOR1_ENCODER_B,
    MOTOR2_ENCODER_B,
    MOTOR3_ENCODER_B,
    MOTOR4_ENCODER_B,
    LAUNCHER_DOWN_B,
    LAUNCHER_UP_B};

// catcher motor
#define catcher_cw 36
#define catcher_ccw 37
#define prox_front 7
#define prox_back 32
#define prox_dribble 31

// esc motor
#define esc_up 2
#define esc_down 33

// MOTOR PINS
#define MOTOR1_IN_A 5
#define MOTOR1_IN_B 6

#define MOTOR2_IN_A 18
#define MOTOR2_IN_B 19

#define MOTOR3_IN_A 3
#define MOTOR3_IN_B 4

#define MOTOR4_IN_A 22
#define MOTOR4_IN_B 10

const int cw[6] = {
    MOTOR1_IN_A,
    MOTOR2_IN_A,
    MOTOR3_IN_A,
    MOTOR4_IN_A};
const int ccw[6] = {
    MOTOR1_IN_B,
    MOTOR2_IN_B,
    MOTOR3_IN_B,
    MOTOR4_IN_B};

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

struct but
{
    int A;
    int B;
    int X;
    int Y;
    int RT;
    int LT;
    int LB;
    int RB;
    int select;
    int start;
    int home;
    int up;
    int down;
    int left;
    int right;
} button;

struct joy
{
    double axis0_x;
    double axis0_y;
    double axis1_x;
    double axis1_y;
    int but_red;
    int but_blue;
    int but_black;
    int but_green;
} joystick;

int solenoid = 9;

int cylinder_upper = 21;
int cylinder_side = 20;

#endif
