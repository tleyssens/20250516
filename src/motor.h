#ifndef MOTOR_H
#define MOTOR_H

#define SIMPLEFOC_DISABLE_TYPEDEF // Voorkom typedef-conflicten
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <SimpleDCMotor.h>

extern DCMotor motorLeft;
extern DCMotor motorRight;
extern DCDriver1PWM1Dir driverLeft;
extern DCDriver1PWM1Dir driverRight;
extern MagneticSensorI2C sensorLeft;
extern MagneticSensorI2C sensorRight;

void motorInit();
void motorRun();
void calculateMotorSpeeds(float speed, float steerAngle);

#endif