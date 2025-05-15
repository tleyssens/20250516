#include "motor.h"
#include "settings.h"
#include "utils.h"

DCMotor motorLeft;
DCMotor motorRight;
DCDriver1PWM1Dir driverLeft = DCDriver1PWM1Dir(PWM1_LPWM, DIR1_RL_ENABLE);    // = DCDriver1PWM1Dir(PWM1_LPWM, DIR1_RL_ENABLE);
DCDriver1PWM1Dir driverRight = DCDriver1PWM1Dir(PWM2_RPWM, DIR1_RL_ENABLE);   // = DCDriver1PWM1Dir(PWM2_RPWM, 5);
MagneticSensorI2C sensorLeft = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensorRight= MagneticSensorI2C(AS5600_I2C);

void motorInit()
{
  if (PWM_Frequency == 0)
  {
    analogWriteFrequency(PWM1_LPWM, 490);
    analogWriteFrequency(PWM2_RPWM, 490);
  }
  else if (PWM_Frequency == 1)
  {
    analogWriteFrequency(PWM1_LPWM, 122);
    analogWriteFrequency(PWM2_RPWM, 122);
  }
  else if (PWM_Frequency == 2)
  {
    analogWriteFrequency(PWM1_LPWM, 3921);
    analogWriteFrequency(PWM2_RPWM, 3921);
  }

  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  pinMode(REMOTE_PIN, INPUT_PULLUP);
  pinMode(DIR1_RL_ENABLE, OUTPUT);
  pinMode(PWM1_LPWM, OUTPUT);
  pinMode(PWM2_RPWM, OUTPUT);
  pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
  pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);

  driverLeft.voltage_power_supply = 12.0f;
  driverLeft.voltage_limit = 12.0f;
  driverLeft.pwm_frequency = 500;
  driverRight.voltage_power_supply = 12.0f;
  driverRight.voltage_limit = 12.0f;
  driverRight.pwm_frequency = 500;

  driverLeft.init();
  driverRight.init();

  sensorLeft.init(&Wire);
  sensorRight.init(&Wire1);

  motorLeft.linkDriver(&driverLeft);
  motorRight.linkDriver(&driverRight);
  motorLeft.linkSensor(&sensorLeft);
  motorRight.linkSensor(&sensorRight);

  motorLeft.voltage_limit = 12.0f;
  motorLeft.velocity_limit = 60.0f;
  motorLeft.controller = MotionControlType::velocity;
  motorLeft.torque_controller = TorqueControlType::voltage;
  motorRight.voltage_limit = 12.0f;
  motorRight.velocity_limit = 60.0f;
  motorRight.controller = MotionControlType::velocity;
  motorRight.torque_controller = TorqueControlType::voltage;

  motorLeft.PID_velocity.P = 0.20f;
  motorLeft.PID_velocity.I = 0.001f;
  motorLeft.PID_velocity.D = 0.001f;
  motorLeft.PID_velocity.output_ramp = 200.0f;
  motorLeft.LPF_velocity.Tf = 0.01f;
  motorRight.PID_velocity.P = 0.20f;
  motorRight.PID_velocity.I = 0.001f;
  motorRight.PID_velocity.D = 0.001f;
  motorRight.PID_velocity.output_ramp = 200.0f;
  motorRight.LPF_velocity.Tf = 0.01f;

  motorLeft.init();
  motorRight.init();
  motorLeft.initFOC();
  motorRight.initFOC();

  motorLeft.enable();
  motorRight.enable();
}

void motorRun()
{
  static uint32_t lastTime = LOOP_TIME;
  if (systick_millis_count - lastTime >= LOOP_TIME)
  {
    lastTime = systick_millis_count;

    encEnable = true;
    if (watchdogTimer++ > 250)
      watchdogTimer = WATCHDOG_FORCE_VALUE;

    workSwitch = digitalRead(WORKSW_PIN);
    if (steerConfig.SteerSwitch == 1)
    {
      steerSwitch = digitalRead(STEERSW_PIN);
    }
    else if (steerConfig.SteerButton == 1)
    {
      reading = digitalRead(STEERSW_PIN);
      if (reading == LOW && previous == HIGH)
      {
        currentState = !currentState;
        steerSwitch = currentState;
      }
      previous = reading;
    }
    else
    {
      if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
      {
        steerSwitch = 0;
        previous = 1;
      }
      if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
      {
        steerSwitch = 1;
        previous = 0;
      }
    }

    if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
    {
      steerSwitch = 1;
      currentState = 1;
      previous = 0;
    }

    if (steerConfig.PressureSensor)
    {
      sensorSample = (float)analogRead(PRESSURE_SENSOR_PIN) * 0.25;
      sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
      if (sensorReading >= steerConfig.PulseCountMax)
      {
        steerSwitch = 1;
        currentState = 1;
        previous = 0;
      }
    }

    if (steerConfig.CurrentSensor)
    {
      sensorSample = (float)analogRead(CURRENT_SENSOR_PIN);
      sensorSample = (abs(775 - sensorSample)) * 0.5;
      sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
      sensorReading = min(sensorReading, 255);
      if (sensorReading >= steerConfig.PulseCountMax)
      {
        steerSwitch = 1;
        currentState = 1;
        previous = 0;
      }
    }

    remoteSwitch = digitalRead(REMOTE_PIN);
    switchByte = (remoteSwitch << 2) | (steerSwitch << 1) | workSwitch;

    steerAngleActual = steerAngleSetPoint;
    float steerAngleScaled = steerAngleSetPoint / 45.0f;
    steeringPosition = (int16_t)((steerAngleScaled * 6805) + 6805);
    helloSteerPosition = steeringPosition - 6805;

    if (steerConfig.InvertWAS)
    {
      steeringPosition = (steeringPosition - 6805 - steerSettings.wasOffset);
      steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
    }
    else
    {
      steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset);
      steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
    }

    if (steerAngleActual < 0)
    {
      steerAngleActual *= steerSettings.AckermanFix;
    }

    steerAngleError = steerAngleActual - steerAngleSetPoint;

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
      steerAngleError = steerAngleActual - steerAngleSetPoint;
      calcSteeringPID();
      float speedRadS = gpsSpeed * 10.0f;
      calculateMotorSpeeds(speedRadS, steerAngleSetPoint);
      motorLeft.loopFOC();
      motorRight.loopFOC();
      motorLeft.move();
      motorRight.move();
      digitalWrite(AUTOSTEER_ACTIVE_LED, HIGH);
      digitalWrite(AUTOSTEER_STANDBY_LED, LOW);
    }
    else
    {
      motorLeft.target = 0;
      motorRight.target = 0;
      motorLeft.loopFOC();
      motorRight.loopFOC();
      motorLeft.move();
      motorRight.move();
      pwmDrive = 0;
      pulseCount = 0;
      digitalWrite(AUTOSTEER_STANDBY_LED, HIGH);
      digitalWrite(AUTOSTEER_ACTIVE_LED, LOW);
    }
  }

  // Speed pulse
  if (gpsSpeedUpdateTimer < 1000)
  {
    if (speedPulseUpdateTimer > 200)
    {
      speedPulseUpdateTimer = 0;
      float speedPulse = gpsSpeed * 36.1111;
      if (gpsSpeed > 0.11)
      {
        tone(velocityPWM_Pin, uint16_t(speedPulse));
      }
      else
      {
        noTone(velocityPWM_Pin);
      }
    }
  }
  else
  {
    noTone(velocityPWM_Pin);
  }

  if (encEnable)
  {
    thisEnc = digitalRead(REMOTE_PIN);
    if (thisEnc != lastEnc)
    {
      lastEnc = thisEnc;
      if (lastEnc)
        EncoderFunc();
    }
  }
}

void calculateMotorSpeeds(float speed, float steerAngle)
{
  float leftSpeed, rightSpeed;
  speed = constrain(speed, 0, 60);
  steerAngle = constrain(steerAngle, -45, 45);

  float baseSpeed = speed;
  float steeringFactor = steerAngle / 45.0f;
  leftSpeed = baseSpeed * (1.0f - steeringFactor);
  rightSpeed = baseSpeed * (1.0f + steeringFactor);

  leftSpeed = constrain(leftSpeed, -60.0f, 60.0f);
  rightSpeed = constrain(rightSpeed, -60.0f, 60.0f);

  motorLeft.target = leftSpeed;
  motorRight.target = rightSpeed;
}