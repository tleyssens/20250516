#include <Arduino.h>
#include "imu.h"
#include "utils.h"
#include "gps.h"
#include "settings.h"

const uint8_t bno08xAddresses[] = {0x4A, 0x4B};
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno080;

bool useCMPS = false;
bool useBNO08x = false;
bool invertRoll = false;
float correctionHeading = 0.0f;

uint32_t READ_BNO_TIME = 0;

void imuInit() {
  if (useBNO08x) {
    // Initialiseer BNO08x (pas aan op basis van je setup)
    Wire.begin();
    if (!bno08x.begin()) {
      // Foutafhandeling
      //uint8_t error;
    }
  }
  if (useCMPS) {
    Wire.begin();
  }
  ImuWire.begin();

  ImuWire.beginTransmission(CMPS14_ADDRESS);
  error = ImuWire.endTransmission();
  if (error == 0) {
    useCMPS = true;
  } else {
    for (int16_t i = 0; i < nrBNO08xAdresses; i++) {
      bno08xAddress = bno08xAddresses[i];
      ImuWire.beginTransmission(bno08xAddress);
      error = ImuWire.endTransmission();
      if (error == 0 && bno08x.begin(bno08xAddress, ImuWire)) {
        ImuWire.setClock(400000);
        delay(300);
        bno08x.enableGameRotationVector(REPORT_INTERVAL);
        useBNO08x = true;
        break;
      }
    }
  }
}

void imuRun() {
  if ((systick_millis_count - READ_BNO_TIME) > REPORT_INTERVAL && useBNO08x) {
    READ_BNO_TIME = systick_millis_count;
    readBNO();
  }
}

void readBNO() {
  if (bno08x.dataAvailable()) {
    float dqx, dqy, dqz, dqw, dacr;
    uint8_t dac;

    bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    float t3 = +2.0 * (dqw * dqz + dqx * dqy);
    float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
    yaw = atan2(t3, t4);

    correctionHeading = -yaw;
    yaw = (int16_t)((yaw * -RAD_TO_DEG_X_10));
    if (yaw < 0) yaw += 3600;

    float t2 = +2.0 * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    if (steerConfig.IsUseY_Axis) {
      roll = asin(t2) * RAD_TO_DEG_X_10;
      pitch = atan2(+2.0 * (dqw * dqx + dqy * dqz), +1.0 - 2.0 * (dqx * dqx + ysqr)) * RAD_TO_DEG_X_10;
    } else {
      pitch = asin(t2) * RAD_TO_DEG_X_10;
      roll = atan2(+2.0 * (dqw * dqx + dqy * dqz), +1.0 - 2.0 * (dqx * dqx + ysqr)) * RAD_TO_DEG_X_10;
    }

    if (invertRoll) {
      roll *= -1;
    }
  }
}

void imuHandler() {
  int16_t temp = 0;
  if (!useDual) {
    if (useCMPS) {
      Wire.beginTransmission(CMPS14_ADDRESS);
      Wire.write(0x1C);
      Wire.endTransmission();
      Wire.requestFrom(CMPS14_ADDRESS, 3);
      while (Wire.available() < 3);
      roll = int16_t(Wire.read() << 8 | Wire.read());
      if (invertRoll) {
        roll *= -1;
      }

      Wire.beginTransmission(CMPS14_ADDRESS);
      Wire.write(0x02);
      Wire.endTransmission();
      Wire.requestFrom(CMPS14_ADDRESS, 3);
      while (Wire.available() < 3);
      temp = Wire.read() << 8 | Wire.read();
      correctionHeading = temp * 0.1;
      correctionHeading = correctionHeading * DEG_TO_RAD;
      itoa(temp, imuHeading, 10);

      int8_t pitch = Wire.read();
      itoa(pitch, imuPitch, 10);

      temp = (int16_t)roll;
      itoa(temp, imuRoll, 10);

      itoa(0, imuYawRate, 10);
    }

    if (useBNO08x) {
      temp = yaw;
      itoa(temp, imuHeading, 10);
      temp = (int16_t)pitch;
      itoa(temp, imuPitch, 10);
      temp = (int16_t)roll;
      itoa(temp, imuRoll, 10);
      itoa(0, imuYawRate, 10);
    }
  }

  if (useDual) {
    dtostrf(rollDual, 4, 2, imuRoll);
    dtostrf(heading, 4, 2, imuHeading);
  }
}