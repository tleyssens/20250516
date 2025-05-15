#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <NativeEthernet.h>

#define GGAReceivedLED 13
#define Power_on_LED 5
#define Ethernet_Active_LED 6
#define GPSRED_LED 9
#define GPSGREEN_LED 10
#define AUTOSTEER_STANDBY_LED 11
#define AUTOSTEER_ACTIVE_LED 12
#define velocityPWM_Pin 36
#define REPORT_INTERVAL 20
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105
#define PWM_Frequency 0

#define PWM1_LPWM 2
#define PWM2_RPWM 3
#define DIR1_RL_ENABLE 4
#define STEERSW_PIN 32
#define WORKSW_PIN 34
#define REMOTE_PIN 37
#define CURRENT_SENSOR_PIN A17
#define PRESSURE_SENSOR_PIN A10

extern const uint16_t LOOP_TIME; // Globale constante voor 40 Hz (25 ms)
extern const uint16_t WATCHDOG_THRESHOLD;
extern const uint16_t WATCHDOG_FORCE_VALUE;

extern bool blink;
extern bool useDual;
extern float roll, pitch, yaw;
extern float heading;
extern float baseline;
extern float relPosD;
extern float rollDual;
extern float headingcorr;
extern uint8_t helloFromIMU[];
extern uint8_t helloFromAutoSteer[];
extern int16_t helloSteerPosition;
extern uint8_t PGN_253[];
extern int8_t PGN_253_Size;
extern uint8_t PGN_250[];
extern int8_t PGN_250_Size;
extern uint8_t aog2Count;
extern float sensorReading, sensorSample;
extern elapsedMillis gpsSpeedUpdateTimer;
extern elapsedMillis speedPulseUpdateTimer;

extern bool isRelayActiveHigh;
extern uint8_t relay, relayHi, uTurn, tram;
extern uint8_t remoteSwitch, workSwitch, steerSwitch, switchByte;
extern uint8_t guidanceStatus, prevGuidanceStatus;
extern bool guidanceStatusChanged;
extern float gpsSpeed;
extern float steerAngleActual, steerAngleSetPoint;
extern int16_t steeringPosition;
extern float steerAngleError;
extern int16_t pwmDrive, pwmDisplay;
extern float pValue, errorAbs, highLowPerDeg;

extern uint8_t currentState, reading, previous;
extern uint8_t pulseCount;
extern bool encEnable;
extern uint8_t thisEnc, lastEnc;

extern uint32_t autsteerLastTime;
extern uint8_t watchdogTimer;

extern IPAddress Eth_ipDestination;
extern bool useBNO08x;
extern bool useCMPS;
extern bool invertRoll;
extern const int32_t baudAOG = 115200;
extern const int32_t baudGPS = 460800;
extern const int32_t baudRTK = 9600;     // most are using Xbee radios with default of 115200
//extern SerialAOG;
extern HardwareSerial* SerialGPS;
extern HardwareSerial* SerialGPS2;

bool Ethernet_running = false;
byte Eth_myip[4] = {192, 168, 1, 121}; //IP-adres van Teensy
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};
unsigned int portMy = 5120;
unsigned int AOGNtripPort = 2233;
unsigned int AOGAutoSteerPort = 8888;
unsigned int portDestination = 9999;
extern uint8_t error;
void errorHandler();
bool calcChecksum();
void calcSteeringPID();
void EncoderFunc();

#endif