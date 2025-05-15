#include "utils.h"

const uint16_t LOOP_TIME = 25; // 40 Hz
const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = 102;
//Serial ports
#define SerialAOG Serial                //AgIO USB connection
HardwareSerial* SerialGPS = &Serial7;   //Main postion receiver (GGA) (Serial2 must be used here with T4.0 / Basic Panda boards - Should auto swap)
HardwareSerial* SerialGPS2 = &Serial2;  //Dual heading receiver 

bool blink = false;
bool useBNO08x = false;
bool useCMPS = false;
bool invertRoll = false;
bool useDual = false;

float roll = 0, pitch = 0, yaw = 0;
uint8_t helloFromIMU[10] = {0x80, 0x81, 0x7E, 0xC5, 0x08, 0, 0, 0, 0, 0};
uint8_t helloFromAutoSteer[10] = {0x80, 0x81, 0x7D, 0xC6, 0x08, 0, 0, 0, 0, 0};
int16_t helloSteerPosition = 0;
uint8_t PGN_253[] = {0x80, 0x81, 0x7F, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t PGN_253_Size = 8;
uint8_t PGN_250[] = {0x80, 0x81, 0x7F, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t PGN_250_Size = 8;
uint8_t aog2Count = 0;
uint8_t error = 0;
float sensorReading = 0, sensorSample = 0;
elapsedMillis gpsSpeedUpdateTimer;
elapsedMillis speedPulseUpdateTimer;

bool isRelayActiveHigh = false;
uint8_t relay = 0, relayHi = 0, uTurn = 0, tram = 0;
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 0, switchByte = 0;
uint8_t guidanceStatus = 0, prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;
float gpsSpeed = 0;
float steerAngleActual = 0, steerAngleSetPoint = 0;
int16_t steeringPosition = 0;
float steerAngleError = 0;
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0, errorAbs = 0, highLowPerDeg = 0;

uint8_t currentState = 0, reading = 0, previous = 0;
uint8_t pulseCount = 0;
bool encEnable = false;
uint8_t thisEnc = 0, lastEnc = 0;

uint32_t autsteerLastTime = LOOP_TIME;
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

IPAddress Eth_ipDestination(192, 168, 1, 255);

float heading = 0.0f;
float baseline = 0.0f;
float relPosD = 0.0f;
float rollDual = 0.0f;
float headingcorr = 0.0f;

void errorHandler() {}
bool calcChecksum() { return true; }
void calcSteeringPID() {}
void EncoderFunc() {}