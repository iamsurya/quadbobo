#define AeroQuad_v18
#define quadPlusConfig
#define RateModeOnly 
#define LASTCHANNEL 6

#include <EEPROM.h>
#include <Wire.h>
#include <GlobalDefined.h>
#include "AeroQuad.h"
#include "PID.h"
#include <AQMath.h>
#include <FourtOrderFilter.h>

#ifdef AeroQuad_v18
  #define LED_Green 13
  #define LED_Red 12
  #define LED_Yellow 12

  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaraion
  #include <Accelerometer_BMA180.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM_Timer

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif

  // Battery Monitor declaration
  #ifdef BattMonitor
    struct BatteryData batteryData[] = {
      BM_DEFINE_BATTERY_V(BattCellCount, 0, ((5.0 / 1024.0) * (15.0 + 7.5) / 7.5), 0.9)};
  #else
    #undef BattMonitorAutoDescent
    #undef BattCellCount
    #undef POWERED_BY_VIN        
  #endif

  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder
  #undef CameraControl
  #undef OSD

  /**
   * Put AeroQuad_v18 specific intialization need here
   */
  void initPlatform() {

    pinMode(LED_Red, OUTPUT);
    digitalWrite(LED_Red, LOW);
    pinMode(LED_Yellow, OUTPUT);
    digitalWrite(LED_Yellow, LOW);

    Wire.begin();
    TWBR = 12;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureAccelSum();
    measureGyroSum();
  }

#endif

#include "Kinematics.h"
  #include "Kinematics_DCM.h"
  #include <Receiver_328p.h>
#include <Motors_PWM_Timer.h>
  #include "FlightControlQuadPlus.h"
//********************************************************
//****************** SERIAL PORT DECLARATION *************
//********************************************************
#define SERIAL_PORT Serial

// Include this last as it contains objects from above declarations
#include "AltitudeControlProcessor.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "HeadingHoldProcessor.h"
#include "DataStorage.h"
#include "SerialCom.h"



/**
 * Main setup function, called one time at bootup
 * initalize all system and sub system of the 
 * Aeroquad
 */
void setup() {
  SERIAL_BEGIN(BAUD);
  pinMode(LED_Green, OUTPUT);
  digitalWrite(LED_Green, LOW);

  #ifdef CHANGE_YAW_DIRECTION
    YAW_DIRECTION = -1;
  #endif
/* Remove ****************************************************************************************************************
  // Read user values from EEPROM
  readEEPROM(); // defined in DataStorage.h
  if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all parameters
    initializeEEPROM();
    writeEEPROM();
  }
/* Remove ****************************************************************************************************************/

  initPlatform();
/* Remove ****************************************************************************************************************
  // Configure motors
  #if defined(quadXConfig) || defined(quadPlusConfig) || defined(quadY4Config) || defined(triConfig)
  /* Remove **************************************************************************************************************** */
       initializeMotors(FOUR_Motors);
  /* Remove ****************************************************************************************************************
  #elif defined(hexPlusConfig) || defined(hexXConfig) || defined (hexY6Config)
     initializeMotors(SIX_Motors);
  #elif defined (octoX8Config) || defined (octoXConfig) || defined (octoPlusConfig)
     initializeMotors(EIGHT_Motors);
  #endif
  /* Remove **************************************************************************************************************** */
  
  
  // Initialize max/min values for all motors
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motorMinCommand[motor] = minArmedThrottle;
    motorMaxCommand[motor] = MAXCOMMAND;
  }

  // Setup receiver pins for pin change interrupts
  initializeReceiver(LASTCHANNEL);
  initReceiverFromEEPROM();

  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  initializeGyro(); // defined in Gyro.h
  initializeAccel(); // defined in Accel.h
  initSensorsZeroFromEEPROM();

  // Calibrate sensors
  calibrateGyro();
  computeAccelBias();
  zeroIntegralError();

  // Flight angle estimation
/* Remove ****************************************************************************************************************
  #ifdef HeadingMagHold
    vehicleState |= HEADINGHOLD_ENABLED;
    initializeMagnetometer();
    initializeKinematics(getHdgXY(XAXIS), getHdgXY(YAXIS));
  #else
  /* Remove **************************************************************************************************************** */
    initializeKinematics(1.0, 0.0);  // with no compass, DCM matrix initalizes to a heading of 0 degrees

//  #endif


  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = 0.375;
  PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = 0.375;
  
  // Optional Sensors
  /* Remove ****************************************************************************************************************
  #ifdef AltitudeHoldBaro
    initializeBaro();
    vehicleState |= ALTITUDEHOLD_ENABLED;
  #endif
  #ifdef AltitudeHoldRangeFinder
    inititalizeRangeFinder(ALTITUDE_RANGE_FINDER_INDEX);
    vehicleState |= RANGE_ENABLED;
  #endif
/* Remove **************************************************************************************************************** */

  // Battery Monitor
  /* Remove ****************************************************************************************************************
  #ifdef BattMonitor
    // batteryMonitorAlarmVoltage updated in readEEPROM()
    initializeBatteryMonitor(sizeof(batteryData) / sizeof(struct BatteryData), batteryMonitorAlarmVoltage);
    vehicleState |= BATTMONITOR_ENABLED;
  #endif
  
  #if defined (UseGPS)
    initializeGps();
  #endif 

  // Camera stabilization setup
  #if defined (CameraControl)
    initializeCameraStabilization();
    setmCameraRoll(318.3); // Need to figure out nice way to reverse servos
    setCenterRoll(1500); // Need to figure out nice way to set center position
    setmCameraPitch(318.3);
    setCenterPitch(1300);
    vehicleState |= CAMERASTABLE_ENABLED;
  #endif

  #if defined(MAX7456_OSD)
    initializeSPI();
    initializeOSD();
  #endif
/* Remove **************************************************************************************************************** */

  #if defined(BinaryWrite) || defined(BinaryWritePID)
    #ifdef OpenlogBinaryWrite
      binaryPort = &Serial1;
      binaryPort->begin(115200);
      delay(1000);
    #else
     binaryPort = &Serial;
    #endif
  #endif

  setupFourthOrder();

  previousTime = micros();
  digitalWrite(LED_Green, HIGH);
  safetyCheck = 0;
}

/*******************************************************************
  // tasks (microseconds of interval)
  ReadGyro        readGyro      (   5000); // 200hz
  ReadAccel       readAccel     (   5000); // 200hz
  RunDCM          runDCM        (  10000); // 100hz
  FlightControls  flightControls(  10000); // 100hz
  ReadReceiver    readReceiver  (  20000); //  50hz
  ReadBaro        readBaro      (  40000); //  25hz
  ReadCompass     readCompass   ( 100000); //  10Hz
  ProcessTelem    processTelem  ( 100000); //  10Hz
  ReadBattery     readBattery   ( 100000); //  10Hz

  Task *tasks[] = {&readGyro, &readAccel, &runDCM, &flightControls,   \
                   &readReceiver, &readBaro, &readCompass,            \
                   &processTelem, &readBattery};

  TaskScheduler sched(tasks, NUM_TASKS(tasks));

  sched.run();
*******************************************************************/
void loop () {
  currentTime = micros();
  deltaTime = currentTime - previousTime;

  measureCriticalSensors();

  // Main scheduler loop set for 100hz
  if (deltaTime >= 10000) {

    frameCounter++;

    // ================================================================
    // 100hz task loop
    // ================================================================
    if (frameCounter % TASK_100HZ == 0) {  //  100 Hz tasks
  
      G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
      hundredHZpreviousTime = currentTime;
      
      evaluateMetersPerSec();
      evaluateGyroRate();

      for (int axis = XAXIS; axis <= ZAXIS; axis++) {
        filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);
      }
      /*
      // ****************** Calculate Absolute Angle *****************
      #if defined FlightAngleNewARG
        calculateKinematics(gyroRate[XAXIS],
                            gyroRate[YAXIS],
                            gyroRate[ZAXIS],
                            filteredAccel[XAXIS],
                            filteredAccel[YAXIS],
                            filteredAccel[ZAXIS],
                            0.0,
                            0.0,
                            0.0,
                            G_Dt);

      #elif defined HeadingMagHold && defined FlightAngleMARG
        calculateKinematics(gyroRate[XAXIS],
                            gyroRate[YAXIS],
                            gyroRate[ZAXIS],
                            filteredAccel[XAXIS],
                            filteredAccel[YAXIS],
                            filteredAccel[ZAXIS],
                            getMagnetometerRawData(XAXIS),
                            getMagnetometerRawData(YAXIS),
                            getMagnetometerRawData(ZAXIS),
                            G_Dt);
      #elif defined FlightAngleARG
        calculateKinematics(gyroRate[XAXIS],
                            gyroRate[YAXIS],
                            gyroRate[ZAXIS],
                            filteredAccel[XAXIS],
                            filteredAccel[YAXIS],
                            filteredAccel[ZAXIS],
                            0.0,
                            0.0,
                            0.0,
                            G_Dt);
      #elif defined HeadingMagHold && !defined FlightAngleMARG && !defined FlightAngleARG
        calculateKinematics(gyroRate[XAXIS],
                            gyroRate[YAXIS],
                            gyroRate[ZAXIS],
                            filteredAccel[XAXIS],
                            filteredAccel[YAXIS],
                            filteredAccel[ZAXIS],
                            accelOneG,
                            getHdgXY(XAXIS),
                            getHdgXY(YAXIS),
                            G_Dt);
      #elif !defined HeadingMagHold && !defined FlightAngleMARG && !defined FlightAngleARG
        calculateKinematics(gyroRate[XAXIS],
                            gyroRate[YAXIS],
                            gyroRate[ZAXIS],
                            filteredAccel[XAXIS],
                            filteredAccel[YAXIS],
                            filteredAccel[ZAXIS],
                            accelOneG,
                            0.0,
                            0.0,
                            G_Dt);
      #endif
*/

      // Evaluate are here because we want it to be synchronized with the processFlightControl
            
      // Combines external pilot commands and measured sensor data to generate motor commands
      processFlightControl();
      
      #ifdef BinaryWrite
        if (fastTransfer == ON) {
          // write out fastTelemetry to Configurator or openLog
          fastTelemetry();
        }
      #endif


    }

    // ================================================================
    // 50hz task loop
    // ================================================================
    if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks

      G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
      fiftyHZpreviousTime = currentTime;

      // Reads external pilot commands and performs functions based on stick configuration
      readPilotCommands(); // defined in FlightCommand.pde
    }

    // ================================================================
    // 10hz task loop
    // ================================================================
    if (frameCounter % TASK_10HZ == 0) {  //   10 Hz tasks

      G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
      tenHZpreviousTime = currentTime;

      // Listen for configuration commands and reports telemetry
      readSerialCommand(); // defined in SerialCom.pde
      sendSerialTelemetry(); // defined in SerialCom.pde
   }

    previousTime = currentTime;
  }
  if (frameCounter >= 100) {
      frameCounter = 0;
  }
}



