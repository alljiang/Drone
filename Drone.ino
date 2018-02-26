/*
 * Resources:
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */
#include <Servo.h>
//=========================================MPU STUFF======================================================================================================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//========================================================================================================================================================

Servo ESCFR, ESCFL, ESCBL, ESCBR;

float yawkp = 0;
float yawki = 0;
float yawkd = 0;
float pitchkp = 7.0;
float pitchki = 5.5;
float pitchkd = 90;
float rollkp = pitchkp;
float rollki = pitchki;
float rollkd = pitchkd;
long lastTime = 0;
double IntegralYaw = 0;
double IntegralPitch = 0;
double IntegralRoll = 0;
double expectedYaw = 0;
double expectedPitch = 0;
double expectedRoll = 0;
double currentYaw = 0;
double currentPitch = 0;
double currentRoll = 0;
double yawOffset = 0;
double pitchOffset = 0;
double rollOffset = 0;
double lastErrorYaw = 0;
double lastErrorPitch = 0;
double lastErrorRoll = 0;
double lastYaw = 0;
double lastPitch = 0;
double lastRoll = 0;
double XAxis = 0;
double YAxis = 0;
double RXAxis = 0;
double RYAxis = 0;
int POVXAxis = 0;
int POVYAxis = 0;
long lastCommandTime = 0;
double maxPitchPID = 15;
double currentMotor0 = 0;
double currentMotor1 = 0;
double currentMotor2 = 0;
double currentMotor3 = 0;

long lastSentTime = 0;
long timeout = 200;
long updateFrequency = 200;

boolean enabled = false;

void setup() {

  delay(1000);
  Serial.begin(115200);
  Serial.setTimeout(0);
  sendConsole("Started");
  pinMode(7, OUTPUT); //CMD
  digitalWrite(7, HIGH); //CMD
  
  ESCFR.attach(6); //Change to PWM pins
//  ESCFL.attach(5);
  ESCBL.attach(4);
//  ESCBR.attach(3);
  setSpeedFR(0);
  setSpeedFL(0);
  setSpeedBL(0);
  setSpeedBR(0);
  delay(1000);
  lastCommandTime = millis();
  mpuSetup();
}

void loop() {
  mpuLoop();
  receiveCommands();
  sendUpdates();
  if(millis() - lastCommandTime > timeout) disable();
  if(enabled) drive();
  delay(.5);
}

void send(String s) {
  Serial.println(s);
  Serial.flush();
}

void sendConsole(String s) {
  String header = "7:";
  String toSend = header + s;
  send(toSend);
}

void sendUpdates()
{
  if(millis() < lastSentTime + updateFrequency) return;
  lastSentTime = millis();
  String motorheader = "9:";
  String currentmotorvalues = motorheader + currentMotor0 + ":" + currentMotor1 + ":" + currentMotor2 + ":" + currentMotor3;
  send(currentmotorvalues);
  String mpuheader = "8:";
  String currentmpureadings = mpuheader + currentYaw + ":" + currentPitch + ":" + currentRoll;
  send(currentmpureadings);
}

//---------------------------------------------------------------------------------------
void receiveCommands() {
  String received = "";
  received = Serial.readString();
  if(received.length() > 0) lastCommandTime = millis();
  Serial.print(received);
  if(received.indexOf("ZERO") > -1)
  {
    digitalWrite(4, LOW);
    calibrateMPU();
    return;
  }
  int index = received.indexOf(":");
  String command = received.substring(0, index);
  received = received.substring(index+1);
  if(index == -1)
  {
    if(command.equals("0")) enable();
    else if(command.equals("1")) disable();
  }
  else
  {
    if(command.equals("2"))
    {
      enabled = false;
      XAxis = 0;
      YAxis = 0;
      RXAxis = 0;
      RYAxis = 0;
      POVXAxis = 0;
      POVYAxis = 0;
      index = received.indexOf(":");
      double speedFR = received.substring(0, index).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      double speedFL = received.substring(0, index).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      double speedBL = received.substring(0, index).toFloat();
      received = received.substring(index+1);
      double speedBR = received.substring(0).toFloat();
      setSpeedFR(speedFR);
      setSpeedFL(speedFL);
      setSpeedBL(speedBL);
      setSpeedBR(speedBR);
    }
    if(command.equals("3"))
    {
//      return;
      index = received.indexOf(":");
      yawkp = (received.substring(0, index)).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      yawki = (received.substring(0, index)).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      yawkd = (received.substring(0, index)).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      pitchkp = (received.substring(0, index)).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      pitchki = (received.substring(0, index)).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      pitchkd = (received.substring(0, index)).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      rollkp = (received.substring(0, index)).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      rollki = (received.substring(0, index)).toFloat();
      received = received.substring(index+1);
      rollkd = (received.substring(0)).toFloat();
      String toReturn = "";
      String space = " ";
      toReturn = yawkp+space+yawki+space+yawkd+space+pitchkp+space+pitchki+space+pitchkd+space+rollkp+space+rollki+space+rollkd;
      sendConsole(toReturn);
    }
    if(command.equals("4"))
    {
      enabled = true;
      index = received.indexOf(":");
      YAxis = received.substring(0, index).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      RXAxis = received.substring(0, index).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      RYAxis = received.substring(0, index).toFloat();
      received = received.substring(index+1);
      index = received.indexOf(":");
      POVXAxis = received.substring(0, index).toInt();
      received = received.substring(index+1);
      POVYAxis = received.substring(0, index).toInt();
    }
  }
}
//---------------------------------------------------------------------------------------

void enable() {

  IntegralYaw = 0;
  IntegralPitch = 0;
  IntegralRoll = 0;
  lastErrorYaw = currentYaw;
  lastErrorPitch = currentPitch;
  lastErrorRoll = currentRoll;
  lastTime = millis();
  enabled = true;
  setSpeedFR(0);  
  setSpeedFL(0);
  setSpeedBL(0);
  setSpeedBR(0);
}

void disable() {
  
  enabled = false;  
  setSpeedFR(0);  
  setSpeedFL(0);
  setSpeedBL(0);
  setSpeedBR(0);
}

void updateGyroValues() {
  yawOffset += currentYaw;  
  pitchOffset += currentPitch;
  rollOffset += currentRoll;
}

void drive() {

  expectedPitch = map(RYAxis, -100, 100, 7, -7);
  expectedRoll = map(RXAxis, -100, 100, -7, 7);
  double changeInTime = millis() - lastTime;
  double errorYaw = currentYaw - expectedYaw;
  double errorPitch = currentPitch - expectedPitch;
  double errorRoll = currentRoll - expectedRoll;
  boolean flipYaw = errorYaw < 0;
  boolean flipPitch = errorPitch < 0;
  boolean flipRoll = errorRoll < 0;
  errorYaw = abs(errorYaw);
  errorPitch = abs(errorPitch);
  errorRoll = abs(errorRoll);
  double changeErrorYaw = errorYaw - lastErrorYaw;
  double changeErrorPitch = errorPitch - lastErrorPitch;
  double changeErrorRoll = errorRoll - lastErrorRoll;
  double changeYaw = currentYaw - lastYaw;
  double changePitch = currentPitch - lastPitch;
  double changeRoll = currentRoll - lastRoll;
  changeErrorPitch = -changeErrorPitch;
  changeErrorRoll = -changeErrorRoll;
  changePitch = -changePitch;
  changeRoll = -changeRoll;

  if(errorYaw < .3) IntegralYaw = 0;
  if(YAxis == 0 || errorPitch < .3) IntegralPitch = 0;
  if(YAxis == 0 || errorRoll < .3) IntegralRoll = 0;
  
//  double yawPIDOutput = yawkp * errorYaw + yawki * IntegralYaw - yawkd * changeErrorYaw / changeInTime;
//  double pitchPIDOutput = pitchkp * errorPitch + pitchki * IntegralPitch - pitchkd * changeErrorPitch / changeInTime;
//  double rollPIDOutput = rollkp * errorRoll + rollki * IntegralRoll - rollkd * changeErrorRoll / changeInTime;
  double yawPIDOutput = yawkp * errorYaw / 100. + yawki * IntegralYaw  / 1000.- yawkd * changeYaw / changeInTime;
  double pitchPIDOutput = pitchkp * errorPitch / 100. + pitchki * IntegralPitch / 1000. - pitchkd * changePitch / changeInTime;
  double rollPIDOutput = rollkp * errorRoll / 100. + rollki * IntegralRoll / 1000. - rollkd * changeRoll / changeInTime;
 
  if(pitchPIDOutput > maxPitchPID) pitchPIDOutput = maxPitchPID;
  if(rollPIDOutput > maxPitchPID) rollPIDOutput = maxPitchPID;
  if(pitchPIDOutput < -maxPitchPID) pitchPIDOutput = -maxPitchPID;
  if(rollPIDOutput < -maxPitchPID) rollPIDOutput = -maxPitchPID;
  
  lastTime = millis();
  lastErrorYaw = errorYaw;
  lastErrorPitch = errorPitch;
  lastErrorRoll = errorRoll;
  lastYaw = currentYaw;
  lastPitch = currentPitch;
  lastRoll = currentRoll;
  IntegralYaw += errorYaw;
  IntegralPitch += errorPitch;
  IntegralRoll += errorRoll;

  double FROutput = YAxis;
  double FLOutput = YAxis;
  double BLOutput = YAxis;
  double BROutput = YAxis;

  //FLIP THESE VALUES IF YOU NEED TO BASED ON THE GYRO
  if(flipYaw) {
    FROutput += -yawPIDOutput;
    FLOutput += yawPIDOutput;
    BLOutput += -yawPIDOutput;
    BROutput += yawPIDOutput;
  }
  else {
    FROutput += yawPIDOutput;
    FLOutput += -yawPIDOutput;
    BLOutput += yawPIDOutput;
    BROutput += -yawPIDOutput;
  }
  if(!flipPitch) {
    FROutput += -pitchPIDOutput;
    FLOutput += -pitchPIDOutput;
    BLOutput += pitchPIDOutput;
    BROutput += pitchPIDOutput;
  }
  else {
    FROutput += pitchPIDOutput;
    FLOutput += pitchPIDOutput;
    BLOutput += -pitchPIDOutput;
    BROutput += -pitchPIDOutput;
  }
  if(flipRoll) {
    FROutput += -rollPIDOutput;
    FLOutput += rollPIDOutput;
    BLOutput += rollPIDOutput;
    BROutput += -rollPIDOutput;
  }
  else {
    FROutput += rollPIDOutput;
    FLOutput += -rollPIDOutput;
    BLOutput += -rollPIDOutput;
    BROutput += rollPIDOutput;
  }
  setSpeedFR(FROutput);
  setSpeedFL(FLOutput);
  setSpeedBL(BLOutput);
  setSpeedBR(BROutput);
}

void calibrateMPU()
{
  yawOffset += currentYaw;
  pitchOffset += currentPitch;
  rollOffset += currentRoll;
  currentYaw = 0;
  currentPitch = 0;
  currentRoll = 0;
  IntegralYaw = 0;
  IntegralPitch = 0;
  IntegralRoll = 0;
  lastErrorYaw = 0;
  lastErrorPitch = 0;
  lastErrorRoll = 0;
  lastYaw = 0;
  lastPitch = 0;
  lastRoll = 0;
}

void setSpeedFR(double speed) {
  if(speed > 0) digitalWrite(4, LOW);
  if(speed > 100) speed = 100;
  if(speed < 0) speed = 0;
  int toWrite = map(speed, 0, 100, 1135, 1832);
  currentMotor0 = speed;
  ESCFR.writeMicroseconds(toWrite);
}

void setSpeedFL(double speed) {
  if(speed < 0) speed = 0;
  if(speed > 100) speed = 100;
  int toWrite = map(speed, 0, 100, 1135, 1832);
  currentMotor1 = speed;
  ESCFL.writeMicroseconds(toWrite);
}

void setSpeedBL(double speed) {
  if(speed < 0) speed = 0;
  if(speed > 100) speed = 100;
  int toWrite = map(speed, 0, 100, 1135, 1832);
  currentMotor2 = speed;
  ESCBL.writeMicroseconds(toWrite);
}

void setSpeedBR(double speed) {
  if(speed < 0) speed = 0;
  if(speed > 100) speed = 100;
  int toWrite = map(speed, 0, 100, 1135, 1832);
  currentMotor3 = speed;
  ESCBR.writeMicroseconds(toWrite);
}


//========================================================MPU 6050 GYRO AND ACCELEROMETER=================================================================
//========================================================================================================================================================
//========================================================================================================================================================
//========================================================================================================================================================
//========================================================================================================================================================

void mpuSetup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    sendConsole(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    sendConsole(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-4271);
    mpu.setYAccelOffset(-1550);
    mpu.setZAccelOffset(1912);
    mpu.setXGyroOffset(-175);
    mpu.setYGyroOffset(-64);
    mpu.setZGyroOffset(-83);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        sendConsole(F("DMP Initialization failed (code "));
        sendConsole(""+devStatus);
    }
}

void mpuLoop()
{
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {}
  mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
//        sendConsole(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

//        #ifdef OUTPUT_READABLE_QUATERNION
//            // display quaternion values in easy matrix form: w x y z
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            Serial.print("quat\t");
//            Serial.print(q.w);
//            Serial.print("\t");
//            Serial.print(q.x);
//            Serial.print("\t");
//            Serial.print(q.y);
//            Serial.print("\t");
//            Serial.println(q.z);
//        #endif
//
//        #ifdef OUTPUT_READABLE_EULER
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
//        #endif

        //GYRO READINGS
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
            currentYaw = ypr[0] * 180/M_PI - yawOffset;
            currentPitch = ypr[1] * 180/M_PI - pitchOffset;
            currentRoll = ypr[2] * 180/M_PI - rollOffset;
            
//        #ifdef OUTPUT_READABLE_REALACCEL
//            // display real acceleration, adjusted to remove gravity
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);
//        #endif
//
//        #ifdef OUTPUT_READABLE_WORLDACCEL
//            // display initial world-frame acceleration, adjusted to remove gravity
//            // and rotated based on known orientation from quaternion
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
//        #endif
    }
}


