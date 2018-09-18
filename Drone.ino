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
float kp = 55.0;
float ki = 17.0;
float kd = 230.0;
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
double XAxis = 0;
double YAxis = 0;
double RXAxis = 0;
double RYAxis = 0;
boolean RTrig = false;
boolean LTrig = false;
int POVXAxis = 0;
int POVYAxis = 0;
long lastCommandTime = 0;
double maxPID = 30;
double maxIntegral = 10;
double currentMotor0 = 0;
double currentMotor1 = 0;
double currentMotor2 = 0;
double currentMotor3 = 0;

long sendPeriod = 200;
long lastSend = 0;
long lastSentTime = 0;
long timeout = 200;
long updateFrequency = 100;
long sampleTime = 7;

boolean enabled = false;

void setup() {

  delay(1000);
  Serial.begin(57600);
  Serial.setTimeout(0);
  sendConsole("Started");
  pinMode(7, OUTPUT); //CMD
  digitalWrite(7, HIGH); //CMD
  
//  ESCBL.attach(6); //Change to PWM pins //FR
  ESCFL.attach(5);						//FL
//  ESCFR.attach(4);						//BL
  ESCBR.attach(3);						//BR
  setSpeedFR(0);
  setSpeedFL(0);
  setSpeedBL(0);
  setSpeedBR(0);
  delay(1000);
  sendConsole("Starting MPU Setup");
  mpuSetup();
  sendConsole("MPU Setup Successful");
  lastCommandTime = millis();
  lastSentTime = millis();
}

void loop() {
  mpuLoop();
  receiveCommands();
  if(millis() - lastSend > sendPeriod) {
    sendUpdates();
    lastSend = millis(); 
  }
  if(millis() - lastCommandTime > timeout) disable();
//  if(enabled) {
//    if(millis() - lastTime >= sampleTime) {
      drive();
//      lastTime = millis();
//    }
//  }
}

void send(byte toSend[], int length) {
	byte packet[length+1];
	for(int i = 0; i < length; i++)
		packet[i] = toSend[i];
	packet[length] = calculateChecksum(packet, length);
	Serial.write(packet, length+1);
	Serial.flush();
}

byte calculateChecksum(byte arr[], int length)
{
    byte sum = 0;
    for(int i = 0; i < length; i++)
      sum += arr[i];
    return sum;
}


void sendConsole(String s) 
{
	int totalLength = s.length() + 2;
	byte byt[s.length()+2];
	byt[0] = 0x7;
	byt[1] = s.length();
	for (int i = 0; i < s.length(); i++)
		byt[i + 2] = s.charAt(i);
	send(byt, totalLength);
}

byte cksum(byte buffer[], int numToCal)
{
	byte ck = 0;
	for (int i = 0; i < numToCal; i++)
		ck += buffer[i];
	return ck;
}

void sendUpdates()
{
	if(millis() < lastSentTime + updateFrequency) return;
	lastSentTime = millis();
	byte motors[] = {0x9, (byte)currentMotor0, (byte)currentMotor1, (byte)currentMotor2, (byte)currentMotor3};
	byte mpu[] = {0x8, (byte)currentYaw, (byte)currentPitch, (byte)currentRoll};
	send(motors, 5);
	send(mpu, 4);
}

/*********************************************************************************************************/
void receiveCommands() {
  if (Serial.available() == 0) return;
  byte cmd[1];
  Serial.readBytes(cmd, 1);
  byte command = cmd[0];
  if(command >= 0 && command < 6) lastCommandTime = millis();
  lastCommandTime = millis();
  if(command == 0x0) //enable / disable
  {
    byte holder[1];
    Serial.readBytes(holder, 1);
	  byte checksum[1];
	  Serial.readBytes(checksum, 1);
	  byte calculatedChecksum = holder[0] + command;
	  if (calculatedChecksum != checksum[0])
	  {
   		Serial.flush();
  		return;
  	}
//    if(holder[0] == 0x0 && enabled) disable();
//    else if(holder[0] == 0x1 && !enabled) enable();
    return;
  }
  else if(command == 0x1) //zero gyro
  {
    byte holder[2];
    readSerial(holder, 2);
	  byte checksum[1];
	  readSerial(checksum, 1);
	  byte calculatedChecksum = calculateChecksum(holder, 2) + command;
	  if (calculatedChecksum != checksum[0])
	  {
  		Serial.flush();
		  return;
	  }
    digitalWrite(4, LOW);
    calibrateMPU();
    return;
  }
  else if(command == 0x2) //set joystick values
  {
    byte holder[5];
    readSerial(holder, 5);
	  byte checksum[1];
	  readSerial(checksum, 1);
	  byte calculatedChecksum = calculateChecksum(holder, 5) + command;
	  if (calculatedChecksum != checksum[0])
	  {
  		Serial.flush();
		  return;
	  }
    if(!enabled) enable();
    YAxis = holder[0];
    RYAxis = holder[1];
    RXAxis = holder[2];
    RTrig = holder[3] == 0 ? false : true;
    LTrig = holder[4] == 0 ? false : true;
  }
  else if(command == 0x3) //set motor testing
  {
    byte holder[4];
	  readSerial(holder, 4);
	  byte checksum[1];
	  readSerial(checksum, 1);
	  byte calculatedChecksum = calculateChecksum(holder, 4) + command;
  	if (calculatedChecksum != checksum[0])
  	{
		   Serial.flush();
		  return;
	  }
    if(enabled) disable();
    XAxis = 0;
    YAxis = 0;
    RXAxis = 0;
    RYAxis = 0;
    POVXAxis = 0;
    POVYAxis = 0;
  	double speedFR = holder[0];
    double speedFL = holder[1];
    double speedBL = holder[2];
    double speedBR = holder[3];
    setSpeedFR(speedFR);
    setSpeedFL(speedFL);
    setSpeedBL(speedBL);
    setSpeedBR(speedBR);
   }
  else if(command == 0x4) //set PID constants
  {   
    byte holder[15];
    readSerial(holder, 15);
	  byte checksum[1];
	  readSerial(checksum, 1);
	  byte calculatedChecksum = calculateChecksum(holder, 15) + command;
	  if (calculatedChecksum != checksum[0])
	  {
  		sendConsole("PID");
		  Serial.flush();
		  return;
	  }
    long kp_sign = holder[0] == 0 ? 1 : -1;
    long kp_1 = ((long)holder[1]<<24);
    long kp_2 = ((long)holder[2]<<16);
    long kp_3 = ((long)holder[3]<<8);
    long kp_4 = ((long)holder[4]<<0);
    kp = kp_sign*(kp_1 + kp_2 + kp_3 + kp_4) / 100000.;
    long ki_sign = holder[5] == 0 ? 1 : -1;
    long ki_1 = ((long)holder[6]<<24);
    long ki_2 = ((long)holder[7]<<16);
    long ki_3 = ((long)holder[8]<<8);
    long ki_4 = ((long)holder[9]<<0);
    ki = ki_sign*(ki_1 + ki_2 + ki_3 + ki_4) / 100000.;
    long kd_sign = holder[10] == 0 ? 1 : -1;
    long kd_1 = ((long)holder[11]<<24);
    long kd_2 = ((long)holder[12]<<16);
    long kd_3 = ((long)holder[13]<<8);
    long kd_4 = ((long)holder[14]<<0);
    kd = kd_sign*(kd_1 + kd_2 + kd_3 + kd_4) / 100000.;
	  sendConsole(String(kp));
  }
  if(command == 0x5) //set trim
  {
    byte holder[1];
    readSerial(holder, 1);
    byte checksum[1];
    readSerial(checksum, 1);
    byte calculatedChecksum = calculateChecksum(holder, 1) + command;
    if (calculatedChecksum != checksum[0]) {
      Serial.flush();
      return;
    }
    sendConsole(String(currentPitch));
    if(holder[0] == 0) pitchOffset += .2;
    else if(holder[0] == 1) rollOffset -= .2;
    else if(holder[0] == 2) pitchOffset += .2;
    else if(holder[0] == 3) rollOffset -= .2;
  }
}

void readSerial(byte arr[], int numToRead) {
  byte temp[1];  
  for(int i = 0; i < numToRead; i++) {
    if(Serial.available() > 0) Serial.readBytes(temp, 1);
    else {
      i--;
      continue;
    }
    arr[i] = temp[0];
  }
  Serial.flush();
}
/*********************************************************************************************************/

void enable() {
  IntegralYaw = 0;
  IntegralPitch = 0;
  IntegralRoll = 0;
  lastErrorYaw = currentYaw;
  lastErrorPitch = currentPitch;
  lastErrorRoll = currentRoll;
  lastYaw = currentYaw;
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

int printLoop = 0;

void drive() {
  if(RYAxis > 100) RYAxis = map(RYAxis, 156, 250, -100, 0);
  if(RXAxis > 100) RXAxis = map(RXAxis, 156, 250, -100, 0);
  double FROutput = YAxis;
  double FLOutput = YAxis;
  double BLOutput = YAxis;
  double BROutput = YAxis;

  //pitch calculations
  expectedPitch = map(RYAxis, -100, 100, -5, 5);
  double errorPitch = currentPitch - expectedPitch;
  double changeErrorPitch = errorPitch - lastErrorPitch;
  lastErrorPitch = errorPitch;
  IntegralPitch += ki * errorPitch / 100000.
  if(IntegralPitch > maxIntegral) IntegralPitch = maxIntegral;
  if(IntegralPitch < -maxIntegral) IntegralPitch = -maxIntegral;
  
  double pitchPIDOutput = kp * errorPitch/1000. + IntegralPitch + kd * changeErrorPitch/100.;
  if(pitchPIDOutput > maxPID) pitchPIDOutput = maxPID;
  if(pitchPIDOutput < -maxPID) pitchPIDOutput = -maxPID;
  FROutput += pitchPIDOutput;
  FLOutput += pitchPIDOutput;
  BLOutput += -pitchPIDOutput;
  BROutput += -pitchPIDOutput;

  //roll calculations
  expectedRoll = map(RXAxis, -100, 100, 5, -5);
  double errorRoll = currentRoll - expectedRoll;
  IntegralRoll += ki * errorRoll / 100000.;
  double changeErrorRoll = errorRoll - lastErrorRoll;
  lastErrorRoll = errorRoll;
  if(IntegralRoll > maxIntegral) IntegralRoll = maxIntegral;
  if(IntegralRoll < -maxIntegral) IntegralRoll = -maxIntegral;
 
  double rollPIDOutput = kp * errorRoll/1000. + IntegralRoll + kd * changeErrorRoll/100.;
  if(rollPIDOutput > maxPID) rollPIDOutput = maxPID;
  if(rollPIDOutput < -maxPID) rollPIDOutput = -maxPID;
  FROutput += -rollPIDOutput;
  FLOutput += rollPIDOutput;
  BLOutput += rollPIDOutput;
  BROutput += -rollPIDOutput;

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
}

void setSpeedFR(double speed) {
  if(speed > 100) speed = 100;
  if(speed < 0) speed = 0;
  int toWrite = map(speed, 0, 100, 1000, 1832);
  currentMotor0 = speed;
  ESCFR.writeMicroseconds(toWrite);
}

void setSpeedFL(double speed) {
  if(speed < 0) speed = 0;
  if(speed > 100) speed = 100;
  int toWrite = map(speed, 0, 100, 1000, 1832);
  currentMotor1 = speed;
  ESCFL.writeMicroseconds(toWrite);
}

void setSpeedBL(double speed) {
  if(speed < 0) speed = 0;
  if(speed > 100) speed = 100;
  int toWrite = map(speed, 0, 100, 1000, 1832);
  currentMotor2 = speed;
  ESCBL.writeMicroseconds(toWrite);
}

void setSpeedBR(double speed) {
  if(speed < 0) speed = 0;
  if(speed > 100) speed = 100;
  int toWrite = map(speed, 0, 100, 1000, 1832);
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
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		sendConsole(F("DMP Initialization failed (code "));
		sendConsole("" + devStatus);
	}
}

void mpuLoop()
{
  if (!dmpReady) return;
  //while (!mpuInterrupt && fifoCount < packetSize) {}
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

        //GYRO READINGS
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("\t");
            //Serial.println(ypr[2] * 180/M_PI);
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
