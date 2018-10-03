/*
 * Resources:
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */
#include <Servo.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>
//=========================================MPU STUFF======================================================================================================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
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

SoftwareSerial hc12(8,9); //rx, tx
Servo ESCFR, ESCFL, ESCBL, ESCBR;

const int SetPin = 7;
float yawkp = 0;
float yawki = 0;
float yawkd = 0;
float kp = 0.0;
float ki = 0.0;
float kd = 0.0;
long lastTime = 0;
double IntegralYaw = 0;
double IntegralPitch = 0;
double IntegralRoll = 0;
double IntegralYawRate = 0;
double IntegralPitchRate = 0;
double IntegralRollRate = 0;
double expectedYaw = 0;
double expectedPitch = 0;
double expectedRoll = 0;
double currentYaw = 0;
double currentPitch = 0;
double currentRoll = 0;
double currentYawAngle = 0;
double currentPitchAngle = 0;
double currentRollAngle = 0;
double currentYawRate = 0;
double currentPitchRate = 0;
double currentRollRate = 0;
double yawOffset = 0;
double pitchOffset = 0;
double rollOffset = 0;
double yawTrim = 0;
double pitchTrim = 0;
double rollTrim = 0;
double lastErrorYaw = 0;
double lastErrorPitch = 0;
double lastErrorRoll = 0;
double lastPitchRate = 0;
double lastRollRate = 0;
double lastErrorYawRate = 0;
double lastErrorPitchRate = 0;
double lastErrorRollRate = 0;
double lastYaw = 0;
double lastPitch = 0;
double lastRoll = 0;
double lastYawRate = 0;
double voltage = 0;
double XAxis = 0;
double YAxis = 0;
double RXAxis = 0;
double RYAxis = 0;
boolean RTrig = false;
boolean LTrig = false;
int POVXAxis = 0;
int POVYAxis = 0;
long lastCommandTime = 0;
double maxPID = 23;
double maxIntegral = 10;
double maxIntegralRate = 10;
double currentMotor0 = 0;
double currentMotor1 = 0;
double currentMotor2 = 0;
double currentMotor3 = 0;
double pitch_level_adjust = 0;
double roll_level_adjust = 0;

long sendPeriod = 200;
long lastSend = 0;
long lastSentTime = 0;
long timeout = 200;
long updateFrequency = 100;
long receiveTimeout = 100;
long sampleTime = 2;

boolean enabled = false;


void setup() {
  wdt_enable(WDTO_8S);
  ESCFR.attach(6); //FR
  ESCFL.attach(5); //FL
  ESCBL.attach(4); //BL
  ESCBR.attach(3); //BR
  pinMode(SetPin, OUTPUT);
  digitalWrite(SetPin, LOW);
//  hc12.begin(9600);
  Serial.begin(57600);
  Serial.setTimeout(0);
  delay(1000);
  wdt_reset();
//  configureHC12();
  sendConsole("Started");
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
  wdt_enable(WDTO_1S);
}

void loop() {
  if(!enabled) {
    setSpeedFR(0);
    setSpeedFL(0);
    setSpeedBL(0);
    setSpeedBR(0);
  }
  receiveCommands();
  if(millis() - lastSend > sendPeriod) {
    sendUpdates();
    lastSend = millis(); 
  }
  if(millis() - lastCommandTime > timeout) disable();
  mpuLoop();
  if(enabled) {
    while (millis() - lastTime <= sampleTime) {
      delayMicroseconds(1);
    }
      drive();
      lastTime = millis();
  }
}

void configureHC12() {
  digitalWrite(SetPin, LOW);
  delay(10);
  hc12.print("AT+B38400"); //set baud to 38400
  delay(50);
  Serial.println(hc12.readString());
  hc12.print("AT+FU3"); //set transmission mode
  delay(50);
  Serial.println(hc12.readString());
  hc12.print("AT+P8"); //set transmission power to max
  delay(50);
  Serial.println(hc12.readString());
  hc12.print("AT+RX"); //get all values
  delay(50);
  Serial.println(hc12.readString());
  digitalWrite(SetPin, HIGH);
  hc12.begin(38400);
}

void send(byte toSend[], int length) {
	byte packet[length+1];
	for(int i = 0; i < length; i++)
		packet[i] = toSend[i];
	packet[length] = calculateChecksum(packet, length);
	Serial.write(packet, length+1);
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
  voltage = getBatteryVoltage();
  byte battery[] = {0xA, (byte)(voltage*10)};
	byte motors[] = {0x9, (byte)currentMotor0, (byte)currentMotor1, (byte)currentMotor2, (byte)currentMotor3};
	byte mpu[] = {0x8, (byte)currentYaw, (byte)currentPitch, (byte)currentRoll};
  send(battery, 2);
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
    byte holder[30];
    readSerial(holder, 30);
	  byte checksum[1];
	  readSerial(checksum, 1);
	  byte calculatedChecksum = calculateChecksum(holder, 30) + command;
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
    long yawkp_sign = holder[15] == 0 ? 1 : -1;
    long yawkp_1 = ((long)holder[16]<<24);
    long yawkp_2 = ((long)holder[17]<<16);
    long yawkp_3 = ((long)holder[18]<<8);
    long yawkp_4 = ((long)holder[19]<<0);
    yawkp = yawkp_sign*(yawkp_1 + yawkp_2 + yawkp_3 + yawkp_4) / 100000.;
    long yawki_sign = holder[20] == 0 ? 1 : -1;
    long yawki_1 = ((long)holder[21]<<24);
    long yawki_2 = ((long)holder[22]<<16);
    long yawki_3 = ((long)holder[23]<<8);
    long yawki_4 = ((long)holder[24]<<0);
    yawki = yawki_sign*(yawki_1 + yawki_2 + yawki_3 + yawki_4) / 100000.;
    long yawkd_sign = holder[25] == 0 ? 1 : -1;
    long yawkd_1 = ((long)holder[26]<<24);
    long yawkd_2 = ((long)holder[27]<<16);
    long yawkd_3 = ((long)holder[28]<<8);
    long yawkd_4 = ((long)holder[29]<<0);
    yawkd = yawkd_sign*(yawkd_1 + yawkd_2 + yawkd_3 + yawkd_4) / 100000.;
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
    if(holder[0] == 0) pitchTrim -= .5;
    else if(holder[0] == 1) rollTrim += .5;
    else if(holder[0] == 2) pitchTrim += .5;
    else if(holder[0] == 3) rollTrim -= .5;
  }
  if(command == 0x6) //set trim
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
    if(holder[0] == 0) yawTrim -= 1;
    else if(holder[0] == 1) yawTrim += 1;
  }
}

void readSerial(byte arr[], int numToRead) {
  byte temp[1];  
  long lastReceived = millis();
  for(int i = 0; i < numToRead; i++) {
    if(Serial.available() > 0) {
      Serial.readBytes(temp, 1);
      lastReceived = millis();
    }
    else {
      i--;
      if(millis() - lastReceived >= receiveTimeout) break;
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
  
  if(currentPitch != lastPitch && currentRoll != lastRoll) {
    if(RYAxis > 100) RYAxis = map(RYAxis, 156, 250, -100, 0);
    if(RXAxis > 100) RXAxis = map(RXAxis, 156, 250, -100, 0);

//    pitch_level_adjust = currentPitchAngle * 1.5;                                    //Calculate the pitch angle correction
//    roll_level_adjust = currentRollAngle * 1.5;                                      //Calculate the roll angle correction

    double expectedPitchRate = map(RYAxis, -100, 100, -27, 27) - pitch_level_adjust;
    double expectedRollRate = map(RXAxis, -100, 100, 27, -27) - roll_level_adjust;
    
    float FROutput = YAxis;
    float FLOutput = YAxis;
    float BLOutput = YAxis;
    float BROutput = YAxis;

    //pitch calculations
    double errorPitchRate = currentPitchRate - expectedPitchRate;
    double changePitchRate = currentPitchRate - lastPitchRate;
    IntegralPitchRate += ki * errorPitchRate / 100000.;
    if(IntegralPitchRate > maxIntegralRate) IntegralPitchRate = maxIntegralRate;
    if(IntegralPitchRate < -maxIntegralRate) IntegralPitchRate = -maxIntegralRate;
    lastPitchRate = currentPitchRate;
    float pitchPIDOutputRate = kp * errorPitchRate/1000. + IntegralPitchRate + kd * changePitchRate/100.;
    
    if(pitchPIDOutputRate > maxPID) pitchPIDOutputRate = maxPID;
    if(pitchPIDOutputRate < -maxPID) pitchPIDOutputRate = -maxPID;

    //roll calculations
    double errorRollRate = currentRollRate - expectedRollRate;
    double changeRollRate = currentRollRate - lastRollRate;
    IntegralRollRate += ki * errorRollRate / 100000.;
    if(IntegralRollRate > maxIntegralRate) IntegralRollRate = maxIntegralRate;
    if(IntegralRollRate < -maxIntegralRate) IntegralRollRate = -maxIntegralRate;
   
    float rollPIDOutputRate = kp * errorRollRate/1000. + IntegralRollRate + kd * changeRollRate/100.;
    lastRollRate = currentRollRate;
    if(rollPIDOutputRate > maxPID) rollPIDOutputRate = maxPID;
    if(rollPIDOutputRate < -maxPID) rollPIDOutputRate = -maxPID;

    //yaw calculations
    double expectedYawRate = 0;
    double errorYawRate = currentYawRate - expectedYawRate;
    double changeErrorYawRate = errorYawRate - lastErrorYawRate;
    IntegralYawRate += yawki * errorYawRate / 100000.;
    if(IntegralYawRate > maxIntegralRate) IntegralYawRate = maxIntegralRate;
    if(IntegralYawRate < -maxIntegralRate) IntegralYawRate = -maxIntegralRate;
   
    float yawPIDOutputRate = yawkp * errorYawRate/100. + IntegralYawRate/100. + yawkd * changeErrorYawRate/100.;
    lastErrorYawRate = errorYawRate;
    if(yawPIDOutputRate > maxPID) yawPIDOutputRate = maxPID;
    if(yawPIDOutputRate < -maxPID) yawPIDOutputRate = -maxPID;
    
    FROutput += pitchPIDOutputRate - rollPIDOutputRate - yawPIDOutputRate + yawTrim + pitchTrim - rollTrim;
    FLOutput += pitchPIDOutputRate + rollPIDOutputRate + yawPIDOutputRate - yawTrim + pitchTrim + rollTrim;
    BLOutput += -pitchPIDOutputRate + rollPIDOutputRate - yawPIDOutputRate + yawTrim - pitchTrim + rollTrim;
    BROutput += -pitchPIDOutputRate - rollPIDOutputRate + yawPIDOutputRate - yawTrim - pitchTrim - rollTrim;
    //compensate for voltage
    FROutput += FROutput*(12.6-voltage)/34.5;
    FLOutput += FLOutput*(12.6-voltage)/34.5;
    BLOutput += BLOutput*(12.6-voltage)/34.5;
    BROutput += BROutput*(12.6-voltage)/34.5;
  
    setSpeedFR(FROutput);
    setSpeedFL(FLOutput);
    setSpeedBL(BLOutput);
    setSpeedBR(BROutput);
  }
}

void slowSendConsole(String str, int skips) {
  if(printLoop++ >= skips) {
    sendConsole(str);
    printLoop = 0;
  }
}

void calibrateMPU()
{
  yawOffset += currentYaw;
  pitchOffset += currentPitch;
  rollOffset += currentRoll;
  yawTrim = 0;
  pitchTrim = 0;
  rollTrim = 0;
  currentYaw = 0;
  currentPitch = 0;
  currentRoll = 0;
  IntegralYaw = 0;
  IntegralPitch = 0;
  IntegralRoll = 0;
  lastErrorYaw = 0;
  lastErrorPitch = 0;
  lastErrorRoll = 0;
  IntegralYawRate = 0;
  IntegralPitchRate = 0;
  IntegralRollRate = 0;
  lastErrorYawRate = 0;
  lastErrorPitchRate = 0;
  lastErrorRollRate = 0;
  lastYaw = 0;
}

void setSpeedFR(double speed) {
  if(speed < 0) speed = 0;
  if(speed > 100) speed = 100;
  int toWrite = map(speed, 0, 100, 995, 1832);
  currentMotor0 = speed;
  ESCFR.writeMicroseconds(toWrite);
  wdt_reset();
}

void setSpeedFL(double speed) {
  if(speed < 0) speed = 0;
  if(speed > 100) speed = 100;
  int toWrite = map(speed, 0, 100, 995, 1832);
  currentMotor1 = speed;
  ESCFL.writeMicroseconds(toWrite);
}

void setSpeedBL(double speed) {
  if(speed < 0) speed = 0;
  if(speed > 100) speed = 100;
  int toWrite = map(speed, 0, 100, 995, 1832);
  currentMotor2 = speed;
  ESCBL.writeMicroseconds(toWrite);
}

void setSpeedBR(double speed) {
  if(speed < 0) speed = 0;
  if(speed > 100) speed = 100;
  int toWrite = map(speed, 0, 100, 995, 1832);
  currentMotor3 = speed;
  ESCBR.writeMicroseconds(toWrite);
}

float getBatteryVoltage() {
  analogReference(INTERNAL); //reference from 1.1V
  double ADCVal = analogRead(A7);
  float val = (ADCVal / 1024.) * 1.1 / (737./(9700.+737.)); //R1 = 9.7k  R2 = 737
  double a = -0.0033;
  double b = 1.0785;
  double c = -0.305;
  float output = ((a*val*val + b*val + c*2)*10)/10;
  if(output < 5.5) output = 0; //probably just voltage from the usb input
  return output;
}

//========================================================MPU 6050 GYRO AND ACCELEROMETER=================================================================
//========================================================================================================================================================
//========================================================================================================================================================
//========================================================================================================================================================
//========================================================================================================================================================

void mpuSetup()
{
  Wire.setClock(400000L);
	Wire.begin();

	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);
	sendConsole(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	sendConsole(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
	mpu.setXAccelOffset(-4004);
	mpu.setYAccelOffset(-1632);
	mpu.setZAccelOffset(1952);
	mpu.setXGyroOffset(-179);
	mpu.setYGyroOffset(-84);
	mpu.setZGyroOffset(-47);

	if (devStatus == 0) {
		mpu.setDMPEnabled(true);
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
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
  while (!mpuInterrupt && fifoCount < packetSize) {
    
  }
  mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();

    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        currentYawAngle = ypr[0] * 180/M_PI - yawOffset;
        currentPitchAngle = ypr[1] * 180/M_PI - pitchOffset;
        currentRollAngle = ypr[2] * 180/M_PI - rollOffset;

        currentYawRate = currentYawRate*.7+ (-mpu.getRotationZ()/65.5)*.3;
        currentPitchRate = currentPitchRate*.7+ (-mpu.getRotationY()/65.5)*.3;
        currentRollRate = currentRollRate*.7+ (mpu.getRotationX()/65.5)*.3;
        
        currentYaw = currentYawRate;
        currentPitch = currentPitchRate;
        currentRoll = currentRollRate;
    }
}
