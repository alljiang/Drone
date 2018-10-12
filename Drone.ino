/*
   Resources:
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
*/
#include <SoftwareSerial.h>
#include <avr/wdt.h>

//=========================================MPU STUFF======================================================================================================
#include <MPU6050_tockn.h>
#include <Wire.h>
MPU6050 mpu6050(Wire);
//========================================================================================================================================================

SoftwareSerial hc12(8, 9); //rx, tx

const int SetPin = 7;
float yawkp = 0;
float yawki = 0;
float yawkd = 0;
float kp = 0.0;
float ki = 0.0;
float kd = 0.0;
float levelkp = 0.0;
float levelki = 0.0;
float levelkd = 0.0;
long lastTime = 0;
double IntegralAnglePitch = 0;
double IntegralAngleRoll = 0;
double IntegralYawRate = 0;
double IntegralPitchRate = 0;
double IntegralRollRate = 0;
double IntegralPitchAngle = 0;
double IntegralRollAngle = 0;
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
double lastPitchAngle = 0;
double lastRollAngle = 0;
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
double maxPID = 50;
double maxIntegral = 50;
double currentMotor0 = 0;
double currentMotor1 = 0;
double currentMotor2 = 0;
double currentMotor3 = 0;
double pitch_level_adjust = 0;
double roll_level_adjust = 0;
int printLoop = 0;

long sendPeriod = 200;
long lastSend = 0;
long lastSentTime = 0;
long timeout = 600;
long updateFrequency = 200;
long receiveTimeout = 100;
long sampleTime = 4;
int controlRate = 35;

boolean enabled = false;
boolean FRESC = true;
boolean FLESC = true;
boolean BLESC = true;
boolean BRESC = true;

long initialBaud = 38400;
long baud = 38400;

void setup() {
  Serial.begin(baud);
  Serial.setTimeout(0);
  mpuSetup();
  wdt_enable(WDTO_8S);

  pinMode(SetPin, OUTPUT);
  digitalWrite(SetPin, LOW);
  hc12.begin(initialBaud);
  hc12.setTimeout(0);

  delay(1000);
  wdt_reset();

  configureHC12();
  sendConsole("Started");
  delay(1000);
  sendConsole("Starting MPU Setup");
  wdt_reset();
  sendConsole("MPU Setup Successful");

  lastCommandTime = millis();
  lastSentTime = millis();

  wdt_enable(WDTO_1S);
  wdt_disable();

  DDRD |= B01111000; //set 3,4,5,6 to output
}

void loop() {
  if ((millis() - lastCommandTime > timeout) || !enabled) disable();
  receiveCommands();
  if (millis() - lastSend > sendPeriod) {
    sendUpdates();
    lastSend = millis();
  }
  mpuLoop();
  if (enabled) {
    while (millis() - lastTime <= sampleTime) {}
    drive();
    lastTime = millis();
  }
}

void configureHC12() {
  digitalWrite(SetPin, LOW);
  delay(10);
  hc12.print("AT+B" + String(baud)); //set baud
  delay(50);
  hc12.print("AT+RX"); //get all values
  delay(50);
  Serial.println(hc12.readString());
  digitalWrite(SetPin, HIGH);
  hc12.begin(38400);
}

void send(byte toSend[], int length) {
  byte packet[length + 1];
  for (int i = 0; i < length; i++)
    packet[i] = toSend[i];
  packet[length] = calculateChecksum(packet, length);
  hc12.write(packet, length + 1);
}

byte calculateChecksum(byte arr[], int length)
{
  byte sum = 0;
  for (int i = 0; i < length; i++)
    sum += arr[i];
  return sum;
}

void sendConsole(String s)
{
  int totalLength = s.length() + 2;
  byte byt[s.length() + 2];
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
  if (millis() < lastSentTime + updateFrequency) return;
  lastSentTime = millis();
  voltage = getBatteryVoltage();
  byte mpuAngles[] = {0xB, (byte)currentPitchAngle, (byte)currentRollAngle};
  byte battery[] = {0xA, (byte)(voltage * 10)};
  byte motors[] = {0x9, (byte)currentMotor0, (byte)currentMotor1, (byte)currentMotor2, (byte)currentMotor3};
  byte mpuRates[] = {0x8, (byte)currentYawRate, (byte)currentPitchRate, (byte)currentRollRate};
  send(mpuAngles, 3);
  send(battery, 2);
  send(motors, 5);
  send(mpuRates, 4);
}

/*********************************************************************************************************/
void receiveCommands() {
  if (hc12.available() == 0) return;
  lastCommandTime = millis();
  byte command = hc12.peek();
  if (command >= 0 && command < 6) lastCommandTime = millis();
  if (command == 0x1) //zero gyro
  {
    if(hc12.available() >= (1+2+1)) {
      byte trash[] = {};
      hc12.readBytes(trash, 1);
    }
    else { 
      return;
    }
    byte holder[2];
    readSerial(holder, 2);
    byte checksum[1];
    readSerial(checksum, 1);
    byte calculatedChecksum = calculateChecksum(holder, 2) + command;
    if (calculatedChecksum != checksum[0])
    {
      hc12.flush();
      return;
    }
    calibrateMPU();
    return;
  }
  else if (command == 0x2) //set joystick values
  {
    if(hc12.available() >= (1+5+1)) {
      byte trash[] = {};
      hc12.readBytes(trash, 1);
    }
    else return;
    byte holder[5];
    readSerial(holder, 5);
    byte checksum[1];
    readSerial(checksum, 1);
    byte calculatedChecksum = calculateChecksum(holder, 5) + command;
    if (calculatedChecksum != checksum[0])
    {
      hc12.flush();
      return;
    }
    if (!enabled) {
      enable();
    }
    YAxis = holder[0];
    RYAxis = holder[1];
    RXAxis = holder[2];
    RTrig = holder[3] == 0 ? false : true;
    LTrig = holder[4] == 0 ? false : true;
  }
  else if (command == 0x3) //set motor testing
  {
    if(hc12.available() >= (1+4+1)) {
      byte trash[] = {};
      hc12.readBytes(trash, 1);
    }
    else return;
    byte holder[4];
    readSerial(holder, 4);
    byte checksum[1];
    readSerial(checksum, 1);
    byte calculatedChecksum = calculateChecksum(holder, 4) + command;
    if (calculatedChecksum != checksum[0])
    {
      hc12.flush();
      return;
    }
    if (enabled) {
      disable();
    }
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
    writeESC(speedFR, speedFL, speedBL, speedBR);
  }
  else if (command == 0x4) //set PID constants
  {
    if(hc12.available() >= (1+30+1)) {
      byte trash[] = {};
      hc12.readBytes(trash, 1);
    }
    else return;
    byte holder[30];
    readSerial(holder, 30);
    byte checksum[1];
    readSerial(checksum, 1);
    byte calculatedChecksum = calculateChecksum(holder, 30) + command;
    if (calculatedChecksum != checksum[0])
    {
      sendConsole("PID");
      hc12.flush();
      return;
    }
    long kp_sign = holder[0] == 0 ? 1 : -1;
    long kp_1 = ((long)holder[1] << 24);
    long kp_2 = ((long)holder[2] << 16);
    long kp_3 = ((long)holder[3] << 8);
    long kp_4 = ((long)holder[4] << 0);
    kp = kp_sign * (kp_1 + kp_2 + kp_3 + kp_4) / 100000.;
    long ki_sign = holder[5] == 0 ? 1 : -1;
    long ki_1 = ((long)holder[6] << 24);
    long ki_2 = ((long)holder[7] << 16);
    long ki_3 = ((long)holder[8] << 8);
    long ki_4 = ((long)holder[9] << 0);
    ki = ki_sign * (ki_1 + ki_2 + ki_3 + ki_4) / 100000.;
    long kd_sign = holder[10] == 0 ? 1 : -1;
    long kd_1 = ((long)holder[11] << 24);
    long kd_2 = ((long)holder[12] << 16);
    long kd_3 = ((long)holder[13] << 8);
    long kd_4 = ((long)holder[14] << 0);
    kd = kd_sign * (kd_1 + kd_2 + kd_3 + kd_4) / 100000.;
    long yawkp_sign = holder[15] == 0 ? 1 : -1;
    long yawkp_1 = ((long)holder[16] << 24);
    long yawkp_2 = ((long)holder[17] << 16);
    long yawkp_3 = ((long)holder[18] << 8);
    long yawkp_4 = ((long)holder[19] << 0);
    yawkp = yawkp_sign * (yawkp_1 + yawkp_2 + yawkp_3 + yawkp_4) / 100000.;
    long yawki_sign = holder[20] == 0 ? 1 : -1;
    long yawki_1 = ((long)holder[21] << 24);
    long yawki_2 = ((long)holder[22] << 16);
    long yawki_3 = ((long)holder[23] << 8);
    long yawki_4 = ((long)holder[24] << 0);
    yawki = yawki_sign * (yawki_1 + yawki_2 + yawki_3 + yawki_4) / 100000.;
    long yawkd_sign = holder[25] == 0 ? 1 : -1;
    long yawkd_1 = ((long)holder[26] << 24);
    long yawkd_2 = ((long)holder[27] << 16);
    long yawkd_3 = ((long)holder[28] << 8);
    long yawkd_4 = ((long)holder[29] << 0);
    yawkd = yawkd_sign * (yawkd_1 + yawkd_2 + yawkd_3 + yawkd_4) / 100000.;
    sendConsole(String(kp));
  }
  else if (command == 0x5) //set translational trim
  {
    if(hc12.available() >= (1+1+1)) {
      byte trash[] = {};
      hc12.readBytes(trash, 1);
    }
    else return;
    byte holder[1];
    readSerial(holder, 1);
    byte checksum[1];
    readSerial(checksum, 1);
    byte calculatedChecksum = calculateChecksum(holder, 1) + command;
    if (calculatedChecksum != checksum[0]) {
      hc12.flush();
      return;
    }
    if (holder[0] == 0) pitchTrim -= .5;
    else if (holder[0] == 1) rollTrim += .5;
    else if (holder[0] == 2) pitchTrim += .5;
    else if (holder[0] == 3) rollTrim -= .5;
  }
  else if (command == 0x6) //set rotational trim
  {
    if(hc12.available() >= (1+1+1)) {
      byte trash[] = {};
      hc12.readBytes(trash, 1);
    }
    else return;
    byte holder[1];
    readSerial(holder, 1);
    byte checksum[1];
    readSerial(checksum, 1);
    byte calculatedChecksum = calculateChecksum(holder, 1) + command;
    if (calculatedChecksum != checksum[0]) {
      hc12.flush();
      return;
    }
    if (holder[0] == 0) yawTrim -= 1;
    else if (holder[0] == 1) yawTrim += 1;
  }
  else if (command == 0x7) //set Angular PID constants
  {
    if(hc12.available() >= (1+5+1)) {
      byte trash[] = {};
      hc12.readBytes(trash, 1);
    }
    else return;
    byte holder[15];
    readSerial(holder, 15);
    byte checksum[1];
    readSerial(checksum, 1);
    byte calculatedChecksum = calculateChecksum(holder, 15) + command;
    if (calculatedChecksum != checksum[0])
    {
      sendConsole("PID");
      hc12.flush();
      return;
    }
    long kp_sign = holder[0] == 0 ? 1 : -1;
    long kp_1 = ((long)holder[1] << 24);
    long kp_2 = ((long)holder[2] << 16);
    long kp_3 = ((long)holder[3] << 8);
    long kp_4 = ((long)holder[4] << 0);
    levelkp = kp_sign * (kp_1 + kp_2 + kp_3 + kp_4) / 100000.;
    long ki_sign = holder[5] == 0 ? 1 : -1;
    long ki_1 = ((long)holder[6] << 24);
    long ki_2 = ((long)holder[7] << 16);
    long ki_3 = ((long)holder[8] << 8);
    long ki_4 = ((long)holder[9] << 0);
    levelki = ki_sign * (ki_1 + ki_2 + ki_3 + ki_4) / 100000.;
    long kd_sign = holder[10] == 0 ? 1 : -1;
    long kd_1 = ((long)holder[11] << 24);
    long kd_2 = ((long)holder[12] << 16);
    long kd_3 = ((long)holder[13] << 8);
    long kd_4 = ((long)holder[14] << 0);
    levelkd = kd_sign * (kd_1 + kd_2 + kd_3 + kd_4) / 100000.;
    sendConsole(String(levelkp));
  }
  else {
    byte trash[] = {};
    hc12.readBytes(trash, 1);
  }
}

void readSerial(byte arr[], int numToRead) {
  byte temp[1];
  long lastReceived = millis();
  for (int i = 0; i < numToRead; i++) {
    if (hc12.available() > 0) {
      hc12.readBytes(temp, 1);
      lastReceived = millis();
    }
    else {
      i--;
      if (millis() - lastReceived >= receiveTimeout) break;
      continue;
    }
    arr[i] = temp[0];
  }
  hc12.flush();
}
/*********************************************************************************************************/

void enable() {
  IntegralAnglePitch = 0;
  IntegralAngleRoll = 0;
  lastErrorYaw = currentYawAngle;
  lastErrorPitch = currentPitchAngle;
  lastErrorRoll = currentRollAngle;
  lastTime = millis();
  enabled = true;
  writeESC(0, 0, 0, 0);
}

void disable() {
  enabled = false;
  writeESC(0, 0, 0, 0);
}

void drive() {
  //  while(currentPitchAngle == lastPitchAngle || currentRollAngle == lastRollAngle) {
  //    mpuLoop();
  //  }
  if(currentPitchAngle == lastPitchAngle || currentRollAngle == lastRollAngle) return;
  if (RYAxis > 100) RYAxis = map(RYAxis, 156, 250, -100, 0);
  if (RXAxis > 100) RXAxis = map(RXAxis, 156, 250, -100, 0);

  //pitch angle calculations
  double errorPitchAngle = currentPitchAngle;
  double changePitchAngle = currentPitchAngle;
  IntegralPitchAngle += levelki * errorPitchAngle / 100000.;
  if (IntegralPitchAngle > maxIntegral) IntegralPitchAngle = maxIntegral;
  if (IntegralPitchAngle < -maxIntegral) IntegralPitchAngle = -maxIntegral;
  lastPitchAngle = currentPitchAngle;
  float pitchPIDOutputAngle = levelkp * errorPitchAngle / 1000. + IntegralPitchAngle + levelkd * changePitchAngle / 100.;

  if (pitchPIDOutputAngle > maxPID) pitchPIDOutputAngle = maxPID;
  if (pitchPIDOutputAngle < -maxPID) pitchPIDOutputAngle = -maxPID;

  //roll angle calculations
  double errorRollAngle = currentRollAngle;
  double changeRollAngle = currentRollAngle;
  IntegralRollAngle += levelki * errorRollAngle / 100000.;
  if (IntegralRollAngle > maxIntegral) IntegralRollAngle = maxIntegral;
  if (IntegralRollAngle < -maxIntegral) IntegralRollAngle = -maxIntegral;

  float rollPIDOutputAngle = levelkp * errorRollAngle / 1000. + IntegralRollAngle + levelkd * changeRollAngle / 100.;
  lastRollAngle = currentRollAngle;
  if (rollPIDOutputAngle > maxPID) rollPIDOutputAngle = maxPID;
  if (rollPIDOutputAngle < -maxPID) rollPIDOutputAngle = -maxPID;

  double expectedPitchRate = map(RYAxis, -100, 100, -controlRate, controlRate) - pitchPIDOutputAngle;
  double expectedRollRate = map(RXAxis, -100, 100, controlRate, -controlRate) - rollPIDOutputAngle;

  //pitch rate calculations
  double errorPitchRate = currentPitchRate - expectedPitchRate;
  double changeErrorPitchRate = errorPitchRate - lastErrorPitchRate;
  double changePitchRate = currentPitchRate - lastPitchRate;
  IntegralPitchRate += ki * errorPitchRate / 100000.;
  if (IntegralPitchRate > maxIntegral) IntegralPitchRate = maxIntegral;
  if (IntegralPitchRate < -maxIntegral) IntegralPitchRate = -maxIntegral;

  float pitchPIDOutputRate = kp * errorPitchRate / 1000. + IntegralPitchRate + kd * changePitchRate / 100.;
  //    float pitchPIDOutputRate = kp * errorPitchRate/1000. + IntegralPitchRate + kd * changeErrorPitchRate/100.;
  lastPitchRate = currentPitchRate;
  lastErrorPitchRate = errorPitchRate;
  if (pitchPIDOutputRate > maxPID) pitchPIDOutputRate = maxPID;
  if (pitchPIDOutputRate < -maxPID) pitchPIDOutputRate = -maxPID;

  //roll rate calculations
  double errorRollRate = currentRollRate - expectedRollRate;
  double changeErrorRollRate = errorRollRate - lastErrorRollRate;
  double changeRollRate = currentRollRate - lastRollRate;
  IntegralRollRate += ki * errorRollRate / 100000.;
  if (IntegralRollRate > maxIntegral) IntegralRollRate = maxIntegral;
  if (IntegralRollRate < -maxIntegral) IntegralRollRate = -maxIntegral;

  float rollPIDOutputRate = kp * errorRollRate / 1000. + IntegralRollRate + kd * changeRollRate / 100.;
  //    float rollPIDOutputRate = kp * errorRollRate/1000. + IntegralRollRate + kd * changeErrorRollRate/100.;
  //  slowSendConsole(String(kp*errorPitchRate/1000.) + " " + String(IntegralPitchRate) + " " + String(kd*changePitchRate/100.), 40);
  lastRollRate = currentRollRate;
  lastErrorRollRate = errorRollRate;
  if (rollPIDOutputRate > maxPID) rollPIDOutputRate = maxPID;
  if (rollPIDOutputRate < -maxPID) rollPIDOutputRate = -maxPID;

  //yaw calculations
  double expectedYawRate = 0;
  double errorYawRate = currentYawRate - expectedYawRate;
  double changeErrorYawRate = errorYawRate - lastErrorYawRate;
  IntegralYawRate += yawki * errorYawRate / 100000.;
  if (IntegralYawRate > maxIntegral) IntegralYawRate = maxIntegral;
  if (IntegralYawRate < -maxIntegral) IntegralYawRate = -maxIntegral;

  float yawPIDOutputRate = yawkp * errorYawRate / 100. + IntegralYawRate / 100. + yawkd * changeErrorYawRate / 100.;
  lastErrorYawRate = errorYawRate;
  if (yawPIDOutputRate > maxPID) yawPIDOutputRate = maxPID;
  if (yawPIDOutputRate < -maxPID) yawPIDOutputRate = -maxPID;

  float FROutput = YAxis + pitchPIDOutputRate - rollPIDOutputRate - yawPIDOutputRate + yawTrim + pitchTrim - rollTrim;
  float FLOutput = YAxis + pitchPIDOutputRate + rollPIDOutputRate + yawPIDOutputRate - yawTrim + pitchTrim + rollTrim;
  float BLOutput = YAxis + -pitchPIDOutputRate + rollPIDOutputRate - yawPIDOutputRate + yawTrim - pitchTrim + rollTrim;
  float BROutput = YAxis + -pitchPIDOutputRate - rollPIDOutputRate + yawPIDOutputRate - yawTrim - pitchTrim - rollTrim;

  //compensate for voltage
  //  FROutput += FROutput*(12.6-voltage)/34.5;
  //  FLOutput += FLOutput*(12.6-voltage)/34.5;
  //  BLOutput += BLOutput*(12.6-voltage)/34.5;
  //  BROutput += BROutput*(12.6-voltage)/34.5;

  writeESC(FROutput, FLOutput, BLOutput, BROutput);
}

void slowSendConsole(String str, int skips) {
  if (printLoop++ >= skips) {
    sendConsole(str);
    printLoop = 0;
  }
}

void calibrateMPU()
{
  yawOffset += currentYawAngle;
  pitchOffset += currentPitchAngle;
  rollOffset += currentRollAngle;
  yawTrim = 0;
  pitchTrim = 0;
  rollTrim = 0;
  lastErrorYaw = 0;
  lastErrorPitch = 0;
  lastErrorRoll = 0;
  IntegralAnglePitch = 0;
  IntegralAngleRoll = 0;
  IntegralYawRate = 0;
  IntegralPitchRate = 0;
  IntegralRollRate = 0;
}

void writeESC(float FR, float FL, float BL, float BR) {
  if (FR < 0 || !FR) FR = 0;
  else if (FR > 100) FR = 100;
  if (FL < 0 || !FL) FL = 0;
  else if (FL > 100) FL = 100;
  if (BL < 0 || !BL) BL = 0;
  else if (BL > 100) BL = 100;
  if (BR < 0 || !BR) BR = 0;
  else if (BR > 100) BR = 100;

  currentMotor0 = FR;
  currentMotor1 = FL;
  currentMotor2 = BL;
  currentMotor3 = BR;

  long escFR = long(map(FR, 0, 100, 995, 1832));
  long escFL = long(map(FL, 0, 100, 995, 1832));
  long escBL = long(map(BL, 0, 100, 995, 1832));
  long escBR = long(map(BR, 0, 100, 995, 1832));
  
  long arr[][2] = {{escFR-900,6-1},{escFL-900,5-1},{escBL-900,4-1},{escBR-900,3-1}};
  bubbleSort(arr, 4);
  arr[3][0] -= arr[2][0];
  arr[2][0] -= arr[1][0];
  arr[1][0] -= arr[0][0];

  noInterrupts();
  PORTD |= B01111000; // set DO 3,4,5,6 to high
  delayMicroseconds(900); //prevent delayMicroseconds from reaching >1000
  long start = micros();
  for(int i = 0; i < 4; i++) {
    delayMicroseconds(arr[i][0]);
    PORTD &= (B11111111^(1<<arr[i][1]));
  }
  interrupts();

  wdt_reset();
}

//{delay,pin}
void bubbleSort(long a[][2], int size) {
  for (int i = 0; i < (size - 1); i++) {
    for (int o = 0; o < (size - (i + 1)); o++) {
      if (a[o][0] > a[o + 1][0]) {
        long t[2] = {a[o][0], a[o][1]};
        a[o][0] = a[o + 1][0];
        a[o][1] = a[o + 1][1];
        a[o + 1][0] = t[0];
        a[o + 1][1] = t[1];
      }
    }
  }
}

float getBatteryVoltage() {
  analogReference(INTERNAL); //reference from 1.1V
  double ADCVal = analogRead(A7);
  float val = (ADCVal / 1024.) * 1.1 / (737. / (9700. + 737.)); //R1 = 9.7k  R2 = 737
  double a = -0.0033;
  double b = 1.0785;
  double c = -0.305;
  float output = ((a * val * val + b * val + c * 2) * 10) / 10;
  if (output < 5.5) output = 0; //probably just voltage from the usb input
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
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void mpuLoop()
{
  mpu6050.update();
  currentYawRate = currentYawRate * .7 + (-mpu6050.getGyroZ()) * .3;
  currentPitchRate = currentPitchRate * .7 + (-mpu6050.getGyroY()) * .3;
  currentRollRate = currentRollRate * .7 + (mpu6050.getGyroX()) * .3;

  currentYawAngle = currentYawAngle * .7 + mpu6050.getAngleZ() * .3;
  currentPitchAngle = currentPitchAngle * .7 + mpu6050.getAngleY() * .3;
  currentRollAngle = currentRollAngle * .7 + mpu6050.getAngleX() * .3;
}
