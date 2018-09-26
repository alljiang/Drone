//This arduino will relay commands from the computer to the drone and back using the HC-12 Module

#include <SoftwareSerial.h>

byte HC12SetPin = 9;
byte HC12TXPin = 7;
byte HC12RXPin = 4;

long receiveBaud = 57600;
long sendBaud = 9600;

SoftwareSerial hc12(HC12TXPin, HC12RXPin); //tx, rx

void setup() {
  pinMode(HC12SetPin, OUTPUT);
  digitalWrite(HC12SetPin, LOW);
  Serial.begin(receiveBaud);
  hc12.begin(sendBaud);
  delay(1000);
  configureHC12();
  digitalWrite(HC12SetPin, HIGH);
}

void loop() {
  if(Serial.available()) {
    mySerial.write(Serial.read());
  }
  if(mySerial.available()) {
    Serial.write(mySerial.read());
  }
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

void send(byte toSend[], int length) {
  byte packet[length+1];
  for(int i = 0; i < length; i++)
    packet[i] = toSend[i];
  packet[length] = calculateChecksum(packet, length);
  Serial.write(packet, length+1);
  Serial.flush();
}

void configureHC12() {
  digitalWrite(HC12SetPin, LOW);
  delay(5);
  setBaud(57600);
  setMode(3);
  setPower(8);
  sendConfig();
  digitalWrite(HC12SetPin, HIGH);
  hc12.close();
  hc12.begin(57600);
}

void testHC12() {
  clearData();
  delay(50);
  hc12.print("AT");
  if(hc12.available() > 0}
  
}

void setBaud(long baud) {
  hc12.print("AT+B" + String(baud));
}

void setMode(int mode) {
  hc12.print("AT+FU" + String(mode));
}

void setPower(int power) {
  hc12.print("AT+P" + String(power));
}

void sendConfig() {
  hc12.print("AT+RX");
}

void clearData() {
  byte trash[] = {};
  hc12.readBytes(trash, hc12.available());
  }
}

