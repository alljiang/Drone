//This arduino will relay commands from the computer to the drone and back using the HC-12 Module

#include <SoftwareSerial.h>

byte HC12SetPin = 2;
byte HC12TXPin = 7;
byte HC12RXPin = 4;

long receiveBaud = 115200;
long sendBaud = 115200;

SoftwareSerial hc12(HC12TXPin, HC12RXPin); //tx, rx

void setup() {
  pinMode(HC12SetPin, OUTPUT);
  digitalWrite(HC12SetPin, LOW);
  Serial.begin(receiveBaud);
  hc12.begin(115200);
  delay(1000);
  configureHC12();
  Serial.println("Done");
  digitalWrite(HC12SetPin, HIGH);
}

void loop() {
  if(Serial.available()) {
    hc12.write(Serial.read());
  }
  if(hc12.available()) {
    Serial.write(hc12.read());
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

byte calculateChecksum(byte arr[], int length)
{
    byte sum = 0;
    for(int i = 0; i < length; i++)
      sum += arr[i];
    return sum;
}

void configureHC12() {
  digitalWrite(HC12SetPin, LOW);
  delay(10);
  hc12.print("AT+B"+String(sendBaud)); //set baud
  delay(50);
  hc12.print("AT+FU3"); //set transmission mode
  delay(50);
  hc12.print("AT+P8"); //set transmission power to max
  delay(50);
  hc12.print("AT+RX"); //get all values
  delay(50);
  hc12.print("AT+U8O1"); //set check bit
  delay(50);
  sendConsole(hc12.readString());
  digitalWrite(HC12SetPin, HIGH);
  hc12.begin(sendBaud);
  hc12.setTimeout(0);
}

void clearData() {
  byte trash[] = {};
  hc12.readBytes(trash, hc12.available());
}

