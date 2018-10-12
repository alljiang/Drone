//This arduino will relay commands from the computer to the drone and back using the HC-12 Module

#include <SoftwareSerial.h>

byte HC12SetPin = 2;
byte HC12TXPin = 7;
byte HC12RXPin = 4;

long receiveBaud = 38400;
long sendBaud = 38400;
long initialHC12Baud = 38400;

SoftwareSerial hc12(HC12TXPin, HC12RXPin); //tx, rx

void setup() {
  pinMode(HC12SetPin, OUTPUT);
  digitalWrite(HC12SetPin, LOW);
  Serial.begin(receiveBaud);
  hc12.begin(initialHC12Baud);
  delay(1000);
  sendConsole("Receiver connected");
  hc12.print(("AT+B"+String(sendBaud)));
  delay(100);
  hc12.begin(sendBaud);
  Serial.println(hc12.readString());
  digitalWrite(HC12SetPin, HIGH);
}

void loop() {
  if(Serial.available() > 0) {
    byte toSend[1];
    Serial.readBytes(toSend, 1);
    hc12.write(toSend[0]);
  }
  if(hc12.available()) {
    byte toSend[1];
    hc12.readBytes(toSend, 1);
    Serial.write(toSend[0]);
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
