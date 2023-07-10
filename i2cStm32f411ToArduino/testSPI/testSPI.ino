#include <SPI.h>

char buf[50] = {0};
// 13 SCK 12 MISO 11 MOSI 10 NSS

byte Data[10] = {0x59, 0x48, 0x56, 0x23};
byte Data2[10] = {0x59, 0x21, 0x56, 0x23};
byte Data3[10]= {0};
#define ss 10
#define sck 13
#define MOSI 11
#define MISO 12
void setup() {
  // put your setup code here, to run once:
pinMode(sck, OUTPUT);
pinMode(MOSI, OUTPUT);
pinMode(MISO, INPUT_PULLUP);
pinMode(ss, OUTPUT);
digitalWrite(sck, HIGH);
digitalWrite(MOSI, HIGH);
digitalWrite(ss, HIGH);
Serial.begin(9600);

delay(3000);
spiSendData(Data, 10, Data3);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(4000);
  
  
  sprintf(buf, "Data3[0] :%x -- %x -- %x -- %x", Data3[0], Data3[1], Data3[2], Data3[3]);
  Serial.print(buf);
  sprintf(buf, "--Data3[0] :%x -- %x -- %x -- %x", Data3[4], Data3[5], Data3[6], Data3[7]);
  Serial.print(buf);
  sprintf(buf, "--Data3[0] :%x -- %x", Data3[8], Data3[9]);
  Serial.println(buf);
  spiSendData(Data2, 10, Data3);

}

void spiSendData(byte *data, byte total, byte *data3)
{
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE1));
  delay(10);
  for(int cnt = 0; cnt < total; cnt ++)
  {
    digitalWrite(ss, LOW);
    delay(5);
    *data3 = SPI.transfer(*data);
    //delay(5);
    digitalWrite(ss, HIGH); 
    data++;
    data3++; 
  }
  
  SPI.endTransaction();  
}
