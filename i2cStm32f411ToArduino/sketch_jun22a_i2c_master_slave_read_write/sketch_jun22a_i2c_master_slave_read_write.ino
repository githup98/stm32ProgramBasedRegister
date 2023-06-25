#include <Wire.h>
//#include <stdio.h>
#define SLAVE_ADDR 0x0f
char buff[48]={0};

char buf[60]={0};
void setup() {
  // put your setup code here, to run once:
Wire.begin();
Serial.begin(9600);
Serial.println("xin chao 2\n");

delay(50);
////slave config//////
#if 0
Wire.begin(0x02);
Wire.onReceive(BlinkLED);
Wire.onRequest(feedBack);
#endif
//////////////////////


/////Master write config//////
#if 0
Wire.beginTransmission(0x0f);
delay(5);
Wire.write(0x97);
delay(5);
Wire.write(0x94);
delay(5);
int error = Wire.endTransmission();
if(error == 0)
{
    Serial.println("transmission done!");
}
else
{
    sprintf(buf, "error value: %d", error);
}
  Serial.println(buf);
#endif
////////////////////////




//////Master read config//////////
#if 1
byte received[5] = {0};
int b = 0;
delay(500);
Wire.requestFrom(SLAVE_ADDR, 3, false); //stop after request not restart
delay(10);
while(Wire.available())
  {
    received[b] = Wire.read();
    b += 1;
  }
  sprintf(buf, "Received data:%x -- %x -- %x", received[0], received[1], received[2]);
  Serial.println(buf);

#endif
}
////////////////////////////////



//byte data;
void loop() {
  // put your main code here, to run repeatedly:
delay(500);
}
int k = 0;
byte data[4] = {0};
void BlinkLED(int Press)
{
  for(k = 0;  k < Press; k++)
  {
  data[k] = Wire.read();  
    }
  for(k = 0;  k < Press; k++)
  {
  sprintf(buff, "received value %d", data[k]); 
  Serial.println(buff);
    }

  
}

byte sendData[2] = {0x89, 0x99};
int i = 0;
void feedBack(int Push)
{
  Wire.write(0x89);
  Wire.write(0x99);
  Wire.write(0xA9);
  Wire.write(0xB9);
  Wire.write(0xC9);
  Wire.write(0xD9);

  Serial.println("write call back");

}
