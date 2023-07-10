#include <Wire.h>

#define SLAVE_ADDR 0x0f

//#define testAsSlave 1
//#define testReadAsMaster 1
#define testWriteAsMaster 1

byte received[5] = {0};

byte sendArr[5] = {0x97, 0x94};
byte sendArr1[5] = {0x95, 0x99};

char buf[60]={0};

int interruptFuncFlag = 0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  
  pinMode(2, INPUT_PULLUP);  //for interrupt
  attachInterrupt(0, tatled, FALLING);
  
  Serial.println("xin chao 2\n");
  
  delay(50);
  ////slave config//////
  #if testAsSlave
  Wire.begin(0x02);
  Wire.onReceive(BlinkLED);
  Wire.onRequest(feedBack);
  #endif
//////////////////////


/////Master write config//////
  #if testWriteAsMaster
  interruptFuncFlag = 2;
  int error = sendByteData(sendArr);
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
  #if testReadAsMaster
  
  interruptFuncFlag = 1;
  
  delay(500);
  receivedData(5);
  
  #endif
}
////////////////////////////////

int oneMore = 0;


void loop() {
  // put your main code here, to run repeatedly:
  if(oneMore == 1)
  {
      if(interruptFuncFlag == 1)
      {
        receivedData(5);
      }
      else if(interruptFuncFlag == 2)
      {
        sendByteData(sendArr1);
      }
      
      oneMore = 0;
  }
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
  sprintf(buf, "received value %d", data[k]); 
  Serial.println(buf);
    }

  
}



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


void tatled(void)
{
  oneMore = 1;
}


void receivedData(byte num)
{
  int b = 0;
  Wire.requestFrom(SLAVE_ADDR, num, true); //stop after request not restart
  delay(5);
  while(Wire.available())
    {
      received[b] = Wire.read();
      b += 1;
    }
  sprintf(buf, "Received data:%x -- %x -- %x -- %x -- %x", received[0], received[1], received[2], received[3], received[4]);
  Serial.println(buf);  
}




int sendByteData(byte *data)
{
  Serial.println("start sending");
  Wire.beginTransmission(0x0f);
  delay(5);
  Wire.write(data[0]);
  delay(5);
  Wire.write(data[1]);
  delay(5);
  int err = Wire.endTransmission();  
  Serial.println("send done");
  return err;
}
