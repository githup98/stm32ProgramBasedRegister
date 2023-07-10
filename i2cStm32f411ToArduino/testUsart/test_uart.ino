#include <string.h>
#include <SoftwareSerial.h>
char buf[40] = {0};
int countInt = 0;
SoftwareSerial mySerial = SoftwareSerial(10, 11); //rx, tx
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(13, OUTPUT);
  pinMode(10, INPUT_PULLUP); //Rx
  pinMode(11, OUTPUT);
  pinMode(2, INPUT_PULLUP);  //for interrupt
  attachInterrupt(0, tatled, FALLING);
  mySerial.begin(9600);
}
int data[5] = {0};
int i = 0;
byte serialCheck = 0;
void loop() {
  // put your main code here, to run repeatedly:
  while(mySerial.available())
  {
    data[i] = mySerial.read();
    i++;
  }

  if(i > 3)
  {
    i = 0;  
    serialCheck = 1;
  }
  
  
  if(serialCheck)
  {
    sprintf(buf, "check value : %x -- %x -- %x -- %x\n", data[0], data[1], data[2], data[3]);
    Serial.println(buf);
    serialCheck = 0;
   
  }
  
  if(countInt == 1)
  {
    Serial.println("sending");
    mySerial.write(0x59);
    mySerial.write(0x48);

    mySerial.write(0x56);
    mySerial.write(0x23);
    Serial.println("send done");
    countInt = 2;
 
  }
  if(countInt == 3)
  {
    Serial.println("sending");
    mySerial.write(0x58);
    mySerial.write(0x47);

    mySerial.write(0x50);
    mySerial.write(0x21);
    Serial.println("send done");
    countInt = 0; 
  }
  if(countInt > 3)
  {
    countInt = 0; 
   }

}


void tatled(void)
{
  countInt ++;  
}
