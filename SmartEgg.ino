#include <Wire.h>
#include <I2Cdev.h>
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <lm75.h>

SoftwareSerial bleSerial(4, 5);

TempI2C_LM75 termo[8] = {TempI2C_LM75(0x48,TempI2C_LM75::nine_bits),
                         TempI2C_LM75(0x49,TempI2C_LM75::nine_bits),
                         TempI2C_LM75(0x4A,TempI2C_LM75::nine_bits),
                         TempI2C_LM75(0x4B,TempI2C_LM75::nine_bits),
                         TempI2C_LM75(0x4C,TempI2C_LM75::nine_bits),
                         TempI2C_LM75(0x4D,TempI2C_LM75::nine_bits),
                         TempI2C_LM75(0x4E,TempI2C_LM75::nine_bits),
                         TempI2C_LM75(0x4F,TempI2C_LM75::nine_bits)
                        };

Quaternion q;
MPU6050 mpu;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;

uint8_t fifoBuffer[64]; // FIFO存储缓冲器
float buff1[10];        //发送数据缓存

unsigned long time_mpu, time_tem;

uint8_t devStatus;

//获取四元数
void dmpGetQuaternion()
{
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if((mpuIntStatus & 0x10)||fifoCount == 1024)
    {
        mpu.resetFIFO(); 
    }else if(mpuIntStatus & 0x02)
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
  //      mpu.resetFIFO(); 
    }
         
}

void setup() {
  Serial.begin(9600);
  bleSerial.begin(9600);
  Wire.begin();
  mpu.initialize();

  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed"); 

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0)
  {
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      mpuIntStatus = mpu.getIntStatus();
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
     while(1)
     {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
     } 
   }

}

void loop() {
  dmpGetQuaternion();
  if(millis() > time_mpu+1000)
  {
      time_mpu = millis();   
      buff1[0] = q.w;
      buff1[1] = q.x;
      buff1[2] = q.y;
      buff1[3] = q.z; 
/* 
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
 */ 
      sendData(0xAA, 16, (uint8_t *)buff1);
  }

  if(millis() > time_tem+5000)
  {
      time_tem = millis();
      for(int i=0; i<8; i++)
      {
          buff1[i] = termo[i].getTemp();
          Serial.print(buff1[i]);
          Serial.print(",");
      }  
       Serial.println(" ");
      sendData(0xBB, 32, (uint8_t *)buff1);
  }
}

void sendData(uint8_t cmd,int _num, uint8_t* _buf)
{
    uint8_t sendBuf[40];

    sendBuf[0] = 0xAA;
    sendBuf[1] = 0xBB;
    sendBuf[2] = cmd;
    if(_num > 0)
      memcpy(sendBuf+3, _buf, _num);
    sendBuf[_num+3] = 0x0d;
    sendBuf[_num+4] = 0x0a;
    
    bleSerial.write(sendBuf, _num+5);      
}



