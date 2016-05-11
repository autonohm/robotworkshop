
/********************************/
/* Sketch for I2C Communication */
/********************************/

#include <Wire.h>

#define ANALOG_MAX 4096 //max. analog-value of the LTC2629


//-----------Commands for I/O (PCA9534)-------------
bool i2c_setDigital(byte value, byte addr)
{
  byte _msg[2];
  
  _msg[0] = 0x01;  //Write-Output-Register
  _msg[1] = value;
  i2c_write(_msg,sizeof(_msg),addr);  
}

bool i2c_stopDigital(byte addr)
{
  byte _msg[2];
  
  _msg[0] = 0x01;   //Write-Output-Register
  _msg[1] = 0x00;
  i2c_write(_msg,sizeof(_msg),addr);  
}

bool i2c_readDigital(byte* buf, byte reg, byte addr)
{  
  i2c_write(&reg,1,addr);
  
  if(i2c_read(buf,1,addr))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool i2c_initDigital()
{
 byte _msg[2]; 
 
 //set in/out 
 _msg[0] = 0x03;
 _msg[1] = 0x30;
 i2c_write(_msg,sizeof(_msg),0x20);
 i2c_write(_msg,sizeof(_msg),0x21);
 i2c_write(_msg,sizeof(_msg),0x22);
 
 //set dir
 i2c_setDigital(0x08,0x20);
 i2c_setDigital(0x00,0x21);
 i2c_setDigital(0x0C,0x22);
}


//-----------Commands for DAC (LTC2629)-------------
bool i2c_setAnalog(short value, byte addr, byte channel)
{
  byte _msg[3];
  
  short _val = abs(value/8);
  if(_val > ANALOG_MAX)
  {
    _val = ANALOG_MAX;
  }
  
  if(channel == 1)
  {
    _msg[0] = 0x30;
  }
  else if(channel == 2)
  {
    _msg[0] = 0x31;
  }
  else
  {
    return false;
  }
  _msg[1] = (byte)((0x0FF0 & _val) >> 4);
  _msg[2] = (byte)((0x000F & _val) << 4);
  
  if(i2c_write(_msg,sizeof(_msg),addr))
  {
    return true;
  }
  else
  {
    return false;
  }  
}

bool i2c_setAnalog(short value, byte addr)
{
  byte _msg[3];
  short _val = abs(value/8);
  
  _msg[0] = 0x2f;  
  _msg[1] = (byte)((0x0FF0 & _val) >> 4);
  _msg[2] = (byte)((0x000F & _val) << 4);
  
  if(i2c_write(_msg,sizeof(_msg),addr))
  {
    return true;
  }
  else
  {
    return false;
  }  
}

bool i2c_setAnalog(short value)
{
  byte _msg[3];
  short _val = abs(value/8);
  
  _msg[0] = 0x2f;  
  _msg[1] = (byte)((0x0FF0 & _val) >> 4);
  _msg[2] = (byte)((0x000F & _val) << 4);
  
  if(i2c_write(_msg,sizeof(_msg),0x73))
  {
    return true;
  }
  else
  {
    return false;
  }  
}

bool i2c_stopAnalog()
{
  byte _msg[3];
  
  _msg[0] = 0x4f;
  _msg[1] = 0;
  _msg[2] = 0;
  
  if(i2c_write(_msg,sizeof(_msg),0x73))
  {
    return true;
  }
  else
  {
    return false;
  }  
}

bool i2c_startAnalog()
{
  byte _msg[3];
  
  _msg[0] = 0x1f;
  _msg[1] = 0;
  _msg[2] = 0;
  
  if(i2c_write(_msg,sizeof(_msg),0x73))
  {
    return true;
  }
  else
  {
    return false;
  }  
}


//-----------------------i2c_general---------------------
void i2c_init()
{
  Wire.begin();
}

bool i2c_write(byte* msg, unsigned int len, byte addr)
{
  unsigned int _written = 0;
  Wire.beginTransmission(addr);
  _written = Wire.write(msg,len); //PROBLEM! schreibt immer len
  Wire.endTransmission();
  if(_written == len)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool i2c_read(byte* msg, unsigned int len, byte addr)
{
  unsigned int _pending = 0;
  
  Wire.requestFrom(addr,len);
  
  _pending = Wire.available();
  if(_pending =! len)
  {
    return false;
  }
  
  for(int i=0; i<len; i++)
  {
    msg[i] = Wire.read();
  }
  
  return true;
}
  


  
