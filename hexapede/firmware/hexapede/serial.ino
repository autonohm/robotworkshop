
/***********************************/
/* Sketch for Serial Communication */
/***********************************/

bool serial_init(unsigned int rate, unsigned int timeout)
{
  Serial.begin(rate);
  Serial.setTimeout(timeout);
  return true;
}

int serial_read(short* msg, unsigned int len)
{
  if(len < 6){         return(-1);} //-------ERROR: min. msg-length 6 Shorts!
  
  byte buf[256];
  int recv_n =  0;
  int checksum = 0;
  short checksum_rcv = 0;
  
  if(Serial.available() >= (len*2+2))
  {
    recv_n = Serial.readBytes(buf, len*2+2);
    
    if(recv_n == (len*2+2))
    {
      for(int i=0; i<len; i++)
      {
        msg[i] = (buf[2*i+1] << 8 | buf[2*i]);
        checksum += abs(msg[i]);    
      }
      checksum_rcv = (buf[len*2+1] << 8) | buf[len*2];
      checksum_rcv -= (checksum & 0xFFFF);
      
      if(checksum_rcv)
      {
        return(0);  //------ERROR: Invalid Checksum!     
      }
    }
    else
    {
      return(-2);  //------ERROR: Failed reading Serial Data!
    }
  }
  else
  {
    return(-3);   //------ERROR: incomplete inc. Data"!
  }
  return(0);
}

bool serial_write(short* msg, unsigned int len)
{
  byte buf[len*2];
  
  for(int i=0; i<len; i++)
  {
    buf[2*i]   = (short)(0x00FF & msg[i]);
    buf[2*i+1] = (short)((0xFF00 & msg[i]) >> 8);
  }
  Serial.write(buf, len*2);
}
  
