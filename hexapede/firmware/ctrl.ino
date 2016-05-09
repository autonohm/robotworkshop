/********************************/
/* Sketch for controlling ic-s  */
/********************************/

//Defines for the digital-Controller
#define ADDR_D_12 0x21
#define ADDR_D_34 0x20
#define ADDR_D_56 0x22

#define REG_IO   0x00
#define REG_OUT  0x01
#define REG_INV  0x02
#define REG_CTRL 0x03

//Defines for the Analog-controller
#define ADDR_A_12 0x11
#define ADDR_A_34 0x10
#define ADDR_A_56 0x12
#define ADDR_A    0x73

#define BIT_D_EN1  0x01
#define BIT_D_EN2  0x02
#define BIT_D_DIR1 0x04
#define BIT_D_DIR2 0x08
#define BIT_D_RDY1 0x10
#define BIT_D_RDY2 0x20

//Adress-Arrays for the I2C-Controller

byte Addr_D[7]{
  //0: no global adress for the Digital-IO-ICs 
  0x00,
  //1-6: adresses of the Digital-IO-ICs controlling the motors 1-6
  ADDR_D_12,
  ADDR_D_12,
  ADDR_D_34, 
  ADDR_D_34,
  ADDR_D_56,
  ADDR_D_56,
};

byte Addr_A[7]{
  //0: the global-Adress for Analog-ICs
  ADDR_A,
  //1-6: adresses of the Analog-ICs controlling the motors 1-6
  ADDR_A_12,
  ADDR_A_12,
  ADDR_A_34, 
  ADDR_A_34,
  ADDR_A_56,
  ADDR_A_56,
};

//----------------------------------------------------------------------------------------------------------------

//setting control-values fo the motorChanels 1-6 (analog+digital)
//Returns 0 if correct!
unsigned int ctrl_setMotor(unsigned int ch, short val)
{
  byte _addrA = Addr_A[ch];
  byte _addrD = Addr_D[ch];
  byte _digRegVal = 0x00;
  byte _digRegValNew = 0x00;
  unsigned int _onboardChanel = ((ch+1)%2)+1; //chanel 1 or 2 on the shields? 
  
  if(!i2c_readDigital(&_digRegVal, REG_IO, _addrD))
  {
    return(1);        //Error Reading Values of the Digital-Controller
  }
  
  //differentiating between CH1 und CH2 of the shields
  //Onboard CH1
  if(_onboardChanel == 1)
  {
    //MaxonController Ready?
    if(!(_digRegVal & BIT_D_RDY1))
    {
      _digRegValNew = _digRegVal;
    }
    else
    {
      if(val < 0)
      {
        _digRegValNew = _digRegVal    | BIT_D_EN1;
        _digRegValNew = _digRegValNew & (~BIT_D_DIR1);
      }
      else if(val > 0)
      {
        _digRegValNew = _digRegVal    | BIT_D_EN1;
        _digRegValNew = _digRegValNew | BIT_D_DIR1;
      }
      else
      {
        _digRegValNew = _digRegVal    & (~BIT_D_EN1);
      }
    }
    i2c_setAnalog(val, _addrA); 
    i2c_setDigital(_digRegValNew, _addrD);
    
  //Onboard CH2  
  }
  else if(_onboardChanel == 2)
  {
    //MaxonController Ready?
    if(!(_digRegVal & BIT_D_RDY2))
    {
      _digRegValNew = _digRegVal;
    }
    else
    {
      if(val < 0)
      {
        _digRegValNew = _digRegVal    | BIT_D_EN2;
        _digRegValNew = _digRegValNew & (~BIT_D_DIR2);
      }
      else if(val > 0)
      {
        _digRegValNew = _digRegVal    | BIT_D_EN2;
        _digRegValNew = _digRegValNew | BIT_D_DIR2;
      }
      else
      {
        _digRegValNew = _digRegVal    & (~BIT_D_EN2);
      }
    }
    i2c_setAnalog(val, _addrA); 
    i2c_setDigital(_digRegValNew, _addrD);
  }
  else
  {
   return(2);          //Error : No valid OnboardChanel!
  }
  
  return(0);  
}

void ctrl_stopMotor(void)
{
  i2c_stopAnalog();
  i2c_stopDigital(ADDR_D_12);
  i2c_stopDigital(ADDR_D_34);
  i2c_stopDigital(ADDR_D_56);
}
  
