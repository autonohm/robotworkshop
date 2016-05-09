/********************************/
/* Sketch for main loop/init    */
/********************************/


#define SERIAL_RATE 115200
#define SERIAL_BUF 6       //buffer-length recv-buffer
#define SERIAL_TO 1000     //Timeout waiting for complete MSG

#define TIMEOUT  100       //timeout for Serial-Com in ms

short buf[SERIAL_BUF];

unsigned int serial_error;    //gives Error-Code when communication-error via serial (0 if ok)
short motor_error[6];         //short array for printing the state of the several motors
unsigned int cnt_to;          //counter for timeout

void setup()
{
  delay(20000); //testing startup-delay 'cause of serial-communication-issues 
  cnt_to = 0;
  
  i2c_init();
  i2c_stopAnalog();
  i2c_initDigital();
  
  serial_init(SERIAL_RATE,SERIAL_TO);
}


void loop()
{
  serial_error = serial_read(buf,SERIAL_BUF);
  if(serial_error == 0){ 
    i2c_setAnalog(buf[0]);
    
    for(int i=0; i<6; i++)
    {
      motor_error[i] = (short)ctrl_setMotor(i+1, buf[i]);
    }
    serial_write(motor_error, SERIAL_BUF);   
    cnt_to = 0;
  }
  else
  {
    cnt_to ++;
  }
  
  //Auto-Stop if no serial data arrives within TIMEOUT  
  if(cnt_to >= TIMEOUT)
  {
    ctrl_stopMotor();
  }
  
  delay(1);
}
