/**
 * @author Stefan May
 * @date 10.10.2015
 * @brief Protocol helper functions (conversion of data types to byte arrays)
 */

#define VALUESCALE 100.f

inline void convertTo12ByteArray(short* vals, char array[])
{
  array[0]  = (vals[0] & 0xFF00) >> 8;
  array[1]  = (vals[0] & 0x00FF);
  array[2]  = (vals[1] & 0xFF00) >> 8;
  array[3]  = (vals[1] & 0x00FF);
  array[4]  = (vals[2] & 0xFF00) >> 8;
  array[5]  = (vals[2] & 0x00FF);
  array[6]  = (vals[3] & 0xFF00) >> 8;
  array[7]  = (vals[3] & 0x00FF);
  array[8]  = (vals[4] & 0xFF00) >> 8;
  array[9]  = (vals[4] & 0x00FF);
  array[10] = (vals[5] & 0xFF00) >> 8;
  array[11] = (vals[5] & 0x00FF);
}

inline void convertTo12ByteArray(int val, char array[])
{
  array[0]  = (val & 0xFF000000) >> 24;
  array[1]  = (val & 0x00FF0000) >> 16;
  array[2]  = (val & 0x0000FF00) >> 8;
  array[3]  = (val & 0x000000FF);
  array[4]  = 0;
  array[5]  = 0;
  array[6]  = 0;
  array[7]  = 0;
  array[8]  = 0;
  array[9]  = 0;
  array[10] = 0;
  array[11] = 0;
}

inline void convertTo12ByteArray(float val, char array[])
{
  int* ival = (int*)&val;
  array[0]  = (*ival & 0xFF000000) >> 24;
  array[1]  = (*ival & 0x00FF0000) >> 16;
  array[2]  = (*ival & 0x0000FF00) >> 8;
  array[3]  = (*ival & 0x000000FF);
  array[4]  = 0;
  array[5]  = 0;
  array[6]  = 0;
  array[7]  = 0;
  array[8]  = 0;
  array[9]  = 0;
  array[10] = 0;
  array[11] = 0;
}

inline void convertFromByteArray(char array[], short retval[6])
{
  retval[0] = ((array[0]  << 8) & 0xFF00) | ((array[1]  << 0) & 0x00FF);
  retval[1] = ((array[2]  << 8) & 0xFF00) | ((array[3]  << 0) & 0x00FF);
  retval[2] = ((array[4]  << 8) & 0xFF00) | ((array[5]  << 0) & 0x00FF);
  retval[3] = ((array[6]  << 8) & 0xFF00) | ((array[7]  << 0) & 0x00FF);
  retval[4] = ((array[8]  << 8) & 0xFF00) | ((array[9]  << 0) & 0x00FF);
  retval[5] = ((array[10] << 8) & 0xFF00) | ((array[11] << 0) & 0x00FF);
}

inline void convertFromByteArray(char array[], int &retval)
{
  retval = ((array[0] << 24) & 0xFF000000) |
           ((array[1] << 16) & 0x00FF0000) |
           ((array[2] << 8) & 0x0000FF00)  |
           ((array[3] << 0) & 0x000000FF);
}

inline void convertFromByteArray(char array[], float &retval)
{
  int val = ((array[0] << 24) & 0xFF000000) |
            ((array[1] << 16) & 0x00FF0000) |
            ((array[2] << 8) & 0x0000FF00)  |
            ((array[3] << 0) & 0x000000FF);
  float* fval = (float*)&val;
  retval = *fval;
}
