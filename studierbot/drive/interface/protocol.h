/**
 * @author Stefan May
 * @date 10.10.2015
 * @brief Protocol helper functions (conversion of data types to byte arrays)
 */

#define VALUESCALE 100.f

inline void convertTo4ByteArray(short* vals, char array[])
{
  array[0] = (vals[0] & 0xFF00) >> 8;
  array[1] = (vals[0] & 0x00FF);
  array[2] = (vals[1] & 0xFF00) >> 8;
  array[3] = (vals[1] & 0x00FF);
}

inline void convertTo4ByteArray(int val, char array[])
{
  array[0] = (val & 0xFF000000) >> 24;
  array[1] = (val & 0x00FF0000) >> 16;
  array[2] = (val & 0x0000FF00) >> 8;
  array[3] = (val & 0x000000FF);
}

inline void convertTo4ByteArray(float val, char array[])
{
  int* ival = (int*)&val;
  array[0] = (*ival & 0xFF000000) >> 24;
  array[1] = (*ival & 0x00FF0000) >> 16;
  array[2] = (*ival & 0x0000FF00) >> 8;
  array[3] = (*ival & 0x000000FF);
}

inline void convertFromByteArray(char array[], short retval[2])
{
  retval[0] = ((array[0] << 8) & 0xFF00) | ((array[1] << 0) & 0x00FF);
  retval[1] = ((array[2] << 8) & 0xFF00) | ((array[3] << 0) & 0x00FF);
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
