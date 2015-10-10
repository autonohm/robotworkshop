/**
 * @author Stefan May
 * @date 10.10.2015
 * @brief Protocol helper functions (conversion of data types to byte arrays)
 */

#define VALUESCALE 100.f

inline void shortValuesTo4ByteArray(short val1, short val2, char array[])
{
  array[0] = (val1 & 0xFF00) >> 8;
  array[1] = (val1 & 0x00FF);
  array[2] = (val2 & 0xFF00) >> 8;
  array[3] = (val2 & 0x00FF);
}

inline void intTo4ByteArray(int val, char array[])
{
  array[0] = (val & 0xFF000000) >> 24;
  array[1] = (val & 0x00FF0000) >> 16;
  array[2] = (val & 0x0000FF00) >> 8;
  array[3] = (val & 0x000000FF);
}

inline void floatTo4ByteArray(float val, char array[])
{
  int* ival = (int*)&val;
  array[0] = (*ival & 0xFF000000) >> 24;
  array[1] = (*ival & 0x00FF0000) >> 16;
  array[2] = (*ival & 0x0000FF00) >> 8;
  array[3] = (*ival & 0x000000FF);
}

inline short byteArrayToShort(char array[])
{
  return ((array[0] << 8) & 0xFF00) | ((array[1] << 0) & 0x00FF);
}

inline int byteArrayToInt(char array[])
{
  return ((array[0] << 24) & 0xFF000000) |
         ((array[1] << 16) & 0x00FF0000) |
         ((array[2] << 8) & 0x0000FF00)  |
         ((array[3] << 0) & 0x000000FF);
}

inline float byteArrayToFloat(char array[])
{
  int val = ((array[0] << 24) & 0xFF000000) |
            ((array[1] << 16) & 0x00FF0000) |
            ((array[2] << 8) & 0x0000FF00)  |
            ((array[3] << 0) & 0x000000FF);
  float* fval = (float*)&val;
  return *fval;
}
