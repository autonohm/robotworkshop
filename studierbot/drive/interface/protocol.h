#define PARAMSCALE 10000.f
#define VALUESCALE 100.f

inline void intTo4ByteArray(int val, char array[])
{
  array[0] = (val & 0xFF000000) >> 24;
  array[1] = (val & 0x00FF0000) >> 16;
  array[2] = (val & 0x0000FF00) >> 8;
  array[3] = (val & 0x000000FF);
}

inline void shortValuesTo4ByteArray(short val1, short val2, char array[])
{
  array[0] = (val1 & 0xFF00) >> 8;
  array[1] = (val1 & 0x00FF);
  array[2] = (val2 & 0xFF00) >> 8;
  array[3] = (val2 & 0x00FF);
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
