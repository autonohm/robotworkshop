#include <iostream>
#include "control.h"

void transferFunctionToStateControl(float bTf[4], float aTf[4], enum tfOrder order, float A[9], float b[3], float c[3], float &d, bool echo)
{
  A[0]=0.f; A[1]=0.f; A[2]=0.f;
  A[3]=0.f; A[4]=0.f; A[5]=0.f;
  A[6]=0.f; A[7]=0.f; A[8]=0.f;
  b[0]=0.f; b[1]=0.f; b[2]=0.f;
  c[0]=0.f; c[1]=0.f; c[2]=0.f;
  switch(order)
  {
  case zero:
    break;
  case first:
    A[0]=-aTf[0]/aTf[1];
    b[0]=1.f/aTf[1];
    c[0]=bTf[0]-bTf[1]*aTf[0]/aTf[1];
    break;
  case second:
    A[0]=0.f;                             A[1]=1.f;
    A[3]=-aTf[0]/aTf[2];                  A[4]=-aTf[1]/aTf[2];
    b[0]=0.f;                             b[1]=1.f/aTf[2];
    c[0]=bTf[0]-bTf[2]*aTf[0]/aTf[2];     c[1]=bTf[1]-bTf[2]*aTf[1]/aTf[2];
    break;
  case third:
    A[0]=0.f;                             A[1]=1.f;
    A[3]=0.f;                             A[4]=0.f;                             A[5]=1.f;
    A[6]=-aTf[0]/aTf[3];                  A[7]=-aTf[1]/aTf[3];                  A[8]=-aTf[2]/aTf[3];
    b[0]=0.f;                             b[1]=0.f;                             b[2]=1.f/aTf[3];
    c[0]=bTf[0]-bTf[3]*aTf[0]/aTf[3];     c[1]=bTf[1]-bTf[3]*aTf[1]/aTf[3];     c[2]=bTf[2]-bTf[3]*aTf[2]/aTf[3];
    break;
  }
  d=bTf[order]/aTf[order];

  if(echo)
  {
    std::cout << "A = [ " << A[0] << " " << A[1] << " " << A[2] << std::endl;
    std::cout << "      " << A[3] << " " << A[4] << " " << A[5] << std::endl;
    std::cout << "      " << A[6] << " " << A[7] << " " << A[8] << " ]" << std::endl;

    std::cout << "b = [ " << b[0] << " " << b[1] << " " << b[2] << std::endl;

    std::cout << "c = [ " << c[0] << " " << c[1] << " " << c[2] << std::endl;

    std::cout << "d = " << d << std::endl;
  }
}

void pidToTransferFunction(float kp, float ki, float kd, float tPar, float bTf[4], float aTf[4], bool echo)
{
  bTf[0]=ki;       bTf[1]=kp+ki*tPar;       bTf[2]=kp*tPar+kd;        bTf[3]=0.f;
  aTf[0]=0.f;      aTf[1]=1.f;              aTf[2]=tPar;              aTf[3]=0.f;

  if(echo)
  {
    std::cout << " b     " << bTf[2] << " s^2 + " << bTf[1] << " s" << std::endl;
    std::cout << "--- = -----------------------------" << std::endl;
    std::cout << " a     " << aTf[2] << " s^2 + " << aTf[1] << " s + " << aTf[0] << std::endl;
  }
}
