/**
 * @author Stefan May
 * @date 10.10.2015
 * @brief Helper functions for conversion of PID and transfer function parameters
 */

/**
 * Enum for supported orders of transfer function
 */
enum tfOrder { zero=0, first=1, second=2, third=3 };

/**
 * Convert transfer function to matrices for state control
 * @param bTf numerator of transfer function
 * @param aTf denominator of transfer function
 * @param order order of transfer function
 * @param A system matrix A
 * @param b input vector b
 * @param c output vector c
 * @param d pass-through coefficient
 * @param echo verbosity flag
 */
void transferFunctionToStateControl(float bTf[4], float aTf[4], enum tfOrder order, float A[9], float b[3], float c[3], float &d, bool echo);

/**
 * Convert PID controller parameters to transfer function
 * @param kp P-Gain
 * @param ki I-Gain
 * @param kd D-Gain
 * @param tPar parasitic time constant
 * @param bTf numerator
 * @param aTf denominator
 * @param echo verbosity flag
 */
void pidToTransferFunction(float kp, float ki, float kd, float tPar, float bTf[4], float aTf[4], bool echo);
