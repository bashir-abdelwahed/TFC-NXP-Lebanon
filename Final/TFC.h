
#include "mbed.h"

/** @file test.h*/

/**
 * @defgroup FRDM-TFC_API FRDM-TFC_API
 *
 * @{
 */


/**

@addtogroup FRDM-TFC_API

@{

Resources used by the TFC Library\n

I/O:\n
-------------------------------------------------------------------------------------------------\n

    PTA12  Servo 1\n
    PTA13  Servo 2\n
\n
\n
    PTB8   (Camera CCD1 SI)\n
    PTB9   (Camera CCD1 CLK)\n
    PTC2   (Camera CCD1 A0)\n
\n

    PTA4    Motor1 PWM1 FTM0_CH1\n
    PTA5    Motor1 PWM2 FTM0_CH2\n
    PTC8    Motor2 PWM1 FTM0_CH4\n
    PTC9    Motor2 PWM2 FTM0_CH5\n
    }
*/



#ifndef _TFC_H
#define _TFC_H

#define TAOS_CLK_HIGH  PTB->PSOR = (1<<9)
#define TAOS_CLK_LOW   PTB->PCOR = (1<<9)
#define TAOS_SI_HIGH   PTB->PSOR = (1<<8)
#define TAOS_SI_LOW    PTB->PCOR = (1<<8)


/**

@addtogroup FRDM-TFC_API
@{
*/

/** Initialized the TFC API.   Call before using any other API calls.
*
*/
void TFC_Init();

/** ServoTicker will increment once every servo cycle.
* It can be used to synchronize events to the start of a servo cycle. ServoTicker is a volatile uint32_t and is updated in the TPM1 overlflow interrupt.  This means you will see ServoTicker increment on the rising edge of the servo PWM signal
*
*/
 extern volatile uint32_t TFC_ServoTicker;


/** Sets the servo channels
*
*  @param ServoNumber  Which servo channel on the FRDM-TFC to use (0 or 1).  0 is the default channel for steering
*  @param Position     Angle setting for servo in a normalized (-1.0 to 1.0) form.   The range of the servo can be changed with the InitServos function.
*                       This is called in the TFC constructor with some useful default values-->  20mSec period,  0.5mS min and 2.0mSec max.  you may need to adjust these for your own particular setup.
*/
void TFC_SetServo(uint8_t ServoNumber, float Position);

/** Initializes TPM for the servoes.  It also sets the max and min ranges
*
*  @param ServoPulseWidthMin    Minimum pulse width (in seconds) for the servo.   The value of -1.0 in SetServo is mapped to this pulse width.  I.E.  .001
*  @param ServoPulseWidthMax    Maximum pulse width (in seconds) for the servo.   The value of +1.0 in SetServo is mapped to this pulse width.  I.E.  .002
*  @param ServoPeriod           Period of the servo pulses (in seconds).  I.e.  .020 for 20mSec
*/

void TFC_InitServos(float ServoPulseWidthMin, float ServoPulseWidthMax, float ServoPeriod);


/** Initialized TPM0 to be used for generating PWM signals for the the dual drive motors.   This method is called in the TFC constructor with a default value of 4000.0Hz
*
*  @param SwitchingFrequency PWM Switching Frequency in floating point format.   Pick something between 1000 and 9000.   Maybe you can modulate it and make a tune.
*/
void TFC_InitMotorPWM(float SwitchingFrequency);

/** Sets the PWM value for each motor.
*
*  @param MotorA    The PWM value for HBridgeA. The value is normalized to the floating point range of -1.0 to +1.0.    -1.0 is 0% (Full Reverse on the H-Bridge) and 1.0 is 100% (Full Forward on the H-Bridge)
*  @param MotorB    The PWM value for HBridgeB. The value is normalized to the floating point range of -1.0 to +1.0.    -1.0 is 0% (Full Reverse on the H-Bridge) and 1.0 is 100% (Full Forward on the H-Bridge)
*/
void TFC_SetMotorPWM(float MotorA ,float MotorB);


/** Pointer to two channels of line scan camera data.   Each channel is 128 points of uint8_t's.  Note that the underlying implementation is ping-pong buffer  These pointers will point to the 
*inactive buffer.   
*
*/

extern volatile uint16_t * TFC_LineScanImage0;
extern volatile uint16_t * TFC_LineScanImage1;


/** This flag will increment when a new frame is ready.  Check for a non zero value (and reset to zero!) when you want to read the camera(s)
*
*/

extern volatile uint8_t TFC_LineScanImageReady;

/** @} */


#endif
