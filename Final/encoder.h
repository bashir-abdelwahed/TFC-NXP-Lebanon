#ifndef _ENCODER_H_
#define _ENCODER_H_
 
#include "mbed.h"
 
/** Encoder class.
 *  Used to read out incremental position encoder. Decodes position in X2 configuration.
 *
 *  Speed estimation is very crude and computationally intensive. Turned off by default
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "Encoder.h"
 *
 *   Encoder motor1(PTD0,PTC9,true);
 *   Serial pc(USBTX,USBRX);
 *   pc.baud(115200);
 *   while(1) {
 *       wait(0.2);
 *       pc.printf("pos: %d, speed %f \r\n",motor1.getPosition(), motor1.getSpeed());
 *   }
 * @endcode
 */
 
 #define LENGTHSPEED   5   // length of the array speeds
 #define MAXSPEED      400 // the class is returning weird values at slow speeds... /// to be adjust later on...
 #define MAXDIFFERENCE 150
class Encoder
{
    public:
    /** Create Encoder instance
    @param int_a Pin to be used as InterruptIn! Be careful, as not all pins on all platforms may be used as InterruptIn.
    @param int_b second encoder pin, used as DigitalIn. Can be any DigitalIn pin, not necessarily on InterruptIn location
    @param speed boolean value to determine whether speed calculation is done in interrupt routine. Default false (no speed calculation)
    */
 
    Encoder(PinName int_a);
    /** Request position
    @returns current position in encoder counts
    */
    //int32_t getPosition(){return m_position;}
    /** Overwrite position
    @param pos position to be written
    */
    //void    setPosition(int32_t pos){m_position = pos;}
    /** Request speed
    @returns current speed
    */
    
    //return m_speed;}
    float   getSpeed(){
        if(zero_speed) return 0; 
        float sp = meanSpeed(); 
        if(sp > MAXSPEED ) return MAXSPEED; 
        return sp;
    }     
        
        
    //private:
    void encoderFalling(void);
    void encoderRising(void);
    void push(float newspeed);
    float meanSpeed();
    
    Timer EncoderTimer;
    Timeout EncoderTimeout;
    InterruptIn pin_a;
  

    void timeouthandler(void);
    bool zero_speed;
    float speeds[LENGTHSPEED];
};
 
 
#endif //_ENCODER_H_
            