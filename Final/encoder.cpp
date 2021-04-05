#include "encoder.h"
 
 
Encoder::Encoder(PinName int_a) : pin_a(int_a)
{
    pin_a.mode(PullNone);
    
    
        EncoderTimer.start();
    pin_a.fall(this,&Encoder::encoderFalling);
    pin_a.rise(this,&Encoder::encoderRising);
    zero_speed = false;
}
 
void Encoder::encoderFalling(void)
{
    //temporary speed storage, in case higher interrupt level does stuff
    float temp_speed=0;
    int motortime_now;
    
        motortime_now = EncoderTimer.read_us();
        EncoderTimer.reset();
        EncoderTimeout.attach(this,&Encoder::timeouthandler,0.1);
        /*calculate as ticks per second*/
        if(zero_speed)
            temp_speed  = 0;
        else
            {temp_speed = 1000000./motortime_now;push(1.5*temp_speed);}
        zero_speed = false;
    
    
    
}
 
void Encoder::encoderRising(void)
{
    //temporary speed storage, in case higher interrupt level does stuff
 
    float temp_speed=0;
    int motortime_now;
    
    motortime_now = EncoderTimer.read_us();
    EncoderTimer.reset();
    EncoderTimeout.attach(this,&Encoder::encoderFalling,0.1);
    /*calculate as ticks per second*/
    if(zero_speed)
        temp_speed  = 0;
    else
        {temp_speed = 1000000./motortime_now; push(temp_speed);}
    zero_speed = false;
    
    
    
    
}
 
void Encoder::timeouthandler(void)
{
    
    zero_speed = true;
}

void Encoder::push(float newspeed){
    for(int i = LENGTHSPEED-1; i>0;i--)
       speeds[i] = speeds[i-1]; 
    
    if(newspeed - speeds[0] < MAXDIFFERENCE || speeds[0] -newspeed > MAXDIFFERENCE) /// not pushing noisy values
        speeds[0] = newspeed; 
     
}

float Encoder::meanSpeed(){
    float temp = 0;
    
      
    for (int i = 0 ; i < LENGTHSPEED; i++)
        temp += speeds[i];
        
    return temp/LENGTHSPEED;    

}