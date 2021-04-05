#include "mbed.h"
#include "lstarks.h"



class PID{
    public:
        PID(float p, float i, float d, float deltat);
        PID(float u, float p, float i, float d, float deltat);
        bool setminmax(float min, float max);   // returns true on success... set minimum and maximum on the pid output 
        bool setminmaxI(float min, float max);  // returns true on success... set minimum and maximum on the integrator output 
        float output(float error);
    
        float deltat;
        float p,d,i;
        float integrator;  
        float min,max;
        bool  limitedoutput;
        float minI,maxI;
        bool  limitedintegrator;  
        float history[HISTORY_LENGTH][2]; // saves the past errors,outputs (100 values)...
    
};