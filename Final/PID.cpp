#include "PID.h"

PID::PID(float p, float i, float d, float deltat){
 this->p = p;
 this->d = d;
 this->i = i;
 this->integrator = 0;   
 this->deltat = deltat;
 this->limitedoutput = false;
 this->limitedintegrator = false;
}

PID::PID(float u, float p, float i, float d, float deltat){
 this->p = u*p;
 this->d = u*d;
 this->i = u*i;
 this->integrator = 0;   
 this->deltat = deltat;
 this->limitedoutput = false;
 this->limitedintegrator = false;
}

bool PID::setminmax(float min, float max){
 if(min<max){
    this->min = min;
    this->max = max;
    this->limitedoutput = true;
    return true; 
 }
 return false;
}   
bool PID::setminmaxI(float min, float max){
 if(min < max){
    this->minI = min;
    this->maxI = max;
    this->limitedintegrator = true;
    return true; 
 }   
 return false;
}
float PID::output(float error){
    integrator += error*deltat;
    if(limitedintegrator){
        if(integrator > maxI) integrator = maxI;
        else if(integrator < minI) integrator = minI;   
    }
    float output = error*p + d*(error - history[0][0])/deltat + i*integrator;
    if(limitedoutput){
        if(output > max)       output = max;  
        else if(output < min)  output = min;
    }
    memmove(history+1,history,(HISTORY_LENGTH-1)*2*sizeof(float));
    history[0][0] = error;
    history[0][1] = output;
    return output;
    
}