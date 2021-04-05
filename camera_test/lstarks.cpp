#include "mbed.h"
#include "TFC.h"
#include "lstarks.h"


Serial Pc(USBTX, USBRX);
Serial Blue(PTE22,PTE23);// using uart pins tx,rx
DigitalOut ground(PTC6);
AnalogIn key(PTC0);

int readkey(){
    
    float val = key.read();
    //Blue.printf("%f \r\n",val);
    if ( val >0.055) return 1;
    if( val < 0.001) return 3;
    if ( val < 0.039) return 2;
    if ( val < 0.045) return 4;
    if ( val < 0.050 ) return 5;
    if (val < 0.055) return 6;
    return 1;// nothing is pressed
    
}

void lstarksinit(){
    // manage your stuff before the race...
    // maybe read the inputs from the buttons... stuff like that...
    // remember to call if before TFC_Init() because of the setup of the resistor...
    ground.write(1);
    wait(0.2);
    ground.write(0);    
    
}

void copycam(volatile uint16_t in[], uint16_t out[]){
    
    for (int i = 0;i<128;i++) out[i] = in[i];
}


float ratioRL(float servo_position){
    // calculated by doing quadratic function y = ax^2 +bx + c

    return  0.2*servo_position* servo_position +0.5 * servo_position + 1;
    
}
int checkcurve(int history[HISTORY_LENGTH][3], int maxindex){
    int i;
    for( i = maxindex; i > 0;i--){
        if(history[i][0] - history[i+1][0] < 0) break;// no conclusions    
    }
    if( i == 0 ) return -1; //right curve
    
    for( i = maxindex; i > 0;i--){
        if(history[i][1] - history[i+1][1] > 0) break;// no conclusions    
    }
    if( i == 0 ) return 1; //left curve
    
    
    return 0; //going straight    
    
    
}
// this is a shitty function ...
bool checkcriticalcase(int history[HISTORY_LENGTH][3], int &BL, int &BR, int maxindex){
    // for example: check last 5 values if all are in critical case => do something about it...
    if(BL < CRITICAL_LEFT_LINE && BR > CRITICAL_RIGHT_LINE) return false; // no problem at all
    if(BL >= CRITICAL_LEFT_LINE){ // this BL might be a BLright
        for (int i = 0;i<maxindex; i++)
            if(history[i][1] > CRITICAL_RIGHT_LINE) return false;
            //if(!(history[i][0] <= 10 && history[i][1] <= CRITICAL_RIGHT_LINE)) return false;
            
        BR = BL; // false BL left
        BL = 1;  
        return true;
    }
    else { // this BR might be a BLleft
        for ( int i = 0;i< maxindex; i++)
            if(history[i][0] < CRITICAL_LEFT_LINE) return false;  
            //if(!(history[i][1] >=118 && history[i][0] >= CRITICAL_LEFT_LINE)) return false;   
        BL = BR; // false BL right
        BR = 128;
        return true;
    }    
    
}

void diff(uint16_t in[], uint16_t out[]){
    // calculate difference between the pixels
    for(int i=64;i<127;i++)                         
    {
        out[i] = abs (in[i] - in[i+1]);
    } 
     for(int i=64;i>5;i--)                           
    {
        out[i] = abs (in[i] - in[i-1]);
    }
    
}
int blineright(uint16_t diff[]){ // end must be less than 122
    // find blackest line on the right                
   int i;
    for( i=64;i<126;i++)   // continue to iterate all the way to the right
        if (diff[i]> THRESHOLD){
            return i;
        }
        
    return 128;
}
int blineleft(uint16_t diff[]){
    // this is problematic at intersections
    // find blackest line on the left                  
    int i;
    
    for(i=64;i>2;i--)// only down to pixel 3, not 1
        if (diff[i] > THRESHOLD){   
            
            return i; // return nearest number to the center
        }// continue to iterate all the way to the left
    return 0;
}

/*
int blineright(uint16_t diff[],int begin, int end){ // end must be less than 122
    // find blackest line on the right                
   int i, max = THRESHOLD, maxindex = end;
    for( i=begin;i<end;i++)   // continue to iterate all the way to the right
        if (diff[i]> max){
            max = diff[i];
            maxindex = i;
        }
        
    return maxindex;
}

int blineleft(uint16_t diff[],int begin, int end){
    // this is problematic at intersections
    // find blackest line on the left                  
    int i, max = THRESHOLD, maxindex = end;
    
    for(i=begin;i>end;i--)// only down to pixel 3, not 1
        if (diff[i] > max){   
            max = diff[i];
            maxindex = i;
            //return i; // return nearest number to the center
        }// continue to iterate all the way to the left
    return maxindex;
}
*/
void countlines(uint16_t diff[],int begin, int end,int &BL, int &BR, int &nbBigLines ){
    int i,j;   BL=BR=-1;
        
    nbBigLines = 0;
    for(i = begin;i<end;i++)
        if(diff[i] > THRESHOLD)
            for (j = i+2;j< i+15;j++) // starting from i+2
                if(diff[j] > THRESHOLD) 
                    if(j < i+7)// found small line if 6 or more => black lines
                        if( i<64) {BL = j; i = j+2;break;} //left line
                        else {BR = i; i = j+2;break;} // right line
                    else {nbBigLines++;i = j+2; break;}// i found big line
    if(BL == -1 ) BL = 0;
    if ( BR == -1 ) BR = 128;         
    /*if(nbBigLines <2) {
        BL = blineleft(diff,64,0); 
        BR = blineright(diff,64,128);
        }
    */
}

int checkForLines(uint16_t camera[])
{ 
   int num = 0;
        for(int i=6;i<=122;i++) if(abs(camera[i+1]-camera[i])>0) num++;   

        return num;
}
void printdata(int BlackLineLeft,int BlackLineRight, float servo_position){
    Pc.printf("Left: %d ",BlackLineLeft);
    Pc.printf("\r\n");    
    Pc.printf("right: %d ",BlackLineRight);
    Pc.printf("\r\n");
    Pc.printf("servo: %f ",servo_position);
    Pc.printf("\r\n");
    
}
    

float maximuminarray(float servo_positions[], int maxindex){
    float max = servo_positions[0];
    for(int i = 1;i< maxindex;i++) 
        if (abs(servo_positions[i]) > max ) 
            max = abs(servo_positions[i]);
    return max;
}



float meaninarray(float servo_positions[], int maxindex){
    float mean = 0;
    for(int i = 0;i< maxindex;i++) 
        mean += abs(servo_positions[i]);
    return mean/maxindex;
}


 


int median(int a, int b, int c){return max(min(a,b), min(max(a,b),c));}
int min(int a, int b){ if(a<=b) return a; return b;}
int max(int a, int b){ if(a<=b) return b; return a;}
/*  // 5 elements ...
void _medianfilter(const element* signal, element* result, int N)
{
   //   Move window through all elements of the signal
   for (int i = 2; i < N - 2; ++i)
   {
      //   Pick up window elements
      element window[5];
      for (int j = 0; j < 5; ++j)
         window[j] = signal[i - 2 + j];
      //   Order elements (only half of them)
      for (int j = 0; j < 3; ++j)
      {
         //   Find position of minimum element
         int min = j;
         for (int k = j + 1; k < 5; ++k)
            if (window[k] < window[min])
               min = k;
         //   Put found minimum element in its place
         const element temp = window[j];
         window[j] = window[min];
         window[min] = temp;
      }
      //   Get result - the middle element
      result[i - 2] = window[2];
   }
}*/