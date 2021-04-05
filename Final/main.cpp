#include "mbed.h"
#include "TFC.h"
#include "lstarks.h"
#include "encoder.h"
#include "ultrasonic.h"
#include "PID.h"


Serial pc(USBTX, USBRX);
Serial blue(PTE22,PTE23);// using uart pins tx,rx
Timer t;

DigitalOut g(LED_GREEN);
DigitalOut r(LED_RED);
DigitalOut b(LED_BLUE);


Ticker ticker;

int i;
uint16_t cam[128];
uint16_t camdiff[128];
int history[HISTORY_LENGTH][3]; //1 second
float servo_positions[HISTORY_LENGTH]; //1 second
          



/// servo related variables START
int BlackLineRight;                        
int BlackLineLeft;                          
int RoadMiddle = 0;                         
int Error = 0;                                                         
float servo_position = 0; 
float integration;
float derivation;
float derivation2;
float derivation3;

/* /// long rod values.
/////PID Servo START @ speed:    
float integrationlimit = 0.4;
float Ku=1.0f/32;
float KP =0.6*Ku; //0.5  ; 0.2;
float KD =0.15*Ku; //0.1  ;0.1
float KDD = 0.15*Ku; // temporary... this is KDD/deltat
float KI = 0.3*Ku;
/////PID Servo END

*/

int target = 64; // by default go to the middle
bool rightside = false; // obstacle avoidance on the left side by defalut

/////PID Servo START @ speed: 
float integrationlimit = 0.9;
float Ku=1.0f/32;
float KP =1*Ku; //1
float KD =0.15*Ku;  //0.15
float KDD = 0.15*Ku; 
float KD3 = 0*Ku;
float KI = 0.3*Ku;
float zero_servo_position = 0.9;

/////PID Servo END


/// servo related variables END




bool onintersection;
float deltat = 0.01;//0.01s => 100 frames/s in reality it takes 50 frames per second 


int keypressed, mode;

float maxrecentservo_pos;

///// for the speed limit zone challenge...   START
bool inslowdownarea = false, finished = false, crossingslowdown = false; 
int nbbiglines;
////// for the speed limit zone challenge...   END

Timer timer;


//// speed control variables   START

/*float maxspeed = 0.25;
float speed = maxspeed ;
float ratiospeed = speed/3;  // ratio of the differential on a turn...  add/subtract to/from left/right motor's pwm
float intersectionspeed = 0.075;// != maxspeed/2;
float KS = speed/3; // speed lower */

float maxspeed = 140;
float speed = maxspeed;
float ratiospeed = 0.5;
float intersectionspeed = 70;
float slowratio = 0; //0.3

Encoder leftspeed(PTD2);  // gives values in range: 0 ----> 200-300 very very very fast. remember not linear.
Encoder rightspeed(PTD3); // same thing


///// PID motors   START

float KPm = 0.003, KIm = 0.05,KDm = 0 , integlimitm = 0.3, maxoutputm = 0.6 , deltatm = 2*deltat ; //ki 0.003 kd 0.0001    // newest KD 0.000125

PID pidright(KPm,KIm,KDm,deltatm);
PID pidleft(KPm,KIm,KDm,deltatm);
//PID pidleft(0.01,0.003,0.0001,2*deltat);

///// PID motors END



//// speed control variables   END


//// prototyping  START
void motorcontrol(float lspeed,float rspeed);
void readablearrtopc( uint16_t in[]);
void readablearrtoblue( uint16_t in[]);
void arrtopc(uint16_t in[]);
void arrtoblue(uint16_t in[]);
void readcamera();
void push(int left,int right,int error, float servo_pos);
void dist(int distance);
void drive_crossing_pid_motors();
void drive();   // currently commented
void drive_crossing();  // currently commeneted
//ultrasonic mu(PTC5, PTD0, .05, 0.1, &dist); //FIRST TRIGGER 2ND ECHO
                                        //have updates every .1 seconds and a timeout after 0.5
                                        //second, and call dist when the distance changes

//// prototyping  END




void motorcontrol(float lspeed,float rspeed){
    
    TFC_SetMotorPWM(pidleft.output(lspeed-leftspeed.getSpeed()),pidright.output(rspeed-rightspeed.getSpeed()));
    //else TFC_SetMotorPWM(pidleft.output(150-rightspeed.getSpeed()),pidright.output(150-rightspeed.getSpeed()));
    //blue.printf("%f %f \r",leftspeed.getSpeed(),rightspeed.getSpeed());     
    
}

int main(){
    ///// intilizations START
    
    r=1;
    g=0.3f;
    b=1;
    
 
    pc.baud(250000);
    blue.baud(115200);
    lstarksinit();
    mode = 3;
    keypressed = readkey();
    
    pidright.setminmaxI(-integlimitm, integlimitm);  // never comment this part of the code
    pidright.setminmax(0, maxoutputm);  // never comment this part of the code
    
    pidleft.setminmaxI(-integlimitm,integlimitm);   // never comment this part of the code
    pidleft.setminmax(0, maxoutputm);    // never comment this part of the code

    TFC_Init(); 
    
    TFC_SetServo(1, zero_servo_position);
    
    TFC_SetMotorPWM(0,0);
    
    
    ///// intilizations END
    
    while(keypressed != 2){
        wait(.2);
        keypressed = readkey();
        if (keypressed != 1) mode = keypressed;
        switch(mode){
            case 3: r = 0; g = 1; b=1;  //last chance ...
                    maxspeed = 160;
                    speed = maxspeed;
                    ratiospeed = 0.25;
                    intersectionspeed = 70;
                    slowratio = 0; //0.3
                    
                    
                    /////PID Servo START @ speed: 
                    integrationlimit = 0.2;
                    Ku=1.0f/32;
                    KP =1*Ku; //1
                    KD =0.15*Ku;  //0.15
                    KDD = 0.15*Ku; //0.15
                    KD3 = 0.15*Ku; //.15
                    KI = 0.5*Ku;
                    /////PID Servo END
                    break;                         
            case 4: r = 1; g = 0; b=1;     // two more chance
                    /*maxspeed = 180;
                    speed = maxspeed;
                    ratiospeed = 0.25;
                    intersectionspeed = 70;
                    slowratio = 0; //0.3
                    
                    
                    /////PID Servo START @ speed: 
                    integrationlimit = 0.5;
                    Ku=1.0f/32;
                    KP =1.5*Ku; //1
                    KD =0.25*Ku;  //0.15
                    KDD = 0.25*Ku; //0.15
                    KD3 = 0.25*Ku; //.15
                    KI = 0.5*Ku;
                    /////PID Servo END*/
                    
                    
                    
                    maxspeed = 180;
                    speed = maxspeed;
                    ratiospeed = 0.25;
                    intersectionspeed = 70;
                    slowratio = 0; //0.3
                    
                    
                    /////PID Servo START @ speed: 
                    integrationlimit = 0.2;
                    Ku=1.0f/32;
                    KP =1.8*Ku; //1
                    KD =0.3*Ku;  //0.15
                    KDD = 0.3*Ku; //0.15
                    KD3 = 0.3*Ku; //.15
                    KI = 0.5*Ku;
                    /////PID Servo END
                    
                    
                    break;
            case 5: r = 1; g = 1; b=0;  // the fastest possible  lap
                 
                    break;    
            case 6: r = 0; g = 0; b=0;
                    
                    break;     
            default : break;
        }
        //pc.printf("%f %f %f %f \r\n",leftspeed.speeds[0],leftspeed.speeds[1],leftspeed.speeds[2],leftspeed.speeds[3]);
        
    }
    
    r = b = g = 1;
    
    g = 0; wait(0.2); g = 1; wait(0.2); g = 0; wait(0.2); g = 1; wait(0.2); g = 0; wait(0.2);  // 3 times blink
    
    ticker.attach_us(&drive_crossing_pid_motors,deltat*1000000);


    //readcamera();
    while(1){}     
    
} 



void drive_crossing_pid_motors(){
    
    if(TFC_LineScanImageReady){
        
        
        copycam(TFC_LineScanImage0,cam);                // copy the array
        TFC_LineScanImageReady = 0;                     // begin next conversion ...
        diff(cam,camdiff);
        
     
        //countlines(camdiff,0, 125, BlackLineLeft, BlackLineRight,nbbiglines);
        
        BlackLineLeft = blineleft(camdiff);
        BlackLineRight = blineright(camdiff);
        
        /*if (!(BlackLineRight > 120 && BlackLineLeft < 8)) {// i am not on intersection 
            if(history[0][1] <70 && history[1][1] <70 && BlackLineLeft>58) { BlackLineRight = BlackLineLeft; BlackLineLeft = 0;}
            else if(history[0][0] >58 && history[0][0] >58 && BlackLineRight<70) { BlackLineLeft = BlackLineRight; BlackLineRight = 128;}
        }*/
        
        //BlackLineLeft = blineleft(camdiff,2,64);
        //BlackLineRight = blineright(camdiff,64,126);
        
        /*
        if(!finished){
            if(nbbiglines > 3){ 
                inslowdownarea = true;// begin the slow down area
                speed = maxspeed/1.5; // divide the speed by 2
                ratiospeed = 0;
                KS = 0;
            }
            else if(nbbiglines > 1 && inslowdownarea && crossingslowdown){ // crossed the slow down area...
            inslowdownarea = false;
            finished = true;
            speed = maxspeed ; // return speed to normal values...
            ratiospeed = 0;
            KS = 0;
            }
            else {crossingslowdown = true;}// detecting 0 big lines => i am in the middle of slow down area
            
        }
        
     */
        RoadMiddle = (BlackLineLeft + BlackLineRight)/2;
        
        // if a line is only on the the left side
        if (BlackLineRight > 126){
            RoadMiddle = BlackLineLeft + 53; //46 on long rod
        }
        // if a line is only on the the right side
        if (BlackLineLeft < 2)
        {
            RoadMiddle = BlackLineRight - 53;
        }
        // if no line on left and right side
        if ((BlackLineRight > 126 && BlackLineLeft < 2)) 
        {

            onintersection = true; integration = 0;
            copycam(TFC_LineScanImage1,cam);
            diff(cam,camdiff);
            
            
            
            countlines(camdiff,10, 112, BlackLineLeft, BlackLineRight, nbbiglines);

            //BlackLineLeft = blineleft(camdiff);
            //BlackLineRight = blineright(camdiff);

            RoadMiddle = (BlackLineLeft + BlackLineRight)/2; // according to the 2nd camera => use another PID
            
            // if a line is only on the the left side
            if (BlackLineRight > 120){
                RoadMiddle = BlackLineLeft + 42;
            }
            // if a line is only on the the right side
            if (BlackLineLeft < 8)
            {
                RoadMiddle = BlackLineRight - 42;
            }

            
            // must include correction in case 1 line is detected...
            
            if ((BlackLineRight > 120) && (BlackLineLeft < 8)){ // narrower field of view 
                //both cameras don't see
                TFC_SetServo(1,zero_servo_position); // go slightly to the left
                motorcontrol(intersectionspeed,intersectionspeed);
                return;
                }

            
        }
        else onintersection = false;
            // the central camera sees the lines...
             
                  // Find difference from real middle
        Error = target - RoadMiddle ; // servo stears 0 to 2 (-1;1)or (-45,45)  calculate actual difference
        
        
        
        // plausibility check
        if (abs (Error - history[0][2]) > 40)  { Error = history[0][2]; }

        //if( integration > 1.8 && history[0][2] > 0 )
        integration += Error*deltat;
        float integ = KI * integration ;        
        derivation  =    (Error - history[0][2])/deltat ;
        derivation2 =    (Error - 2*history[0][2]+history[1][2])/deltat;
        derivation3 =    (Error - 3*history[0][2]+3*history[1][2] - history[2][2])/deltat;
           
        if(integ> integrationlimit) integration = integrationlimit/KI;                       // implementing non linearity on the integration part of the loop controle
        else if(integ < -integrationlimit) integration = -integrationlimit/KI; 
        
        
        
        if(onintersection)servo_position =0.5*(KP*Error + KD*derivation);// + KD*derivation + KI * integration; //2*kp because the camera gives a much narrower field of view !! reminder don't accept values above/below certian limits // -1 < servo_position < 1
        else servo_position = KP*Error + KD*derivation + KDD*derivation2 + KD3*derivation3 +  KI * integration; // -1 < servo_position < 1
         
        if(servo_position > 1 ) servo_position = 1;
        else if(servo_position <-1 ) servo_position = -1;
        
        push(BlackLineLeft,BlackLineRight,Error,servo_position);// push current frame
        
        TFC_SetServo(1,servo_position + zero_servo_position );
        
         maxrecentservo_pos = meaninarray(servo_positions, 5);
         blue.printf("%f %f %f %f %f %d\r",servo_position, KP*Error, KD * derivation , KDD*derivation2, KI * integration , RoadMiddle);
        
        
        motorcontrol(speed * (1 - servo_position * ratiospeed - slowratio * maxrecentservo_pos ),
                     speed *( 1 + servo_position * ratiospeed - slowratio * maxrecentservo_pos));  
        
        
        
       /*
       if(!onintersection ) blue.printf("l: %d ;; r: %d ;;\r\n", BlackLineLeft,BlackLineRight);
       else blue.printf("far: l: %d ;; r: %d ;;\r\n", BlackLineLeft,BlackLineRight);
        */
        
    } 
    
}






void readablearrtopc( uint16_t in[]){
    pc.printf("\r\n");
    
    for( int i =0; i<128; i++) {
        pc.printf("%d,",in[i]); 
    }  
}
void readablearrtoblue( uint16_t in[]){
    for( int i =0; i<128; i++) {
        blue.printf("%d,",in[i]); 
    }  
}
void arrtopc(uint16_t in[]){ 
    int div = 16;// sending info on 8 states; cahnge to 32 if conversion on 8 bit, 512 if conversion on 12 bit
    pc.printf("\r\n");
    
    for(int i=0; i<128; i++) {
        pc.printf("%c",in[i]/div); 
    }      
}

void arrtoblue(uint16_t in[]){ 
    int div = 16;// sending info on 8 states; cahnge to 32 if conversion on 8 bit, 512 if conversion on 12 bit
    
    
    for(int i=0; i<128; i++) {
        blue.printf("%c,",in[i]/div); 
    }      
}

void readcamera(){
   
      while(1){  
        if(TFC_LineScanImageReady>0) {
            g=0;  r = !r;
            TFC_LineScanImageReady = 0;
            copycam(TFC_LineScanImage0,cam);
            readablearrtoblue(cam);
            copycam(TFC_LineScanImage1,cam);
            readablearrtoblue(cam);
            blue.printf("\r\n");
        }
    }

}

void push(int left,int right,int error, float servo_pos){
    /*for (int i = HISTORY_LENGTH -1; i>=1;i--)
    {   
        history[i][0] = history[i-1][0];
        history[i][1] = history[i-1][1];
        history[i][2] = history[i-1][2];
        servo_positions[i] = servo_positions[i-1]; 
    }memmove(history+1,history,(HISTORY_LENGTH-1)*2*sizeof(float));*/
    
    memmove(history+1,history,(HISTORY_LENGTH-1)*3*sizeof(int));
    memmove(servo_positions+1,servo_positions,(HISTORY_LENGTH-1)*sizeof(float));
    history[0][0] = left;
    history[0][1] = right;
    history[0][2] = error;
    servo_positions[0] = servo_pos;
}

/* void dist(int distance){
    //put code here to happen when the distance is changed
    blue.printf("Distance changed to %dmm\r\n", distance);
    if(distance < 1500 && !rightside) {rightside = true; target = 40;}
}
*/



/*

void drive_crossing(){
    
    if(TFC_LineScanImageReady){
        
        
        copycam(TFC_LineScanImage0,cam);                // copy the array
        TFC_LineScanImageReady = 0;                     // begin next conversion ...
        diff(cam,camdiff);
        
        //BlackLineRight = blineright(camdiff,64,122);
        //BlackLineLeft = blineleft(camdiff,64,5);
        

     
        countlines(camdiff,0, 112, BlackLineLeft, BlackLineRight,nbbiglines);
        
        //blue.printf("biglines: %d ;;;; near, left: %d ;;;; near, right: %d \r\n",nbbiglines,BlackLineLeft,BlackLineRight);
        
        if(!finished){
            if(nbbiglines > 3){ 
                inslowdownarea = true;// begin the slow down area
                speed = maxspeed/2 ; // divide the speed by 2
                ratiospeed = speed/3;
                KS = speed / 2;
            }
            else if(nbbiglines > 1 && inslowdownarea && crossingslowdown){ // crossed the slow down area...
            inslowdownarea = false;
            finished = true;
            speed = maxspeed ; // return speed to normal values...
            ratiospeed = speed/3;
            KS = speed / 2;
            }
            else {crossingslowdown = true;}// detecting 0 big lines => i am in the middle of slow down area
            
        }
      //  blue.printf("countlines: %d ;;;; near, left: %d ;;;; near, right: %d \r\n",nbbiglines,BlackLineLeft,BlackLineRight);
     
        RoadMiddle = (BlackLineLeft + BlackLineRight)/2;
        
        // if a line is only on the the left side
        if (BlackLineRight > 120){
            RoadMiddle = BlackLineLeft + 64;
        }
        // if a line is only on the the right side
        if (BlackLineLeft < 5)
        {
            RoadMiddle = BlackLineRight - 64;
        }
        // if no line on left and right side
        if ((BlackLineRight > 120 && BlackLineLeft < 6)) 
        {
            //RoadMiddle = 64;
            onintersection = true;
            copycam(TFC_LineScanImage1,cam);
            diff(cam,camdiff);
            
            // blue.printf("far, left: %d ;;;; far, right: %d \r\n",BlackLineLeft,BlackLineRight);
            
            countlines(camdiff,0, 112, BlackLineLeft, BlackLineRight, nbbiglines);
        
           // blue.printf("countlines: %d ;;;; near, left: %d ;;;; near, right: %d \r\n",nbbiglines,BlackLineLeft,BlackLineRight);
            
            //BlackLineRight = blineright(camdiff,64,110);
            //BlackLineLeft = blineleft(camdiff,64,20);
            
            RoadMiddle = (BlackLineLeft + BlackLineRight)/2; // according to the 2nd camera => use another PID
            
            // if a line is only on the the left side
            if (BlackLineRight > 120){
                RoadMiddle = BlackLineLeft + 42;
            }
            // if a line is only on the the right side
            if (BlackLineLeft < 5)
            {
                RoadMiddle = BlackLineRight - 42;
            }

            
            // must include correction in case 1 line is detected...
            
            if ((BlackLineRight > 120) && (BlackLineLeft < 6)){ // narrower field of view 
                //both cameras don't see
                TFC_SetServo(0,1.2); // go slightly to the left
                TFC_SetMotorPWM(intersectionspeed,intersectionspeed);
                return;
                }

            
        }
        else {onintersection = false;}
            // the central camera sees the lines...
             
                  // Find difference from real middle
        Error = 64 - RoadMiddle ; // servo stears 0 to 2 (-1;1)or (-45,45)  calculate actual difference
        
        // plausibility check
        if (abs (Error - history[0][2]) > 40)   Error = history[0][2];

        //if( integration > 1.8 && history[0][2] > 0 )
        integration += Error*deltat;
        float integ = KI * integration ;        
        derivation  =    (Error - history[0][2])/deltat ;
        derivation2 =    (Error - 2*history[0][2]+history[1][2])/deltat;
        
           
        if(integ> integrationlimit) integration = integrationlimit/KI;                       // implementing non linearity on the integration part of the loop controle
        else if(integ < -integrationlimit) integration = -integrationlimit/KI; 
        
        
        
        if(onintersection)servo_position =1.5*KP*Error + KD*derivation + KI * integration; //2*kp because the camera gives a much narrower field of view !! reminder don't accept values above/below certian limits // -1 < servo_position < 1
        else servo_position = KP*Error + KD*derivation + KDD*derivation2 + KI * integration; // -1 < servo_position < 1
         
        if(servo_position > 1 ) servo_position = 1;
        else if(servo_position <-1 ) servo_position = -1;
        
        push(BlackLineLeft,BlackLineRight,Error,servo_position);// push current frame
        
        TFC_SetServo(0,servo_position + 1 );
        
         maxrecentservo_pos = meaninarray(servo_positions, 10);
         //blue.printf("%f %f %f %f %f %d\r",servo_position, KP*Error, KD * derivation , KDD*derivation2, KI * integration , RoadMiddle);

         TFC_SetMotorPWM(speed - maxrecentservo_pos * KS - servo_position * ratiospeed,
                         speed - maxrecentservo_pos * KS + servo_position * ratiospeed); // lower speed + differenctial
    
    
    } 
    
}
*/

/*
void drive(){
    
    
    if(TFC_LineScanImageReady){
        
        
        copycam(TFC_LineScanImage0,cam);                // copy the array
        TFC_LineScanImageReady = 0;                     // begin next conversion ...
        diff(cam,camdiff);
        
        //BlackLineRight = blineright(camdiff,64,122);
        //BlackLineLeft = blineleft(camdiff,64,5);
        
        
      //  blue.printf("near, left: %d ;;;; near, right: %d \r\n",BlackLineLeft,BlackLineRight);
     
        countlines(camdiff,0, 112, BlackLineLeft, BlackLineRight,nbbiglines);
        
      //  blue.printf("countlines: %d ;;;; near, left: %d ;;;; near, right: %d \r\n",nbbiglines,BlackLineLeft,BlackLineRight);
     
        RoadMiddle = (BlackLineLeft + BlackLineRight)/2;
        
        // if a line is only on the the left side
        if (BlackLineRight > 120){
            RoadMiddle = BlackLineLeft + 64;
        }
        // if a line is only on the the right side
        if (BlackLineLeft < 5)
        {
            RoadMiddle = BlackLineRight - 64;
        }
        // if no line on left and right side
        if ((BlackLineRight > 120) && (BlackLineLeft < 6))
        {
            //RoadMiddle = 64;
            onintersection = true;
            copycam(TFC_LineScanImage1,cam);
            diff(cam,camdiff);
            
            // blue.printf("far, left: %d ;;;; far, right: %d \r\n",BlackLineLeft,BlackLineRight);
            
            countlines(camdiff,0, 112, BlackLineLeft, BlackLineRight, nbbiglines);
        
           // blue.printf("countlines: %d ;;;; near, left: %d ;;;; near, right: %d \r\n",nbbiglines,BlackLineLeft,BlackLineRight);
            
            //BlackLineRight = blineright(camdiff,64,110);
            //BlackLineLeft = blineleft(camdiff,64,20);
            
            RoadMiddle = (BlackLineLeft + BlackLineRight)/2; // according to the 2nd camera => use another PID
            
            // if a line is only on the the left side
            if (BlackLineRight > 120){
                RoadMiddle = BlackLineLeft + 42;
            }
            // if a line is only on the the right side
            if (BlackLineLeft < 5)
            {
                RoadMiddle = BlackLineRight - 42;
            }

            
            // must include correction in case 1 line is detected...
            
            if ((BlackLineRight > 120) && (BlackLineLeft < 6)){ // narrower field of view 
                //both cameras don't see
                TFC_SetServo(0,1.2); // go slightly to the left
                TFC_SetMotorPWM(intersectionspeed,intersectionspeed);
                return;
                }

            
        }
        else {onintersection = false;}
            // the central camera sees the lines...
             
                  // Find difference from real middle
        Error = 64 - RoadMiddle ; // servo stears 0 to 2 (-1;1)or (-45,45)  calculate actual difference
        
        // plausibility check
        if (abs (Error - history[0][2]) > 40)   Error = history[0][2];

        //if( integration > 1.8 && history[0][2] > 0 )
        integration += Error*deltat;
        float integ = KI * integration ;        
        derivation  =    (Error - history[0][2])/deltat ;
        derivation2 =    (Error - 2*history[0][2]+history[1][2])/deltat;
        
           
        if(integ> integrationlimit) integration = integrationlimit/KI;                       // implementing non linearity on the integration part of the loop controle
        else if(integ < -integrationlimit) integration = -integrationlimit/KI; 
        
        
        
        if(onintersection)servo_position =1.5*KP*Error + KD*derivation + KI * integration; //2*kp because the camera gives a much narrower field of view !! reminder don't accept values above/below certian limits // -1 < servo_position < 1
        else servo_position = KP*Error + KD*derivation + KDD*derivation2 + KI * integration; // -1 < servo_position < 1
         
        if(servo_position > 1 ) servo_position = 1;
        else if(servo_position <-1 ) servo_position = -1;
        
        push(BlackLineLeft,BlackLineRight,Error,servo_position);// push current frame
        
        TFC_SetServo(0,servo_position + 1 );
        
         maxrecentservo_pos = meaninarray(servo_positions, 20);
         //blue.printf("%f %f %f %f %f %d\r",servo_position, KP*Error, KD * derivation , KDD*derivation2, KI * integration , RoadMiddle);
        
         TFC_SetMotorPWM(speed - maxrecentservo_pos * KS - servo_position * ratiospeed,
                         speed - maxrecentservo_pos * KS + servo_position * ratiospeed); // lower speed + differenctial
    
         
    } 
    
}
     

*/

  /*
    while(1){
        //blue.printf("%d\r\n", keypressed);
        
        switch(keypressed){
            case 1: readcamera(); break;
            case 2: break;
            case 3: break;
            case 4: break;
            case 5: break;
            case 6: break;
            default: break;  
        }

        keypressed = readkey();
        
        while(keypressed == 1){keypressed = readkey();}
   
    }
    */
   



 
 
 
 
 // call this in the main function once... 
    //mu.startUpdates();//start mesuring the distance
    
    /*
    mu.checkDistance();     //call checkDistance() as much as possible, as this is where
                                //the class checks if dist needs to be called.
    */
 
 
 
 
   
// stable values
/*float maxspeed = 0.25;
float Ku=1.0f/32;
float KP =0.3*Ku; //0.5  ; 0.2;
float KD =0.1*Ku; //0.1  ;0.1
float KI = 0.15*Ku;
float KS = maxspeed /2; // speed lower 
float ratiospeed = maxspeed /3;  // ratio of the differential on a turn...  add/subtract to/from left/right motor's pwm
float intersectionspeed = 0.075;// != maxspeed/2;*/
  
 /* float maxspeed = 0.3;
float ratiospeed = maxspeed/3 ;  // ratio of the differential on a turn...  add/subtract to/from left/right motor's pwm
float intersectionspeed = 0.075;// != maxspeed/2;
float integrationlimit = 0.5;
float Ku=1.0f/32;
float KP =0.3*Ku; //0.5  ; 0.2;
float KD =0.15*Ku; //0.1  ;0.1
float KDD = 0.11*Ku; // temporary... this is KDD/deltat
float KI = 2*Ku;
float KS = maxspeed /2; // speed lower 
    */