#include "mbed.h"
#include "TFC.h"

void readablearrtopc(uint16_t in[]);
void camera();
void copycam(volatile uint16_t in[], uint16_t out[]);

Serial pc(USBTX, USBRX);

uint16_t cam[128];




void readablearrtopc( uint16_t in[]){
    pc.printf("\r\n");
    
    for( int i =0; i<128; i++) {
        pc.printf("%d,",in[i]); 
    }  
}
void camera(){
    
    if(TFC_LineScanImageReady){
        
        
        copycam(TFC_LineScanImage0,cam);  
        readablearrtopc(cam);
        }   



    

    //readcamera();
    while(1){
        camera();
        }     
    
} 