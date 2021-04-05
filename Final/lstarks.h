
#include "mbed.h"

#define THRESHOLD               250     // must make it variable  
#define THRESHOLD_BLACK        1800
#define THRESHOLD_WHITE        3500 
#define THRESHOLD_WHITE2       2500     
#define KDP                     25
#define HISTORY_LENGTH          30
#define CRITICAL_LEFT_LINE      57
#define CRITICAL_RIGHT_LINE     71

// prototyping... :( :(
//void arrtopc( uint16_t in[]);
int readkey();



//void arrtoblue(uint16_t in[]);

void copycam(volatile uint16_t in[], uint16_t out[]);

void diff(uint16_t in[], uint16_t out[]);

int blineright(uint16_t in[]);

int blineleft(uint16_t diff[]);

/*
int blineright(uint16_t in[],int begin, int end);

int blineleft(uint16_t diff[],int begin, int end);
*/
int checkcurve(int history[HISTORY_LENGTH][3], int maxindex);

bool checkcriticalcase(int history[HISTORY_LENGTH][3], int &BL, int &BR, int maxindex);

void printdata(int BlackLineLeft,int BlackLineRight, float servo_position);

void lstarksinit();

float maximuminarray(float servo_positions[], int maxindex);

float meaninarray(float servo_positions[], int maxindex);


float ratioRL(float servo_position);

void countlines(uint16_t diff[],int begin, int end,int &BL, int &BR, int &nbBigLines );


int median(int a, int b, int c);

int min(int a, int b);

int max(int a, int b);

int checkForLines(uint16_t camera[]);

int countlargelines(uint16_t diff[]);
//////// end prototyping ////////



