import processing.serial.*;
 Serial usb,in,out;
 int baud=115200;
 int lf = 10;
 byte[] inBuffer = new byte[2000];
 String str;
 int[] data = new int[256];
 String[] arrstring;
 int div = 16;
int mul = 1;//32
void setup() {
   size(1280, 500);

  background(0);
  fill(255, 0, 0, 30);
  println(Serial.list());
  //usb=new Serial(this, Serial.list()[2], baud);
  //in=new Serial(this, Serial.list()[2], baud);
  in=new Serial(this, Serial.list()[0], baud); //com 9 is incoming not outgoing

}

void draw(){

  if (in.available() > 0) {
    in.readBytesUntil(lf, inBuffer);
    str = new String(inBuffer);
    //println(str);
    histogramreadable(str);
  }

}


void histogramreadable(String s){

  try{
  s = trim(s);
  
  arrstring = split(s,',');
  for (int i =0;i< data.length ; i++)
    data[i] =  Integer.parseInt(arrstring[i]);

  printArray(data);
  background(0);
  for (int x=0; x < 128 ; x++) {
    fill(mul*data[x]/div,mul*data[x]/div,mul*data[x]/div);//println(data[x]);
    rect(10*x, 50, 10, 50);
  }
 for (int x=128; x < 256 ; x++) {
    fill(mul*data[x]/div,mul*data[x]/div,mul*data[x]/div);//println(data[x]);
    rect(10*(x-128),200, 10, 50);
  }
  
  }catch(Exception e){println(e.getMessage());}
  
}

/*
void histogram(String s){
  try{
  s = trim(s);
  
  
  for (int i =0;i< s.length() -1 ; i++)
    data[i] =  (int)(s.charAt(i));
    //data[i] =  Character.getNumericValue(s.charAt(i));
   //println(min(data));
  
  
  printArray(data);
  background(0);
  for (int x=0; x < 128 ; x++) {
    fill( data[x],data[x],data[x]);//println(data[x]);
    rect(10*x, 50, 10, 50);
  }
  for (int x=128; x < 256 ; x++) {
    fill(data[x],data[x],data[x]);//println(data[x]);
    rect(10*(x-128),200, 10, 50);
  }
  
  }catch(Exception e){println(e.getMessage());}
  
}

*/ 
