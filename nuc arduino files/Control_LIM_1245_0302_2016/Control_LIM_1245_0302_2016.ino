#include <MeccaBrain.h>

//operates 4 modules with 4 motors through serial
//modified serial reading
#include <string.h>
#include <MeccaBrain.h>

int intDaisyPin[4] = {
  2,3,5,7};
int intServo[4] = {
  0,1,2,3};
MeccaBrain objDaisy[4]={
  MeccaBrain(intDaisyPin[0]),MeccaBrain(intDaisyPin[1]),MeccaBrain(intDaisyPin[2]),MeccaBrain(intDaisyPin[3])};
//byte bytPosition[4][4]={{0x75,0x89,0x84,0xE6},{0x7A,0xE7,0x7A,0x75},{0x0,0x0,0x0,0x0},{0x0,0x0,0x0,0x0}};// initial position of dinosaur
byte bytPosition[4][4]={
  {
    0x7A,0xEE,0x17,0x1B  }
  ,{
    0x84,0xE,0xE7,0xE3  }
  ,{
    0x0,0x0,0x0,0x0  }
  ,{
    0x0,0x0,0x0,0x0  }
};// initial position of meccanoid
byte basePosition[4][4]={
  {
    0x7A,0xEE,0x17,0x1B  }
  ,{
    0x84,0xE,0xE7,0xE3  }
  ,{
    0x0,0x0,0x0,0x0  }
  ,{
    0x0,0x0,0x0,0x0  }
};// initial position of meccanoid
int conductor[4][4]={
  {
    124,128,56,55  }
  ,{
    122,128,180,194  }
  ,{
    0,0,0,0  }
  ,{
    0,0,0,0  }
};

byte bytSense[4][4]={
  {
    0x58,0x58,0x58,0x58  }
  ,{
    0x58,0x58,0x58,0x58  }
  ,{
    0x58,0x58,0x58,0x58  }
  ,{
    0x58,0x58,0x58,0x58  }
};
int intpos[4][4];
int LIM=0;
int firstOp=0;
byte tempByte[4];

//char correctModule[4][4]={{'S','S','S','S'},{'S','S','S','S'},{'_','_','_','_'},{'_','_','_','_'}}; // robot configuration
char correctModule[4][4]={
  {
    'S','S','S','S'  }
  ,{
    'S','S','S','S'  }
  ,{
    '_','_','_','_'  }
  ,{
    '_','_','_','_'  }
}; // robot configuration


float Xi[4][4];
float Xf[4][4];
int Nsteps=15; //number of steps in each motion
int stepDelay=1; //delay between steps
float a[4][4];
float X1[4][4];
float X2[4][4];
float V[4][4];
float rise=0.5;
float t1;
float t2;
float X[4][4];
int newCommand=0; //changes to 1 when a message is received and the starts smooth motion 
unsigned long time;

bool noprint=false;
int changePos=0;




void setup() {
  Serial.begin(9600);
}


void loop() {
  check_connectivity();
  for (int i=0; i < 2; i++){
    my_communicate(i, false);
  }

  //------------------ sets the initial output angles to base posture--------------------------------
  if (firstOp == 0) {
    for (int i=0; i < 2; i++){
      for (int j=0; j < 4; j++){
        objDaisy[i].setServoPosition(intServo[j],bytPosition[i][j]);
      }
      my_communicate(i, true);
    }
    firstOp=firstOp+1;
  }

  //----------------------------------- read serial input ------------------------------------------

  for (int i=0; i < 2; i++){ //save the current angle commands before updating 
    for (int j=0; j < 4; j++){
      Xi[i][j]=(float)bytPosition[i][j];
    }
  }

  String str = "";
  while (Serial.available()) 
  {
    if (Serial.available()>0)
    {
      str = Serial.readStringUntil('\n');
    }
  }
  if (str != "")
  {
    int k = 0;
    int prev_comma = 0;
    int comma = str.indexOf(',');
    while(comma >= 0) 
    {
      int i = k / 4;
      int j = k % 4;
      String motor = str.substring(prev_comma, comma);
      if(motor.length()==0)
        intpos[i][j] = -1;
      else
        intpos[i][j] = atoi(motor.c_str());
      if (intpos[i][j]==300){
        LIM=1;  
      }
      if (intpos[i][j]==301){
        LIM=0;
      }
      bytPosition[i][j]=(byte)intpos[i][j];
      if(noprint){
        Serial.print(k);
        Serial.print("=");
        Serial.print(i);
        Serial.print(",");
        Serial.print(j);
        Serial.print(" ==> ");
        Serial.print(intpos[i][j]);
        Serial.println();
      }
      k++;
      prev_comma = comma + 1;
      comma = str.indexOf(',', prev_comma); 
    }
    int i = k / 4;
    int j = k % 4;
    String motor = str.substring(prev_comma);
    if(motor.length()==0)
      intpos[i][j] = -1;
    else
      intpos[i][j] = atoi(motor.c_str());
    if (intpos[i][j]==300){
      LIM=1;  
    }
    if (intpos[i][j]==301){
      LIM=0;
    }
    bytPosition[i][j]=(byte)intpos[i][j];
    if(noprint){
      Serial.print(k);
      Serial.print("=");
      Serial.print(i);
      Serial.print(",");
      Serial.print(j);
      Serial.print(" ==> ");
      Serial.print(intpos[i][j]);
      Serial.println();
    }

    for (int i=0; i < 2; i++){ //save the next angle commands  
      for (int j=0; j < 4; j++){
        Xf[i][j]=(float)bytPosition[i][j];
      }
    }
    newCommand=1;    
  }

  //----------------------------------------------------------------------------------------------
  //--------------------manualy set new command---------------------------------------------------
  if (changePos==1) {
    Serial.println("conductor");
    for (int i=0; i < 2; i++){ //save the next angle commands  
      for (int j=0; j < 4; j++){
        Xi[i][j]=(float)bytPosition[i][j];
        bytPosition[i][j]=(byte)conductor[i][j];
        Xf[i][j]=(float)bytPosition[i][j];
      }
    }
    changePos=0;
    newCommand=1;   
  }
  else {
    Serial.println("base");
    for (int i=0; i < 2; i++){ //save the next angle commands  
      for (int j=0; j < 4; j++){
        Xi[i][j]=(float)bytPosition[i][j];
        bytPosition[i][j]=basePosition[i][j];
        Xf[i][j]=(float)bytPosition[i][j];
      }
    }
    changePos=1;
    newCommand=1;       
  }




  //--------------------smooth motion between current and next position---------------------------
  if (newCommand==1) {
    for (int i=0; i < 2; i++){  //calculate trajectory profile parameters
      for (int j=0; j < 4; j++){
        t1=(float)Nsteps*rise;
        t2=(float)Nsteps-t1;
        a[i][j]=(Xf[i][j]-Xi[i][j])/(t1*((float)Nsteps-t1));
        X1[i][j]=Xi[i][j]+0.5*a[i][j]*t1*t1;
        V[i][j]=a[i][j]*t1;
        X2[i][j]=X1[i][j]+V[i][j]*(t2-t1);
      }
    }

    for (int t=0; t<=Nsteps; t++) {
      if (t<=t1) {
        for (int i=0; i < 2; i++){
          for (int j=0; j < 4; j++){
            X[i][j]=Xi[i][j]+0.5*a[i][j]*t*t;
            bytPosition[i][j]=(byte)X[i][j];             
            objDaisy[i].setServoPosition(intServo[j],bytPosition[i][j]);                    
          }  
        }    
      }
      else if (t<=t2){
        for (int i=0; i < 2; i++){
          for (int j=0; j < 4; j++){
            X[i][j]=X1[i][j]+V[i][j]*(t-t1);
            bytPosition[i][j]=(byte)X[i][j];
            objDaisy[i].setServoPosition(intServo[j],bytPosition[i][j]);   
          }  
        }
      }  
      else {
        for (int i=0; i < 2; i++){
          for (int j=0; j < 4; j++){
            X[i][j]=X2[i][j]+V[i][j]*(t-t2)-0.5*a[i][j]*(t-t2)*(t-t2);
            bytPosition[i][j]=(byte)X[i][j];
            objDaisy[i].setServoPosition(intServo[j],bytPosition[i][j]);   
          }  
        }

      }
      for (int i=0; i < 2; i++){    
        objDaisy[i].communicate2();

      }   
    }
    newCommand=0;
  }




  //---------------------------- send new commands to servos -------------------------------------

  if (LIM==0) {
    /*
    for (int i=0; i < 2; i++){
     for (int j=0; j < 4; j++){ 
     objDaisy[i].setServoPosition(intServo[j],bytPosition[i][j]);
     }
     }
     for (int i=0; i < 2; i++){
     
     for (int j=0; j < 4; j++){
     Serial.print(i+1);
     Serial.print(",");                 
     Serial.print(j+1);
     Serial.print(": ");  
     my_communicate(i);
     delay(70);
     Serial.println();
     }
     delay(70);        
     }   */
  }

  //-----------------------------------------------------------------------------------------------

  //------------------------------------- go to LIM mode ------------------------------------------
  else {
    if(noprint){
      Serial.print("LIM: ");
    }
    for (int i=0; i < 4; i++){
      for (int j=0; j < 4; j++){ 
        objDaisy[i].setServotoLIM(intServo[j]);
        my_communicate(i, true);
        delay(100);              
      }
    }


    for (int i=0; i < 4; i++){
      for (int j=0; j < 4; j++){ 
        bytSense[i][j]=objDaisy[i].getServoPosition(intServo[j]);
        delay(100);
        my_communicate(i, true);
        delay(100); 
        if(noprint){
          Serial.print(bytSense[i][j]);
          Serial.print(",");
        }
      }  
    } 
    if(noprint){   
      Serial.println();
    }
  }
  //----------------------------------------------------------------------------------------------

}

void my_communicate(int i, bool do_print)
//------------- prints the current commands to the servos, sends the commands and then prints the message received from the servo-----
{
  if(do_print){
    // print output
    for (int k=0; k < 4; k++){
      if(noprint){
        Serial.print(objDaisy[i].outputByte[k]);
        Serial.print(",");
      }
    }
    for (int k=0; k < 4; k++){
      if(noprint){
        Serial.print(objDaisy[i].moduleType[k]);
        Serial.print(",");
      }
    }  
  }
  objDaisy[i].communicate();
  //delay(5);
  // print input
  if(do_print){
    if(noprint){  
      Serial.print(objDaisy[i].inputByte);
      Serial.println();
    }
  }

}

bool is_connected_correctly(){
  bool correct_connection = true;
  for (int i=0; i < 4; i++){
    for (int j=0; j < 4; j++){
      if(objDaisy[i].moduleType[j]!=correctModule[i][j]){
        correct_connection = false;
      }      
    }
  }
  return correct_connection;
}

void check_connectivity(){
  //----Test if module types were identified correctly and if not run identification process again---
  bool correct_connection = is_connected_correctly();
  while(!correct_connection){
    if(noprint){
      Serial.print("identifying : ");
      Serial.println();
    }
    for (int i=0; i < 2 && !correct_connection; i++){
      if(noprint){
        Serial.print(i);
        Serial.print(" : ");
      }
      my_communicate(i, true);
      if(noprint){
        Serial.println();
      }
      correct_connection = is_connected_correctly();
    }
  }
}


//--------------------------------------------------------------------------------------------------------------------------------------




