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
int intRx[4][4];
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
int startMes = 400;




void setup() {
  Serial.begin(9600);
}

void my_communicate(int i, bool do_print)
//------------- prints the current commands to the servos, sends the commands and then prints the message received from the servo-----
{
  if(do_print){
    // print output
    for (int k=0; k < 4; k++){
      Serial.print(objDaisy[i].outputByte[k]);
      Serial.print(",");
    }
    for (int k=0; k < 4; k++){
      Serial.print(objDaisy[i].moduleType[k]);
      Serial.print(",");
    }  
  }
  objDaisy[i].communicate();
  //delay(5);
  // print input
  if(do_print){
    Serial.print(objDaisy[i].inputByte);
    Serial.println();
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------

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
    Serial.print("identifying : ");
    Serial.println();
    for (int i=0; i < 2 && !correct_connection; i++){
      Serial.print(i);
      Serial.print(" : ");
      my_communicate(i, true);
      Serial.println();  
      correct_connection = is_connected_correctly();
    }
  }
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


  bool checkSum = false;
  while (!checkSum) {
    while (Serial.available()) {  //read string from $ to @
      int calcSum = 0;
      int recSum = -1; 
      int intSearch = Serial.parseInt();

      if (intSearch == startMes) { // if received the start int start reading message
        Serial.print("Listen :");
        for (int i=0; i<4; i++) { // read 16 integers
          for (int j=0; j<4; j++) {
            intRx[i][j] = Serial.parseInt();
            Serial.print(intRx[i][j]);
            Serial.print(',');          
            calcSum +=intRx[i][j];  // calculates the sum of incoming message       
          }
        }
        Serial.print(calcSum);
        recSum = Serial.parseInt(); // read the 17th integer- this the the checksum
        Serial.println(recSum);
        if (recSum == calcSum){ // if checksum isn't good start reading looking for start int again
          checkSum = true;
          newCommand=1;
        }
      }     
    }
  }

  //----------------------------------------------------------------------------------------------

  //--------------------smooth motion between current and next position---------------------------
  if (newCommand==1) {
    for (int i=0; i < 2; i++){  //calculate trajectory profile parameters
      for (int j=0; j < 4; j++){
        bytPosition[i][j]=(byte)intRx[i][j];
        Xf[i][j]=(float)bytPosition[i][j];
        Serial.print(i);
        Serial.print(",");
        Serial.print(j);
        Serial.print(" ==> ");
        Serial.print(intRx[i][j]);
        Serial.println();
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
    Serial.print("LIM: ");
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
        Serial.print(bytSense[i][j]);
        Serial.print(",");
      }  
    }    
    Serial.println();
  }
  //----------------------------------------------------------------------------------------------

}




