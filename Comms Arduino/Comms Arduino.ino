#include <Wire.h>
#include <Servo.h>

//Maximum Number of chars per Packet
#define numChars 13

//IR sensors Pins
#define irFrontPin
#define irBackPin
#define irLeftPin
#define irRightPin

//Bluetooth State Pin
#define btStatePin 2

//Battery Voltage Divider Pin
#define batteryVoltPin 15
#define batteryMaxVoltage 8.5
#define batteryMinVoltage 5.5

//Values of the Different Parameters
#define Range 180 //Max value of the control signal (0-Range)
#define midPoint Range/2
#define throttleHover 0 //Value of the throttle at which the quad hovers

//Variables for Control
byte Controls[4]; // Throttle[0] Roll[1] Pitch[2] Yaw[3]

//Serial Variables
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;
char oneChar=0;

//Timing Variables
long batteryPrevMillis = -1000*10;
long batteryInterval = 1000*10;

//Bluetooth State Variable 
bool btDisconnected=0;
bool btConnected=0;

void setup() {
  pinMode(batteryVoltPin,INPUT);
  pinMode(btStatePin,INPUT);
  pinMode(13,OUTPUT);
  pinMode(irFrontPin,INPUT);
  pinMode(irBackPin,INPUT);
  pinMode(irLeftPin,INPUT);
  pinMode(irRighPin,INPUT);
  Wire.begin(1);
  Wire.onRequest(requestEvent);
  Serial.begin(9600);
}

void loop() {
  //unsigned long currentMillis = millis();
  //getBatteryVoltage(currentMillis);
  checkBluetooth();
  if(btDisconnected==0){
    recieveSerial();
    checknewData();
  }
}
//=============================***FUNCTIONS***=================================================================
void requestEvent(){ //I2C function to send controls array
  Wire.write(Controls,4);
}

void checkBluetooth(){
  bool btState = digitalRead(btStatePin);
  if(btState){
    btConnected = 1;
    btDisconnected = 0;
  }
  else if(btState==0 && btConnected){
    btDisconnected = 1;
    btConnected = 0;
    noSignal();    
  }
  else{
    btConnected=0;
    btDisconnected=0;
  }
}

void recieveSerial() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '@';
  char endMarker = '!';
  char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();     
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) { 
      recvInProgress = true;
    }
    else { //If it gets here then only once char was sent
      oneChar=rc;
      newData = true;
    }
  }
}

void checknewData(){
  if (newData == true) {
    if(oneChar){
      processOneChar();
      oneChar=0;
    }
    else{
      processPacket();     
    }
  newData = false; 
  }
}

//**********************SERIAL PROCESSING FUNCTIONS*************************************************
void processOneChar(){
  switch(oneChar){         
  }
}

void processPacket(){
  strcpy(tempChars, receivedChars);
  char identifier = receivedChars[0];
  switch(identifier){
      break;
    case 'L': //leftstick
      parseXYpad(Controls[3],Controls[0]);
      break;
    case 'R': //rightstick
      parseXYpad(Controls[1],Controls[2]); 
      break;
  }
}

//************************PACKET PARSING FUNCTIONS*******************************************************************
void parseXYpad(byte &x,byte &y){ //Format: <*X###Y###>
  char * strIndx; 
  strIndx = &tempChars[1];     
  strIndx = strtok(strIndx+1, "Y");
  x = atoi(strIndx);    
  strIndx = strtok(NULL, " ");   
  y = atoi(strIndx);   
}

void parseSlider(int &value){ //Format <*###>
  char * strIndx;
  strIndx = &tempChars[1];
  value = atoi(strIndx);
}

//*****************************TIMING FUNCTIONS**********************************************************
byte getBatteryVoltage(unsigned long &currentMillis){
  if(currentMillis-batteryPrevMillis>=batteryInterval){
    batteryPrevMillis=currentMillis;
    float Vin;
    int  voltage;
    Vin = analogRead(batteryVoltPin);
    Vin= (Vin/1023)*10;
    Vin= (Vin-batteryMinVoltage)/(batteryMaxVoltage-batteryMinVoltage);
    Vin= Vin*100;
    voltage = (int)Vin;
    if(voltage<0){
      voltage=0;
    }
    if(voltage<=17){
     // Serial.print("*D*");   
    }
    //Serial.print("*V"+String(voltage)+'*');
  }     
}

//******************************AUTO-FUNCTIONS*****************************************************************************
void noSignal(){
  digitalWrite(13,btState);
  Controls[1]=Range/2;
  Controls[2]=Range/2;
  Controls[3]=Range/2;
  if(Controls[0]>(throttleHover*2)){
    Controls[0]=Controls[0]-2;
    delay(50);    
  }
  if(Controls[0]>throttleHover){
    Controls[0]=Controls[0]-1;
    delay(50);    
  }
}

void AutoLand(){
  Controls[1]=Range/2;
  Controls[2]=Range/2;
  Controls[3]=Range/2;
  if(Controls[0]>(throttleHover*2)){
    Controls[0]=Controls[0]-2;
    delay(50);    
  }
  while(Controls[0]>throttleHover){
    Controls[0]=Controls[0]-1;
    delay[50];
  }
}

void processirSensors(){
  bool irFront,irBack,irLeft,irRight;
  irFront=digitalRead(irFrontPin);
  irBack=digitalRead(irBackPin);
  irLeft=digitalRead(irLeftPin);
  irRight=digitalRead(irRightPin);
  
  if(irFront && irBack && irLeft && irRight){
    AutoLand();    
  }
  else if(irFront && irBack && irRight){
    evadeLeft();
  }
  else if(irFront && irBack && irLeft){
    evadeRight();
  }
  else if(irFront && irLeft){
    evadeBackRight();
  }
  else if(irFront && irRight){
    evadeBackLeft();
  }
  else if(irBack && irLeft){
    evadeFrontRight();
  }
  else if(irBack && irRight){
    evadeFrontLeft();
  }
  else if(irLeft){
    evadeRight();
  }
  else if(irRight){
    evadeLeft();
  }
}

//*****************************SEMI AUTO CONTROL FUNCTIONS****************************************************************************
void processControls(){
  bool irFront,irBack,irLeft,irRight;
  int upperLimit,lowerLimit;

  upperLimit = midPoint+Range/6;
  lowerLimit = midPoint-Range/6;
  
  irFront=digitalRead(irFrontPin);
  irBack=digitalRead(irBackPin);
  irLeft=digitalRead(irLeftPin);
  irRight=digitalRead(irRightPin);
  
  if(Controls[1]>upperLimit){
    Controls[1] = upperLimit;
  }
  else if(Controls[1]<lowerLimit){
    Controls[1] = lowerLimit;
  }
  if(Controls[2]>upperLimit){
    Controls[2] = upperLimit;
  }
  else if(Controls[2]<lowerLimit){
    Controls[2] = lowerLimit;
  }
    
  if(irFront && irBack && irLeft && irRight){
    Controls[1] =  midPoint;
    Controls[2] =  midPoint;
    Controls[3] =  midPoint;   
  }
  else if(irFront && irBack && irRight){
    (Controls[1]>midPoint)? Controls[1]=midPoint;
  }
  else if(irFront && irBack && irLeft){
    (Controls[1]<midPoint)? Controls[1]=midPoint;
  }
  else if(irFront && irLeft){
    (Controls[1]<midPoint)? Controls[1]=midPoint;
    (Controls[2]<midPoint)? Controls[2]=midPoint;
  }
  else if(irFront && irRight){
    (Controls[1]>midPoint)? Controls[1]=midPoint;
    (Controls[2]<midPoint)? Controls[2]=midPoint;
  }
  else if(irBack && irLeft){
    (Controls[1]<midPoint)? Controls[1]=midPoint;
    (Controls[2]>midPoint)? Controls[2]=midPoint;
  }
  else if(irBack && irRight){
    (Controls[1]>midPoint)? Controls[1]=midPoint;
    (Controls[2]>midPoint)? Controls[2]=midPoint;
  }
  else if(irLeft){
    (Controls[1]<midPoint)? Controls[1]=midPoint;
  }
  else if(irRight){
    (Controls[1]>midPoint)? Controls[1]=midPoint;
  }  
}
