#include <Wire.h>

byte  QuadData[4];                        //  [0] = Throttle  | [1] = Roll  | [2] = Pitch | [3] = Yaw
int   xyz[3];                             //  Accelerometer Data
byte  XYZ[3];                             //  Mapped accelerometer data

volatile boolean s;

void setup()
{
  Serial.begin(9600);                     //  Setup serial with PC for debug
  Wire.begin();                           //  Setup I2C

  pinMode(A0, INPUT);                     //  Analog Read Pins
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  int i;
  for(i=0; i<4; i++)
  {
    QuadData[i] = 0;
  }

  //  Timer1 setup

  cli();                                  //  Disable interupts
  TCCR1A = 0;                             //  Set Registers to 0
  TCCR1B = 0;
  OCR1A = 62499;                          //  Prescaler for 1Hz

  TCCR1B |= (1 << WGM12);                 //  Turn on CTC mode and 256 prescaler
  TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  TIMSK1 |= (1 << OCIE1A);                //  Enable timer compare interupt
  sei();                                  //  Enable interupts

  s = false;

  pinMode(13, OUTPUT);
}

ISR(TIMER1_COMPA_vect)
{
  s = true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

void loop()
{
  recData();
  accelData();

  if(s)
  {
    Serial.print("\n\tSEND DATA!!!\t\n");
    printData();
    sendData();
    s = false;
  }
  
}

void recData()
{
  Wire.requestFrom(1,4);                  //  Request 4 bytes from address 1
  while(Wire.available() < 4);
  QuadData[0] = Wire.read();              //  Throttle
  QuadData[1] = Wire.read();              //  Roll
  QuadData[2] = Wire.read();              //  Pitch
  QuadData[3] = Wire.read();              //  Yaw
}

void sendData()
{
  Wire.beginTransmission(1);              //  Send to address 1

  Wire.write(XYZ, 3);                     //  Send 3 bytes

  Wire.endTransmission(1);
}

void printData()
{
  /*
  Serial.print("\nThrottle  = ");
  Serial.print(QuadData[0]);
  Serial.print("\t|\tRoll  = ");
  Serial.print(QuadData[1]);
  Serial.print("\t|\tPitch = ");
  Serial.print(QuadData[2]);
  Serial.print("\t|\tYaw = ");
  Serial.print(QuadData[3]);
  //*/

  ///*
  Serial.print( "\nX: " );
  Serial.print( XYZ[0] );
  Serial.print( "\tY: " );
  Serial.print( XYZ[1] );
  Serial.print( "\tZ: " );
  Serial.print( XYZ[2] );
  //*/
}

void accelData()
{
  xyz[0] = analogRead(A0);                  //  Read accelerometer
  xyz[1] = analogRead(A1);
  xyz[2] = analogRead(A2);

  XYZ[0] = map( xyz[0], 180, 420, 0, 255);  //  Map data to byte range
  XYZ[1] = map( xyz[1], 180, 420, 0, 255);
  XYZ[2] = map( xyz[2], 180, 420, 0, 255);
}

