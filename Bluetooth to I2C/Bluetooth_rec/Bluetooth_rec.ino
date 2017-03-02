/*  Code Adapted from Robin2 on Arduino Forums

*/

#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial BT(2, 3);                //  Pin 2 = RX  | Pin 3 = TX 

#define numChars 32                     //  Number of bytes sent by BT
char receivedChars[numChars];           //  Received string
char tempChars[numChars];               //  Temporary string for manipulation

boolean newData = false;

struct XY
{
  byte x;
  byte y;
} A;

byte QuadData[4];                       //  [0] = Throttle  | [1] = Roll  | [2] = Pitch | [3] = Yaw
byte XYZ[3];                            //  Accelerometer data

void setup()
{
  Serial.begin(9600);                   //  Setup serial with PC for debug
  Serial.println("<Arduino is ready>");

  Wire.begin(1);
  Wire.onRequest(requestEvent);         //  Register data request event
  Wire.onReceive(receiveEvent);         //  Register data receive event

  BT.begin(9600);                       //  Setup BT Serial

  int i;
  for(i=0; i<4; i++)
  {
    QuadData[i] = 127;
  }
}

void loop()
{
  recvWithStartEndMarkers();
  showNewData();
}

void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';               //  Start marker for BT data
  char endMarker = '>';                 //  End marker for BT data
  char rc;

  // if( Serial.available() > 0 ) {
  while ( BT.available() > 0 && newData == false )
  {
    rc = BT.read();
    if ( recvInProgress == true )
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if ( rc == startMarker )
    {
      recvInProgress = true;
    }
  }
}

void showNewData()
{
  if (newData == true)
  {
    parseData();
    newData = false;
  }
}

void parseData()
{
  strcpy(tempChars, receivedChars);
  switch ( receivedChars[0] )
  {
    case 'L':
      parseXY();
      QuadData[0] = A.y;                //  Throttle
      QuadData[3] = A.x;                //  Roll
      break;
    case 'R':
      parseXY();
      QuadData[1] = A.y;                //  Pitch
      QuadData[2] = A.x;                //  Yaw
      break;
  }
}

void parseXY()                          //  !X###Y###
{
  char* str =  &tempChars[1];           //  !X {###} Y###
  str = strtok(str + 1, "Y");
  A.x = atoi(str);
  str = strtok(NULL, " ");              //  !X###Y  {###}
  A.y = atoi(str);
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

void requestEvent()
{
  Wire.write(QuadData, 4);                //  Send 4 byte array
}

void receiveEvent(int n)
{
  int i = 0;
  while( Wire.available() > 0 )           //  Receive 3 bytes
  {
    XYZ[i] = Wire.read();                 //  Store in XYZ array
    i++;
  }

  Serial.print("\n\tNEW DATA!!!\t\n");
  printData();

  sendData();
}

void sendData()
{
  BT.print( "\nX: " );
  BT.print( XYZ[0] );
  BT.print( "\tY: " );
  BT.print( XYZ[1] );
  BT.print( "\tZ: " );
  BT.print( XYZ[2] );
}

