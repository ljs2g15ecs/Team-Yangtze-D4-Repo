/*  Basic Quadcopter controller with 2 analog sticks
 *  connected to the anlog read pins (1-4). 
 *  Sends data over spi to RF24 module with a delay.
 *  
 *  Analog Sticks | Arduino
 *        GND     |   GND
 *        5V      |   VCC
 *       VRx1     |   A1
 *       VRy1     |`  A2
 *       VRx2     |   A3
 *       VRy2     |`  A3
 *       
 *        RF24    | Arduino
 *        GND     |   GND
 *       3.3V     |   VCC
 *         CE     |    D9
 *         CS     |   D10
 *        SCK     |   D13
 *       MOSI     |   D11
 *       MISO     |   D12
 */

#include <SPI.h>
#include <RF24.h>

#define CE_PIN 9
#define CS_PIN 10

RF24 RDO(CE_PIN, CS_PIN);                       //  Initiase with CE and CS Pins

const byte address[5] = {'R','x','A','A','A'};  //  Addresses for data
byte data[4] = { 0, 0, 0, 0 };                  //  Data to be sent


void setup()
{
  Serial.begin(9600);
  Serial.write("\nSimple RF24 Master\n");

  RDO.begin();                                  //  Setup SPI

  RDO.setDataRate( RF24_250KBPS );              //  Set to min for longest range
  RDO.setRetries(3,5);                          //  Set delay and number of retries
  RDO.openWritingPipe( address );               //  Open data pipe
}

void loop()
{
  sendData();
  showData();
  delay(50);
}

void sendData()
{
  if( RDO.write( &data, sizeof(data) ) )        //  Check if data is sent
  {
    updateData();
  }
  else
  {
    Serial.write( "\n\nData transmission FAILED!\n" );
  }
}

void showData()
{
  Serial.write( "\nData Sent:\t" );
  int i;
  for(i=0; i<4; i++)
  {
    Serial.print( data[i] );
    Serial.write( "\t|\t" );
  }
}

void updateData()                                 //  Read the analog sticks and map to byte
{
  data[0] = map( analogRead(A1), 0, 1023, 0, 255);
  data[1] = map( analogRead(A2), 0, 1023, 0, 255);
  data[2] = map( analogRead(A3), 0, 1023, 0, 255);
  data[3] = map( analogRead(A4), 0, 1023, 0, 255);
}

