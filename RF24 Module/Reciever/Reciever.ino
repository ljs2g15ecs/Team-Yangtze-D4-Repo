/*  Receives data from master with a RF24
 *  module using SPI
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
byte data[4] = { 0, 0, 0, 0 };                  //  Data to bereceived


void setup()
{
  Serial.begin(9600);
  Serial.write("\nSimple RF24 Master\n");

  RDO.begin();                                  //  Setup SPI

  RDO.setDataRate( RF24_250KBPS );              //  Set to min for longest range
  RDO.setRetries(3,5);                          //  Set delay and number of retries
  RDO.openReadingPipe( 1, address );            //  Open receive pipe
  RDO.startListening();
}

void loop()
{
  recData();
  showData();
}

void recData()
{
  if( RDO.available() )                         //  Wait for data to be sent
  {
    RDO.read( &data, sizeof(data) );            //  Store in data byte array
    newData = true;
  }
}

void showData()
{
  if( newData )
  {
    Serial.write( "\nData Received:\t" );
    int i;
    for(i=0; i<4; i++)
    {
      Serial.print( data[i] );
      Serial.write( "\t|\t" );
    }
    newData = false;
  }
}



