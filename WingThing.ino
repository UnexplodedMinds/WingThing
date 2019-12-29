/*
 * WingThing (c) 2019 Allen K. Lair, Sky Fun
 
  MPL3115A2 Altimiter I2C address is 0x60.
  BNO055 9DOF Orientation and Magnetometer sensor is 0x29 with 3V applied to ADR pin, otherwise 0x28.
  You will need to modify the BNO055 code in your Arduino library to use the pre-defined ADDRESS_B instead of ADDRESS_A.
  The 4525DO Pitot-Static sensor address is 0x28 - it is software configurable but a pain to change.
 */

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_MPL3115A2.h>


SoftwareSerial  Bluetooth( 10, 9 ); // RX, TX
char            btBuff[7];
int             btAvail = 0, btPos = 0;

Adafruit_BNO055 bno = Adafruit_BNO055( 55, 0x28 );
sensors_event_t orientationData, magData; // Note there is more data available from the BNO055 sensor but this is all we are interested in.
int8_t          boardTemp;
double          oX = 0.0, oY = 0.0, oZ = 0.0, mX = 0.0, mY = 0.0, mZ = 0.0;
double          head = 0.0, airspeed = 0.0;

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
float                     pressure = 0.0f, temp = 0.0f, altitude = 0.0f;

char     buff[4];
char     b1, b2, t1, t2;
uint16_t usBridge, usTemp;
int      iBuffPos;
float    pitotTemp = 0.0f;
float    pitotPressure = 0.0f;
float    seaPress = 101320.76;

int iCheck;

void setup()
{
  // Set the baud rate between the HC-06 and Arduino - higher baud rates will work but 9600 is fast enough and less prone to errors
  Bluetooth.begin( 9600 );
  delay( 100 );
  // Set the Bluetooth device name on the HC-06 module
  Bluetooth.print( "AT+NAMEWingThing" );

// Serial is only required for debugging for this project.
// Serial.begin( 9600 );

  // Initialize the BNO055 sensor. Flash the built-in LED very fast forever if it was not successful.
  if( !bno.begin() )
    flashLEDforever( 250 );

  // We are only interested in the orientation and magnetometer with this sensor so put it in that mode
  bno.setMode( Adafruit_BNO055::OPERATION_MODE_COMPASS );
  // Orientation for how it's mounted in the WingThing - if it's positioned differently this will need to be changed accordingly.
  bno.setAxisRemap( Adafruit_BNO055::REMAP_CONFIG_P2 );
  bno.setAxisSign( Adafruit_BNO055::REMAP_SIGN_P2 );

  delay( 100 );
}
 
void loop()
{
  // This is required for every sampling of the MPL3115A2.
  if( !baro.begin() )
    flashLEDforever( 2000 );

  String btString;

  // Get the barometric pressure set from the app, if sent.
  // Stratofier will send "NN.NN" with no qualifiers in exactly that format (without quotes) when it's set in the app.
  // The MPL3115A2 will use this to internally calibrate the altitude to the correct pressure altitude.
  btAvail = Bluetooth.available();
  btPos = 0;
  while( btAvail > 0 )
  {
    btBuff[btPos] = Bluetooth.read();
    btPos++;
    if( btPos > 5 )
      break;
  }
  if( btPos == 5 )
  {
    btBuff[5] = '\0';
    String newBaro( btBuff );
    seaPress = newBaro.toFloat() * 3386.39;
  }

  // There is other data available from this sensor but we are only interested in orientation and magnetic heading
  bno.getEvent( &orientationData, Adafruit_BNO055::VECTOR_EULER );
  bno.getEvent( &magData, Adafruit_BNO055::VECTOR_MAGNETOMETER );

  // Assign the data to their respective structs
  populate( &orientationData );
  populate( &magData );

  // Calculate the magnetic heading; 30 degrees seems to be a constant offset - may need adjustment or even a calibration table.  It's very sensitive.
  head = (atan2( mY, mX ) * 57.29578) - 30.0;
  if( head < 0.0 )
    head = 360.0 + head;

  // Get all the MPL3115A2 data
  baro.setSeaPressure( seaPress );
  pressure = baro.getPressure() / 3377.0;
  altitude = baro.getAltitude() * 3.28084;
  temp = baro.getTemperature(); // This is the temperature sent to Stratofier. pitotTemp below is used internally to calibrate the airspeed sensor.

  // Get the 4525DO sensor data
  // Note that thers is no Arduino library available for his sensor although there may be a compatible one for a sensor that uses the same hardware.
  // The one used in WingThing is from 3D Robotics.
  getPitotStatic();
  pitotTemp = ((static_cast<float>( usTemp ) * 200.0f) / 2047.0f) - 53.3f;  // 3.3 deg C offset from datasheet
  airspeed = (static_cast<float>( usBridge ) - 8192.0f) / 8192.0f * 200.0f; // Airspeed as a ratio of pressure count to pressure range - this very likely needs calibration.
                                                                            // It is assumed the count represents the full stated range of the sensor which is probably not accurate.
                                                                            // The equations in the datasheet cancel out so we're left with this simple calculation that probably needs tweaking.
  // Temperature multipliers for extremes of the operating range
  if( (pitotTemp < -15.0f) || (pitotTemp > 97.0f) )
    airspeed *= 1.25f;
  else if( (pitotTemp < -25.0f) || (pitotTemp > 115.0f) )
    airspeed *= 1.5f;
  // No backward flying :)
  if( airspeed < 0.0f )
    airspeed = 0.0f;

  // Send the data
  btString = String( airspeed, 2 );
  btString.concat( "," );
  btString.concat( String( altitude, 2 ) );
  btString.concat( "," );
  btString.concat( String( head, 2 ) );
  btString.concat( "," );

  // Barometric pressure placeholder - we could use the uncalibrated altimiter reading here or
  // a reliable pressure sensor.  Good ones are actually pretty expensive.  For now it's just a placeholder
  btString.concat( "-1," );

  btString.concat( String( oX, 4 ) );
  btString.concat( "," );
  btString.concat( String( oY, 4 ) );
  btString.concat( "," );
  btString.concat( String( oZ, 4 ) );
  btString.concat( "," );

  // Simple checksum
  iCheck = static_cast<int>( (airspeed + altitude + head + oX + oY + oZ) / 6.0 );

  btString.concat( String( iCheck ) );
  
  btString.concat( ";" );

  // Send the data over Bluetooth
  Bluetooth.print( btString );

  delay( 100 );
}

// Used for simple debugging without a terminal
void flashLEDforever( int iMS )
{
  pinMode( LED_BUILTIN, OUTPUT );
  while( true )
  {
    digitalWrite( LED_BUILTIN, HIGH );
    delay( iMS );
    digitalWrite( LED_BUILTIN, LOW );
    delay( iMS );
  }
}

// Fill in the data structs from the BNO055 sensor data
void populate( sensors_event_t *se )
{
  if( se->type == SENSOR_TYPE_ORIENTATION )
  {
    oX = se->orientation.x;
    oY = se->orientation.y;
    oZ = se->orientation.z;
  }
  else if( se->type == SENSOR_TYPE_MAGNETIC_FIELD )
  {
    mX = se->magnetic.x;
    mY = se->magnetic.y;
    mZ = se->magnetic.z;
  }
}

// Get the data from the 4525DO sensor (airspeed and temperature)
// This function is used in place of what would otherwise be done by a library but there doesn't appear to be one for this sensor.
void getPitotStatic()
{
  memset( buff, 0x00, sizeof( char ) * 4 );
  Wire.requestFrom( 0x28, 4 );
  iBuffPos = 0;
  while( Wire.available() )
  {
    buff[iBuffPos] = Wire.read();
    iBuffPos++;
  }

  // Bridge and Temp data in the 4-byte return
  b1 = buff[0];
  b2 = buff[1];
  t1 = buff[2];
  t2 = buff[3];

  b1 &= 0x3F;                                                     // Mask out the two high bits of the pressure
  t2 &= 0xE0;                                                     // Mask out the 5 low bits of the temperature
  usBridge = (static_cast<unsigned short int>( b1 ) << 8);
  usBridge |= (static_cast<unsigned short int>( b2 ) & 0x00FF);
  usTemp = ((static_cast<unsigned short int>( t1 ) << 8) | static_cast<unsigned short int>( t2 )) >> 5;
}
