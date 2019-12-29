// Altimiter is 0x60
// BNO055 is 0x29 with 3V, 0x28 without
// Pitot is 0x28

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
sensors_event_t orientationData, magData;
// sensors_event_t angVelocityData, linearAccelData;
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

int iCheck;

void setup()
{
  Bluetooth.begin( 9600 );
  delay( 100 );
  Bluetooth.print( "AT+NAMEWingThing" );
  Serial.begin( 9600 );

  if( !bno.begin() )
    flashLEDforever( 250 );

  bno.setMode( Adafruit_BNO055::OPERATION_MODE_COMPASS );
  bno.setAxisRemap( Adafruit_BNO055::REMAP_CONFIG_P2 );
  bno.setAxisSign( Adafruit_BNO055::REMAP_SIGN_P2 );

//  Wire.begin( 0x28 );

  delay( 100 );
}
 
void loop()
{
  if( !baro.begin() )
    flashLEDforever( 2000 );

  String btString;

  // Get the barometric pressure set from the app
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
    baro.setSeaPressure( newBaro.toFloat() * 3386.39 );
  }

  bno.getEvent( &orientationData, Adafruit_BNO055::VECTOR_EULER );
//  bno.getEvent( &angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE );
//  bno.getEvent( &linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL );
  bno.getEvent( &magData, Adafruit_BNO055::VECTOR_MAGNETOMETER );

  populate( &orientationData );
// populate( &angVelocityData );
// populate( &linearAccelData );
  populate( &magData );
  
  head = (atan2( mY, mX ) * 57.29578) - 30.0;
  if( head < 0.0 )
    head = 360.0 + head;

  baro.setSeaPressure( 101320.76 );
  pressure = baro.getPressure() / 3377.0;
  altitude = baro.getAltitude() * 3.28084;
  temp = baro.getTemperature();

  getPitotStatic();
  pitotTemp = ((static_cast<float>( usTemp ) * 200.0f) / 2047.0f) - 53.3f;  // 3.3 deg C offset from datasheet
  airspeed = (static_cast<float>( usBridge ) - 8192.0f) / 8192.0f * 200.0f; // Airspeed as a ratio of pressure count to pressure range
  // Temperature multipliers for extremes of the operating range
  if( (pitotTemp < -15.0f) || (pitotTemp > 97.0f) )
    airspeed *= 1.25f;
  else if( (pitotTemp < -25.0f) || (pitotTemp > 115.0f) )
    airspeed *= 1.5f;
  // No backward flying :)
  if( airspeed < 0.0f )
    airspeed = 0.0f;

  btString = String( airspeed, 2 );
  btString.concat( "," );
  btString.concat( String( altitude, 2 ) );
  btString.concat( "," );
  btString.concat( String( head, 2 ) );
  btString.concat( "," );

  // Barometric pressure placeholder
  btString.concat( "-1," );

  btString.concat( String( oX, 4 ) );
  btString.concat( "," );
  btString.concat( String( oY, 4 ) );
  btString.concat( "," );
  btString.concat( String( oZ, 4 ) );
  btString.concat( "," );

  iCheck = static_cast<int>( (airspeed + altitude + head + oX + oY + oZ) / 6.0 );

  btString.concat( String( iCheck ) );
  
  btString.concat( ";" );

  Bluetooth.print( btString );

  delay( 100 );
}

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
