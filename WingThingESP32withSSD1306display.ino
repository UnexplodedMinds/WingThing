/*
  WingThing (c) 2019 Allen K. Lair, Sky Fun
 
  MPL3115A2 Altimiter I2C address is 0x60.
  BNO055 9DOF Orientation and Magnetometer sensor is 0x29 with 3V applied to ADR pin, otherwise 0x28.
  You will need to modify the BNO055 code in your Arduino library to use the pre-defined ADDRESS_B instead of ADDRESS_A.
  The 4525DO Pitot-Static sensor address is 0x28 - it is software configurable but a pain to change.
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPL3115A2.h>
#include <utility/imumaths.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiUdp.h>


Adafruit_BNO055 bno = Adafruit_BNO055( 55, 0x29 );

sensors_event_t magData; //, orientationData; // Note there is more data available from the BNO055 sensor but this is all we are interested in.
int8_t          boardTemp;
double          mX = 0.0, mY = 0.0, mZ = 0.0; //, oX = 0.0, oY = 0.0, oZ = 0.0;
double          head = 0.0, airspeed = 0.0;

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
float              pressure = 0.0f, temp = 0.0f, altitude = 0.0f;

uint8_t  btString[32];
char     pitotBuff[4];
char     dispString[64];
char     b1, b2, t1, t2;
uint16_t usBridge, usTemp;
int      iBuffPos;
float    pitotTemp = 0.0f;
float    pitotPressure = 0.0f;
float    seaPress = 101320.76;
int      iCheck;
bool     bHeart = true;


TwoWire          oled_i2c( 0 );
Adafruit_SSD1306 disp( 128, 64, &oled_i2c, -1 );
WiFiUDP          udp;
int              iFromStratofier = 0;


void setup()
{
  printf( "Starting\n" );

   // Built-in LED
  pinMode( LED_BUILTIN, OUTPUT );

  // Initialize the BNO055 sensor. Flash the built-in LED very fast forever if it was not successful.
  if( !bno.begin() )
    flashLEDforever( 250 );

  // We are only interested in the orientation and magnetometer with this sensor so put it in that mode
  bno.setMode( Adafruit_BNO055::OPERATION_MODE_COMPASS );
  // Orientation for how it's mounted in the WingThing - if it's positioned differently this will need to be changed accordingly.
  bno.setAxisRemap( Adafruit_BNO055::REMAP_CONFIG_P2 );
  bno.setAxisSign( Adafruit_BNO055::REMAP_SIGN_P2 );

  pinMode( 16, OUTPUT );
  digitalWrite( 16, LOW );    // set GPIO16 low to reset OLED
  delay( 50 );
  digitalWrite( 16, HIGH );   // while OLED is running, must set GPIO16 to high

  oled_i2c.begin( 4, 15 );
  disp.begin( SSD1306_SWITCHCAPVCC, 0x3C, true, true );
  disp.clearDisplay();
  disp.setTextSize( 1 );

  disp.setCursor( 0, 0 );
  disp.setTextColor( SSD1306_WHITE, SSD1306_BLACK );
  disp.print( "CONNECTING..." );
  disp.display();
  WiFi.begin( "stratux" );
  while( WiFi.status() != WL_CONNECTED )
    delay( 500 );
  disp.setCursor( 0, 24 );
  disp.print( "Stratux Connected" );
  disp.setCursor( 0, 48 );
  disp.print( "IP:" );
  disp.setCursor( 25, 48 );
  disp.print( WiFi.localIP() );
  disp.display();
  udp.begin( 45678 );
  delay( 2000 );

  disp.clearDisplay();
  disp.setCursor( 0, 0 );
  disp.setTextColor( SSD1306_BLACK, SSD1306_WHITE );
  disp.print( "SKY FUN - WINGTHING" );
  disp.setTextColor( SSD1306_WHITE, SSD1306_BLACK );
  disp.setCursor( 0, 12 );
  disp.print( "IAS:" );
  disp.setCursor( 0, 24 );
  disp.print( "ALT:" );
  disp.setCursor( 0, 36 );
  disp.print( "HED:" );
  disp.setCursor( 0, 50 );
  disp.print( "TMP/BAR:" );
}


void loop()
{
  if( !baro.begin() )
    flashLEDforever( 2000 );

  // Get the barometric pressure set from the app, if sent.
  // Stratofier will send "NN.NN" with no qualifiers in exactly that format (without quotes) when it's set in the app.
  // The MPL3115A2 will use this to internally calibrate the altitude to the correct pressure altitude.

  iFromStratofier = udp.parsePacket();
  if( iFromStratofier > 0 )
  {
    udp.read( dispString, iFromStratofier );
    seaPress = atof( dispString ) * 3386.39;
  }

  // There is other data available from this sensor but we are only interested in orientation and magnetic heading
  // bno.getEvent( &orientationData, Adafruit_BNO055::VECTOR_EULER );
  bno.getEvent( &magData, Adafruit_BNO055::VECTOR_MAGNETOMETER );

  // Assign the data to their respective structs
  // populate( &orientationData );
  populate( &magData );

  // Calculate the magnetic heading; 30 degrees seems to be a constant offset - may need adjustment or even a calibration table.  It's very sensitive.
  head = (atan2( mY, mX ) * 57.29578);
  if( head < 0.0 )
    head = 360.0 + head;

  // Get all the MPL3115A2 data
  baro.setSeaPressure( seaPress );          // seaPress is in Pascals, converted from inches Hg in the set routine above
  pressure = baro.getPressure() / 3377.0;   // Pressure in inches Hg
  altitude = baro.getAltitude() * 3.28084;  // Altitude in feet
  temp = baro.getTemperature();             // This is the temperature sent to Stratofier. pitotTemp below is used internally to calibrate the airspeed sensor.
                                            // Also note that 20 degrees is a fixed offset.  It may not be necessary when mounted outside; inside the sensor seems to artificially gradually climb.

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
  // No flying in the ground (some places are below sea level but you'd never fly there)
  if( altitude < 0.0f )
    altitude = 0.0f;

  // Simple checksum
  iCheck = static_cast<int>( (airspeed + altitude + head + temp) / 4.0 );

  // Send the data over UDP
  sprintf( dispString, "%.2f,%.2f,%.2f,-1,%.2f,%d;", airspeed, altitude, head, temp, iCheck );
  // printf( "%s\n", dispString );
  memcpy( btString, dispString, strlen( dispString ) );
  udp.beginPacket( "192.168.10.255", 45678 );
  udp.write( static_cast<uint8_t *>( btString ), strlen( dispString ) );
  udp.endPacket();
  
  // Blink the built-in LED as a heartbeat and 
  if( bHeart )
  {
    digitalWrite( LED_BUILTIN, HIGH );
    disp.begin( SSD1306_SWITCHCAPVCC, 0x3C, true, true );
    sprintf( dispString, "%.2f", airspeed );
    disp.setCursor( 50, 12 );
    disp.print( dispString );
    sprintf( dispString, "%.2f", altitude );
    disp.setCursor( 50, 24 );
    disp.print( dispString );
    sprintf( dispString, "%.2f", head );
    disp.setCursor( 50, 36 );
    disp.print( dispString );
    sprintf( dispString, "%.2f / %.2f", temp, seaPress / 3386.39 );
    disp.setCursor( 50, 50 );
    disp.print( dispString );
    disp.display();
  }
  else
    digitalWrite( LED_BUILTIN, LOW );

  bHeart = (!bHeart);
}

// Used for simple debugging without a terminal or display
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
  if( se->type == SENSOR_TYPE_MAGNETIC_FIELD )
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
  memset( pitotBuff, 0x00, sizeof( char ) * 4 );
  Wire.requestFrom( 0x28, 4 );
  iBuffPos = 0;
  while( Wire.available() )
  {
    pitotBuff[iBuffPos] = Wire.read();
    iBuffPos++;
  }

  // Bridge and Temp data in the 4-byte return
  b1 = pitotBuff[0];
  b2 = pitotBuff[1];
  t1 = pitotBuff[2];
  t2 = pitotBuff[3];

  b1 &= 0x3F;                                                     // Mask out the two high bits of the pressure
  t2 &= 0xE0;                                                     // Mask out the 5 low bits of the temperature
  usBridge = (static_cast<unsigned short int>( b1 ) << 8);
  usBridge |= (static_cast<unsigned short int>( b2 ) & 0x00FF);
  usTemp = ((static_cast<unsigned short int>( t1 ) << 8) | static_cast<unsigned short int>( t2 )) >> 5;
}
