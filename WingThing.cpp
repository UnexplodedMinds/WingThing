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
#include <EEPROM.h>


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

Adafruit_BNO055 bno = Adafruit_BNO055( 55, 0x29 );


// Set up an interrupt fot the PRG button to initiate a magnetometer calibration
#define ESP_INTR_FLAG_DEFAULT 0
#define BUTTON_PIN 0

SemaphoreHandle_t semaphore = NULL;


float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;
float scaleX = 1.0f, scaleY = 1.0f, scaleZ = 1.0f;
bool  bCalibrateMag = false;


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


void dispStaticText()
{
    disp.clearDisplay();
    disp.setCursor( 0, 0 );
    disp.setTextSize( 1 );
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


void calibrateMagnetometer( float *dest1, float *dest2 ) 
{
    uint16_t ii = 0;
    int32_t  mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t  mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767};
    char     cal[16];

    disp.begin( SSD1306_SWITCHCAPVCC, 0x3C, true, true );
    disp.clearDisplay();
    disp.setTextSize( 1 );
    disp.setCursor( 0, 0 );
    disp.setTextColor( SSD1306_WHITE, SSD1306_BLACK );
    disp.print( "CALIBRATE" );
    disp.setCursor( 0, 12 );
    for( int iCal = 5; iCal > 0; iCal-- )
    {
        disp.setCursor( 0, 12 );
        sprintf( cal, "IN %d SECONDS...", iCal );
        disp.print( cal );
        disp.display();
        delay( 1000 );
    }
    disp.clearDisplay();
    disp.setTextSize( 2 );
    disp.setCursor( 0, 0 );
    disp.setTextColor( SSD1306_BLACK, SSD1306_WHITE );
    disp.print( "CALIBRATE" );
    disp.display();

    bno.begin();
    for( ii = 0; ii < 128; ii++ )
    {
        digitalWrite( LED_BUILTIN, HIGH );

        // Read the magnetometer data
        bno.getEvent( &magData, Adafruit_BNO055::VECTOR_MAGNETOMETER );
        populate( &magData );
        printf( "%.4f, %.4f, %.4f\n", mX, mY, mZ );
        if( mX > mag_max[0] )
            mag_max[0] = mX;
        if( mX < mag_min[0] )
            mag_min[0] = mX;
        if( mY > mag_max[1] )
            mag_max[1] = mY;
        if( mY < mag_min[1] )
            mag_min[1] = mY;
        if( mZ > mag_max[2] )
            mag_max[2] = mZ;
        if( mZ < mag_min[2] )
            mag_min[2] = mZ;

        delay( 135 );  // at 8 Hz ODR, new mag data is available every 125 ms
        digitalWrite( LED_BUILTIN, LOW );
    }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]; // *MPU9250mRes*MPU9250magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]; // *MPU9250mRes*MPU9250magCalibration[1];   
    dest1[2] = (float) mag_bias[2]; // *MPU9250mRes*MPU9250magCalibration[2];  

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    int   addr = 0x00;

    avg_rad /= 3.0;

    dest2[0] = avg_rad / ((float)mag_scale[0]);
    dest2[1] = avg_rad / ((float)mag_scale[1]);
    dest2[2] = avg_rad / ((float)mag_scale[2]);

    EEPROM.begin( sizeof( float ) * 6 );
    for( ii = 0; ii < 3; ii++ )
    {
        EEPROM.writeFloat( addr, dest1[ii] );
        addr += sizeof( float );
    }
    for( ii = 0; ii < 3; ii++ )
    {
        EEPROM.writeFloat( addr, dest2[ii] );
        addr += sizeof( float );
    }
    EEPROM.commit();

    dispStaticText();
}
 

void IRAM_ATTR handler( void* arg )
{
    xSemaphoreGiveFromISR( semaphore, NULL );
}

 
void button_task( void* arg )
{
    for( ;; )
    {
        if( xSemaphoreTake( semaphore, portMAX_DELAY ) == pdTRUE )
            bCalibrateMag = true;
    }
}


void setup()
{
    // Create a binary semaphore
    semaphore = xSemaphoreCreateBinary();

    // Setup the button GPIO pin
    gpio_pad_select_gpio( static_cast<gpio_num_t>( BUTTON_PIN ) );

    // Quite obvious, a button is a input
    gpio_set_direction( static_cast<gpio_num_t>( BUTTON_PIN ), GPIO_MODE_INPUT );

    // Trigger the interrupt when going from HIGH -> LOW ( == pushing button)
    gpio_set_intr_type( static_cast<gpio_num_t>( BUTTON_PIN ), GPIO_INTR_NEGEDGE );

    // Associate button_task method as a callback
    xTaskCreate( button_task, "button_task", 4096, NULL, 10, NULL );

    // Install default ISR service 
    gpio_install_isr_service( ESP_INTR_FLAG_DEFAULT );

    // Add our custom button handler to ISR
    gpio_isr_handler_add( static_cast<gpio_num_t>( BUTTON_PIN ), handler, NULL );

    // Built-in LED
    pinMode( LED_BUILTIN, OUTPUT );

    oled_i2c.begin( 4, 15 );

    // Initialize the BNO055 sensor. Flash the built-in LED very fast forever if it was not successful.
    if( !bno.begin() )
        flashLEDforever( 250 );

    // We are only interested in the orientation and magnetometer with this sensor so put it in that mode
    bno.setMode( Adafruit_BNO055::OPERATION_MODE_COMPASS );
    // Orientation for how it's mounted in the WingThing - if it's positioned differently this will need to be changed accordingly.
    bno.setAxisRemap( Adafruit_BNO055::REMAP_CONFIG_P0 );
    bno.setAxisSign( Adafruit_BNO055::REMAP_SIGN_P0 );

    pinMode( 16, OUTPUT );
    digitalWrite( 16, LOW );    // set GPIO16 low to reset OLED
    delay( 50 );
    digitalWrite( 16, HIGH );   // while OLED is running, must set GPIO16 to high

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

    int addr = 0x00;
    EEPROM.begin( sizeof( float ) * 6 );
    biasX = EEPROM.readFloat( addr );
    addr += sizeof( float );
    biasY = EEPROM.readFloat( addr );
    addr += sizeof( float );
    biasZ = EEPROM.readFloat( addr );
    addr += sizeof( float );
    scaleX = EEPROM.readFloat( addr );
    addr += sizeof( float );
    scaleY = EEPROM.readFloat( addr );
    addr += sizeof( float );
    scaleZ = EEPROM.readFloat( addr );

    dispStaticText();
}


void loop()
{
    if( !baro.begin() )
        flashLEDforever( 2000 );

    // Get the barometric pressure set from the app, if sent.
    // Stratofier will send "BARO:NN.NN" with no qualifiers in exactly that format (without quotes) when it's set in the app.
    // The MPL3115A2 will use this to internally calibrate the altitude to the correct pressure altitude.
    // Stratofier will send "CALIBRATE" with no qualifiers to trigger a calibration.  You'll normally want to trigger this on the
    // ground so you can move the device in figure eights during the calibration to get a good sample of all limits.

    iFromStratofier = udp.parsePacket();
    if( iFromStratofier > 0 )
    {
        udp.read( dispString, iFromStratofier );
        if( strncmp( dispString, "CALIBRATE", 9 ) == 0 )
            bCalibrateMag = true;
        else
            seaPress = atof( dispString ) * 3386.39;
    }

    // There is other data available from this sensor but we are only interested in orientation and magnetic heading
    // bno.getEvent( &orientationData, Adafruit_BNO055::VECTOR_EULER );
    bno.getEvent( &magData, Adafruit_BNO055::VECTOR_MAGNETOMETER );

    // Assign the data to their respective structs
    // populate( &orientationData );
    populate( &magData );

    // Calculate the magnetic heading
    head = atan2( (mY + biasY) * scaleY, (mX + biasX) * scaleX ) * 57.29578;
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
    memcpy( btString, dispString, strlen( dispString ) );
    udp.beginPacket( "192.168.10.255", 45678 );
    udp.write( static_cast<uint8_t *>( btString ), strlen( dispString ) );
    udp.endPacket();

    // Blink the built-in LED as a heartbeat and display the current sensor data on the tiny screen
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

    if( bCalibrateMag )
    {
        float dest1[3], dest2[3];
        
        calibrateMagnetometer( dest1, dest2 );
        biasX = dest1[0];
        biasY = dest1[1];
        biasZ = dest1[2];
        scaleX = dest2[0];
        scaleY = dest2[1];
        scaleZ = dest2[2];
    }
    bCalibrateMag = false;

    bHeart = (!bHeart);
}

