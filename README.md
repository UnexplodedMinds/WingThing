WingThing started as a Raspberry Pi project and morphed into an Arduino project since it is just easier to get the sensors to work correctly on an Arduino since that's what they're targeted for, and the libraries are easily obtainable.

The initial application is to work with the Stratofier Android app but with some modification of the sending protocol it could work with lots of other things.  All of the parts are easily available from Amazon or directly from Adafruit.

Stratofier will send the barometric pressure setting from your Android device so the altimiter sensor will give you an altitude corrected for the local barometric pressure.

Bluetooth send/recieve is handled by an HC-06 bluetooth module.

The orientation and magnetic heading are from an Adafruit BNO055 9DOF sensor.

Pressure altitude is from an Adafruit MPL3115A2 pressure sensor.

Pitot-static airspeed data is provided by a 3D Robotics pitot-static sensor that incorporates a 4525DO air pressure sensor.

The BNO055, luckily, can easily change its I2C address to 0x29 so it doesn't collide with the 4525DO that defaults to 0x28 and is a real pain to change.  Other air data sensors may be better or easier to use.  I chose this one since when I bought it, it had the highest pressure range (supposedly up to 200 mph - not tested).  A simple voltage divider off the 5V supply from the Arduino is required to send 3V to the ADR pin of the BNO055 to get it to change its address.  A voltage divider of the exact same type is required to bring the 5V logic between the Arduino TX signal and the HC-06 bluetooth module RX, down to 3V or you'll fry the HC-06.  A 2K2 and a 1K resistor for each circuit will do the trick:

       2.2 kOhm          1 kOhm
5V -----/\/\/\-----+-----/\/\/\-----|| Ground
                   |
                   3V

The code sets the HC-06 name to WingThing but you can set it to whatever you want if you want to get creative.  The Stratofier app doesn't actually care what the device is called since it's looking for the correct underlying serial stream.

Enjoy!
