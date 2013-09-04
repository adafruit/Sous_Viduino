Sous Videno
============
Sous-vide powered by Arduino - The Sous Videno!

This project was designed as an alternative interface to the [Adafruit Sous-Vide controller](http://learn.adafruit.com/sous-vide-powered-by-arduino-the-sous-viduino).
The code is a direct drop-in replacement, with a number of additional features:

* Added a Menu system for easier navigation.
* Added a large display font for increased display readablilty.
* Improved configuration screens, allowing setting of individual digits for all numeric values.
* Support for Fahrenheit temperature display and setting (Celsius is used internally and the values are converted to/from Fahrenheit).
* Compilation Options:
  * Support for non-RGB backlit LCD displays.
  * Support for 20 character wide displays (in addition to the default 16).
  * Ability to disable the logging feature.


Instructions
------------

###Build

1. Download.
2. Install Required 3rd Party Libraries:
  * [Adafruit RGB LCD Shield Library](https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library)
  * [Arduino PID Library](http://playground.arduino.cc/Code/PIDLibrary)
  * [Arduino AutoTune Library](http://playground.arduino.cc//Code/PIDAutotuneLibrary)
3. Assemble hardware according to the [Adafruit Sous-Vide controller tutorial](http://learn.adafruit.com/sous-vide-powered-by-arduino-the-sous-viduino).
4. Build and Deply.
 
###Usage

On first boot you will be brought to the main menu (You will get an error screen if the temperature sensor is not detected).
To navigate the menus use the UP and DOWN buttons.  To select a menu item use the RIGHT button.
To return to the menu from a mode, use the SELECT button, which for simplicity we'll call the MENU button from here on out.

When setting numeric values (temperature, Kp, Ki, Kd) use the LEFT and RIGHT buttons to select the digit.
Use the UP and DOWN buttons to change the value for that digit.  Pressing the MENU button will save the value and return you to the menu.
Pressing the LEFT and RIGHT buttons simultaneously will reset the the previously saved value.

When setting the display units, use the UP and DOWN buttons to switch between Celsius and Fahrenheit.
Use the RIGHT button to save the current selection as the units.

The Run mode will cycle between displaying the current temperature, the setpoint, and the duty cycle percentage.
When in the Run mode, you can navigate the menus while the controller runs in the background.
To return to the Run mode screen, select the Running menu option, which replaced the Run option.
To turn off the controller, select the Off menu option (This replaces the Show Temperature option when running).
While on the Run mode screen, press LEFT and RIGHT simultaneously to display the detailed display. This will dislpay the
current temperature, setpoint, and duty cycle percentage all on one screen.

References
----------

Based on the Sous Vide Controller by Bill Earl:  
https://github.com/adafruit/Sous_Viduino

Which is in turn based on the Arduino PID and PID AutoTune Libraries by Brett Beauregard:  
http://playground.arduino.cc/Code/PIDLibrary  
http://playground.arduino.cc//Code/PIDAutotuneLibrary

Uses the Adafruit RGB LCD Shield library  
https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library

Large LCD font based on Instructable:  
Custom Large Font for 16x2 LCDs by mpilchfamily  
http://www.instructables.com/id/Custom-Large-Font-For-16x2-LCDs/
