//-------------------------------------------------------------------
// Sous Videno
// by Brady Doll
//
// Based on the Sous Vide Controller
// by Bill Earl - for Adafruit Industries
// https://github.com/adafruit/Sous_Viduino
// 
// which is in turn
// Based on the Arduino PID and PID AutoTune Libraries
// by Brett Beauregard
// http://playground.arduino.cc/Code/PIDLibrary
// http://playground.arduino.cc//Code/PIDAutotuneLibrary
//
// Uses the Adafruit RGB LCD Shield library
// https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library
// 
//-----------
// Large LCD font based on Instructable:
// Custom Large Font for 16x2 LCDs by mpilchfamily
// http://www.instructables.com/id/Custom-Large-Font-For-16x2-LCDs/
//------------------------------------------------------------------
// Pins Used:
// Analog 4 & 5			I2C (LCD Shield)
// Digital 2, 3, 4		DS18B20 Temperature Sensor
// Digital 7			Output control relay
//------------------------------------------------------------------


// ************************************************
// Compilation options
// 
// Comment/Uncomment to disable/enable options
// ************************************************

// LCD Options
#define CHARS 16		// Screen characters, Supports 16 or 20

// Support color LCD backlight
#define COLOR true		// Comment for monochrome backlight

// Enable serial logging
#define LOGGING true	// Comment to disable logging

// Default setpoint
#define SETPOINT_DEF 64.5

// Default Kp, Ki, & Kd values
#define KP_DEF 850
#define KI_DEF 0.5
#define KD_DEF 0.1


// ************************************************
// Libraries
// ************************************************

// Libraries for Adafruit LCD shield
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// So we can save and retrieve settings
#include <EEPROM.h>


// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint = SETPOINT_DEF;
double Input;
double Output;
// Output relay on time
volatile long onTime = 0;

// pid tuning parameters
double Kp = KP_DEF;
double Ki = KI_DEF;
double Kd = KD_DEF;

// EEPROM addresses for persisted data
const int SpAddress		= 0;
const int KpAddress		= 8;
const int KiAddress		= 16;
const int KdAddress		= 24;
const int configAddress	= 32;	// Bitmask, Unit settings, pump settings

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
const int WindowSize = 10000; 
unsigned long windowStartTime;


// ************************************************
// Auto Tune Variables and constants
// ************************************************

byte ATuneModeRemember = 2;

const double aTuneStep = 500;
const double aTuneNoise = 1;
const unsigned int aTuneLookBack = 20;

// Tuning mode
boolean tuning = false;
// Auto Tuner
PID_ATune aTune(&Input, &Output);


// ************************************************
// Display variables and constants
// ************************************************

// Backlight colors
#if defined(COLOR)
	#define RED 0x1
	#define YELLOW 0x3
	#define GREEN 0x2
	#define TEAL 0x6
	#define BLUE 0x4
	#define VIOLET 0x5
#endif
#define WHITE 0x7

// Large font segments
const int TOP_LEFT       = 0;
const int TOP_BAR        = 1;
const int TOP_RIGHT      = 2;
const int LOWER_LEFT     = 3;
const int LOWER_BAR      = 4;
const int LOWER_RIGHT    = 5;
const int TOP_MIDDLE_BAR = 6;
const int DEGREE         = 7;
const int BLANK          = 254;
const int SOLID          = 255;

// Define custom characters that make up segmetns
// Top left
uint8_t tl[8] = { B00111, B01111, B11111, B11111, B11111, B11111, B11111, B11111, };
// Top bar
uint8_t ub[8] = { B11111, B11111, B11111, B00000, B00000, B00000, B00000, B00000, };
// Top right
uint8_t tr[8] = { B11100, B11110, B11111, B11111, B11111, B11111, B11111, B11111, };
// Bottom left
uint8_t ll[8] = { B11111, B11111, B11111, B11111, B11111, B11111, B01111, B00111, };
// Bottom bar
uint8_t lb[8] = { B00000, B00000, B00000, B00000, B00000, B11111, B11111, B11111 };
// Bottom right
uint8_t lr[8] = { B11111, B11111, B11111, B11111, B11111, B11111, B11110, B11100 };
// Upper middle bar
uint8_t umb[8] = { B11111, B11111, B11111, B00000, B00000, B00000, B11111, B11111 };
// Degree
uint8_t deg[8] = { B01110, B11111, B11011, B11111, B01110, B00000, B00000, B00000 };

// Define large characters
uint8_t zero[6]  = { TOP_LEFT,       TOP_BAR,        TOP_RIGHT,      LOWER_LEFT, LOWER_BAR,  LOWER_RIGHT };
uint8_t one[6]   = { TOP_BAR,        TOP_RIGHT,      BLANK,          LOWER_BAR,  SOLID,      LOWER_BAR   };
uint8_t two[6]   = { TOP_MIDDLE_BAR, TOP_MIDDLE_BAR, TOP_RIGHT,      LOWER_LEFT, LOWER_BAR,  LOWER_BAR   };
uint8_t three[6] = { TOP_MIDDLE_BAR, TOP_MIDDLE_BAR, TOP_RIGHT,      LOWER_BAR,  LOWER_BAR,  LOWER_RIGHT };
uint8_t four[6]  = { LOWER_LEFT,     LOWER_BAR,      SOLID,          BLANK,      BLANK,      SOLID       };
uint8_t five[6]  = { LOWER_LEFT,     TOP_MIDDLE_BAR, TOP_MIDDLE_BAR, LOWER_BAR,  LOWER_BAR,  LOWER_RIGHT };
uint8_t six[6]   = { TOP_LEFT,       TOP_MIDDLE_BAR, TOP_MIDDLE_BAR, LOWER_LEFT, LOWER_BAR,  LOWER_RIGHT };
uint8_t seven[6] = { TOP_BAR,        TOP_BAR,        TOP_RIGHT,      BLANK,      BLANK,      SOLID       };
uint8_t eight[6] = { TOP_LEFT,       TOP_MIDDLE_BAR, TOP_RIGHT,      LOWER_LEFT, LOWER_BAR,  LOWER_RIGHT };
uint8_t nine[6]  = { TOP_LEFT,       TOP_MIDDLE_BAR, TOP_RIGHT,      BLANK,      BLANK,      SOLID       };
uint8_t c[6]     = { TOP_LEFT,       TOP_BAR,        TOP_BAR,        LOWER_LEFT, LOWER_BAR,  LOWER_BAR   };
uint8_t f[6]     = { TOP_LEFT,       TOP_MIDDLE_BAR, TOP_MIDDLE_BAR, SOLID,      BLANK,      BLANK       };
uint8_t s[6]     = { TOP_LEFT,       TOP_MIDDLE_BAR, TOP_MIDDLE_BAR, LOWER_BAR,  LOWER_BAR,  LOWER_RIGHT };
uint8_t v[6]     = { LOWER_LEFT,     BLANK,          BLANK,          BLANK,      LOWER_LEFT, LOWER_RIGHT };
uint8_t blank[6] = { BLANK, BLANK, BLANK, BLANK, BLANK, BLANK };

// Initalize the Adafruit LCD shield
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// Flash state
unsigned long previousFlash = 0;
const int flashInterval = 500;
bool flashed = false;
int8_t prevFlash = -1;

// Current display offset
uint8_t x = 0;


// ************************************************
// Relay control
// ************************************************

// Output Relay
#define RelayPin 7



// ************************************************
// Logging
// ************************************************

// Optional logging support
#if defined(LOGGING)
	const int logInterval = 10000; // log every 10 seconds
	unsigned long lastLogTime = 0;
#endif


// ************************************************
// Mode variables and constants
// ************************************************

// Maximum number of menu items
const int MENU_MAX = 4;
const int SETTINGS_MENU_MAX = 7;
const int TUNE_MENU_MAX = 11;

// Menu timeout
const long menuTimeout = 20000; // 20s

enum mode {
	MENU = 0,
	RUN,			// --> begin Main menu
	SET_TEMP,
	DISPLAY_TEMP,
	SETTINGS,		// <-- end Main menu
	CHOOSE_UNITS,	// --> begin Settings menu
	TUNING,
	PUMP_CONTROL,	// <-- end Settings menu
	TUNE_KP,		// --> begin Tuning menu
	TUNE_KI,
	TUNE_KD,
	AUTOTUNE		// <-- end Tuning menu
};
int8_t currentMode = MENU;
uint8_t previousMode = MENU;
unsigned long menuReturn = 0;

// Run PID
bool runPID = false;

// ************************************************
// Button variables and constants
// ************************************************

#define BUTTON_MENU BUTTON_SELECT

// Current selected button(s)
uint8_t buttons = 0;
// Delay interval for button registration
const unsigned long selectInterval = 90;
// Previous states of buttons
bool buttonStates[5] = {
	false,	// BUTTON_LEFT
	false,	// BUTTON_RIGHT
	false,	// BUTTON_UP
	false,	// BUTTON_DOWN
	false	// SELECT
};




// ************************************************
// Temperature variables and constants
// ************************************************

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)
#define ONE_WIRE_BUS 2
#define ONE_WIRE_PWR 3
#define ONE_WIRE_GND 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
// arrays to hold device address
DeviceAddress tempSensor;

// Temperature (for viewing)
float currentTemp;
// Change temperature
float changeTemp;
// Selected digit
int8_t selected = 0;

// Units
enum units { C = 0, F };
// Current unit
uint8_t currentUnits = C;
// Selected unit
uint8_t selectedUnits = currentUnits;

enum cycle {
	SHOW_TEMP = 0,
	SETP_TEMP,
	DUTY,
	DETAILED
};
uint8_t cycleDisplay = 0;
const int cycleInterval = 5000;


// Current time
unsigned long currentMillis = 0;
//// Misc Interval
unsigned long previousInterval = 0;
// Last selected item
int8_t lastSelect;
float changeValue = 0.0;
float prevValue = 0.0;




// ************************************************
// Setup
// ************************************************
void setup()
{
	// Initialize the LCD
	lcd.begin(CHARS, 2);

	lcd.clear();
#if defined(COLOR)
	lcd.setBacklight(GREEN);
#endif

	// Create custom characters segements
	lcd.createChar(TOP_LEFT, tl);
	lcd.createChar(TOP_BAR, ub);
	lcd.createChar(TOP_RIGHT, tr);
	lcd.createChar(LOWER_LEFT, ll);
	lcd.createChar(LOWER_BAR, lb);
	lcd.createChar(LOWER_RIGHT, lr);
	lcd.createChar(TOP_MIDDLE_BAR, umb);
	lcd.createChar(DEGREE, deg);

	// Display splash screen
	x = (CHARS - 16) / 2;
	customChar(s);
	lcd.setCursor(x, 1);
	lcd.print(F("ous"));
	x += 4;
	customChar(v);
	lcd.setCursor(x, 0);
	lcd.write(LOWER_RIGHT);
	lcd.setCursor(x, 1);
	lcd.write(BLANK);
	lcd.print(F("ideno"));

	// Set up Ground & Power for the sensor from GPIO pins
	// Ground
	pinMode(ONE_WIRE_GND, OUTPUT);
	digitalWrite(ONE_WIRE_GND, LOW);
	// Power
	pinMode(ONE_WIRE_PWR, OUTPUT);
	digitalWrite(ONE_WIRE_PWR, HIGH);

	// Load temperature sensor
	sensors.begin();
	if (!sensors.getAddress(tempSensor, 0)) {
		delay(1500);
#if defined(COLOR)
		lcd.setBacklight(RED);
#endif
		lcd.clear();

		uint8_t exclamation1[] = { B11111, B11100, B11100, B11100, B11100, B11100, B11100, B11100 };
		lcd.createChar(0, exclamation1);
		uint8_t exclamation2[] = { B11111, B00111, B00111, B00111, B00111, B00111, B00111, B00111 };
		lcd.createChar(1, exclamation2);
		uint8_t exclamation3[] = { B11100, B11100, B11111, B11100, B11100, B11100, B11100, B11111 };
		lcd.createChar(2, exclamation3);
		uint8_t exclamation4[] = { B00111, B00111, B11111, B00111, B00111, B00111, B00111, B11111 };
		lcd.createChar(3, exclamation4);

		x = (CHARS - 16) / 2;

		lcd.setCursor(x, 0);
		lcd.write(SOLID);
		lcd.write(0);
		lcd.write(1);
		lcd.write(SOLID);

		lcd.setCursor(x, 1);
		lcd.write(SOLID);
		lcd.write(2);
		lcd.write(3);
		lcd.write(SOLID);

		x += 5;
		lcd.setCursor(x, 0);
		lcd.print(F("Sensor"));
		lcd.setCursor(x, 1);
		lcd.print(F("Error"));

		x += 7;
		lcd.setCursor(x, 0);
		lcd.write(SOLID);
		lcd.write(0);
		lcd.write(1);
		lcd.write(SOLID);

		lcd.setCursor(x, 1);
		lcd.write(SOLID);
		lcd.write(2);
		lcd.write(3);
		lcd.write(SOLID);

		currentMode = -1;
		return;
	}

#if defined(LOGGING)
	Serial.begin(9600);
#endif

	sensors.setResolution(tempSensor, 12);
	sensors.setWaitForConversion(false);
	// Start an asynchronous temperature reading
	sensors.requestTemperatures();

	// Initialize Output Relay Control
	pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
	digitalWrite(RelayPin, LOW);  // Make sure it is off to start
	
	// Initialize the PID and related variables
	LoadParameters();
	myPID.SetTunings(Kp, Ki, Kd);

	myPID.SetSampleTime(1000);
	myPID.SetOutputLimits(0, WindowSize);

	// Run timer2 interrupt every 15 ms 
	TCCR2A = 0;
	TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

	//Timer2 Overflow Interrupt Enable
	TIMSK2 |= 1<<TOIE2;

	myPID.SetMode(MANUAL);

	// Delay for splash screen
	delay(3000);
	lcd.clear();
	// Load menu
	changeMode(MENU);
}

// ************************************************
// Main Loop
// ************************************************
void loop()
{
	currentMillis = millis();
	buttons = lcd.readButtons();

	// Determine the current mode
	switch (currentMode) {
		case MENU:
			displayMenu(MENU);
			break;
		case RUN:
			Run();
			break;
		case SET_TEMP:
			setValue(true, 1, true, MENU);
			break;
		case DISPLAY_TEMP:
			displayTemp();
			break;
		case SETTINGS:
			displayMenu(SETTINGS);
			break;
		case CHOOSE_UNITS:
			chooseUnits();
			break;
		case TUNING:
			displayMenu(TUNING);
			break;
		case TUNE_KP:
			setValue(false, 1, true, TUNING);
			break;
		case TUNE_KI:
		case TUNE_KD:
			setValue(false, 2, false, TUNING);
			break;
	}

	if (runPID) {
		doControl();
	}
}



// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
	if (runPID) {
		DriveOutput();
	} else {
		digitalWrite(RelayPin, LOW);  // make sure relay is off
	}
}



// ************************************************
// Mode Functions
// ************************************************

/// Handles mode changes
///
/// <param name="newMode"></param>
void changeMode( uint8_t newMode )
{
	sensors.requestTemperatures();

	// Reset the button states
	for (uint8_t i = 0; i < 5; i++) {
		buttonStates[i] = false;
	}

	bool celsius = (C == currentUnits);

	previousMode = currentMode;

	switch (previousMode) {
		case SET_TEMP:
			Setpoint = (celsius) ? changeValue : (changeValue - 32) * 0.555555555555556;
			SaveParameters();
			break;
		case TUNE_KP:
			Kp = changeValue;
			SaveParameters();
			myPID.SetTunings(Kp, Ki, Kd);
			break;
		case TUNE_KI:
			Ki = changeValue;
			SaveParameters();
			myPID.SetTunings(Kp, Ki, Kd);
			break;
		case TUNE_KD:
			Kd = changeValue;
			SaveParameters();
			myPID.SetTunings(Kp, Ki, Kd);
			break;
	}

	switch (newMode) {
		case MENU:
		case SETTINGS:
		case TUNING:
#if defined(COLOR)
			lcd.setBacklight(BLUE);
#endif
			lastSelect = -1;
			selected = currentMode;
			menuReturn = previousInterval = currentMillis;
			break;
		case RUN:
			if (!runPID) {
				myPID.SetMode(AUTOMATIC);
				windowStartTime = millis();
				runPID = true;
			}
			previousInterval = currentMillis;
			cycleDisplay = SHOW_TEMP;
			getTemp(celsius);
			break;
		case SET_TEMP:
#if defined(COLOR)
			lcd.setBacklight(TEAL);
#endif
			prevValue = changeValue = (celsius) ? Setpoint : (Setpoint * 1.8) + 32;
			selected = 2;
			lcd.clear();
			printTemp(changeValue, celsius, true, true, 1);
			prevFlash = -1;
			break;
		case DISPLAY_TEMP:
			if (runPID) {
				turnOff();
				changeMode(MENU);
				return;
			} else {
#if defined(COLOR)
				lcd.setBacklight(GREEN);
#endif
				getTemp(celsius);
			}
			break;
		case CHOOSE_UNITS:
#if defined(COLOR)
			lcd.setBacklight(TEAL);
#endif
			lcd.clear();
			
			lcd.setCursor(0, 0);
			lcd.print(F("Units"));

			selectedUnits = currentUnits;
			displayUnitChooser();
			break;
		case TUNE_KP:
#if defined(COLOR)
			lcd.setBacklight(TEAL);
#endif
			prevValue = changeValue = Kp;
			selected = 2;
			lcd.clear();
			x = (CHARS - 13) / 2;
			printLargeNumber(changeValue, 1, true);
			prevFlash = -1;
			break;
		case TUNE_KI:
#if defined(COLOR)
			lcd.setBacklight(TEAL);
#endif
			prevValue = changeValue = Ki;
			selected = 2;
			lcd.clear();
			x = (CHARS - 10) / 2;
			printLargeNumber(changeValue, 2, false);
			prevFlash = -1;
			break;
		case TUNE_KD:
#if defined(COLOR)
			lcd.setBacklight(TEAL);
#endif
			prevValue = changeValue= Kd;
			selected = 2;
			lcd.clear();
			x = (CHARS - 10) / 2;
			printLargeNumber(changeValue, 2, false);
			prevFlash = -1;
			break;
		case AUTOTUNE:
			if (abs(Input - Setpoint) < 0.5) {
				// Remember the mode we're in
				ATuneModeRemember = myPID.GetMode();

				// set up the auto-tune parameters
				aTune.SetNoiseBand(aTuneNoise);
				aTune.SetOutputStep(aTuneStep);
				aTune.SetLookbackSec(int(aTuneLookBack));
				tuning = true;

				// Switch to the RUN mode
				changeMode(RUN);
			} else {
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print(F("The temperature must"));
				lcd.setCursor(0, 1);
				lcd.print(F("be within 0.5"));
				lcd.write(B11011111);
				lcd.print(F("C"));
				delay(cycleInterval);

				changeMode(SETTINGS);
			}
			return;
			break;
	}

	currentMode = newMode;
}

/// Displays the menu
/// 
///
void displayMenu( uint8_t menu )
{
	bool change = false;

	if (buttons) {
		if ((currentMillis - menuReturn) > flashInterval ) {
			//if (checkButton(BUTTON_MENU)) {
			//	if ((currentMillis - menuReturn) > flashInterval ) {
			//		// Change back to the previous mode
			//		changeMode(previousMode);
			//	}
			//} else if (checkButton(BUTTON_UP)) {
			if (checkButton(BUTTON_UP)) {
				selected--;
			} else if(checkButton(BUTTON_DOWN)) {
				selected++;
			} else if (checkButton(BUTTON_RIGHT)) {
				changeMode(selected);
				return;
			} else if (MENU != menu && (checkButton(BUTTON_LEFT) || checkButton(BUTTON_MENU))) {
				if (SETTINGS == menu) {
					changeMode(MENU);
					selected = SETTINGS;
				} else if (TUNING == menu) {
					changeMode(SETTINGS);
					selected = TUNING;
				}
			}

			previousInterval = currentMillis;
		}
	} else {
		// Reset the button states
		for (uint8_t i = 0; i < 5; i++) {
			buttonStates[i] = false;
		}

		
		// Check if we have reached our timout interval
		if ((currentMillis - previousInterval) > menuTimeout) {
			if (runPID) {
				changeMode(RUN);
			} else {
				turnOff();
				// Turn off the display backlight
				lcd.setBacklight(0);

				// Wait until a button is pressed
				while (!buttons) {
					buttons = lcd.readButtons();
				}

				menuReturn = previousInterval = millis();

				// Start an asynchronous temperature reading
				sensors.requestTemperatures();

				// Turn the backlight back on
#if defined(COLOR)
				lcd.setBacklight(BLUE);
#else
				lcd.setBacklight(WHITE);
#endif
			}
		}
	}

	x = 0;

	uint8_t min;
	uint8_t max;

	// Get the item ranges for the selected menu
	if (MENU == menu) {
		min = 1;
		max = MENU_MAX;
	} else if (SETTINGS == menu) {
		min = MENU_MAX + 1;
		max = SETTINGS_MENU_MAX - 1;
	} else if (TUNING == menu) {
		min = SETTINGS_MENU_MAX + 1;
		max = TUNE_MENU_MAX;
	}

	// Make sure the currently selected item is within the range
	if (min > selected) {
		selected = min;
	} else if (max < selected) {
		selected = max;
	}

	// Check if the selected item has changed
	if (selected != lastSelect) {
		bool celsius = (C == currentUnits);

		lcd.clear();

		int selected_num = selected;
		if (SETTINGS == menu) {
			selected_num -= MENU_MAX;
		} else if (TUNING == menu) {
			selected_num -= SETTINGS_MENU_MAX;
		}

		printLargeDigit(selected_num);
		x++;
		lcd.setCursor(x, 0);

		switch ( selected ) {
			case RUN:
				if (runPID) {
					if (tuning) {
						lcd.print(F("Tuning"));
					} else {
						lcd.print(F("Running"));
					}
				} else {
					lcd.print(F("Run"));
				}

				lcd.setCursor(( (celsius) ? CHARS - 9 : CHARS - 10 ), 1);
				lcd.print(F("SP:"));
				lcd.print( (celsius) ? Setpoint : (Setpoint * 1.8) + 32 );
				lcd.setCursor(CHARS - 2, 1);
				lcd.write(B11011111); // Degree symbol
				lcd.print( (celsius) ? 'C' : 'F' );
				break;
			case SET_TEMP:
				lcd.print(F("Set"));
				lcd.setCursor(x, 1);
				lcd.print(F("Temperature"));
				break;
			case DISPLAY_TEMP:
				if (runPID) {
					lcd.print(F("Stop"));
					lcd.setCursor(x, 1);
					lcd.print(F("Running"));
				} else {
					lcd.print(F("Display"));
					lcd.setCursor(x, 1);
					lcd.print(F("Temperature"));
				}
				break;
			case SETTINGS:
				lcd.print(F("Settings"));
				lcd.setCursor(x,1);
				break;
		// Settings Menu
			case CHOOSE_UNITS:
				lcd.print(F("Set"));
				lcd.setCursor(x, 1);
				lcd.print(F("Units"));
				lcd.setCursor(CHARS - 1, 1);
				lcd.print( (celsius) ? 'C' : 'F' );
				break;
			case TUNING:
				lcd.print(F("Configure"));
				lcd.setCursor(x, 1);
				lcd.print(F("Tuning"));
				break;
		// Tuning Menu
			case TUNE_KP:
				lcd.print(F("Tune"));
				lcd.setCursor(x, 1);
				lcd.print(F("Kp"));
				if (Kp < 100) {
					lcd.setCursor(CHARS - 5, 1);
				} else if (Kp < 1000) {
					lcd.setCursor(CHARS - 6, 1);
				} else {
					lcd.setCursor(CHARS - 7, 1);
				}
				lcd.print(Kp);
				break;
			case TUNE_KI:
				lcd.print(F("Tune"));
				lcd.setCursor(x, 1);
				lcd.print(F("Ki"));
				lcd.setCursor(CHARS - 4, 1);
				lcd.print(Ki);
				break;
			case TUNE_KD:
				lcd.print(F("Tune"));
				lcd.setCursor(x, 1);
				lcd.print(F("Kd"));
				lcd.setCursor(CHARS - 4, 1);
				lcd.print(Kd);
				break;
			case AUTOTUNE:
				lcd.print(F("Auto"));
				lcd.setCursor(x,1);
				lcd.print(F("Tuning"));
				break;
		}

		lcd.setCursor(CHARS - 1, 0);
		lcd.write(B01111110);

		lastSelect = selected;
	}
}

// Handles the run state
//
//
void Run() {
	bool celsius = (C == currentUnits);

	if (buttons) {
		if (checkButton(BUTTON_MENU)) {
			changeMode(MENU);
			return;
		} else if (buttons & BUTTON_LEFT && buttons & BUTTON_RIGHT) {
			if (DETAILED != cycleDisplay) {
				cycleDisplay = DETAILED;
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print(F("Sp: "));
				lcd.print((celsius) ? Setpoint : (Setpoint * 1.8) + 32);
				lcd.write(B11011111);	// degree symbol
				lcd.print((celsius) ? 'C' : 'F');

				lcd.setCursor(CHARS - 1, 0);
				if (tuning) {
					lcd.print(F("T"));
				} else {
					lcd.write(BLANK);
				}
			}
		}
	} else {
		for (uint8_t i = 0; i < 5; i++) {
			buttonStates[i] = false;
		}
	}

	if (DETAILED != cycleDisplay) {
		if ((SHOW_TEMP == cycleDisplay && (currentMillis - previousInterval) > menuTimeout) || ((SHOW_TEMP != cycleDisplay) && (currentMillis - previousInterval) > cycleInterval)) {
			cycleDisplay++;

			if (cycleDisplay > 2) {
				cycleDisplay = 0;
			}

			if (SETP_TEMP == cycleDisplay) {
				float setpTemp = (celsius) ? Setpoint : (Setpoint * 1.8) + 32;
				printTemp(setpTemp, celsius, true, true, 1);
				lcd.setCursor(0, 0);
				lcd.print(F("S"));
				lcd.setCursor(0, 1);
				lcd.print(F("P"));
				
				lcd.setCursor(CHARS - 1, 0);
				if (tuning) {
					lcd.print(F("T"));
				} else {
					lcd.write(BLANK);
				}
			} else if (DUTY == cycleDisplay) {
				lcd.clear();
				lcd.setCursor(CHARS - 2, 0);
				lcd.print(F("%"));

				lcd.setCursor(CHARS - 1, 0);
				if (tuning) {
					lcd.print(F("T"));
				} else {
					lcd.write(BLANK);
				}
			}

			previousInterval = currentMillis;
		}
	}

	float pct;

	switch (cycleDisplay) {
		case SHOW_TEMP:
			printTemp(currentTemp, celsius, false, true, 2);
			
			lcd.setCursor(CHARS - 1, 0);
			if (tuning) {
				lcd.print(F("T"));
			} else {
				lcd.write(BLANK);
			}
			break;
		case DUTY:
			pct = (map(Output, 0, WindowSize, 0, 1000) / 10);
			x = (CHARS - 14) / 2;
			printLargeNumber(pct, 2, false);
			break;
		case DETAILED:
			lcd.setCursor(0, 1);
			lcd.print(currentTemp);
			lcd.write(B11011111);	// degree symbol
			if (celsius) {
				lcd.print(F("C :"));
			} else {
				lcd.print(F("F :"));
			}
			pct = (map(Output, 0, WindowSize, 0, 1000) / 10);
			if (pct < 100) {
				lcd.write(BLANK);
			}
			lcd.print(pct);
			lcd.print("%");
			break;
	}

#if defined(COLOR)
	setBacklight();
#endif
}

///
///
///
void turnOff() {
	runPID = false;

	myPID.SetMode(MANUAL);
	// Turn off the relay
	digitalWrite(RelayPin, LOW); // make sure it is off
}

// ************************************************
// Execute the control loop
// ************************************************
void doControl() {
	getTemp(C == currentUnits);

	if (tuning) {
		if (aTune.Runtime()) {	// returns 'true' when done
			FinishAutoTune();
		}
	} else {
		// TODO: Add feed forward/quick heat mode
		myPID.Compute();
	}

	 // Time Proportional relay state is updated regularly via timer interrupt.
	onTime = Output;

#if defined(LOGGING)
	// Periodically log to the serial port in CSV format
	if (currentMillis - lastLogTime > logInterval) {
		Serial.print(Input);
		Serial.print(",");
		Serial.print(Output);
	}
#endif
}

///
///
///
void displayTemp() {
	if (buttons) {
		if (checkButton(BUTTON_MENU)) {
			changeMode(MENU);
			return;
		}
	} else {
		for (int i = 0; i < 5; i++) {
			buttonStates[i] = false;
		}
	}

	bool celsius = (C == currentUnits);

	getTemp(celsius);

	printTemp(currentTemp, celsius, false, true, 2);
}

///
///
///
void setValue( bool temperature, uint8_t decimalPlaces, bool leadingZero, uint8_t returnMenu )
{
	if (buttons) {
		if (checkButton(BUTTON_MENU)) {
			changeMode(returnMenu);
			return;
		} else {
			// Check if both left and right are pressed at the same time
			if (buttons & BUTTON_LEFT && buttons & BUTTON_RIGHT) {
				// Reset the value
				changeValue = prevValue;
				if (temperature) {
					printTemp(changeValue, (C == currentUnits), leadingZero, true, decimalPlaces);
					selected = (2 == decimalPlaces) ? 1 : 2;
				} else {
					uint8_t len = (leadingZero) ? 13 : 10;
					x = (CHARS - len) / 2;
					printLargeNumber(changeValue, decimalPlaces, leadingZero);
					selected = 2;
				}
			} else if (checkButton(BUTTON_LEFT)) {
				selected--;
			} else if (checkButton(BUTTON_RIGHT)) {
				selected++;
			} else {
				bool up = checkButton(BUTTON_UP);
				bool down = checkButton(BUTTON_DOWN);

				if (up || down) {
					uint8_t digits[4];
					uint16_t tempInt = int(changeValue);

					if (2 == decimalPlaces) {
						digits[0] = (tempInt / 10) % 10;
						digits[1] = tempInt % 10;
						digits[2] = int(changeValue * 10) % 10;
						digits[3] = int(changeValue * 100) % 10;
					} else {
						digits[0] = (tempInt / 100) % 10;
						digits[1] = (tempInt / 10) % 10;
						digits[2] = tempInt % 10;
						digits[3] = int(changeValue * 10) % 10;
					}

					// Increment the section
					if (up) {
						digits[selected]++;
					} else if(down) {
						digits[selected]--;
					}

					// Wrap from 9 to 0 and vice versa
					if ( digits[selected] > 9 ) {
						digits[selected] = 0;
					} else if (digits[selected] < 0 ) {
						digits[selected] = 9;
					}

					// Rebuild the full value
					if (2 == decimalPlaces) {
						changeValue = (digits[0] * 10) + digits[1] + (float(digits[2]) / 10) + (float(digits[3]) / 100);
					} else {
						changeValue = (digits[0] * 100) + (digits[1] * 10) + digits[2] + (float(digits[3]) / 10);
					}
				}
			}
		}
	} else {
		// Reset button states
		for (int i = 0; i < 5; i++) {
			buttonStates[i] = false;
		}
	}

	// Wrap the digit selection
	if (leadingZero) {
		if (selected < 0) {
			selected = 3;
		} else if (selected > 3) {
			selected = 0;
		}
	} else {
		if (selected < 1) {
			selected = 3;
		} else if (selected > 3) {
			selected = 1;
		}
	}

	// Flash the currently selected digit
	uint8_t offset;

	if (temperature) {
		offset = 1;
	} else {
		uint8_t len = (leadingZero) ? 13 : 10;
		offset = (CHARS - len) / 2;
	}

	flashLargeDigit(changeValue, selected, offset, decimalPlaces, leadingZero);
}

/// Choose the display units.
///
///
void chooseUnits()
{
	if (buttons) {
		if (checkButton(BUTTON_MENU)) {
			changeMode(SETTINGS);
			return;
		} else if (checkButton(BUTTON_UP)) {
			selectedUnits = (C == selectedUnits) ? F : C;
			displayUnitChooser();
		} else if(checkButton(BUTTON_DOWN)) {
			selectedUnits = (C == selectedUnits) ? F : C;
			displayUnitChooser();
		} else if(checkButton(BUTTON_RIGHT)) {
			lcd.setCursor(CHARS - 1, 0);
			lcd.write(B00101010);

			// Save the new unit selection
			currentUnits = selectedUnits;
			SaveParameters();
		}
	} else {
		for (uint8_t i = 0; i < 5; i++) {
			buttonStates[i] = false;
		}
	}
}

/// Displays the Unit Chooser screen.
///
///
void displayUnitChooser() {
	x = 6;
	
	if (C == selectedUnits) {
		customChar(c);
		lcd.setCursor(9, 1);
		lcd.print(F("elsius "));
	} else {
		customChar(f);
		lcd.setCursor(7, 1);
		lcd.print(F("ahrenheit"));
	}

	lcd.setCursor(CHARS - 1, 0);
	if (selectedUnits == currentUnits) {
		lcd.write(B00101010);
	} else {
		lcd.write(BLANK);
	}
}


/// Checks the button states.
///
/// <param name="button">The button to check.</param>
/// <return></return>
bool checkButton( uint8_t button )
{
	bool buttonState = (buttons & button);
	
	uint8_t buttonID;
	switch (button) {
		case BUTTON_LEFT:
			buttonID = 0;
			break;
		case BUTTON_RIGHT:
			buttonID = 1;
			break;
		case BUTTON_UP:
			buttonID = 2;
			break;
		case BUTTON_DOWN:
			buttonID = 3;
			break;
		case BUTTON_MENU:
			buttonID = 4;
			break;
	}

	if (buttonState != buttonStates[buttonID]) {
		buttonStates[buttonID] = buttonState;
		if (buttonState) {
			return true;
		}
	}

	return false;
}

/// Gets the current temperature.
///
/// 
void getTemp( bool celsius ) {
	if (sensors.isConversionAvailable(0)) {
		//currentTemp = (celsius) ? sensors.getTempC(tempSensor) : sensors.getTempF(tempSensor);
		Input = sensors.getTempC(tempSensor);
		currentTemp = (celsius) ? Input : (Input * 1.8) + 32; // Convert C to F  Tf = Tc * 9/5 + 32
		sensors.requestTemperatures();
	}
}


// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
	long now = millis();
	// Set the output
	// "on time" is proportional to the PID output
	if ((now - windowStartTime) > WindowSize) { //time to shift the Relay Window
		windowStartTime += WindowSize;
	}
	if((onTime > 100) && (onTime > (now - windowStartTime))) {
		digitalWrite(RelayPin,HIGH);
	} else {
		digitalWrite(RelayPin,LOW);
	}
}


// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp, Ki, Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}


// ************************************************
// Display functions
// ************************************************


void flashLargeDigit( float num, uint8_t digit, uint8_t offset, uint8_t decimalPlaces, bool leadingZero )
{
	// Check if the flashed digit has changed or the flash time has elapsed
	if (digit != prevFlash || (currentMillis - previousFlash) > flashInterval)
	{
		uint8_t divisor;
		
		// Check if the flashed digit has changed
		if (digit != prevFlash) {
			// Check if we were already flashing a digit
			if (flashed && -1 != prevFlash) {
				// Re print the digit
				x = offset + 3 * prevFlash;
				if (2 == decimalPlaces) {
					if (!leadingZero) {
						x -= 3;
					}
					if (3 == prevFlash) {
						x++;
						printLargeDigit(int(num * 100) % 10);
					} else if (2 == prevFlash) {
						x++;
						printLargeDigit(int(num * 10) % 10);
					} else if (1 == prevFlash) {
						printLargeDigit(int(num) % 10);
					}
				} else {
					if (3 == prevFlash) {
						x++;
						printLargeDigit(int(num * 10) % 10);
					} else {
						divisor = (0 == prevFlash) ? 100 : (1 == prevFlash) ? 10 : 1;
						printLargeDigit((int(num) / divisor) % 10);
					}
				}
			}
			flashed = false;
		}

		previousFlash = currentMillis;

		flashed = !flashed;

		// Get the digit offset
		x = offset + 3 * digit;

		// Check if we are flashing the digit
		if (flashed) {
			if (2 == decimalPlaces && !leadingZero) {
				x -= 3;
			}
			if (3 == digit || (2 == decimalPlaces && 2 == digit)) {
				x++;
			}
			// Clear the digit
			customChar( blank );
		} else {
			// Re-add the digit
			if (2 == decimalPlaces) {
				if (!leadingZero) {
					x -= 3;
				}
				if (3 == digit) {
					x++;
					printLargeDigit(int(num * 100) % 10);
				} else if (2 == digit) {
					x++;
					printLargeDigit(int(num * 10) % 10);
				} else if (1 == digit) {
					printLargeDigit(int(num) % 10);
				}
			} else {
				if (3 == digit) {
					x++;
					printLargeDigit(int(num * 10) % 10);
				} else {
					divisor = (0 == digit) ? 100 : (1 == digit) ? 10 : 1;
					printLargeDigit((int(num) / divisor) % 10);
				}
			}
		}

		prevFlash = digit;
	}
}

/// Prints out a temperature to the LCD display.
///
/// <param name="temp">The temperature to display. This can be in the range -999 to 999.</param>
/// <param name="celsius">True if the displayed temperature is in Celsius or false if it is in Fahrenheit.</param>
/// <param name="leadingZero">True if a leading zero should be displayed, otherwise false.</param>
/// <param name="center">True if the temperature display should be centered, otherwise false.</param>
/// <param name="decimalPlaces">The number of decimal places to print (if the temp is less than 100).
void printTemp( float temp, bool celsius, bool leadingZero, bool center, uint8_t decimalPlaces )
{
	// Check that the temperature is in the supported range
	if (temp > -1000 && temp < 1000) {
		uint8_t len = 8;
		bool smallUnit = false;

		// Check if we are centering the display
		if (center) {
			// Get the individual digits
			uint8_t digits[4];

			bool negative = false;
			if (temp < 0) {
				temp *= -1;
				negative = true;
				len++;
			}


			uint8_t curDigit = 2;
			uint8_t decimal;
			uint16_t tempInt = int(temp);

			if (temp >= 100) {
				curDigit++;
				len += 3;
			} else if (temp < 10 || (temp > 10 && 1 == decimalPlaces)) {
				curDigit++;
				if (leadingZero) {
					len += 3;
				}
			}

			if (temp >= 10) {
				curDigit++;
				len += 3;
			}

			if (curDigit < 4) {
				len += 3;
			}

			// Check if we need to show small units
			if ((CHARS - 1 - len) < 3) {
				len++;
				smallUnit = true;
			} else {
				len += 3;
			}

			// Get the indentation amount
			x = (CHARS - len) / 2;
			if (0 == x && len < CHARS) {
				x = 1;
			}

			// Print blank characters before the temperature
			if (x > 0) {
				for (uint8_t line = 0; line < 2; line++) {
					lcd.setCursor(0, line);
					for (uint8_t i = 0; i < x; i++ ) {
						lcd.write(BLANK);
					}
				}
			}

			if (negative) {
				temp *= -1;
			}
		} else { // Not centered
			x = 0;
		}

		printLargeNumber(temp, decimalPlaces, leadingZero);

		// Display the degreen symbol
		lcd.setCursor(x, 0);
		lcd.write(DEGREE);
		lcd.setCursor(x, 1);
		lcd.write(BLANK);
		x++;

		// Display the unit
		if (celsius) {	// Celsius
			if (smallUnit) {
				lcd.setCursor(x, 0);
				lcd.write(BLANK);
				lcd.setCursor(x, 1);
				lcd.print(F("C"));
				x++;
			} else {
				customChar(c);
			}
		} else {		// Fahrenheit
			if (smallUnit) {
				lcd.setCursor(x, 0);
				lcd.write(BLANK);
				lcd.setCursor(x, 1);
				lcd.print(F("F"));
				x++;
			} else {
				customChar(f);
			}
		}

		// Print blank characters at the end
		while (CHARS >= x) {
			lcd.setCursor(x, 0);
			lcd.write(BLANK);
			lcd.setCursor(x, 1);
			lcd.write(BLANK);
			x++;
		}
	}
}

/// Prints a large font number to the LCD display.
/// The start position must be set first by setting the variable 'x'.
///
/// <param name="num">The number to display.</param>
/// <param name="decimalPlaces">The number of decimal places to print (if the temp is less than 100).
/// <param name="leadingZero">True if a leading zero should be displayed, otherwise false.</param>
void printLargeNumber( float num, uint8_t decimalPlaces, bool leadingZero )
{
	bool negative = false;

	// Check if the temperature is negative
	if (num < 0) {
		num = num * -1;
		negative = true;
	}

	// Get the individual digits
	uint8_t digits[4];

	uint8_t curDigit = 0;
	uint8_t decimal;
	uint16_t tempNum = int(num);

	if (num >= 100) {
		digits[curDigit] = (tempNum / 100) % 10;
		curDigit++;
	} else if (num < 10 || (num > 10 && 1 == decimalPlaces)) {
		digits[curDigit] = 0;
		curDigit++;
	}

	if (num >= 10) {
		digits[curDigit] = (tempNum / 10) % 10;
		curDigit++;
	} else if (num < 10 && 1 == decimalPlaces) {
		digits[curDigit] = 0;
		curDigit++;
	}
	digits[curDigit] = tempNum % 10;
	curDigit++;

	decimal = curDigit;

	digits[curDigit] = int(num * 10) % 10;
	int roundUp;
	if (curDigit == 3) {
		roundUp = int(num * 100) % 10;
		// Round up the last digit
		if (roundUp > 4) {
			digits[3]++;
		}
	}
	curDigit++;

	if (curDigit < 4) {
		digits[curDigit] = int(num * 100) % 10;
		if (curDigit == 3) {
			roundUp = int(num * 1000) % 10;
			// Round up the last digit
			if (roundUp > 4) {
				digits[3]++;
			}
		}
	}

	// Check if we are showing a negative number
	if (negative) {
		lcd.setCursor(x, 0);
		lcd.write(LOWER_BAR);
		lcd.setCursor(x, 1);
		lcd.write(BLANK);
		x++;
	}

	// Display each digit
	for ( uint8_t i = 0; i < 4; i++ ) {
		if (decimal == i) {
			// Print the decimal point
			lcd.write(B00101110);
			lcd.setCursor(x, 0);
			lcd.write(BLANK);
			x++;
		}

		// Check if we are showing leading zeros
		if (0 == digits[i]) {
			if ( 3 == i || 2 == i || (1 == i && (2 == decimalPlaces || 100 <= num)) || ( leadingZero && ((0 == i && 100 > num) || (1 == i && 10 > num)) ) ) {
				printLargeDigit(digits[i]);
			}
		} else {
			printLargeDigit(digits[i]);
		}
	}
}

// ************************************************
// Display large font number digit.
// ************************************************
void printLargeDigit( uint8_t num )
{
	switch ( num ) {
		case 0:
			customChar(zero);
			break;
		case 1:
			customChar(one);
			break;
		case 2:
			customChar(two);
			break;
		case 3:
			customChar(three);
			break;
		case 4:
			customChar(four);
			break;
		case 5:
			customChar(five);
			break;
		case 6:
			customChar(six);
			break;
		case 7:
			customChar(seven);
			break;
		case 8:
			customChar(eight);
			break;
		case 9:
			customChar(nine);
			break;
	}
}

// ************************************************
// Display large font character.
// ************************************************
void customChar( uint8_t character[] )
{
	lcd.setCursor(x, 0);
	lcd.write(character[0]);
	lcd.write(character[1]);
	lcd.write(character[2]);

	lcd.setCursor(x, 1);
	lcd.write(character[3]);
	lcd.write(character[4]);
	lcd.write(character[5]);

	x += 3;
}

// ************************************************
// Set Backlight based on the state of control
// ************************************************
#if defined(COLOR)
void setBacklight()
{
   if (tuning) {
      lcd.setBacklight(VIOLET); // Tuning Mode
   } else if (abs(Input - Setpoint) > 1.0)   {
      lcd.setBacklight(RED);  // High Alarm - off by more than 1 degree
   } else if (abs(Input - Setpoint) > 0.2)   {
      lcd.setBacklight(YELLOW);  // Low Alarm - off by more than 0.2 degrees
   } else {
      lcd.setBacklight(WHITE);  // We're on target!
   }
}
#endif




// ************************************************
// EEPROM functions
// ************************************************


// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }

   if (currentUnits != EEPROM.read(configAddress)) {
	   EEPROM.write(configAddress, currentUnits);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   currentUnits = EEPROM.read(configAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = SETPOINT_DEF;
   }
   if (isnan(Kp) || Kp < 0)
   {
     Kp = KP_DEF;
   }
   if (isnan(Ki) || Ki < 0)
   {
     Ki = KI_DEF;
   }
   if (isnan(Kd) || Kd < 0)
   {
     Kd = KD_DEF;
   }
   if (F != currentUnits) {
	   currentUnits = C;
   }
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;

   for (int i = 0; i < sizeof(value); i++) {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;

   for (int i = 0; i < sizeof(value); i++) {
      *p++ = EEPROM.read(address++);
   }

   return value;
}