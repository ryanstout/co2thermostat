// TODO: Needs to have a trigger on the up (too much), and then needs to have a value it goes below to close.

// 11 - IR LED positive
// 8 - Relay
// 3 - Blue button
// 2 - Red button


#include <IRremote.h>
#include <Wire.h>
#include "SparkFun_SCD4x_Arduino_Library.h" // Include the SCD40 library

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

#include <Adafruit_MCP9808.h>

#define LOG 0

enum VanState {
  IDLE, // Not heating or venting
  HEATING,
  VENTING
};

VanState vanState = IDLE;

volatile int displayState = 0;
volatile int editing = 0; // when 1, we're editing in the current display state



// How many degrees (F) below the setpoint do we stop heating
#define LOW_TEMP_HYSTERESIS 2

// How many degrees (F) above the setpoint do we open the fan
#define HIGH_TEMP_HYSTERESIS 2


// What CO2 reading do we start venting at
#define START_VENTING 1200

// What CO2 reading do we stop venting at
#define STOP_VENTING 700

// TIMINGS ----------------------------
// how many ticks do we average over before running the main loop (to avg CO2)
#define MAIN_RUN_TICKS 6

// How many ms to sleep for between main ticks
#define SLEEP_TIME 500

// The minimum run time for the heater
#define RUN_HEATING_HOLD_TIME 60*1000

// How long to wait after the heater stops (before venting)
#define STOP_HEATING_HOLD_TIME 120*1000 // 2 mins

// How long to wait for the vent to close
#define VENTING_HOLD_TIME 30*1000


// TICKS ------------------------


// How many ticks to hold a state for before looking at changing it
#define RUN_HEATING_HOLD_TICKS ((long)(RUN_HEATING_HOLD_TIME * MAIN_RUN_TICKS) / SLEEP_TIME)

// How many ticks to hold a state for before looking at changing it
#define STOP_HEATING_HOLD_TICKS ((long)(STOP_HEATING_HOLD_TIME * MAIN_RUN_TICKS) / SLEEP_TIME)

// How many ticks to hold a state for before looking at changing it
#define VENTING_HOLD_TICKS (((long)VENTING_HOLD_TIME * MAIN_RUN_TICKS) / SLEEP_TIME)

// the current temperature
int vanTemp = -1000;

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

Adafruit_7segment matrix = Adafruit_7segment();

SCD4x co2Sensor;

IRsend irsend(11); // TODO: was 3
int isOpen = 0; // 0 = closed, 1 = open
unsigned int ticks = 0;

// When we don't want to do anything for a while, we set this to some number of ticks
// and it will decrement. When it reaches zero, we start running again.
uint16_t skipTicks = 0;

// Example sequence (converted to positive integers)
// unsigned int irSignalOpen[] = { 1664, 828, 844, 1644, 844, 816, 844, 1644, 844, 816, 844, 820, 1664, 824, 840, 2476, 6644, 2484, 848, 5792, 840, 1648, 5828, 816, 840, 1644, 844, 4968, 840, 2472, 5000, 816, 1668, 1648, 840, 4140, 840, 3304, 4168, 820, 2492, 1656, 2492, 1660, 1672, 3300, 836, 828, 844, 1644, 3340, 1636, 2524, 3288, 1672, 1644, 844, 820, 840, 2476, 848, 812, 848, 1640, 844, 8280, 840, 1652, 2496, 824, 1664, 1652, 4176, 1636, 840, 2476, 844, 820, 5828 };
// unsigned int irSignalClose[] = { 1668, 824, 840, 1648, 848, 812, 848, 1640, 844, 820, 844, 812, 1672, 820, 840, 2476, 6644, 2488, 844, 5796, 836, 1652, 5820, 824, 844, 1644, 844, 4964, 844, 2472, 4996, 820, 1668, 1648, 848, 4132, 840, 3308, 4168, 820, 2492, 1656, 2492, 1660, 1672, 3300, 848, 816, 844, 1644, 3348, 1632, 2516, 3296, 1664, 1652, 844, 824, 840, 2476, 844, 812, 848, 1640, 848, 8280, 836, 1652, 2496, 828, 1668, 1648, 4172, 1640, 844, 2472, 840, 824, 5820 };

// Open 50%
unsigned int irSignalOpen[] = { 1672, 856, 816, 1672, 816, 816, 848, 1668, 816, 820, 844, 844, 1644, 820, 840, 2476, 6656, 2480, 844, 5796, 852, 1668, 5804, 816, 844, 1644, 844, 4964, 844, 2476, 4996, 852, 1636, 1652, 844, 4140, 844, 3300, 4172, 852, 2460, 1688, 2464, 1692, 1640, 3304, 844, 820, 844, 1676, 3348, 1632, 1640, 860, 1636, 1680, 1644, 1676, 812, 828, 844, 2472, 840, 820, 840, 1648, 840, 8292, 840, 1676, 2476, 848, 1636, 1684, 1640, 4144, 840, 2476, 848, 820, 5828  };
// Close 50%
unsigned int irSignalClose[] = { 1676, 820, 840, 1648, 852, 840, 808, 1648, 840, 852, 820, 816, 1672, 820, 840, 2476, 6648, 2488, 844, 5800, 848, 1668, 5804, 848, 812, 1672, 816, 4968, 840, 2480, 5004, 844, 1644, 1644, 840, 4144, 844, 3304, 4176, 816, 2500, 1648, 2496, 1660, 1668, 3304, 2492, 868, 4148, 1660, 1644, 856, 1640, 1676, 1648, 1672, 816, 852, 820, 2468, 844, 820, 840, 1672, 816, 8288, 848, 1672, 2464, 860, 1640, 1648, 848, 2480, 844, 1668, 816, 2476, 840, 852, 5804 };


const int upButtonPin = 2;     // the number of the pushbutton pin
const int downButtonPin = 3;     // the number of the pushbutton pin
volatile int lowSetPoint = 50; // target low temperature
volatile int highSetPoint = 95; // target high temperature
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 200;    // the debounce time; increase if the output flickers


void setup() {
  Serial.begin(9600);
  Serial.println("---------------- starting ------------------------");

  Wire.begin();

  pinMode(upButtonPin, INPUT_PULLUP); // initialize the pushbutton pin as an input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(upButtonPin), upButtonPressed, RISING); // attach interrupt

  pinMode(downButtonPin, INPUT_PULLUP); // initialize the pushbutton pin as an input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(downButtonPin), downButtonPressed, RISING); // attach interrupt

  // Initialize pin 8 as an output
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  matrix.begin(0x70);
  matrix.setBrightness(1); // out of 15

  // Begin communication with the sensor
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808!");
    while (1);
  }

  if (co2Sensor.begin() == false) {
    Serial.println("CO2 Sensor not detected. Please check wiring. Freezing...");
    while (1);
  }
}

void upButtonPressed() {
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the current time is greater than the last debounce time plus the debounce delay
    // Serial.println("Button pressed");
    // skipTicks = 0;
    if (displayState == 2) {
      setSetPoint(highSetPoint + 1, true);
    } else {
      setSetPoint(lowSetPoint + 1, false);
      displayState = 1;
    }
    editing = 1; // enter editing mode if we're not there already
    lastDebounceTime = millis(); // reset the debounce timer

  }
}

void downButtonPressed() {
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the current time is greater than the last debounce time plus the debounce delay
    // Serial.println("Button pressed");
    // skipTicks = 0;
    if (displayState == 2) {
      setSetPoint(highSetPoint - 1, true);
    } else {
      displayState = 1;
      setSetPoint(lowSetPoint - 1, false);
    }
    editing = 1; // enter editing mode if we're not there already
    lastDebounceTime = millis(); // reset the debounce timer
  }
}

void showTempAndSet(int left, int right) {
  matrix.writeDigitNum(0, left / 10);     // Tens place of the left number
  matrix.writeDigitNum(1, left % 10);     // Units place of the left number
  matrix.writeDigitNum(3, right / 10);    // Tens place of the right number
  matrix.writeDigitNum(4, right % 10);    // Units place of the right number
  matrix.writeDisplay();                  // Update the display with the new values
}

void setSetPoint(int newValue, bool isHighPoint) {
  if (isHighPoint) {
    if (newValue <= lowSetPoint + 5) {
      lowSetPoint = newValue - 5;
    }
    highSetPoint = newValue;
  } else {
    if (newValue >= highSetPoint - 5) {
      highSetPoint = newValue + 5;
    }
    lowSetPoint = newValue;
  }
}

void loop() {

  /* Self Test ------------------------------------------------*/
  //We need to stop periodic measurements before we can run the self test
  // if (co2Sensor.stopPeriodicMeasurement() == true)
  // {
  //   Serial.println(F("Periodic measurement is disabled!"));
  // }  

  // //Now we can run the self test:
  // Serial.println(F("Starting the self-test. This will take 10 seconds to complete..."));

  // bool success = co2Sensor.performSelfTest();

  // Serial.print(F("The self test was "));
  // if (success == false)
  //   Serial.print(F("not "));
  // Serial.println(F("successful"));
  /* ENd Self Test ------------------------------------------------*/

  switch(displayState) {
    case 0:
      showSegmentDisplay(' ', vanTemp); // Display current temperature
      break;
    case 1:
      showSegmentDisplay('L', lowSetPoint); // Display "L" and low set point
      break;
    case 2:
      showSegmentDisplay('H', highSetPoint); // Display "H" and high set point
      break;
    case 3:
      uint16_t co2Val = co2Sensor.getCO2();
      showSegmentDisplay('C', (int)co2Val); // Display "C" and current CO2 reading
      break;
  }

  // showTempAndSet(lowSetPoint, (int)vanTemp);

  ticks += 1;
  if (ticks > MAIN_RUN_TICKS) {
    // Read temperature from the sensor
    float rawTemp = tempsensor.readTempC();
    if (rawTemp != 0.0) {
      vanTemp = (int)celsiusToFahrenheit(rawTemp);
    }

    // Always read the CO2 Value
    if (LOG) {
      Serial.print("CO2(ppm): ");
    }
    bool co2read = co2Sensor.readMeasurement();
    uint16_t co2Val = co2Sensor.getCO2();
    if (LOG) {
      Serial.println(co2Val);
    }


    // Print the temperature to the Serial Monitor
    if (LOG) {
      Serial.print(vanStateToStr(vanState));
      Serial.print(" Set: ");
      Serial.print(lowSetPoint);
      Serial.print(" Temp: ");
      Serial.println(vanTemp);
    }

    if (editing == 0) {
      displayState = (displayState + 1) % 4;
    } else {
      editing += 1;
      if (editing > 2) {
        // 10 seconds since last edit
        editing = 0;
      }
    }


    if (skipTicks == 0) {
      if (vanTemp != -1000) {
        if (vanState == HEATING && vanTemp > (lowSetPoint+LOW_TEMP_HYSTERESIS)) {
          // stop heating if we're above lowSetPoint+LOW_TEMP_HYSTERESIS
          changeVanState(IDLE);
        }

        if (vanState == IDLE && vanTemp < lowSetPoint) {
          // heat if we need
          Serial.println("Get Heat");
          changeVanState(HEATING);
        }

        if (vanState == IDLE && vanTemp > (highSetPoint+HIGH_TEMP_HYSTERESIS)) {
          // open the vent if we're above highSetPoint+HIGH_TEMP_HYSTERESIS
          changeVanState(VENTING);
        }

        // If we're not heating or we are venting, read the CO2
        if ((vanState == IDLE || vanState == VENTING) && co2read) {

          // Serial.print(" Temperature(C): ");
          // Serial.print(co2Sensor.getTemperature(), 1);

          // Serial.print(" Humidity(%): ");
          // Serial.println(co2Sensor.getHumidity(), 1);

          // See if we're over the threshold to start venting (co2 or temp)
          if (vanState == IDLE && (co2Val > START_VENTING || vanTemp > (highSetPoint+HIGH_TEMP_HYSTERESIS))) {
            changeVanState(VENTING);
          } else if (vanState == VENTING && (co2Val < STOP_VENTING && ((int)vanTemp) < highSetPoint)){
            // See if we're under the stop point for venting, both co2 and temp
            changeVanState(IDLE);
          }
        }
      }
      
    } else {
      // skipTicks > 0
      skipTicks -= MAIN_RUN_TICKS; // only being run every MAIN_RUN_TICKS
      // Serial.print(".");
      if (LOG) {
        Serial.print("S: ");
        Serial.println(skipTicks);
      }
    }

    ticks = 0;
  }

  delay(SLEEP_TIME);
}

void showSegmentDisplay(char letter, int value) {
    matrix.clear();
    
    // Display the letter
    if (value >= 1000) {
      // No space for C, show the full value
      matrix.writeDigitNum(0, value / 1000);
      value %= 1000; // remove the thousands place from the value
    } else {
      matrix.writeDigitRaw(0, encode(letter));
    }

    // If the letter is 'C', display the hundreds place of the value in the second digit
    if (value >= 100) {
        matrix.writeDigitNum(1, value / 100);
        value %= 100; // remove the hundreds place from the value
    } else {
        matrix.writeDigitRaw(1, 0x00);
    }

    // Display the tens place of the value
    matrix.writeDigitNum(3, value / 10);
    value %= 10; // remove the tens place from the value

    // Display the units place of the value
    matrix.writeDigitNum(4, value);

    // Send the data to the display
    matrix.writeDisplay();
}

// void showSegmentDisplay(char letter, int value) {
//     matrix.clear();
    
//     matrix.writeDigitNum(1, value / 10);    // Tens place of the right number
//     matrix.writeDigitNum(2, value % 10);    // Units place of the right number
    
//     matrix.writeDigitNum(3, value / 10);    // Tens place of the right number
//     matrix.writeDigitNum(4, value % 10);    // Units place of the right number
//     matrix.writeDisplay();                  // Update the display with the new values

// }

uint8_t encode(char letter) {
  switch(letter) {
    case ' ':
      return 0x00; // All segments off
    case 'L':
      return 0x38; // Segments for 'L'
    case 'H':
      return 0x76; // Segments for 'H'
    case 'C':
      return 0x39; // Segments for 'C'
    default:
      return 0x00; // All segments off for unknown characters
  }
}



const char* vanStateToStr(VanState s) {
    switch (s) {
        case IDLE: return "IDLE";
        case HEATING: return "HEATING";
        case VENTING: return "VENTING";
        default: return "[Unknown State]";
    }
}

void changeVanState(VanState newState) {
  Serial.print("States: ");
  Serial.print(vanStateToStr(vanState));
  Serial.print(" => ");
  Serial.println(vanStateToStr(newState));

  // Longer holds after heating to let things dissipate.
  if (newState == HEATING) {
    // Hold any changes for a minimum amount of time
    Serial.print("hold for ");
    Serial.println(RUN_HEATING_HOLD_TICKS);
    skipTicks = RUN_HEATING_HOLD_TICKS;
  }

  if (vanState == HEATING) {
    // how long to hold after heating finishes
    Serial.print("hold for ");
    Serial.println(STOP_HEATING_HOLD_TICKS);
    skipTicks = STOP_HEATING_HOLD_TICKS;
  }

  // After venting, we can go to heating pretty quikcly
  if (vanState == VENTING || newState == VENTING) {
    Serial.print("hold for ");
    Serial.println(VENTING_HOLD_TICKS);
    skipTicks = VENTING_HOLD_TICKS;
  }

  if (vanState == VENTING && newState == IDLE) {
    // We're leaving venting, close the vent
    vanState = newState;
    Serial.println("Close Fan");
    irsend.sendRaw(irSignalClose, sizeof(irSignalClose) / sizeof(irSignalClose[0]), 38);  // 38 kHz is a common frequency
    isOpen = 0;
  }

  if (vanState == IDLE && newState == VENTING) {
    // Open the vent
    vanState = newState;
    Serial.println("Open Fan");
    irsend.sendRaw(irSignalOpen, sizeof(irSignalOpen) / sizeof(irSignalOpen[0]), 38);  // 38 kHz is a common frequency
    isOpen = 1;
  }
  if (vanState == IDLE && newState == HEATING) {
    // Start heating the van
    vanState = newState;
    Serial.println("Start Heating");
    digitalWrite(8, HIGH);
  }

  if (vanState == HEATING && newState == IDLE) {
    vanState = newState;
    Serial.println("Stop Heating");
    digitalWrite(8, LOW);
  }

}

float convertToCelsius(int sensorValue) {
  // Add the conversion logic based on your sensor's specification
  // This is just a placeholder
  float temperatureC = (sensorValue / 1024.0) * 5.0 * 100.0; // Example conversion
  return temperatureC;
}

// Function to convert Celsius to Fahrenheit
float celsiusToFahrenheit(float celsius) {
  return (celsius * 9.0 / 5.0) + 32.0;
}
