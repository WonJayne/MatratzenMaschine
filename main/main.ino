#include <Controllino.h> /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch.*/

// Disable debug output by setting DEBUG_ENABLED to 0
#define DEBUG_ENABLED 1


// the setup function runs once when you press reset (CONTROLLINO RST button) or connect the CONTROLLINO to the PC

// the cylinders are accessed in an array. there are two functions to interact with all cylinders:
// 1) obtain_cylinder_state([array])
//    Input is an array with all indices of the cylinders which are to read
//    returns an array with one entry per cylinder. The states are
//      -1: Cylinder is contracted
//       1: Cylinder is extracted
//       0: The state of the cylinder is unknown (somewhere in between)
//    It iterates over the sensor inputs twice within a short period of time,
//    When both measurements are within the same range, the state is given, otherwise 0 is returned
// 2) change_cylinder_state([array], [array])
//    Input is an array encoded with 0, -1, 1:
//    For each nonzero entry the function will either send a signal to contract or extract the cylinder whose id matches


// Some Notion:
// Each Cylinder has two outputs and one input.
//      Therefore, the state array is 1xn,
//      and the do array is 2xn
// Example with three cylinders, set state and then wait for completion:
// array doStat = {1,-1,0}
// the machine will only care about cylinders in pos 0 and 1, as they differ from 0
// All Zeros are neglected.
//
// changelog:
// 1) introduced f_valve_activation_timer() to allow for longer times
// when firing the valves, f_short_delay might be too short
// 2) changed f_change_cylinder_state from bool to void, as there is no return value


// ########### Global I/O Mapping Def ###################:
// all cylinder Outputs
byte doZ01short = CONTROLLINO_D0;
byte doZ01long  = CONTROLLINO_D1;
byte doZ02short = CONTROLLINO_D2;
byte doZ02long  = CONTROLLINO_D3;
byte doZ03short = CONTROLLINO_D4;
byte doZ03long  = CONTROLLINO_D5;
byte doZ04short = CONTROLLINO_D6;
byte doZ04long  = CONTROLLINO_D7;
byte doZ05short = CONTROLLINO_D8;
byte doZ05long  = CONTROLLINO_D9;
byte doZ06short = CONTROLLINO_D10;
byte doZ06long  = CONTROLLINO_D11;
byte doZ07short = CONTROLLINO_D12;
byte doZ07long  = CONTROLLINO_D13;
byte doZ08short = CONTROLLINO_D14;
byte doZ08long  = CONTROLLINO_D15;
byte doZ09short = CONTROLLINO_D16;
byte doZ09long  = CONTROLLINO_D17;
byte doZ10short = CONTROLLINO_D18;
byte doZ10long  = CONTROLLINO_D19;

// all cylinder inputs/positions --> constants
int isZ01Pos = CONTROLLINO_A0;
int isZ02Pos = CONTROLLINO_A1;
int isZ03Pos = CONTROLLINO_A2;
int isZ04Pos = CONTROLLINO_A3;
int isZ05Pos = CONTROLLINO_A4;
int isZ06Pos = CONTROLLINO_A5;
int isZ07Pos = CONTROLLINO_A6;
int isZ08Pos = CONTROLLINO_A7;
int isZ09Pos = CONTROLLINO_A8;
int isZ10Pos = CONTROLLINO_A9;
int isZ11Pos = CONTROLLINO_A10;

// conclude all information in the arrays:
// if wiring is messed up, one can permutate the sequence the cylinders here
// last one is actually two cylinders that are set by same valve but checked by two set of sensors
int isZylStateMap[] = {
  isZ01Pos, isZ02Pos, isZ03Pos, isZ04Pos, isZ05Pos, isZ06Pos, isZ07Pos, isZ08Pos, isZ09Pos, isZ10Pos, isZ11Pos
  };

// define all outputs which are given by the user via control (note that doZ05short is duplicated on purpose):
byte doZylShortMap[] = {
  doZ01short, doZ02short, doZ03short, doZ04short, doZ05short, doZ05short, doZ06short, doZ07short, doZ08short, doZ09short, doZ10short
  };

// define all outputs which are given by the user via control (note that doZ05long is duplicated on purpose):
byte  doZylLongMap[] = {
  doZ01long, doZ02long, doZ03long, doZ04long, doZ05long, doZ05long, doZ06long, doZ07long, doZ08long, doZ09long, doZ10long
};

// define all inputs which are given by the user via control:
const byte mapNcStart = CONTROLLINO_I16;
// switch set to cycles
const byte mapDoCycle = CONTROLLINO_I18;
const byte mapModeChoice = CONTROLLINO_A15;
// define outputs which are displayed to the user via control panel:
const byte mapStartLight = CONTROLLINO_D23;
const byte mapStopLight  = CONTROLLINO_D22;
const byte mapResetLight = CONTROLLINO_D21;
// last are the buttons, they are interrupts as we have to remember any push
// parallel switch and sensor to detect end of cycle iteration
// const byte mapStopAtCycleEnd = CONTROLLINO_I17;
const byte stopInterruptPin = CONTROLLINO_IN1;
const byte resetInterruptPin = CONTROLLINO_IN0;
// button push is stored in global flag which has to be volatile!
volatile bool resetFlag = true;
volatile bool stopFlag = false;
const byte mapDebugRelay = CONTROLLINO_R1;
const int numCylinders = sizeof(isZylStateMap) / sizeof(isZylStateMap[0]);
// flag for end of line:
const byte mapEndOfLine = CONTROLLINO_A14;
// Number of times to read the sensor repetitively
const int NUM_SENSOR_READINGS = 5;

// ############# Define all Cylinder states: #############

// Are all Constant, therefore no problem that they are global

const int doStateArray[][numCylinders] = {
  // Cylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  {-1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
  { 0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
  { 0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0},
  { 0,  0,  0,  0, -1, -1,  0,  0,  0,  0,  0},
  { 0, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
  { 0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
  { 1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
  { 0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
  { 0,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0},
  { 0,  0,  0,  0,  0,  0, -1, -1,  0,  0,  0},
  { 0,  0,  0, -1,  0,  0,  0,  0,  0, -1, -1},
  { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
  { 0,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0},
  { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
  { 0,  0,  0,  0,  0,  0,  0,  0, -1,  1,  0},
  { 0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0},
  { 0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
};

const int toSafeStateArray[][numCylinders] = {
  // Cylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  { 0, -1,  0,  0,  0,  0, -1, -1,  0,  0,  1}, // row00
  { 1,  0,  0,  0, -1, -1,  0,  0, -1,  1,  0}, // row01
  { 0,  0,  1, -1,  1,  1,  0,  0,  0,  0,  0}, // row02
  { 0,  0,  1,  0,  0,  0,  0,  0,  1,  0,  0}
};

const int toUnlockStateArray[][numCylinders] = {
  // Cylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  { 0,  0,  0,  0,  0,  0,  0,  0,  0, -1,  0}, // row00
  { 0,  0,  0,  0,  0,  0,  0,  0, -1,  0, -1}, // row01
};

const int toLockStateArray[][numCylinders] = {
  // Cylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  { 0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0}, // row00
  { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}, // row01
};

// ############# Actual Code: #############
// Only include the following code if debugging is enabled
void debugPrint(const String& message) {
  Serial.println(message);
}

void debugPrint(int value) {
  Serial.println(value);
}

void debugPrint(double value) {
  Serial.println(value, 6); // Adjust precision as needed
}
// Add more overloads for other data types as necessary
#else // DEBUG_ENABLED not defined --> create empty implementations
void debugPrint(const String& message) {}
void debugPrint(int value) {}
void debugPrint(double value) {}
// Empty implementations for other data types
#endif

// ############# Functions to interact with Machine: #############
void setup ()
{
  // initialise all used digital output pins as outputs
  // first, start with serial port:
  // define volatile flags for interrupts:
    resetFlag = true; // reset flag is set to true, as we want to start in reset state --> machine moves to safe state
    stopFlag = false;
  #ifdef DEBUG_ENABLED
    Serial.begin(9600);
    // Wait for serial port to connect. Needed for native USB
    while (!Serial) {}
  #endif
  // next, continue with interrupts:
  attachInterrupt (digitalPinToInterrupt (resetInterruptPin), isr_reset, CHANGE);  // attach interrupt handler --> detects if circuit is opened
  attachInterrupt (digitalPinToInterrupt (stopInterruptPin), isr_stop_at_end_of_cycle, CHANGE); // attach interrupt handler --> detects if circuit is opened
  // initialise all inputs from user control panel:
  pinMode(mapNcStart, INPUT);
  // pinMode(mapStopAtCycleEnd, INPUT);
  pinMode(mapDoCycle, INPUT);
  pinMode(mapModeChoice, INPUT);
  pinMode(mapEndOfLine,INPUT);
  // initialise all cylinder in& outputs:
  for (int i = 0; i < numCylinders; i++) {
    pinMode(isZylStateMap[i], INPUT);
    pinMode(doZylShortMap[i], OUTPUT);
    pinMode(doZylLongMap[i], OUTPUT);
  }
  // define all leds which we display to the user:
  pinMode(mapStartLight, OUTPUT);
  pinMode(mapStopLight, OUTPUT);
  pinMode(mapResetLight, OUTPUT);
  pinMode(mapDebugRelay, OUTPUT);

  // actual machine start up:
  f_wait_for_nc_start();
}


// ############# main loop: #############

// the loop function runs forever
void loop() {
  bool newStateReached;
  bool machineUnlocked;
  bool isEndOfLine;
  int tempArr[numCylinders];
  debugPrint("Start of Loop");
  // if start_up --> we want the user to give us a go.
  if (resetFlag) {
    resetFlag = false;
    //dummy reset, to remove later!!
    debugPrint("Machine is in Reset State of Loop");
    //illuminate the reset switch
    f_display_reset_state();
    // bring machine from any state in starting/safe state
    f_initialise_machine_starting_pos();
    machineUnlocked = false;
    // wait for user to switch in manual mode or nc start
    while (digitalRead(mapNcStart) == LOW) {
      while (digitalRead(mapModeChoice) == HIGH) {
        // while in manual mode, wait for user to again switch on machine mode
        // after moving the machine to starting position
        if (not(machineUnlocked)) {
          f_unlock_machine();
          machineUnlocked = true;
        }
      }
      if (machineUnlocked) {
        f_lock_machine();
        machineUnlocked = false;
      }
    }
    f_display_running_state();
    // machine has to be at starting points
    f_initialise_machine_starting_pos();
    stopFlag = true;
    // user gave permission to go ahead
  } else {
    debugPrint("Machine is in Running State of Loop");
    // run the code here:
    f_display_running_state();
    // check if we are at the end of line:
    isEndOfLine = f_check_if_line_end();

    if (not(stopFlag) & not (isEndOfLine)) {
      for (int i = 0; i < (sizeof(doStateArray) / sizeof(doStateArray[0])); i++) {
        if (resetFlag) {
          // reset flag has been raised, interrupt immediately!
          break;
        }
        // copy the temp array slice to make sure that it has the right dimension
        for (int j = 0; j < numCylinders; j++) {
          // debugPrint("Extracting State Vec  ");
          tempArr[j] = doStateArray[i][j];
        }
        newStateReached = false;
        //1 do state
        //2 wait for completion
        //3 break if reset is pressed
        f_change_cylinder_state(tempArr, sizeof(tempArr), i + 1);
        debugPrint("State set, wait for completion");
        while (not(newStateReached)) {
          newStateReached = f_check_defined_state(tempArr, sizeof(tempArr));
          // newStateReached = newStateReached && f_check_defined_state(tempArr, sizeof(tempArr));
          if (resetFlag) {
            break;
            // reset flag has been raised, interrupt immediately!
          }
        }
        f_break_cycle_on_request();
        if (resetFlag) { break;} // reset flag has been raised, interrupt immediately!
      }
    } else {
      // stop flag has been raised, interrupt at end of cycle
      f_break_cycle_at_end_on_stop_request();
      stopFlag = false;
    }
  }
}

//############# Functions to bring Machine in different initial states #################

void f_unlock_machine() {
  // brings machine from locked start state to unlock state
  bool newStateReached;
  int tempArr[numCylinders];
  for (int i = 0; i < (sizeof(toUnlockStateArray) / sizeof(toUnlockStateArray[0])); i++) {
    for (int j = 0; j < numCylinders; j++) {
      tempArr[j] = toUnlockStateArray[i][j];
    }
    debugPrint("Moving machine to Unlock State");
    newStateReached = false;
    // 1 do state
    // 2 wait for completion
    f_change_cylinder_state(tempArr, sizeof(tempArr), i + 1);
    while (not(newStateReached)) {
      newStateReached = f_check_defined_state(tempArr, sizeof(tempArr));
      if (resetFlag) {break;}
    }
  }
}

void f_lock_machine() {
  // brings machine from unlock to lock state
  int tempArr[numCylinders];
  bool newStateReached;
  for (int i = 0; i < (sizeof(toLockStateArray) / sizeof(toLockStateArray[0])); i++) {
    debugPrint("Moving machine to Lock State");
    // copy temporare array
    for (int j = 0; j < numCylinders; j++) {
      tempArr[j] = toLockStateArray[i][j];
    }
    // if (resetFlag){break;}
    newStateReached = false;
    //1 do state
    //2 wait for completion
    f_change_cylinder_state(tempArr, sizeof(tempArr), i + 1);
    while (not(newStateReached)) {
      newStateReached = f_check_defined_state(tempArr, sizeof(tempArr));
      if (digitalRead(mapModeChoice) == LOW){break;}
    }
  }
}


void f_initialise_machine_starting_pos() {
  int i;
  int j;
  int tempArr[numCylinders];
  bool newStateReached;
  // toSafeStateArray
  for (i = 0; i < (sizeof(toSafeStateArray)/sizeof(toSafeStateArray[0])); i++) {
    debugPrint("Moving machine to Safe State");
    // copy temporary array
    for (j = 0; j < numCylinders; j++) {
      tempArr[j] = toSafeStateArray[i][j];
    }
    newStateReached = false;
    //1 do state
    //2 wait for completion
    f_change_cylinder_state(tempArr, sizeof(tempArr), i + 1);
    while (not(newStateReached)) {
      newStateReached = f_check_defined_state(tempArr, sizeof(tempArr));
      if (resetFlag) {break;}
    }
  }
}


//############# Function to Interrupt loop #################

// ## A) Interrupt Service Routine (ISR) one for stop and one for reset
void isr_stop_at_end_of_cycle()
{
  debugPrint("stop flag");
  stopFlag = true;
}  // end of isr

// Interrupt Service Routine (ISR)
void isr_reset()
{
  debugPrint("reset flag");
  resetFlag = true;
}  // end of isr

bool f_check_if_line_end() {
    // check if circuit of end detector is closed --> HIGH --> not touched yet
    if (digitalRead(mapEndOfLine)== HIGH){
        debugPrint("End of Line not reached");
        return false;
    };
    debugPrint("End of Line");
    return true;
} //f_check_if_line_end

//############# Function to Interrupt loop #################

// ## Ordinary Functions
void f_small_delay() {
  //debugPrint("f_smallDelay");
  // is a helper function to set all smallDelays at one place
  #ifdef DEBUG_ENABLED
    digitalWrite(mapDebugRelay, HIGH);
  #endif
  delay(20);
  #ifdef DEBUG_ENABLED
    digitalWrite(mapDebugRelay, LOW);
    delay(20);
  #endif
} // small delay

void f_valve_activation_timer() {
  // is a helper function which gives the valves time to react
  debugPrint("f_valve_activation_timer");
  delay(20);
} // f_valve_activation_timer

void  f_break_cycle_at_end_on_stop_request() {
  debugPrint("f_break_cycle_at_end_on_stop_request");
  f_wait_for_nc_start_or_reset();
} // f_break_cycle_at_end_on_stop_request

void f_break_cycle_on_request() {
  debugPrint("f_break_cycle_on_request");
  // checks if cycle switch is set to step or cycle
  if (digitalRead(mapDoCycle) == HIGH) {
    f_wait_for_nc_start_or_reset();
  }
} // f_break_cycle_on_request

void f_wait_for_nc_start_or_reset() {
  int i = 0;
  debugPrint("f_wait_for_nc_start_or_reset");
  // illuminate stop light
  digitalWrite(mapStopLight, HIGH);
  digitalWrite(mapStartLight, LOW);
  while (digitalRead(mapNcStart) == LOW && not(resetFlag)) {
    delay(1);
    i++;
    // after 500 steps illuminate start button to wait for user to press
    if (i == 499) {
      digitalWrite(mapStartLight, HIGH);
      debugPrint("f_wait_for_nc_start_or_reset, start light on");
    };
    // after 1000 steps no longer illuminate start button to wait for user to press
    if (i >= 999) {
      digitalWrite(mapStartLight, LOW);
      debugPrint("f_wait_for_nc_start_or_reset, start light off");
      i = 0;
    };
  }
} // f_wait_for_nc_start_or_reset()

void f_wait_for_nc_start() {
  int i = 0;
  debugPrint("f_wait_for_nc_start");
  // illuminate stop light
  digitalWrite(mapStartLight, LOW);
  while (digitalRead(mapNcStart) == LOW) {
    delay(1);
    i++;
    // after 250 steps illuminate start button to wait for user to press
    if (i == 499) {
      digitalWrite(mapStartLight, HIGH);
      debugPrint("f_wait_for_nc_start, start light on");
    };
    // after 500 steps no longer illuminate start button to wait for user to press
    if (i >= 499) {
      digitalWrite(mapStartLight, LOW);
        debugPrint("f_wait_for_nc_start, start light off");
      i = 0;
    };
  }
} // f_wait_for_nc_start()


//############# Function to Interact with Cylinders #################

void f_change_cylinder_state(int doState[], int doStateSize, int currStep) {
    // changes the state of the cylinders
  debugPrint("f_change_cylinder_state");
  debugPrint(currStep);
  // give command to extract cylinder --> fire valves
  for (int i = 0; i < (doStateSize / sizeof(doState[0])); i++) {
    if (doState[i] == 1) {
      digitalWrite(doZylLongMap[i], HIGH);
    };
    if (doState[i] == -1) {
      digitalWrite(doZylShortMap[i], HIGH);
    };
  }
  f_valve_activation_timer();
  // take command back --> to prevent from overheating
  for (int i = 0; i < (doStateSize / sizeof(doState[0])); i++) {
    debugPrint(doState[i]);
    if (doState[i] == 1) {
      digitalWrite(doZylLongMap[i], LOW);
    };
    if (doState[i] == -1) {
      digitalWrite(doZylShortMap[i], LOW);
    };
  }
} // f_change_cylinder_state


bool f_check_defined_state(const int doState[], const int doStateSize) {
    // copy the arrays first, call by value
    int readings[numCylinders][NUM_SENSOR_READINGS]; // Array to store all readings

    // Take NUM_SENSOR_READINGS for each sensor
    for (int j = 0; j < NUM_SENSOR_READINGS; j++) {
        for (int i = 0; i < numCylinders; i++) {
            readings[i][j] = analogRead(isZylStateMap[i]);
        }
        if (j < NUM_SENSOR_READINGS - 1){
            f_small_delay(); // Delay between each set of readings
        }
    }

    for (int i = 0; i < numCylinders; i++) {
        if (!f_check_delta_within_range(readings[i], NUM_SENSOR_READINGS)) {
            return false; // If any sensor reading is out of range, set condition to false and return
        }
    }

    // If all sensor readings are within range, proceed to compare with desired states
    float averagedReadings[numCylinders];
    for (int i = 0; i < numCylinders; i++) {
        int sum = 0;
        for (int j = 0; j < NUM_SENSOR_READINGS; j++) {
            sum += readings[i][j];
        }
        // cast to float to avoid integer division
        averagedReadings[i] = (float)sum / (float)NUM_SENSOR_READINGS;
    }
    return f_compare_do_and_is(doState, averagedReadings, doStateSize);
} // f_check_defined_state

bool f_check_delta_within_range(const int readings[], const int numReadings) {
    int minReading = readings[0];
    int maxReading = readings[0];

    for (int i = 1; i < numReadings; i++) {
        if (readings[i] < minReading) minReading = readings[i];
        if (readings[i] > maxReading) maxReading = readings[i];
    }
    // Check if the difference between min and max is within a threshold, e.g., less than 5
    return (maxReading - minReading) < 5;
} // f_check_delta_within_range

const int allowedHighRange[][numCylinders] = {
  // Cylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  {798, 797, 780, 798, 778, 780, 796, 795, 795, 795, 795}, // row00
  {808, 807, 790, 808, 788, 790, 806, 805, 805, 805, 805}, // row01
};

const int allowedLowRange[][numCylinders] = {
  // Cylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  {711, 710, 695, 710, 708, 708, 710, 710, 710, 710, 710}, // row00
  {721, 720, 705, 720, 718, 718, 720, 720, 720, 720, 720}, // row01
};

bool f_compare_do_and_is(int doState[], float isState[], int doStateSize) {
  // compares the states, if not in specified range break immediately by return false, else return true
  for (int i = 0; i < numCylinders; i++) {
    debugPrint(isState[i]);
    debugPrint(isState[i]);
    if (doState[i] == 1) {
      if (not(isState[i] > allowedHighRange[0][i] && isState[i] < allowedHighRange[1][i])) {
        debugPrint(i);
        debugPrint(" at ");
        debugPrint(1);
        debugPrint(" with value ");
        debugPrint(isState[i]);
        debugPrint(" is not yet where it should be ");
        return false;
      }
    }
    if (doState[i] == -1) {
      if (not(isState[i] > allowedLowRange[0][i] && isState[i] < allowedLowRange[1][i])) {
        debugPrint(i);
        debugPrint(" at ");
        debugPrint(-1);
        debugPrint(" with value ");
        debugPrint(isState[i]);
        debugPrint(" is not yet where it should be \n");
        return false;
      }
    }
  }
  
  #ifdef DEBUG_ENABLED
      for (int i = 0; i < numCylinders; i++) {
        debugPrint(i);
        debugPrint(" at ");
        debugPrint(isState[i]);
        debugPrint(" ");
        debugPrint(" Everything is fine, every cylinder is where it should be \n");
      }
  #endif
  return true;
}

//############# Functions for Illumination: #############
void f_display_reset_state() {
  digitalWrite(mapStopLight, LOW);
  digitalWrite(mapStartLight, LOW);
  digitalWrite(mapResetLight, HIGH);
}
void f_display_running_state() {
  digitalWrite(mapStopLight, LOW);
  digitalWrite(mapStartLight, HIGH);
  digitalWrite(mapResetLight, LOW);
}
