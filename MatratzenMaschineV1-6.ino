#include <Controllino.h> /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch.*/

// the setup function runs once when you press reset (CONTROLLINO RST button) or connect the CONTROLLINO to the PC

// the zylinders are accessed in an array. there are two functions to interact with all zylinders:
// 1) obtain_zylinder_state([array])
//    Input is an array with all indices of the zylinders which are to read
//    returns an array with one entry per zylinder. The states are
//      -1: Zylinder is contracted
//       1: Zylinder is extracted
//       0: The state of the zylinder is unkown (somewhere in between)
//    It iterates over the sensor inputs twice within a short period of time,
//    When both measurements are within the same range, the state is given, otherwhise 0 is returned
// 2) change_zylinder_state([array])
//    Input is an array encoded with 0, -1, 1:
//    For each nonzero entry the function will either send a signal to contract or extract the zylinder whose id matches


// Some Notion:
// Each Zylinder has two outputs and one input.
//      Therefore, the state array is 1xn,
//      and the do array is 2xn
// Example with three zylinders, set state and then wait for completion:
// array doStat = {1,-1,0}
// the machine will only care about zylinders in pos 0 and 1, as they differ from 0
// All Zeros are neclected.
//
// changelog: 
// 1) introduced f_valve_activation_timer() to allow for longer times
// when firing the valves, f_short_delay might be too short
// 2) changed f_change_zylinder_state from bool to void, as there is no return value


// ########### Global I/O Mapping Def ###################:
// all zylinder Outputs
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

// all zylinder inputs/positions --> constants
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
// if wiring is messed up, one can permutate the sequence the zylinders here
// last one is actually two zylinders that are set by same valve but checked by two set of sensors
int isZylStateMap[] = {
  isZ01Pos, isZ02Pos, isZ03Pos, isZ04Pos, isZ05Pos, isZ06Pos, isZ07Pos, isZ08Pos, isZ09Pos, isZ10Pos, isZ11Pos          
  };

byte doZylShortMap[] = {
  doZ01short, doZ02short, doZ03short, doZ04short, doZ05short, doZ05short, doZ06short, doZ07short, doZ08short, doZ09short, doZ10short
  };

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
const byte mapDebugRelais = CONTROLLINO_R1;
const int numZylinders = sizeof(isZylStateMap) / sizeof(isZylStateMap[0]);
// flag for end of line:
const byte mapEndOfLine = CONTROLLINO_A14;


void setup ()
{
  // initialise all used digital output pins as outputs
  // beginn with interrupts:
  attachInterrupt (digitalPinToInterrupt (resetInterruptPin), isr_reset, CHANGE);  // attach interrupt handler --> detects if circuit is opened
  attachInterrupt (digitalPinToInterrupt (stopInterruptPin), isr_stop_at_end_of_cycle, CHANGE); // attach interrupt handler --> detects if circuit is opened
  // initialise all inputs from user control panel:
  pinMode(mapNcStart, INPUT);
  // pinMode(mapStopAtCycleEnd, INPUT);
  pinMode(mapDoCycle, INPUT);
  pinMode(mapModeChoice, INPUT);
  pinMode(mapEndOfLine,INPUT);
  // initialise all zylinder in& outputs:
  for (int i = 0; i < numZylinders; i++) {
    pinMode(isZylStateMap[i], INPUT);
    pinMode(doZylShortMap[i], OUTPUT);
    pinMode(doZylLongMap[i], OUTPUT);
  }
  // define all leds which we display to the user:
  pinMode(mapStartLight, OUTPUT);
  pinMode(mapStopLight, OUTPUT);
  pinMode(mapResetLight, OUTPUT);
  pinMode(mapDebugRelais, OUTPUT);
  Serial.begin(9600);
  f_wait_for_nc_start();
}

// ############# Define all Zylinder states: #############

// Are all Constant, therefore no problem that they are global

const int doStateArray[][numZylinders] = {
  // Zylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  {-1, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  {00,  1, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  {00, 00, 00, 00, 00, 00,  1, 00, 00, 00, 00},
  {00, 00, 00, 00, -1, -1, 00, 00, 00, 00, 00},
  {00, -1, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  {00, 00, 00,  1, 00, 00, 00, 00, 00, 00, 00},
  { 1, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  {00, 00, 00, 00, 00, 00, 00,  1, 00, 00, 00},
  {00, 00, 00, 00,  1,  1, 00, 00, 00, 00, 00},
  {00, 00, 00, 00, 00, 00, -1, -1, 00, 00, 00},
  {00, 00, 00, -1, 00, 00, 00, 00, 00, -1, -1},
  {00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  {00, 00, -1, 00, 00, 00, 00, 00, 00, 00, 00},
  {00, 00, 00, 00, 00, 00, 00, 00, 00, 00,  1},
  {00, 00, 00, 00, 00, 00, 00, 00, -1,  1, 00},
  {00, 00,  1, 00, 00, 00, 00, 00, 00, 00, 00},
  {00, 00, 00, 00, 00, 00, 00, 00,  1, 00, 00},
};

const int doTestArray[][numZylinders] = {
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  {00, -1, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  {00,  1, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  {00, -1, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  {-1, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  { 1, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  {-1, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00},
  {00, 00, -1, 00, 00, 00, 00, 00, 00, 00, 00},
  {00, 00,  1, 00, 00, 00, 00, 00, 00, 00, 00},
  {00, 00, 00,  1, 00, 00, 00, 00, 00, 00, 00},
  {00, 00, 00, -1, 00, 00, 00, 00, 00, 00, 00},
  {00, 00, 00, 00,  1,  1, 00, 00, 00, 00, 00},
  {00, 00, 00, 00, -1, -1, 00, 00, 00, 00, 00},
  {00, 00, 00, 00, 00, 00,  1, 00, 00, 00, 00},
  {00, 00, 00, 00, 00, 00, -1, 00, 00, 00, 00},
  {00, 00, 00, 00, 00, 00, 00,  1, 00, 00, 00},
  {00, 00, 00, 00, 00, 00, 00, -1, 00, 00, 00},
  {00, 00, 00, 00, 00, 00, 00,  1, 00, 00, 00},
  {00, 00, 00, 00, 00, 00, 00, -1, 00, 00, 00},
  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00},
  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, 00},
  {00, 00, 00, 00, 00, 00, 00, 00,  1, 00, 00},
  {00, 00, 00, 00, 00, 00, 00, 00, 00, 00, -1},
  {00, 00, 00, 00, 00, 00, 00, 00, 00, 00,  1},
  {00, 00, 00, 00, 00, 00, 00, 00, -1, 00, 00},
};
// {{00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00},
//  {00, 00, 00, 00, 00, 00, -1, -1, 00, 00,  1}, // row00
//  { 1, 00, 00, -1,  1,  1, 00, 00, -1,  1, 00}, // row01
//  {00, 00,  1, 00, 00, 00, 00, 00, 00, 00, 00}, // row02
//  {00, 00,  1, 00, 00, 00, 00, 00,  1, 00, 00}, //
// };
//{
//  // test ende
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1,  1}, // row00
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, -1}, // row01
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, -1}, // row02
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1,  1}, // row03
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, 00}, // row04
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00}, // row04
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, 00}, // row05
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00}, // row06
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, 00}, // row07
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00}, // row08
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, 00}, // row09
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00}, // row10
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, 00}, // row11
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00}, // row12
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, 00}, // row13
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00}, // row14
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, 00}, // row15
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00}, // row16
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, 00}, // row17
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00}, // row18
//  {00, 00, 00, 00, 00, 00, 00, 00, 00,  1, 00}, // row19
//  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00}, // row20
//};

const int toSafeStateArray[][numZylinders] = {
  // Zylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  {00, -1, 00, 00, 00, 00, -1, -1, 00, 00,  1}, // row00
  { 1, 00, 00, 00, -1, -1, 00, 00, -1,  1, 00}, // row01
  {00, 00,  1, -1,  1,  1, 00, 00, 00, 00, 00}, // row02
  {00, 00,  1, 00, 00, 00, 00, 00,  1, 00, 00}
};

const int toUnlockStateArray[][numZylinders] = {
  // Zylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  {00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00}, // row00
  {00, 00, 00, 00, 00, 00, 00, 00, -1, 00, -1}, // row01
};

const int toLockStateArray[][numZylinders] = {
  // Zylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  {00, 00, 00, 00, 00, 00, 00, 00,  1, 00, 00}, // row00
  {00, 00, 00, 00, 00, 00, 00, 00, 00, 00,  1}, // row01
};

// ############# main loop: #############

// the loop function runs forever
void loop() {
  bool newStateReached;
  bool machineUnlocked;
  bool isEndOfLine;
  int i;
  int tempArr[numZylinders];
  byte j;
  Serial.print("Start of Loop");
  Serial.print("\n");
  // if start_up --> we want the user to give us a go.
  if (resetFlag) {
    resetFlag = false;
    //dummy reset, to remove later!!
    Serial.print("Machine is in Reset State of Loop");
    Serial.print("\n");
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
    Serial.print("Machine is in Running State of Loop");
    Serial.print("\n");
    // run the code here:
    f_display_running_state();
    // check if we are at the end of line:
    isEndOfLine = f_check_if_line_end();
    
    if (not(stopFlag) & not (isEndOfLine)) {
      for (i = 0; i < (sizeof(doStateArray) / sizeof(doStateArray[0])); i++) {
        if (resetFlag) {
          // reset flag has been raised, interrupt immediately!
          break;
        }
        // copy the temp array slice to make sure that it has the right dimension
        for (j = 0; j < numZylinders; j++) {
          // Serial.print("Extracting State Vec  ");
          tempArr[j] = doStateArray[i][j];
        }
        newStateReached = false;
        //1 do state
        //2 wait for completion
        //3 break if reset is pressed
        f_change_zylinder_state(tempArr, sizeof(tempArr), i + 1);
        Serial.print("State set, wait for completion");
        Serial.print("\n");
        while (not(newStateReached)) {
          newStateReached = f_check_defined_state(tempArr, sizeof(tempArr));
          // newStateReached = newStateReached && f_check_defined_state(tempArr, sizeof(tempArr));
          if (resetFlag) {
            break;
            // reset flag has been raised, interrupt immediately!
          }
        }
        f_break_cycle_on_request();
        if (resetFlag) {
          // reset flag has been raised, interrupt immediately!
          break;
        }
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
  int i;
  int j;
  bool newStateReached;
  int tempArr[numZylinders];
  for (i = 0; i < (sizeof(toUnlockStateArray) / sizeof(toUnlockStateArray[0])); i++) {
    for (j = 0; j < numZylinders; j++) {
      tempArr[j] = toUnlockStateArray[i][j];
    }
    Serial.print("Moving machine to Unlock State");
    Serial.print("\n");
    newStateReached = false;
    //1 do state
    //2 wait for completion
    f_change_zylinder_state(tempArr, sizeof(tempArr), i + 1);
    while (not(newStateReached)) {
      newStateReached = f_check_defined_state(tempArr, sizeof(tempArr));
      // newStateReached = newStateReached && f_check_defined_state(tempArr, sizeof(tempArr));
      if (resetFlag) {
        break;
      }
    }
  }
}

void f_lock_machine() {
  // brings machine from unlock to lock state
  int i;
  int j;
  int tempArr[numZylinders];
  bool newStateReached;
  for (i = 0; i < (sizeof(toLockStateArray) / sizeof(toLockStateArray[0])); i++) {
    Serial.print("Moving machine to Lock State");
    Serial.print("\n");
    // copy temporare array
    for (j = 0; j < numZylinders; j++) {
      tempArr[j] = toLockStateArray[i][j];
    }
    // if (resetFlag){break;}
    newStateReached = false;
    //1 do state
    //2 wait for completion
    f_change_zylinder_state(tempArr, sizeof(tempArr), i + 1);
    while (not(newStateReached)) {
      newStateReached = f_check_defined_state(tempArr, sizeof(tempArr));
      if (digitalRead(mapModeChoice) == LOW){break;}
    }
  }
}


void f_initialise_machine_starting_pos() {
  int i;
  int j;
  int tempArr[numZylinders];
  bool newStateReached;
  // toSafeStateArray
  for (i = 0; i < (sizeof(toSafeStateArray)/sizeof(toSafeStateArray[0])); i++) {
    Serial.print("Moving machine to Safe State");
    Serial.print("\n");
    // if (resetFlag){break;}
    // copy temporare array
    for (j = 0; j < numZylinders; j++) {
      tempArr[j] = toSafeStateArray[i][j];
    }
    newStateReached = false;
    //1 do state
    //2 wait for completion
    f_change_zylinder_state(tempArr, sizeof(tempArr), i + 1);
    while (not(newStateReached)) {
      newStateReached = f_check_defined_state(tempArr, sizeof(tempArr));
      // newStateReached = newStateReached && f_check_defined_state(tempArr, sizeof(tempArr));
      if (resetFlag) {
        break;
      }
    }
  }
}


//############# Function to Interrupt loop #################

// ## A) Interrupt Service Routine (ISR) one for stop and one for reset
void isr_stop_at_end_of_cycle()
{
  Serial.print("stop flag");
  Serial.print("\n");
  stopFlag = true;
}  // end of isr

// Interrupt Service Routine (ISR)
void isr_reset()
{
  Serial.print("reset flag");
  Serial.print("\n");
  resetFlag = true;
}  // end of isr

bool f_check_if_line_end() {
  bool isEndOfLine = true;
  // check if circuit of end detector is closed --> HIGH --> not touched yet
  if (digitalRead(mapEndOfLine)== HIGH){
    isEndOfLine = false;
    Serial.print("Beginning with next cycle");
  }
  return isEndOfLine;
} //f_check_if_line_end

//############# Function to Interrupt loop #################

// ## Ordinary Functions
void f_small_delay() {
  //Serial.print("f_smallDelay");
  // is a helper function to set all smallDelays at one place
  digitalWrite(mapDebugRelais, HIGH);
  delay(100);
  digitalWrite(mapDebugRelais, LOW);
  delay(10);
} // small delay

void f_valve_activation_timer() {
  //Serial.print("f_smallDelay");
  // is a helper function which gives the valves time to react
     delay(20);
} // small delay

void  f_break_cycle_at_end_on_stop_request() {
  Serial.print("f_break_cycle_at_end_on_stop_request");
  Serial.print("\n");
  f_wait_for_nc_start_or_reset();
} // f_break_cycle_at_end_on_stop_request

void f_break_cycle_on_request() {
  Serial.print("f_break_cycle_on_request");
  Serial.print("\n");
  // checks if cycle switch is set to step or cycle
  if (digitalRead(mapDoCycle) == HIGH) {
    f_wait_for_nc_start_or_reset();
  }
} // f_break_cycle_on_request

void f_wait_for_nc_start_or_reset() {
  byte i = 0;
  Serial.print("f_wait_for_nc_start_or_reset");
  Serial.print("\n");
  // illuminate stop light
  digitalWrite(mapStopLight, HIGH);
  digitalWrite(mapStartLight, LOW);
  while (digitalRead(mapNcStart) == LOW && not(resetFlag)) {
    digitalWrite(mapDebugRelais, HIGH);
    delay(10);
    i++;
    // after 50 steps illuminate start button to wait for user to press
    if (i == 49) {
      digitalWrite(mapStartLight, HIGH);
    };
    // after 100 steps no longer illuminate start button to wait for user to press
    if (i >= 99) {
      digitalWrite(mapStartLight, LOW);
      // reset counter to prevent overflow.
      i = 0;
    };
    digitalWrite(mapDebugRelais, LOW);
  }
} // f_wait_for_nc_start_or_reset()

void f_wait_for_nc_start() {
  byte i = 0;
  Serial.print("f_wait_for_nc_start");
  Serial.print("\n");
  // illuminate stop light
  digitalWrite(mapStartLight, LOW);
  while (digitalRead(mapNcStart) == LOW) {
    digitalWrite(mapDebugRelais, HIGH);
    delay(10);
    i++;
    // after 50 steps illuminate start button to wait for user to press
    if (i == 49) {
      digitalWrite(mapStartLight, HIGH);
    };
    // after 100 steps no longer illuminate start button to wait for user to press
    if (i >= 99) {
      digitalWrite(mapStartLight, LOW);
      // reset counter to prevent overflow.
      i = 0;
    };
    digitalWrite(mapDebugRelais, LOW);
  }
} // f_wait_for_nc_start()


//############# Function to Interact with Zylinders #################

void f_change_zylinder_state(int doState[], int doStateSize, int currStep) {
  int i;
  Serial.print("current step ");
  Serial.print(currStep);
  Serial.print("\n");
  // give command to extract zylinder --> fire valves
  for (i = 0; i < (doStateSize / sizeof(doState[0])); i++) {
    if (doState[i] == 1) {
      digitalWrite(doZylLongMap[i], HIGH);
    };
    if (doState[i] == -1) {
      digitalWrite(doZylShortMap[i], HIGH);
    };
  }
  f_valve_activation_timer();
  // take command back --> to prevent from overheating
  for (i = 0; i < (doStateSize / sizeof(doState[0])); i++) {
    //Serial.print(doState[i]);
    //Serial.print("\n");
    if (doState[i] == 1) {
      digitalWrite(doZylLongMap[i], LOW);
    };
    if (doState[i] == -1) {
      digitalWrite(doZylShortMap[i], LOW);
    };
  }
}




bool f_check_defined_state(int doState[], int doStateSize) {
  // copy the arrays first, call by value
  int arr1[numZylinders];
  int arr2[numZylinders];
  byte i;
  bool cond = false;
  for (i = 0; i < numZylinders; i++)
  {
    //Serial.print("Copying States  ");
    //Serial.print(doState[i]);
    //Serial.print(i);
    arr1[i] = 0;
    arr2[i] = 0;
    //Serial.print("\n");
  }
  // now take each signal twice, with a short break in between
  // first selective iteration
  for (i = 0; i < (doStateSize / sizeof(doState[0])); i++) {
    //if (doState[i] != 0) {
    //Serial.print("Reading States, Zylinder ");
    //Serial.print(i);
    //Serial.print("State ");
    //Serial.print(doState[i]);
    //Serial.print("\n");
    arr1[i] = analogRead(isZylStateMap[i]);
    //Serial.print(arr1[i]);
    //Serial.print("\n");
    //}
  }
  // small delay before second signal recording
  f_small_delay();
  // second selective iteration
  for (i = 0; i < (doStateSize / sizeof(doState[0])); i++) {
    //if (doState[i] != 0) {
    arr2[i] = analogRead(isZylStateMap[i]);
    //Serial.print(arr2[i] );
    //Serial.print("\n");
    //}
  }
  // are all measurements identical (did the zylinder state not change?)
  cond = f_check_delta_within_range(arr1, arr2, sizeof(arr2));
  if (cond) {
    // the measurements were the same, but are the zylinder already in the do state?
    cond = f_compare_do_and_is(doState, arr2, sizeof(arr2));
  };
  return cond;
}

bool f_check_delta_within_range(int A1[], int A2[], int A2size) {
  // calculates the pairwise delta and checks if its in range
  // if any delta is not in range, return false
  // otherwise return true
  int i;
  int k;
  for (i = 0; i < numZylinders; i++) {
    k = A1[i] - A2[i];
    if (abs(k) > 10) {
      return false;
    }
  }
  // Serial.print("\nCheckComplete\n");
  return true;
}

const int allowedHighRange[][numZylinders] = {
  // Zylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  {798, 797, 780, 798, 778, 780, 796, 795, 795, 795, 795}, // row00
  {808, 807, 790, 808, 788, 790, 806, 805, 805, 805, 805}, // row01
};

const int allowedLowRange[][numZylinders] = {
  // Zylinder ID:
// 01, 02, 03, 04, 05, 05, 06, 07, 08, 09, 10
  {711, 710, 695, 710, 708, 708, 710, 710, 710, 710, 710}, // row00
  {721, 720, 705, 720, 718, 718, 720, 720, 720, 720, 720}, // row01
};

bool f_compare_do_and_is(int doState[], int isState[], int doStateSize) {
  // compares the states, if not in specified range break immediately by return false
  // if run is complete, return true
  bool cond = true;
  byte i;
  for (i = 0; i < numZylinders; i++) {
    //Serial.print(isState[i]);
    //Serial.print("\n");
    //Serial.print(isState[i]);
    //Serial.print("\n");
    if (doState[i] == 1) {
      if (not(isState[i] > allowedHighRange[0][i] && isState[i] < allowedHighRange[1][i])) {
        cond = false;
        /*Serial.print(i);
        Serial.print(" at ");
        Serial.print(1);
        Serial.print(" with value ");
        Serial.print(isState[i]);
        Serial.print(" is not yet where it should be \n");
        */
      }
    }
    if (doState[i] == -1) {
      if (not(isState[i] > allowedLowRange[0][i] && isState[i] < allowedLowRange[1][i])) {
        cond = false;
        /*
        Serial.print(i);
        Serial.print(" at ");
        Serial.print(-1);
        Serial.print(" with value ");
        Serial.print(isState[i]);
        Serial.print(" is not yet where it should be \n");
        */
      }
    }
  }
  if (cond == true) {
      for (i = 0; i < numZylinders; i++) {
        Serial.print(i);
        Serial.print(" at ");
        Serial.print(isState[i]);
        Serial.print(" ");
      }
     Serial.print(" Everything is fine, every zylinder is where it should be \n");
  }
  
  return cond;
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
