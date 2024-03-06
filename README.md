# CONTROLLINO Mega Zylinder Control Script

## Introduction

This Arduino script is specifically designed for the CONTROLLINO Mega platform to manage and control pneumatic cylinders (termed as "zylinders" within the script). It employs the CONTROLLINO library for easier interaction with the hardware's I/O pins and provides robust functions for reading and adjusting the states of multiple cylinders based on sensor feedback.

## Overview

The script controls zylinders through a series of input and output mappings, leveraging digital and analog I/O pins of the CONTROLLINO Mega. It includes functions for obtaining zylinder states, changing zylinder states, initializing the system to a safe starting position, and handling manual or automatic control modes with interrupt capabilities for user inputs or completion of cycles.

### Key Functionalities

- **Zylinder State Management**: Detect and change the states (extracted, contracted, unknown) of up to 11 zylinders.
- **Double-check Mechanism**: Ensures accuracy by double-checking sensor readings before determining a zylinder's state.
- **Safety Initialization**: On startup or reset, initializes the system to a predefined safe state.
- **User Control Modes**: Supports manual and automatic modes, allowing for flexible operation based on user needs.
- **Interrupt Handling**: Provides mechanisms to handle stop and reset requests during operation.

## Setup Instructions

### Hardware Setup

1. **Zylinders Connection**: Connect each pneumatic cylinder to the CONTROLLINO Mega according to the specified output mappings for short and long movements (e.g., `doZ01short` for a short movement of cylinder 1, `doZ01long` for a long movement).
2. **Sensor Setup**: Attach sensors monitoring the positions of zylinders to the analog inputs as defined in the script (e.g., `isZ01Pos` for the position sensor of cylinder 1).
3. **Control Inputs**: Connect switches or buttons to the designated control input pins to manage start, stop, and mode selection functionalities.

### Software Setup

1. Ensure the Arduino IDE and CONTROLLINO library are installed on your computer.
2. Open the script in the Arduino IDE.
3. Upload the script to your CONTROLLINO Mega device.

## Script Walkthrough

### Initialization (`setup()`)

- Configures I/O pin modes for zylinder control and user inputs.
- Attaches interrupt service routines for handling stop and reset buttons.
- Waits for a start command to transition from an initial safe state to active operation.

### Main Operation Loop (`loop()`)

- **Start/Reset Handling**: Checks for reset flags to reinitialize the system or proceed with operational sequences.
- **Operational Sequence**: Iterates through predefined zylinder state arrays to execute sequences of zylinder movements.
- **Sensor Feedback**: Uses sensor readings to confirm zylinder states before proceeding to the next step in the sequence.

### Zylinder Control Functions

- `f_change_zylinder_state()`: Sends signals to zylinders to change their states (extracted or contracted) based on input arrays specifying the desired states for each zylinder.
- `f_check_defined_state()`: Compares sensor readings against expected values to verify if zylinders have reached their intended states.

### Safety and Manual Control

- Functions to lock (`f_lock_machine()`), unlock (`f_unlock_machine()`), and initialize machine to safe positions (`f_initialise_machine_starting_pos()`) are provided to ensure operational safety and manual intervention capabilities.

### Utility Functions

- Include delays (`f_small_delay()`, `f_valve_activation_timer()`) for timing adjustments and display functions (`f_display_reset_state()`, `f_display_running_state()`) for visual feedback through onboard LEDs.

## Usage

- **Automatic Mode**: Once started, the script autonomously cycles through the predefined zylinder state sequences until interrupted.
- **Manual Mode**: Allows users to manually intervene, reposition zylinders, or reset the system to its safe state.
- **Interruption Handling**: Users can stop the cycle at any time or initiate a system reset to return to the safe state.

