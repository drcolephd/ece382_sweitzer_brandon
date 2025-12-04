
#include <stdbool.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/Nokia5110.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA2.h"
#include "../inc/TimerA1.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Bump.h"
#include "../inc/UART0.h"


// Macro to constrain a value within a given range [MIN, MAX]
// If X < MIN, X is set to MIN.
// If X > MAX, X is set to MAX.
// Example:
// x = MINMAX(0, 100, x);  // x will be between 0 and 100.
#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ( (X) > (Max)? (Max) : (X) ) )

// variables to average tachometer readings
#define TACHBUFF_SIZE 10
uint16_t LeftTachoPeriod[TACHBUFF_SIZE];
uint16_t RightTachoPeriod[TACHBUFF_SIZE];


bool IsCollectionDone = false;

uint16_t LeftPeriod;        // Timer period for the left motor
uint16_t RightPeriod;       // Timer period for the right motor
int16_t  LeftDistance_mm;   // Distance traveled by the left motor (in mm)
int16_t  RightDistance_mm;  // Distance traveled by the right motor (in mm)
uint16_t LeftSpeed_rpm;     // Speed of the left motor (in RPM)
uint16_t RightSpeed_rpm;    // Speed of the right motor (in RPM)

#define BUFFER_SIZE 1000    // Size of data buffers for storing measurements

uint16_t SpeedBufferL[BUFFER_SIZE];     // Buffer to store left motor speed data
uint16_t SpeedBufferR[BUFFER_SIZE];     // Buffer to store right motor speed data
uint16_t DistanceBufferL[BUFFER_SIZE];  // Buffer to store left motor distance data
uint16_t DistanceBufferR[BUFFER_SIZE];  // Buffer to store right motor distance data
uint16_t DutyBuffer[BUFFER_SIZE];       // Buffer to store duty cycle values

// Transmit data stored in buffers to the PC via UART.
// This function asks the user whether to transmit data and responds accordingly.
// The decision is made based on the state of the left switch (SWL) to toggle Yes/No and
// the bump sensor to confirm the action. If transmission is selected, buffer data is
// sent to the PC via UART. The function exits if No is selected or after data transmission is complete.
static void TxBuffer(void) {

    // Wait until both the LaunchPad switch and bump sensors are released
    // Ensures that data transmission does not start until both input controls are inactive.
    while(LaunchPad_SW() || Bump_Read()) {
        // Flash the red LED to indicate that the program is waiting for input
        Clock_Delay1ms(200);
        LaunchPad_RGB(RGB_OFF); // LED off
        Clock_Delay1ms(200);
        LaunchPad_RGB(RED); // LED on (red)
    }

    // Variable to track whether transmission is enabled
    bool isTxEnabled = false;

    while(1) {

        // Clear the LCD and display transmission query message
        Nokia5110_Clear();
        Nokia5110_OutString("Tx Buffer?");
        Nokia5110_SetCursor2(2,1);
        Nokia5110_OutString("SWL: Y/N"); // Prompt user to press left switch for Yes/No
        Nokia5110_SetCursor2(4,1);
        Nokia5110_OutString("Bump 2 Enter"); // Instruct user to press bump sensor to confirm action

        // Loop until the bump sensor is pressed to confirm the action
        while(!Bump_Read()) {
            // Display "Y" (Yes) or "N" (No) on the LCD depending on the current state of isTxEnabled
            if (isTxEnabled) {
                Nokia5110_SetCursor2(1,12);
                Nokia5110_OutString("Y");
            } else {
                Nokia5110_SetCursor2(1,12);
                Nokia5110_OutString("N");
            }

            // If the left switch is pressed, toggle the state of isTxEnabled
            // Each press changes the selection between Yes (Y) and No (N)
            if (LaunchPad_SW() == SWL_ON) {
                isTxEnabled ^= 1;  // Toggle Yes/No selection
            }

            // Flash the blue LED to indicate that the system is ready and awaiting user input
            BLUELED ^= 1;
            Clock_Delay1ms(200); // Small delay to create a flashing effect
        }

        // If the bump sensor is pressed and transmission is enabled (Y)
        if(isTxEnabled) {
            // Send buffer data to the PC via UART
            UART0_OutString("\n\r***Receiving buffer data***\n\r");

            // Transmit data stored in Speed, Distance, and Duty buffers
            for (int i = 0; i < BUFFER_SIZE; i++) {
                UART0_OutUDec(i); UART0_OutChar(','); // Send index
                UART0_OutSDec(SpeedBufferL[i]); UART0_OutChar(','); // Send left wheel speed
                UART0_OutSDec(SpeedBufferR[i]); UART0_OutChar(','); // Send right wheel speed
                UART0_OutSDec(DistanceBufferL[i]); UART0_OutChar(','); // Send left wheel distance
                UART0_OutSDec(DistanceBufferR[i]); UART0_OutChar(','); // Send right wheel distance
                UART0_OutSDec(DutyBuffer[i]); UART0_OutString("\n\r"); // Send motor duty cycle
            }

            // Notify the user that transmission is complete
            Nokia5110_SetCursor2(4,1);
            Nokia5110_OutString("TX is Done   ");
            Nokia5110_SetCursor2(5,1);
            Nokia5110_OutString("Bump 2 Cont");

            // Wait for the bump sensor to be pressed again to confirm and exit transmission mode
            while(!Bump_Read());
            Clock_Delay1ms(500);

        } else {
            // If transmission is not enabled (No), exit the function
            break;
        }
    }

    // After transmission or exit, flash the yellow LED 5 times to indicate the function is complete
    for(int k = 0; k < 5; k++){
        LaunchPad_RGB(YELLOW);  // Flash yellow LED
        Clock_Delay1ms(100);    // Delay to create flashing effect
        LaunchPad_RGB(RGB_OFF); // Turn off LED
        Clock_Delay1ms(100);
    }
}

// ------------average------------
// Calculate the average value from an array of 16-bit unsigned integers
// Input: data is an array of 16-bit unsigned numbers
//        data_length is the number of elements in data
// Output: the average value of the data
// Note: overflow is not considered
uint16_t average(uint16_t *data, int data_length) {

    uint32_t sum = 0;
    for (int i = 0; i < data_length; i++) {
        sum += data[i];
    }
    return sum/data_length;
}

// Constants for desired speeds and PWM duty cycle bounds
#define DESIREDMAX 120  // Maximum desired RPM
#define DESIREDMIN 30   // Minimum desired RPM
#define MAXDUTY 999     // Maximum duty cycle (99.9%)
#define MINDUTY 10      // Minimum duty cycle (1.0%)

// Variables to control motor speed
static uint16_t DesiredL_rpm = 80;  // Desired left motor RPM, initially set to 80
static uint16_t DesiredR_rpm = 80;  // Desired right motor RPM, initially set to 80

// Semaphore to control whether the speed controller is enabled
bool IsControllerEnabled = false;

void LCDDesired2(void){
    // Display the desired motor speeds for both left and right motors
    Nokia5110_SetCursor2(2, 2);         // Desired left RPM
    Nokia5110_OutUDec(DesiredL_rpm, 5);
    Nokia5110_SetCursor2(2, 8);         // Desired right RPM
    Nokia5110_OutUDec(DesiredR_rpm, 5);
}

// ========== Update the Desired Speeds ==========
static void UpdateParameters(void) {

    // Wait until any button press is released
    while(LaunchPad_SW()) {
        Clock_Delay1ms(200); LaunchPad_RGB(RGB_OFF);  // Flash off
        Clock_Delay1ms(200); LaunchPad_RGB(RED);      // Flash red LED
    }

    // Stop the motors temporarily while updating parameters
    Motor_Coast();
    Clock_Delay1ms(300);  // Delay for a smooth stop

    // Prompt the user to adjust the desired speeds using switches
    // The loop continues until a bump sensor is pressed
    while(!Bump_Read()) {

        // Display current desired speeds on the LCD
         LCDDesired2();

         // Right switch pressed: Increase desired right speed by 10 RPM, with wrap-around
         if((LaunchPad_SW() & SWR_ON)) {
            // write your code here
            DesiredR_rpm += 10;
            if (DesiredR_rpm > DESIREDMAX) {
                DesiredR_rpm = DESIREDMIN;
            }
            Clock_Delay1ms(250);
        }

         // Left switch pressed: Increase desired left speed by 10 RPM, with wrap-around
         if((LaunchPad_SW() & SWL_ON)){
             // write your code here
             DesiredR_rpm += 10;
             if (DesiredR_rpm > DESIREDMAX) {
                 DesiredR_rpm = DESIREDMIN;
             }
             Clock_Delay1ms(250);
        }

         // Flash the blue LED while updating speeds to provide feedback
         BLUELED ^= 1;
         Clock_Delay1ms(200);
     }

     // After updating the desired speeds, flash the yellow LED for 2 seconds to indicate confirmation
     for(int k = 0; k < 10; k++){
         LaunchPad_RGB(YELLOW);
         Clock_Delay1ms(100);
         LaunchPad_RGB(RGB_OFF);
         Clock_Delay1ms(100);
     }
}

int16_t X_pos_mm = 360;   // starting position
int16_t Y_pos_mm = 0;

uint16_t avgDistTravelled = 0;

typedef enum {
    Stop = 0,      // Stop state
    Forward,       // Move forward
    Backward,      // Move backward
    LeftTurn,      // Turn left
    RightTurn      // Turn right
} state_t;

typedef enum {
    North,
    South,
    East,
    West
} face;

static state_t CurrentState = Forward;  // Initial state: Forward
static state_t NextState = Forward;     // Next state variable

static face faceNow = North;
static face faceNext = North;

bool ReachedStop = false;

// Structure to define robot movement commands
typedef struct command {
    uint16_t left_permil;     // PWM duty cycle for the left wheel
    uint16_t right_permil;    // PWM duty cycle for the right wheel
    void (*MotorFunction)(uint16_t, uint16_t);  // Motor function to call (e.g., Motor_Forward)
    int32_t dist_mm;          // Wheel displacement in mm
} command_t;

// Control parameters for various states (distances and duty cycles)
#define FRWD_DIST       700   // Replace this line for forward distance
#define BKWD_DIST       90   // Replace this line for backward distance
#define TR90_DIST       80   // Replace this line for 90 degree turn
#define TR60_DIST       74   // Replace this line for 60 degree turn
#define TR30_DIST       37   // Replace this line for 30 degree turn

#define NUM_STATES      5     // Number of robot states
#define LEN_STR_STATE   5     // String length for state names
static char strState[NUM_STATES][LEN_STR_STATE] = {"STOP", "FRWD", "BKWD", "LFTR", "RGTR"};  // State names
static char strFace[4][5] = {"NRTH", "SOUT", "EAST", "WEST"};

// Control commands for each state
command_t ControlCommands[NUM_STATES] = {
    {0,   0,   &Motor_Stop,        0},          // Stop indefinitely
    {600, 635 , &Motor_Forward,     FRWD_DIST},  // Move forward until bump sensor triggered
    {350, 350, &Motor_Backward,    BKWD_DIST},  // Move backward for 90mm
    {350, 350, &Motor_TurnLeft,    0},          // Turn left
    {350, 350, &Motor_TurnRight,   0}           // Turn right
};

// Clear the LCD and display initial state


static void LCDClear3 (void) {
    Nokia5110_Clear();
}

static void LCDOut3(void) {
    LCDClear3();

    Nokia5110_SetCursor2(1,1);
    Nokia5110_OutString("H: ");
    Nokia5110_OutString(strFace[faceNow]);
    Nokia5110_OutString(" ");

    Nokia5110_SetCursor2(2,1);
    Nokia5110_OutString("X:");
    Nokia5110_OutString(" ");
    Nokia5110_SetCursor2(2,3);
    Nokia5110_OutSDec(X_pos_mm, 5);

    Nokia5110_SetCursor2(3,1);
    Nokia5110_OutString("Y:");
    Nokia5110_OutString(" ");
    Nokia5110_SetCursor2(3,3);
    Nokia5110_OutSDec(Y_pos_mm, 5);

    Nokia5110_SetCursor2(4,1);
    Nokia5110_OutString("L:");
    Nokia5110_OutString(" ");
    Nokia5110_SetCursor2(4,3);
    Nokia5110_OutSDec(LeftDistance_mm, 5);

    Nokia5110_SetCursor2(5,1);
    Nokia5110_OutString("R:");
    Nokia5110_OutString(" ");
    Nokia5110_SetCursor2(5,3);
    Nokia5110_OutSDec(RightDistance_mm, 5);

    Nokia5110_SetCursor2(6,1);
    Nokia5110_OutString(strState[CurrentState]);
}


// Counter for how many times the controller function has been called
static uint8_t NumControllerExecuted = 0;  // Updated every 20ms


// Main control logic for the robot, executed by the TimerA2 ISR every 20ms
static void Controller3(void) {

    static uint16_t timer_20ms = 0;   // Timer to track elapsed time
    static uint8_t bumpRead = 0x00;   // Stores the bump sensor reading

    NumControllerExecuted++;  // Increment counter every time the controller runs

    // Get the PWM duty cycles for the current state
    uint16_t left_permil = ControlCommands[CurrentState].left_permil;
    uint16_t right_permil = ControlCommands[CurrentState].right_permil;


    ControlCommands[CurrentState].MotorFunction(left_permil, right_permil);
    // State transition logic based on bump sensors and distance
    bumpRead = Bump_Read();  // Read bump sensor status
    Tachometer_GetDistances(&LeftDistance_mm, &RightDistance_mm);  // Get current wheel distances

    static int32_t stateL = 0, stateR = 0;   // reset on state entry

    // compute incremental motion
    int32_t dL = LeftDistance_mm - stateL;
    int32_t dR = RightDistance_mm - stateR;

    avgDistTravelled = 0.5*(dL + dR);

    // save for next time
    stateL = LeftDistance_mm;
    stateR = RightDistance_mm;


    switch (faceNow) {
        case North:
            Y_pos_mm += avgDistTravelled;
            break;
        case South:
            Y_pos_mm -= avgDistTravelled;
            break;
        case East:
            X_pos_mm += avgDistTravelled;
            break;
        case West:
            X_pos_mm -= avgDistTravelled;
            break;
    }

    if (faceNow == South && X_pos_mm < 100 && Y_pos_mm < 20 ) {
        NextState = Stop;
    }
    switch (CurrentState) {

        case Stop: {
            NextState = Stop;
            ReachedStop = true;

        }
        case Forward:
            if (bumpRead) { //separate bump conditions from full dist

                // Determine next turn based on which bump sensor triggered
                if (bumpRead & 0x01) {
                    ControlCommands[LeftTurn].dist_mm =TR30_DIST;
                    NextState = LeftTurn;
                }
                else if (bumpRead & 0x02) {
                    ControlCommands[LeftTurn].dist_mm =TR60_DIST;
                    NextState = LeftTurn;

                }
                else if (bumpRead & 0x10) {
                    ControlCommands[RightTurn].dist_mm = TR60_DIST;
                    NextState = RightTurn;
                }
                else if (bumpRead & 0x20) {
                    ControlCommands[RightTurn].dist_mm = TR30_DIST;
                    NextState = RightTurn;

                }
                else if ((bumpRead & 0x04) || (bumpRead & 0x08)) {
                    NextState = Backward;
                    ControlCommands[LeftTurn].dist_mm = TR90_DIST;
                }
            }


            break;

        case Backward: //account for current position based off direction
            switch (faceNow) {
                case North:
                    Y_pos_mm -= avgDistTravelled;
                    break;

                case South:
                    Y_pos_mm += avgDistTravelled;
                    break;

                case East:
                    X_pos_mm -= avgDistTravelled;
                    break;

                case West:
                    X_pos_mm += avgDistTravelled;
                    break;
            }

            if (abs(LeftDistance_mm) >= ControlCommands[Backward].dist_mm) {
                NextState = LeftTurn;
            }
            break;

        case LeftTurn:
            if (abs(RightDistance_mm) >= ControlCommands[LeftTurn].dist_mm) {

                if (ControlCommands[LeftTurn].dist_mm == TR90_DIST) { //change direction when cranking nineties
                    switch (faceNow) {
                        case North:
                            faceNext = West;
                            break;
                        case South:
                            faceNext = East;
                            break;
                        case East:
                            faceNext = North;
                            break;
                        case West:
                            faceNext = South;
                            break;
                    }
                }

                NextState = Forward;
            }
            break;

        case RightTurn:
            if (abs(LeftDistance_mm) >= ControlCommands[RightTurn].dist_mm) {

                if (ControlCommands[RightTurn].dist_mm == TR90_DIST) {
                    switch (faceNow) {
                        case North:
                            faceNext = East;
                            break;
                        case South:
                            faceNext = West;
                            break;
                        case East:
                            faceNext = South;
                            break;
                        case West:
                            faceNext = North;
                            break;
                    }
                }
                NextState = Forward;
            }
            break;

        default:
            NextState = Stop;
            break;
    }

    // Update the timer or reset if transitioning to a new state
    if (CurrentState == NextState) {
        timer_20ms++;  // Stay in current state, increment timer
    } else {
        timer_20ms = 0;  // New state, reset timer and distance measurements
        stateL = 0;
        stateR = 0;
        Tachometer_ResetSteps();
    }

    // Set the current state to the next state for the next iteration
    CurrentState = NextState;
    faceNow = faceNext;

    uint8_t ledTick = 0;

    if (ReachedStop) {
                ledTick++;
                if(ledTick >= 25){   // 25 * 20ms = 500ms
                    ledTick = 0;

                    static uint8_t toggle = 0;

                    if(toggle){
                        LaunchPad_RGB(RED);
                    } else {
                        LaunchPad_RGB(BLUE);
                    }
                    toggle ^= 1;
                }
            }
}

void Level1(void) {

    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Bump_Init();
    Motor_Init();
    Nokia5110_Init();
    Tachometer_Init();

    uint8_t const contrast = 0xA8;
    Nokia5110_SetContrast(contrast);

    // Set TimerA2 to call the Controller3() function every 20 ms (50 Hz)
    const uint16_t period_4us = 5000;       // 20 ms period
    TimerA2_Init(&Controller3, period_4us); // Initialize TimerA2

//    UpdateParameters();
    TxBuffer();
    LCDClear3();
    Tachometer_ResetSteps();  // Reset tachometer distance measurements

    // Set the LCD update rate to 10 Hz (update every 5 controller cycles)
    const uint16_t LcdUpdateRate = 25;    // 50 Hz / 5 = 10 Hz
    LCDClear3();  // Clear the LCD and display initial state

    EnableInterrupts();

    while(1) {

        // Enter low-power mode while waiting for the next interrupt
        WaitForInterrupt();

        // Update the LCD every 10 Hz
        if (NumControllerExecuted == LcdUpdateRate) {
            LCDOut3();  // Output current state and motor data to the LCD
            NumControllerExecuted = 0;  // Reset the execution count
        }



    }
}
