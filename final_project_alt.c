//#############################################################################
// FILE:   final_project_alt.c
//
// TITLE:  ME 461 Final Project C Code File - Added Controls
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
//#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

#define C4NOTE ((uint16_t)(((50000000/2)/2)/261.63))
#define D4NOTE ((uint16_t)(((50000000/2)/2)/293.66))
#define E4NOTE ((uint16_t)(((50000000/2)/2)/329.63))
#define F4NOTE ((uint16_t)(((50000000/2)/2)/349.23))
#define G4NOTE ((uint16_t)(((50000000/2)/2)/392.00))
#define A4NOTE ((uint16_t)(((50000000/2)/2)/440.00))
#define B4NOTE ((uint16_t)(((50000000/2)/2)/493.88))
#define C5NOTE ((uint16_t)(((50000000/2)/2)/523.25))
#define D5NOTE ((uint16_t)(((50000000/2)/2)/587.33))
#define E5NOTE ((uint16_t)(((50000000/2)/2)/659.25))
#define F5NOTE ((uint16_t)(((50000000/2)/2)/698.46))
#define G5NOTE ((uint16_t)(((50000000/2)/2)/783.99))
#define A5NOTE ((uint16_t)(((50000000/2)/2)/880.00))
#define B5NOTE ((uint16_t)(((50000000/2)/2)/987.77))
#define A4FLATNOTE ((uint16_t)(((50000000/2)/2)/415.3))
#define A5FLATNOTE ((uint16_t)(((50000000/2)/2)/830.61))
#define B5FLATNOTE ((uint16_t)(((50000000/2)/2)/932.33))
#define D5FLATNOTE ((uint16_t)(((50000000/2)/2)/554.37))
#define E5FLATNOTE ((uint16_t)(((50000000/2)/2)/622.25))
#define G5FLATNOTE ((uint16_t)(((50000000/2)/2)/739.99))
#define A6FLATNOTE ((uint16_t)(((50000000/2)/2)/1661.22))
#define B6NOTE ((uint16_t)(((50000000/2)/2)/1975.53))
#define F5SHARPNOTE ((uint16_t)(((50000000/2)/2)/739.99))
#define C5SHARPNOTE ((uint16_t)(((50000000/2)/2)/554.37))
#define OFFNOTE 0
#define SONG_LENGTH 6
int16_t song_index = 0;
uint16_t songarray[SONG_LENGTH] = {

                                   C5NOTE,
                                   A5NOTE,
                                   C5NOTE,
                                   A5NOTE,
                                   C5NOTE,
                                   A5NOTE
};

// ----- code for CAN start here -----
#include "F28379dCAN.h"
//#define TX_MSG_DATA_LENGTH    4
//#define TX_MSG_OBJ_ID         0  //transmit

#define RX_MSG_DATA_LENGTH    8
#define RX_MSG_OBJ_ID_1       1  //measurement from sensor 1
#define RX_MSG_OBJ_ID_2       2  //measurement from sensor 2
#define RX_MSG_OBJ_ID_3       3  //quality from sensor 1
#define RX_MSG_OBJ_ID_4       4  //quality from sensor 2
// ----- code for CAN end here -----


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

__interrupt void ADCA_ISR (void);
// ----- code for CAN start here -----
__interrupt void can_isr(void);
// ----- code for CAN end here -----
__interrupt void SPIB_isr(void);
void setupSpib(void);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM2A(float controleffort); // RP,HH: Function prototype to control EPWM2A (right motor)
void setEPWM2B(float controleffort); // RP,HH: Function prototype to control EPWM2B (left motor)
// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0; // Define for Ex 5
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
// From Lab 4 - ADCA global variables
uint32_t ADCvoltscounter = 0; // Global variable for counter in ADC to call printf
uint16_t ADCIND0raw = 0; // Global variable for storing ADCIND0 reading
uint16_t ADCIND1raw = 0; // Global variable for storing ADCIND1 reading
uint16_t ADCINA2raw = 0; // Global variable for storing ADCINA2 reading
uint16_t ADCINA3raw = 0; // Global variable for storing ADCINA3 reading
uint16_t ADCINA4raw = 0;
uint16_t ADCINA5raw = 0;
uint16_t ADCINB4raw = 0; // Global variable for storing ADCINB4 reading

float ADCIND0scaled = 0; // Global variable for storing ADCIND0 scaled voltage
float ADCIND1scaled = 0; // Global variable for storing ADCIND1 scaled voltage
float ADCINA2scaled = 0; // Global variable for storing ADCINA2 scaled voltage
float ADCINA3scaled = 0; // Global variable for storing ADCINDA3 scaled voltage
float ADCINA4scaled = 0;
float ADCINA5scaled = 0;
float ADCINB4scaled = 0; // Global variable for storing ADCINDB4 scaled voltage

int16_t accelx_raw = 0;
int16_t accely_raw = 0;
int16_t accelz_raw = 0;
int16_t gyrox_raw = 0;
int16_t gyroy_raw = 0;
int16_t gyroz_raw = 0;

float accelx = 0;
float accely = 0;
float accelz = 0;

float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

// Setup global variables for robot dimensions
float LeftWheel = 0.0;
float RightWheel = 0.0;
float left_meter_per_radian = 8.619; // meter per radian that we measured for left wheel
float right_meter_per_radian = 8.554;//meter per radian that we measured for right wheel
float W_R = 0.173; // Robot width (m)
float R_Wh = 0.0593; // Robot wheel radii (m)

// Setup global variables for derived values/PI control
float distance_left = 0.0; // Distance traveled by left wheel
float distance_right = 0.0; // Distance traveled by right wheel
float uLeft = -0.0;
float uRight = -0.0;
float PosLeft_K = 0.0;
float PosLeft_K_1 = 0.0;
float PosRight_K = 0.0;
float PosRight_K_1 = 0.0;
//float VLeftK = 0.0;
//float VRightK = 0.0;
float Ki = 25.0;
float Kp = 3.0;
float Kturn = 0.25;
float Vref = 0.0;
float Ik_left = 0.0;
float Ik_left_1 = 0.0;
float Ik_right = 0.0;
float Ik_right_1 = 0.0;
float ek_left = 0.0;
float ek_left_1 = 0.0;
float ek_right = 0.0;
float ek_right_1 = 0.0;
float uk_left = 0.0;
float uk_right = 0.0;
float turn = 0.0; // Turn setpoint
float e_turn = 0.0; // Turn error
float e_left = 0.0; //
float e_right = 0.0; //
float Kp_turn = 3.0; // Coupled turn gain
float VLeftK = 0.0;
float VRightK = 0.0;

// LIGHT-TRACKING CONTROL VARIABLES (Ryan P)
float front_sensor_follow_limit = -2.79; // Upper limit for forward movement
float front_sensor_prox_limit = -2.80; // Hard limit for a very close light source (stop + buzzer)
float Kp_light = 1.5; // Controls how aggressive the robot will react to differences in intensity between the sensors
float Ki_light = 0.0; // Keep at 0 as errors will accumulate for photosensors - potentially causing integral windup/overshooting
float Kd_light = 0.01; // Dampens the turning rate as the robot orients itself towards the direction of the light source
float EW_error_1 = 0.0; // Previous state for derivative calculation
float light_integral = 0.0; // State variable for PID Integral

float alpha = 0.1; // Smoothing coefficient used in light derivative low-pass IIR filter

float activation_threshold = -0.010; // At least one sensor must be this bright to turn
float deadband = 0.25;               // Ignore error if difference is smaller than this
float light_derivative_raw = 0.0;   // Raw rate of change of error in light intensity (calculated with norm_EW_error)
float light_derivative_raw_1 = 0.0; // Past state for raw light derivative
float light_derivative_smoothed = 0.0;       // Rate of change of error in light intensity
float light_derivative_smoothed_1 = 0.0;     // Past state of change of error
float m = 0.0;
float m_min = 0.25;                  // Minimum vector magnitude to trust direction
float max_turn_rate = 0.75;          // Cap turn rate to prevent overshoot (rad/s)

float norm_EW_error = 0.0;
float norm_EW_error_1 = 0.0;        // Past state of normalized EW error

uint16_t tracking_state = 0; // 0 = Searching, 1 = Tracking
float m_tracklimit_start = 0.30; // Must be higher than this to START tracking
float m_tracklimit_stop = 0.10;  // Must drop below this to STOP tracking

uint32_t lost_light_counter = 0; // State counter variable for debounce filter (Light signal lost)
uint32_t found_light_counter = 0; // State counter variable for debounce filter (Light signal found)

// Rear Spin State Variables
// Used to commit to a 180-degree turn without wiggling.
static uint16_t spin_counter = 0;
static float spin_val = 0.0;

// LABVIEW communication global variables
float printLV3 = 0.0;
float printLV4 = 0.0;
float printLV5 = 0.0;
float printLV6 = 0.0;
float printLV7 = 0.0;
float printLV8 = 0.0;
float x = 0.0;
float y = 0.0;
float bearing = 0.0;
extern uint16_t NewLVData;
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

// Steering motor PI control variables
float theta_left = 0.0; // Left wheel rotation angle
float theta_right = 0.0; // Right wheel rotation angle
float theta_left_1 = 0.0; // Previous state for theta_left
float theta_right_1 = 0.0; // Previous state for theta_right
float x_R = 0.0; // Robot pose X-coordinate
float y_R = 0.0; // Robot pose Y-coordinate
float x_R_1 = 0.0; // Previous state for x_R
float y_R_1 = 0.0; // Previous state for y_R
float Theta_R = 0.0; // Robot pose angle/bearing
float theta_avg = 0.0; // Average rotation angle of left and right wheels
float thetadot_left = 0.0; // Left wheel angular velocity
float thetadot_right = 0.0; // Right wheel angular velocity
float thetadot_avg = 0.0; // Average angular velocity of left and right wheels
float xdot_R = 0.0; // Robot X-coordinate velocity
float ydot_R = 0.0; // Robot Y-coordinate velocity
float xdot_R_1 = 0.0; // Previous state for xdot_R
float ydot_R_1 = 0.0; // Previous state for ydot_R

// Wall-following loop
float distright = 0.0; // Reading from right distance sensor
float distfront = 0.0; // Reading from front distance sensor
uint16_t forward = 0; //
uint16_t leftturn = 0;
uint16_t rightturn = 0;
float Kpright = 0.0005; // Initialize right turn proportional constant
float Kpfront = 0.0015; // Initialize left turn proportional constant
float ref_right = 400; // Initialize right reference distance
float ref_front = 600; // Initialize front reference distance
float V_front = 0.25; // Initialize LEFT TURN velocity
float V_right = 0.50; // Initialize FRONT/right wall follow velocity
float threshold1 = 400.0; // Front wall NEAR threshold
float threshold2 = 600.0; // Front wall FAR threshold

// Exercise 8
extern uint16_t beep[];      // Matches "uint16_t beep[3]" in song.h
uint16_t beep_song_index = 0;      // Note tracking index (Local to this file)
uint16_t play_beep = 0;            // Start flag to begin beep (Local to this file)

// Function prototype for our new PWM init function
void InitEPwm9(void);

float u = 0.0; // RP,HH: Create global variable for increment in setEPWM2A/B functions
uint16_t motorflag = 0; // RP,HH: Create a global variable for the duty cycle counter toggle

int32_t SpibNumCalls = 0;


// Kalman Filter Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -.76;
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000; //50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t AverageIndex = -1;
float pred_P = 0;
float kalman_K = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;
float velLeftK = 0.0;
float velRightK = 0.0;
float velLeftK_1 = 0.0;
float velRightK_1 = 0.0;
float gyro_valueK_1 = 0.0;
float gyrorate_dot = 0.0;
float gyrorate_dotK_1 = 0.0;
float K1 = -60.0;
float K2 = -4.5;
float K3 = -1.1;
float K4 = -0.1;
float ubal = 0.0;
float LeftWheelK_1 = 0.0;
float RightWheelK_1 = 0.0;

// ----- code for CAN start here -----
// volatile uint32_t txMsgCount = 0;
// extern uint16_t txMsgData[4];

volatile uint32_t rxMsgCount_1 = 0;
volatile uint32_t rxMsgCount_3 = 0;
extern uint16_t rxMsgData[8];

uint32_t dis_raw_1[2];
uint32_t dis_raw_3[2];
uint32_t dis_1 = 0;
uint32_t dis_3 = 0;

uint32_t quality_raw_1[4];
uint32_t quality_raw_3[4];
float quality_1 = 0.0;
float quality_3 = 0.0;

uint32_t lightlevel_raw_1[4];
uint32_t lightlevel_raw_3[4];
float lightlevel_1 = 0.0;
float lightlevel_3 = 0.0;

uint32_t measure_status_1 = 0;
uint32_t measure_status_3 = 0;

volatile uint32_t errorFlag = 0;

float voltage1 = 0.0;
float voltage2 = 0.0;
// ----- code for CAN end here -----

// 21st-order FIR low pass filter
float b[22]={   6.5869928109880361e-03,
                8.3822503875666966e-03,
                1.3510034452386821e-02,
                2.1618819668201512e-02,
                3.2055852101738572e-02,
                4.3925915740345808e-02,
                5.6175631845700291e-02,
                6.7694958280951845e-02,
                7.7425648094147204e-02,
                8.4465591749038393e-02,
                8.8158304868934750e-02,
                8.8158304868934750e-02,
                8.4465591749038393e-02,
                7.7425648094147204e-02,
                6.7694958280951845e-02,
                5.6175631845700291e-02,
                4.3925915740345808e-02,
                3.2055852101738572e-02,
                2.1618819668201512e-02,
                1.3510034452386821e-02,
                8.3822503875666966e-03,
                6.5869928109880361e-03};

// Initialize filtered values for A2 and A3
// #define filter_order 21
uint32_t filter_order = 21;

float xk_A2[22] = {0};
float xk_A3[22] = {0};
float xk_A4[22] = {0};
float xk_A5[22] = {0};

float W = 0.0;
float E = 0.0;
float S = 0.0;
float N = 0.0;

float W_offset = 0.0;
float E_offset = 0.0;
float S_offset = 0.0;
float N_offset = 0.0;

float buzzer = 0;
//adca1 pie interrupt
__interrupt void ADCA_ISR (void) {
    ADCINA2raw = AdcaResultRegs.ADCRESULT0; // W photosensor
    ADCINA3raw = AdcaResultRegs.ADCRESULT1; // E photosensor
    ADCINA4raw = AdcaResultRegs.ADCRESULT2; // S photosensor
    ADCINA5raw = AdcaResultRegs.ADCRESULT3; // N photosensor
    // Here covert ADCINA2, ADCINA3 to volts
    ADCINA2scaled = (3.0/4096.0) * ADCINA2raw;
    ADCINA3scaled = (3.0/4096.0) * ADCINA3raw;
    ADCINA4scaled = (3.0/4096.0) * ADCINA4raw;
    ADCINA5scaled = (3.0/4096.0) * ADCINA5raw;
    W = (3.0/4096.0) * ADCINA2raw;
    E = (3.0/4096.0) * ADCINA3raw;
    S = (3.0/4096.0) * ADCINA4raw;
    N = (3.0/4096.0) * ADCINA5raw;
    // Set the first past state element in the array as the scaled ADCINA2, ADCINA3, ADCINA4, ADCINA5 Voltage
    xk_A2[0] = ADCINA2scaled;
    xk_A3[0] = ADCINA3scaled;
    xk_A4[0] = ADCINA4scaled;
    xk_A5[0] = ADCINA5scaled;
    // For loop to calculate filtered values
    if (calibration_state == 2) {
        W = 0;
        N = 0;
        S = 0;
        E = 0;
        for (int i = 0; i <= filter_order; i++) {
            W += b[i]*xk_A2[i];
            E += b[i]*xk_A3[i];
            S += b[i]*xk_A4[i];
            N += b[i]*xk_A5[i];
        }
    }
    /*
     * W.Wu (12/4/25 1:50 PM) The watch expressions for N S E W might look like it flickers to 2.5 and back to 0 however that is fine. Because the order of our interrupts goes from
     * ADCA_ISR to SPIB_ISR to SWI_ISR. After the ADCA_ISR the 2.5 gets sent to the SPIB_ISR where the offset gets subtracted. Afterwards that subtracted value is used
     * for the logic in the SWI_ISR. We shouldn't need to change the m_min value either I think 0.1 is good enough but you should check on the floor.
     * The logic should all be done in the SWI_ISR. Try not to over complicate the left and right turn. We probably need to detect a change in light and to make sure that we aren't just sensing a wall
     * on one side which makes the other side look brighter. This could be done with an && in the if statement condition.
     */
    // Print ADCINA2 and ADCINA3 voltage value to TeraTerm every 100ms
    ADCvoltscounter++;
    //    if ((ADCvoltscounter % 100) == 0) {
    //        UARTPrint = 0;
    //    }

    //Update past states array using a for loop
    for (int i = filter_order; i >= 1; i--) {
        xk_A2[i] = xk_A2[i-1];
        xk_A3[i] = xk_A3[i-1];
        xk_A4[i] = xk_A4[i-1];
        xk_A5[i] = xk_A5[i-1];
    }

    //    SpibRegs.SPITXBUF = 2200; // PWM Value
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = 0xBA00;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    // Buzzer (GPIO16, Mux 2 for EPWM9A)
    //    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); // Mux 2 selects EPWM9A
    //    GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL);

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // ----- code for CAN start here -----
    //GPIO17 - CANRXB
    GPIO_SetupPinMux(17, GPIO_MUX_CPU1, 2);
    GPIO_SetupPinOptions(17, GPIO_INPUT, GPIO_ASYNC);

    //GPIO12 - CANTXB
    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 2);
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);
    // ----- code for CAN end here -----



    // ----- code for CAN start here -----
    // Initialize the CAN controller
    InitCANB();

    // Set up the CAN bus bit rate to 1000 kbps
    setCANBitRate(200000000, 1000000);

    // Enables Interrupt line 0, Error & Status Change interrupts in CAN_CTL register.
    CanbRegs.CAN_CTL.bit.IE0= 1;
    CanbRegs.CAN_CTL.bit.EIE= 1;
    // ----- code for CAN end here -----

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.ADCA1_INT = &ADCA_ISR; // Address to ADCA interrupt function

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    // ----- code for CAN start here -----
    PieVectTable.CANB0_INT = &can_isr;
    // ----- code for CAN end here -----
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 4000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000); //to call cpu1 every 4ms
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 100000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    // ADC Initialization
    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    // ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; // Set SOC0 to convert pin A2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; // Sets SOC1 to convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA will trigger SOC1
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4; // Set SOC0 to convert pin A2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA will trigger SOC0
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 5; // Sets SOC1 to convert pin A3
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //set to SOC3, the last converted, and it will set INT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; // enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // make sure INT1 flag is cleared
    EDIS;

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD (010)
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (pulse is the same as trigger) (01)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    //    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms (1kHz) sample. Input clock is 50MHz.
    //    EPwm5Regs.TBPRD = 12500; // Set Period to 0.25ms (4kHz) sample. Input clock is 50MHz.
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms (1kHz) sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode (00)
    EDIS;

    // EPWM2A/B settings (right and left motors)
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Set TBCTL CTRMODE bitfield to Up-count mode
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; // Set TBCTL FREE_SOFT bitfield to freerun mode: 1x
    EPwm2Regs.TBCTL.bit.PHSEN = 0; // Set TBCTL PHSEN bitfield to disable phase loading: 0
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; // Set TBCTL CLKDIV bitfield to clock divide by 1

    // TBCTR register
    EPwm2Regs.TBCTR = 0; // Starts the TBCTR timer at zero

    // TMPRD register
    EPwm2Regs.TBPRD = 2500; // Sets the carrier frequency - when TBCTR = TBPRD

    // CMPA register (Compare Control Register): Value is continuously compared to the value in TBCTR
    // When TBCTR = CMPA, an "event" is generated and sent to the AQCTLA register
    // The AQCTLA register determines what to do with the signal pin when it receives an event.
    // By changing the value in CMPA, the duty cycle of the signal pin can be changed.
    EPwm2Regs.CMPA.bit.CMPA = 1250;
    EPwm2Regs.CMPB.bit.CMPB = 1250; // Starts duty cycle at 0 (Change value relative to TBPRD to set duty cycle)

    // AQCTLA register (Action Qualifier Control Register)
    EPwm2Regs.AQCTLA.bit.CAU = 1; // Sets the CAU bitfield to clear the signal pin (LOW) when TBCTR register reaches the value in CMPA (01 binary)
    EPwm2Regs.AQCTLB.bit.CBU = 1; // Sets the CBU bitfield to clear the signal pin (LOW) when TBCTR register reaches the value in CMPB (01 binary)
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // Sets the PRD bitfield to force the EPWMxA output (HIGH) when TBCTR register reaches the value in TMCTR (10 binary)
    EPwm2Regs.AQCTLB.bit.ZRO = 2; // Sets the PRD bitfield to force the EPWMxA output (HIGH) when TBCTR register reaches the value in TMCTR (10 binary)

    // TBPHS register
    EPwm2Regs.TBPHS.bit.TBPHS = 0; // Set the phase to zero (ignores synchronization event, time-base counter not loaded with the phase)

    // Function to set PinMux
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); // Sets the PinMux to GPIO 2, CPU 1, Mux Index/Mux Position
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); // Sets the PinMux to GPIO 3 (2B), CPU 1, Mux Index/Mux Position

    init_serialSCIA(&SerialA,115200);
    setupSpib();

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.
    IER |= M_INT1;
    IER |= M_INT6;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA  CANB
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;  //SPiB

    // Enable INT1 in the PIE: Group 1 interrupt 1 for ADCA1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // ----- code for CAN start here -----
    // Enable CANB in the PIE: Group 9 interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx7 = 1;
    // ----- code for CAN end here -----

    // ----- code for CAN start here -----
    // Enable the CAN interrupt signal
    CanbRegs.CAN_GLB_INT_EN.bit.GLBINT0_EN = 1;
    // ----- code for CAN end here -----

    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);
    init_eQEPs(); //call the function that given in LAB6 EX1
    InitEPwm9(); // Initialize the buzzer PWM

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
    // ----- code for CAN start here -----

    //    // Transmit Message
    //    // Initialize the transmit message object used for sending CAN messages.
    //    // Message Object Parameters:
    //    //      Message Object ID Number: 0
    //    //      Message Identifier: 0x1
    //    //      Message Frame: Standard
    //    //      Message Type: Transmit
    //    //      Message ID Mask: 0x0
    //    //      Message Object Flags: Transmit Interrupt
    //    //      Message Data Length: 4 Bytes
    //    //
    //    CANsetupMessageObject(CANB_BASE, TX_MSG_OBJ_ID, 0x1, CAN_MSG_FRAME_STD,
    //                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
    //                           TX_MSG_DATA_LENGTH);

    // Measured Distance from 1
    // Initialize the receive message object 1 used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 1
    //      Message Identifier: 0x060b0101
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 8 Bytes (Note that DLC field is a "don't care"
    //      for a Receive mailbox)
    //
    CANsetupMessageObject(CANB_BASE, RX_MSG_OBJ_ID_1, 0x060b0101, CAN_MSG_FRAME_EXT,
                          CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,
                          RX_MSG_DATA_LENGTH);

    // Measured Distance from 2
    // Initialize the receive message object 2 used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 2
    //      Message Identifier: 0x060b0102
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 8 Bytes (Note that DLC field is a "don't care"
    //      for a Receive mailbox)
    //

    CANsetupMessageObject(CANB_BASE, RX_MSG_OBJ_ID_2, 0x060b0103, CAN_MSG_FRAME_EXT,
                          CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,
                          RX_MSG_DATA_LENGTH);

    // Measurement Quality from 1
    // Initialize the receive message object 2 used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 3
    //      Message Identifier: 0x060b0201
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 8 Bytes (Note that DLC field is a "don't care"
    //      for a Receive mailbox)
    //

    CANsetupMessageObject(CANB_BASE, RX_MSG_OBJ_ID_3, 0x060b0201, CAN_MSG_FRAME_EXT,
                          CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,
                          RX_MSG_DATA_LENGTH);

    // Measurement Quality from 2
    // Initialize the receive message object 2 used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 4
    //      Message Identifier: 0x060b0202
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 8 Bytes (Note that DLC field is a "don't care"
    //      for a Receive mailbox)
    //

    CANsetupMessageObject(CANB_BASE, RX_MSG_OBJ_ID_4, 0x060b0203, CAN_MSG_FRAME_EXT,
                          CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,
                          RX_MSG_DATA_LENGTH);

    //
    // Start CAN module operations
    //
    CanbRegs.CAN_CTL.bit.Init = 0;
    CanbRegs.CAN_CTL.bit.CCE = 0;

    //    // Initialize the transmit message object data buffer to be sent
    //    txMsgData[0] = 0x12;
    //    txMsgData[1] = 0x34;
    //    txMsgData[2] = 0x56;
    //    txMsgData[3] = 0x78;


    //    // Loop Forever - A message will be sent once per second.
    //    for(;;)
    //    {
    //
    //        CANsendMessage(CANB_BASE, TX_MSG_OBJ_ID, TX_MSG_DATA_LENGTH, txMsgData);
    //        txMsgCount++;
    //        DEVICE_DELAY_US(1000000);
    //    }


    // ----- code for CAN end here -----


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            //            serial_printf(&SerialA,"Tilt Value: %.3f| Gyro Value: %.3f \r\n Leftwheelangle: %.3f | Rightwheelangle: %.3f\r\n\n",tilt_value,gyro_value,LeftWheel,RightWheel);
            serial_printf(&SerialA,"N: %.4f| Forward: %d \r\n E: %.4f W: %.4f S: %.4f | Spin Value: %.4f | Turn strength: %.4f \r\n", N , forward, E, W, S, spin_val, turn);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    static uint16_t first_SWI_run = 1;

    // State variables for Smart Search
    static float search_dir = 1.0;      // 1.0 = Left/CCW, -1.0 = Right/CW
    static float last_valid_error = 0.0; // Remembers where the light was before it was lost

    // Insert SWI ISR Code here.......

    // LIGHT-TRACKING LOGIC //
    float EW_error = E - W;             // Difference between East and West sensors

    m = sqrt(pow(N-S,2) + pow(W-E,2)); // Light source directionality - if low, indicates that light in all directions is similar (e.g., ambient lighting)

    // 1. Calculate Normalized Error FIRST.
    //    Do before deadband check. Otherwise, weak signals (far away) are rejected before normalization can boost them.
    if (m > 0.05) {
        norm_EW_error = EW_error / (m + 0.25);
        last_valid_error = norm_EW_error; // Update memory of where the light is. Use this if light is lost to decide which way to search.
    }

    // PD Calculation
    // Derivative = (Current Error - Previous Error) / Sample Time (0.004s)
    light_derivative_raw = (norm_EW_error - norm_EW_error_1) / 0.004;
    light_derivative_smoothed = (alpha * light_derivative_raw) + ((1.0 - alpha) * light_derivative_smoothed_1);

    // Update State based on tracking memory
    if (tracking_state == 0) {
        light_integral = 0; // Reset integral when not tracking to prevent windup history
        // Currently Searching: Look for strong signal
        // Enter tracking if light is strong AND South is NOT the strongest sensor.
        // This allows side detection (E or W dominant) but prevents locking onto back-light (S dominant).
        if (m > m_tracklimit_start) {
            // Robot continues to search until light is roughly centered on North sensor
            found_light_counter++; // Start incrementing when light is detected

            if (found_light_counter > 50) { // Switch to tracking state when a strong source is detected for ~ 200 ms
                tracking_state = 1; // Switch to Tracking
                lost_light_counter = 0; // Reset counter if light is detected
                found_light_counter = 0; // Reset counter

                if (fabs(norm_EW_error) > 0.40) {
                    spin_counter = 80; // Lock into tracking/turning for 80 cycles (~320ms)

                    // Force max turn rate in the direction of the error
                    if (norm_EW_error > 0) {
                        spin_val = max_turn_rate;
                    } else {
                        spin_val = -max_turn_rate;
                    }
                }
            }
        } else {
            found_light_counter = 0; // Light signal not sustained, reset count
        }
    } else {
        // Currently Tracking: Tolerate weak signal down to lower limit
        if (m < m_tracklimit_stop && spin_counter == 0) {
            lost_light_counter++; // Increment counter

            // Switch to search mode ONLY if no strong light source for 100 cycles (~400 ms)
            if (lost_light_counter > 100) {
                tracking_state = 0;
                found_light_counter = 0;
                light_integral = 0; // Reset integral on loss of lock
            }

            // If last error was Negative (Light to Right), search Right.
            // If last error was Positive (Light to Left), search Left.
            if (last_valid_error < -0.1) {
                search_dir = -1.0;
            } else if (last_valid_error > 0.1) {
                search_dir = 1.0;
            }
        } else {
            lost_light_counter = 0;
        }
    }

    if ( N <= -0.1 && N >= front_sensor_follow_limit) { // Robot detected the light in front and is not too close to light
        if (fabs(EW_error) < (0.2*m)) {
            forward = 1;
        } else {
            forward = 0; //
        }
    } else {
        forward = 0; // If no light detected stop going forward
        Vref = 0.0;
    }
    if (N <= front_sensor_prox_limit) { // When light source is very close, stop forward movement and sound buzzer
        forward = 0;
        Vref = 0.0;
        turn = 0.0;
        light_integral = 0; // Reset integral when stopped at target
        buzzer = 1;
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1,5);
        // Play buzzer
    }

    // --- OUTER LOOP: PD LIGHT CONTROL ---
    if (tracking_state == 1 && N > front_sensor_prox_limit) {

        //        if (W < activation_threshold || E < activation_threshold || N < activation_threshold || S < activation_threshold) {

        // Committed Spin Logic
        // Priority 1: If robot is already in a committed spin, continue regardless of sensors. This overrides noise/wiggles.
        if (spin_counter > 0) {
            forward = 0;
            light_integral = 0;
            turn = spin_val; // Maintain the established direction
            spin_counter--;
        }

        // Check if South is the strongest (most negative) signal.
        if (S < N && S < E && S < W) {
            // South is dominating. The light is behind the robot.
            forward = 0; // Force stop forward motion while spinning
            light_integral = 0; // Prevent accumulation of integral during a 180 spin

            // Set the lock timer (50 cycles = 200ms)
            spin_counter = 50;

            // Spin Direction Logic:
            // Compare East vs West to find the shortest turn path.
            // If E < W (East is brighter/more negative), turn Right (negative turn).
            // If W < E (West is brighter), turn Left (positive turn).
            if (E < W) {
                spin_val = -max_turn_rate; // Turn Right
            } else {
                spin_val = max_turn_rate;  // Turn Left
            }
            turn = spin_val;
        }
        else { // Standard PID tracking loop
            // Diagonal Deadzone Fix
            // Explicitly check for 45-degree overlaps where mathematical error might be small but geometry is clear.
            float diag_threshold = -0.20;

            if (N < diag_threshold && E < diag_threshold) {
                // North + East are both active. Light is in the NE diagonal.
                // Force a right turn to push error outside deadband.
                norm_EW_error -= 0.5;
            }
            else if (N < diag_threshold && W < diag_threshold) {
                // North + West are both active. Light is in the NW diagonal.
                // Force a left turn to push error outside deadband.
                norm_EW_error += 0.5;
            }

            // 2. Check Deadband against NORMALIZED Error.
            //    Now checking the directionality instead of raw voltage.
            if (fabs(norm_EW_error) > deadband) {

                // Variable Gain
                // Reduce aggression if moving forward to prevent swerving
                // Ternary operators (if condition 1 is TRUE, set current_Kp to Kp_light * 0.5; else set to Kp_light
                float current_Kp = (forward == 1) ? (Kp_light * 0.5) : Kp_light;
                float current_Kd = (forward == 1) ? (Kd_light * 0.5) : Kd_light;
                float current_Ki = (forward == 1) ? (Ki_light * 0.5) : Ki_light;

                // 1. Calculate Integral (Accumulate Error over time)
                //    We multiply by 0.004 (sample time) to keep units consistent
                light_integral += ((norm_EW_error + norm_EW_error_1)/2) * 0.004;

                // 2. Anti-Windup Clamping
                //    Crucial: Prevents integral from growing excessively if the robot physically can't turn faster.
                //    Clamped to +/- 2.0 (arbitrary reasonable limit for this robot).
                if (light_integral > 2.0) light_integral = 2.0;
                if (light_integral < -2.0) light_integral = -2.0;

                // Control Law: Turn = P_term + D_term
                turn = (current_Kp * norm_EW_error) + (current_Ki * light_integral) + (current_Kd * light_derivative_smoothed);

                // Saturation
                if (turn > max_turn_rate) turn = max_turn_rate;
                if (turn < -max_turn_rate) turn = -max_turn_rate;

            } else {
                turn = 0.0;
                light_integral = 0.0; // Reset integral if error is within deadband to prevent drift
            }
        }
    } else if (N > front_sensor_prox_limit) {
        // Search State (No Light Detected / Lost Tracking)
        // Case: m <= m_hyst_low (Light is too weak/far away)
        // Action: Rotate in place to "scan" for the light source.
        turn = 0.25 * search_dir; // This allows the robot to "rewind" if it overshoots the light, instead of spinning away.
        forward = 0; // Ensure the robot does not drift forward while searching
    }

    // Update Previous Error State for next cycle's derivative.
    // IMPORTANT: We update using the value calculated at the TOP of the function (re-calculating to be safe, or we could have stored it).
    // Re-calculating raw error to ensure we don't save the "+/- 0.25" diagonal modified version.
    float EW_error_rec = E - W;
    if (m > 0.05) {
        norm_EW_error_1 = EW_error_rec / (m + 0.25);
    } else {
        norm_EW_error_1 = 0;
    }

    // 3. VELOCITY SETTING
    if (forward == 1) {
        Vref = 0.5;
        if (fabs(turn) > 0.5) {
            Vref = 0.1;
        }
    } else {
        Vref = 0;
    }

    LeftWheel=readEncLeft();
    RightWheel=readEncRight();

    distance_left= LeftWheel/left_meter_per_radian/2.0;
    distance_right= RightWheel/right_meter_per_radian/2.0;
    PosLeft_K = distance_left;
    PosRight_K = distance_right;

    // On the first run, PosLeft_K_1 is 0 but PosLeft_K might be non-zero (if robot settled).
    // This creates a massive (PosLeft_K - 0)/0.004 velocity spike.
    // We catch this here, update the history, and output 0 for one cycle.
    if (first_SWI_run) {
        PosLeft_K_1 = PosLeft_K;
        PosRight_K_1 = PosRight_K;

        setEPWM2A(0);
        setEPWM2B(0);

        first_SWI_run = 0;
        DINT;
        return; // Exit early
    }

    VLeftK = (PosLeft_K - PosLeft_K_1)/0.004;
    VRightK = (PosRight_K - PosRight_K_1)/0.004;

    // --- INNER LOOP: COUPLED MOTOR CONTROL ---
    // The PD Light Loop (Outer) provides the 'turn' command.
    // This loop (Inner) forces the wheels to obey that command.
    e_turn = turn + (VLeftK - VRightK); // Error = Desired Turn - Actual Differential Velocity

    ek_left = Vref - VLeftK - Kp_turn * e_turn;
    ek_right = Vref - VRightK + Kp_turn * e_turn;

    Ik_left = Ik_left_1 + 0.004*(ek_left + ek_left_1)/2.0;
    Ik_right = Ik_right_1 + 0.004*(ek_right + ek_right_1)/2.0;

    // Anti-windup
    if (fabs(Ik_left) > 5.0) {
        if (Ik_left > 0) Ik_left = 5.0; else Ik_left = -5.0;
    }
    if (fabs(Ik_right) > 5.0) {
        if (Ik_right > 0) Ik_right = 5.0; else Ik_right = -5.0;
    }

    // Explicit Zeroing when stopped
    if (Vref == 0 && turn == 0) {
        Ik_left = 0;
        Ik_right = 0;
        uk_left = 0;
        uk_right = 0;
    } else {
        uk_left = Kp*ek_left + Ki*Ik_left;
        uk_right = Kp*ek_right + Ki*Ik_right;
    }

    if (fabs(uk_left) > 10.0) {
        if(uk_left > 0) uk_left = 10.0; else uk_left = -10.0;
    }
    if (fabs(uk_right) > 10.0) {
        if(uk_right > 0) uk_right = 10.0; else uk_right = -10.0;
    }

    // MODIFICATION: Check doneCal before writing to motors to prevent startup jump
    if (doneCal == 0) {
        setEPWM2A(0);
        setEPWM2B(0);
        // Force integrators to zero while calibrating to prevent windup
        Ik_left = 0;
        Ik_right = 0;
        uk_left = 0;
        uk_right = 0;
    } else {
        setEPWM2A(uk_right);
        setEPWM2B(-uk_left);
    }

    // Update previous states with current
    // Ik_left/right updates are commented out or ignored as we use Ik_fwd/gyro now
    PosLeft_K_1 = PosLeft_K; // Update old position value 4 ms ago with current value (left motor)
    PosRight_K_1 = PosRight_K; // Update old position value 4 ms ago with current value (right motor)
    EW_error_1 = EW_error;
    norm_EW_error_1 = norm_EW_error;
    light_derivative_raw_1 = light_derivative_raw;
    light_derivative_smoothed_1 = light_derivative_smoothed;

    // Legacy state updates (kept for safety if variables are monitored elsewhere)
    ek_left_1 = ek_left;
    ek_right_1 = ek_right;
    theta_left_1 = theta_left;
    theta_right_1 = theta_right;
    xdot_R_1 = xdot_R;
    ydot_R_1 = ydot_R;
    x_R_1 = x_R;
    y_R_1 = y_R;
    Ik_left_1 = Ik_left;
    Ik_right_1 = Ik_right;

    if (NewLVData == 1) {
        NewLVData = 0;
        Vref = fromLVvalues[0];
        turn = fromLVvalues[1];
        //W.Wu Added the segbot_refSpeed and turn Rate so that we can control and steer the robot while it is on the floor through labview.
        printLV3 = fromLVvalues[2];
        printLV4 = fromLVvalues[3];
        printLV5 = fromLVvalues[4];
        printLV6 = fromLVvalues[5];
        printLV7 = fromLVvalues[6];
        printLV8 = fromLVvalues[7];
    }
    if((numSWIcalls%62) == 0) { // change to the counter variable of you selected 4 ms. timer
        DataToLabView.floatData[0] = x_R; // x_R
        DataToLabView.floatData[1] = y_R; // y_R
        DataToLabView.floatData[2] = m;
        DataToLabView.floatData[3] = N;
        DataToLabView.floatData[4] = E;
        DataToLabView.floatData[5] = S;
        DataToLabView.floatData[6] = W;
        DataToLabView.floatData[7] = norm_EW_error;
        LVsenddata[0] = '*'; // header for LVdata
        LVsenddata[1] = '$';
        for (int i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
            } else {
                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
            }
        }
        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }
    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    //    if ((numTimer0calls%250) == 0) {
    //        displayLEDletter(LEDdisplaynum);
    //        LEDdisplaynum++;
    //        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
    //            LEDdisplaynum = 0;
    //        }
    //    }


    //Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    //GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 2; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    //    SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 3; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0xDA;
    //    SpibRegs.SPITXBUF = 500; // PWM value

    if ((numTimer0calls%50) == 0) {
        // Blink LaunchPad Red LED
        //        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    CpuTimer1.InterruptCount++;

    numTimer1calls++;



    //    // Exercise 6
    theta_left = LeftWheel; // Read left wheel rotation angle from left encoder (radian)
    theta_right = RightWheel; // Read right wheel rotation angle from right encoder (radian)
    thetadot_left = (theta_left - theta_left_1)/0.004; // Left wheel angular velocity (radian/s)
    thetadot_right = (theta_right - theta_right_1)/0.004 ; // Right wheel angular velocity (radian/s)

    Theta_R = R_Wh/W_R * (theta_right - theta_left); // Calculate robot bearing using differential between wheel angles
    theta_avg = 0.5 * (theta_right + theta_left); // Calculate robot bearing from average of two wheels
    thetadot_avg = 0.5 * (thetadot_right + thetadot_left); // Calculate robot angular velocity using average of two wheels

    xdot_R = R_Wh * thetadot_avg * cos(Theta_R); // Calculate robot x-velocity (converted to m/s)
    ydot_R = R_Wh * thetadot_avg * sin(Theta_R); // Calculate robot y-velocity (converted to m/s)

    x_R = x_R_1 + ((xdot_R + xdot_R_1)/2) * 0.004; // Numerical integration for x_R
    y_R = y_R_1 + ((ydot_R + ydot_R_1)/2) * 0.004; // Numerical integration for y_R
    //
    //    // Exercise 7 - Wall-following
    //    // Wall-following control algorithm

    //
    //    // Wall-following control algorithm (Exercise 8)
    //    if (rightwallfollow == 1) {
    //        turn = Kpright * (ref_right - distright); // Turn right to track wall
    //        Vref = V_right;
    //
    //        // Flash LEDs to indicate right wall following
    //        if ((numTimer1calls % 25) == 0) { // Flash approx. every 100ms
    //            // Forward row of LEDs
    //            GpioDataRegs.GPDSET.bit.GPIO111 = 1;
    //            GpioDataRegs.GPASET.bit.GPIO27 = 1;
    //            GpioDataRegs.GPESET.bit.GPIO159 = 1;
    //            GpioDataRegs.GPFSET.bit.GPIO160 = 1;
    //            // Right row
    //            GpioDataRegs.GPASET.bit.GPIO22 = 1;
    //            GpioDataRegs.GPCSET.bit.GPIO94 = 1;
    //            GpioDataRegs.GPCSET.bit.GPIO95 = 1;
    //            GpioDataRegs.GPDSET.bit.GPIO97 = 1;
    //
    //            // Turn off left row of LEDs when moving forward
    //            GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
    //            GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
    //            GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;
    //            GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
    //
    //        }
    //
    //        if (distfront < threshold1) { // Front wall is close
    //            rightwallfollow = 0; // Activate left turn
    //
    //            play_beep = 1; // Trigger beep
    //            //beep_song_index = 0; // Reset beep index
    //            // Turn off forward row of LEDs
    //            GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;
    //            GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
    //            GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;
    //            // Turn off right row of LEDs
    //            GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
    //            GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;
    //            GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;
    //            GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;
    //
    //            if ((numTimer1calls % 25) == 0) { // Flash approx. every 100ms
    //                GpioDataRegs.GPBSET.bit.GPIO60 = 1; // Turn on left row of LEDs
    //                GpioDataRegs.GPBSET.bit.GPIO61 = 1; // Turn on left row of LEDs
    //                GpioDataRegs.GPESET.bit.GPIO157 = 1; // Turn on left row of LEDs
    //                GpioDataRegs.GPESET.bit.GPIO158 = 1; // Turn on left row of LEDs
    //            }
    //        }
    //
    //    } else {
    //        turn = Kpfront * (ref_front - distfront); // Turn left when front wall is close
    //        Vref = V_front;
    //
    //        play_beep = 1; // Trigger beep
    //        //beep_song_index = 0; // Reset beep index
    //
    //        if ((numTimer1calls % 25) == 0) { // Flash approx. every 100ms
    //            GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Turn on left row of LEDs
    //            GpioDataRegs.GPBTOGGLE.bit.GPIO61 = 1; // Turn on left row of LEDs
    //            GpioDataRegs.GPETOGGLE.bit.GPIO157 = 1; // Turn on left row of LEDs
    //            GpioDataRegs.GPETOGGLE.bit.GPIO158 = 1; // Turn on left row of LEDs
    //        }
    //
    //        // Turn off forward row of LEDs
    //        GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;
    //        GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
    //        GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;
    //
    //        // Turn off right row of LEDs
    //        GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
    //        GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;
    //        GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;
    //        GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;
    //
    //        if (distfront > threshold2) { // Front wall is far away
    //            rightwallfollow = 1; // Resume right wall following
    //        }
    //    }
    //
    //
    // Update previous states with current
    //    Ik_left_1 = Ik_left; //updating old integral value 4 ms ago with current value (left motor)
    //    Ik_right_1 = Ik_right; //updating old integral value 4 ms ago with current value (right motor)
    //    PosLeft_K_1 = PosLeft_K; // Update old position value 4 ms ago with current value (left motor)
    //    PosRight_K_1 = PosRight_K; // Update old position value 4 ms ago with current value (right motor)
    //    ek_left_1 = ek_left;
    //    ek_right_1 = ek_right;
    //    theta_left_1 = theta_left;
    //    theta_right_1 = theta_right;
    //    xdot_R_1 = xdot_R;
    //    ydot_R_1 = ydot_R;
    //    x_R_1 = x_R;
    //    y_R_1 = y_R;


    if ((CpuTimer1.InterruptCount % 200) == 0) {
        UARTPrint = 0;}
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    //    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    song_index++;
    if (song_index >= SONG_LENGTH) {
        song_index = 0;
        buzzer = 0;
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0); //GPIO PinName, CPU, Mux Index
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
    }
    else {
        if (buzzer == 1) {
            EPwm9Regs.TBPRD = songarray[song_index];
        }
    }

    //    if ((CpuTimer2.InterruptCount % 10) == 0) {
    //        UARTPrint = 0;
    //    }
}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a part of the SPIB setup.
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected

    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset

    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16-bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason

    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL

    //SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPIFFCT.bit.TXDLY = 0;

    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I dont think this is needed. Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL =16; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below

    //-----------------------------------------------------------------------------------------------------------------

    //Step 2.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low


    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16-bit transfers, so two registers at a time after the first 16-bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = (0x1300 | 0x0000);
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = (0x0000 | 0x0013);
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = (0x0200 | 0x0000);
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = (0x0800 | 0x0006);
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = (0x2300 | 0x0000);
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = (0x4000 | 0x008C);
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = (0x0200 | 0x0088);
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = (0x0C00 | 0x000A);

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
    // perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = (0x2A00 | 0x0081);
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x0019); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x008A); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0011); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x007C); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0028); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x00B2); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;


}

int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;


__interrupt void SPIB_isr(void) {
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    spivalue1 = SpibRegs.SPIRXBUF;
    accelx_raw = SpibRegs.SPIRXBUF;
    accely_raw = SpibRegs.SPIRXBUF;
    accelz_raw = SpibRegs.SPIRXBUF;
    spivalue2 = SpibRegs.SPIRXBUF;
    gyrox_raw = SpibRegs.SPIRXBUF;
    gyroy_raw = SpibRegs.SPIRXBUF;
    gyroz_raw = SpibRegs.SPIRXBUF;

    accelx = accelx_raw*(4.0/32767.0);
    accely = accely_raw*(4.0/32767.0);
    accelz = accelz_raw*(4.0/32767.0);

    gyrox = gyrox_raw*(250.0/32767.0);
    gyroy = gyroy_raw*(250.0/32767.0);
    gyroz = gyroz_raw*(250.0/32767.0);

    //    W = (3.0/4096.0) * ADCINA2raw;
    //    E = (3.0/4096.0) * ADCINA3raw;
    //    S = (3.0/4096.0) * ADCINA4raw;
    //    N = (3.0/4096.0) * ADCINA5raw;
    //    LeftWheel=readEncLeft();
    //    RightWheel=readEncRight();
    //Code to be copied into SPIB_isr interrupt function after the IMU measurements have been collected.
    /* W.Wu
     * In the SPIB we start by having 2 seconds of doing nothing. Then we have 2 seconds where the robot calculates the accel and gyro offsets. During this time the BLUE LED on the
     * red board will be flashing and when the offsets are done calculating it turns off. Afterwards we now have the Kalman filter which averages the previous four tilt and gyro
     * values and also the previous four left and right wheel angular velocities. In calibration state 2 it also saves the current accelz balance point which tells the robot
     * what tilt that it should focus on reaching.
     */
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1) {
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
        W_offset+=W;
        E_offset+=E;
        N_offset+=N;
        S_offset+=S;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            W_offset/=2000.0;
            E_offset/=2000.0;
            N_offset/=2000.0;
            S_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2) {
        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        N -= N_offset;
        E -= E_offset;
        S -= S_offset;
        W -= W_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        AverageIndex++;
        // Kalman Filter used
        tilt_array[AverageIndex] = kalman_tilt;
        gyro_array[AverageIndex] = tiltrate;
        LeftWheelArray[AverageIndex] = readEncLeft();
        RightWheelArray[AverageIndex] = readEncRight();
        if (AverageIndex >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            AverageIndex = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }

    }
    if((SpibNumCalls%200) == 0) {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
    }
    if ((SpibNumCalls % 200) == 0) {
        UARTPrint = 1;
    }
    SpibNumCalls++;

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    // Loops to drive wheels on completed board back and forth periodically

    // Call setEPWM2A functions
    //    setEPWM2A(uRight);
    //    setEPWM2B(-uLeft);
}

// ----- code for CAN start here -----
__interrupt void can_isr(void)
{
    int i = 0;

    uint32_t status;

    GpioDataRegs.GPBSET.bit.GPIO52 = 1;
    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    status = CANgetInterruptCause(CANB_BASE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CANgetStatus(CANB_BASE);

    }

    //
    // Check if the cause is the transmit message object 1
    //
    //    else if(status == TX_MSG_OBJ_ID)
    //    {
    //        //
    //        // Getting to this point means that the TX interrupt occurred on
    //        // message object 1, and the message TX is complete.  Clear the
    //        // message object interrupt.
    //        //
    //        CANclearInterruptStatus(CANB_BASE, TX_MSG_OBJ_ID);
    //
    //        //
    //        // Since the message was sent, clear any error flags.
    //        //
    //        errorFlag = 0;
    //    }

    //
    // Check if the cause is the receive message object 2
    //
    else if(status == RX_MSG_OBJ_ID_1)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_1, rxMsgData);

        for(i = 0; i<2; i++)
        {
            dis_raw_1[i] = rxMsgData[i];
        }

        dis_1 = 256*dis_raw_1[1] + dis_raw_1[0];

        measure_status_1 = rxMsgData[2];

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_1);

        //
        // Increment a counter to keep track of how many messages have been
        // received. In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        rxMsgCount_1++;

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
        GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    }

    else if(status == RX_MSG_OBJ_ID_2)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_2, rxMsgData);

        for(i = 0; i<2; i++)
        {
            dis_raw_3[i] = rxMsgData[i];
        }

        dis_3 = 256*dis_raw_3[1] + dis_raw_3[0];

        measure_status_3 = rxMsgData[2];

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_2);

        //
        // Increment a counter to keep track of how many messages have been
        // received. In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        rxMsgCount_3++;

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
        GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    }

    else if(status == RX_MSG_OBJ_ID_3)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_3, rxMsgData);

        for(i = 0; i<4; i++)
        {
            lightlevel_raw_1[i] = rxMsgData[i];
            quality_raw_1[i] = rxMsgData[i+4];

        }

        lightlevel_1 = ((256.0*256.0*256.0)*lightlevel_raw_1[3] + (256.0*256.0)*lightlevel_raw_1[2] + 256.0*lightlevel_raw_1[1] + lightlevel_raw_1[0])/65535;
        quality_1 = ((256.0*256.0*256.0)*quality_raw_1[3] + (256.0*256.0)*quality_raw_1[2] + 256.0*quality_raw_1[1] + quality_raw_1[0])/65535;


        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_3);

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
        GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    }


    else if(status == RX_MSG_OBJ_ID_4)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_4, rxMsgData);

        for(i = 0; i<4; i++)
        {
            lightlevel_raw_3[i] = rxMsgData[i];
            quality_raw_3[i] = rxMsgData[i+4];

        }

        lightlevel_3 = ((256.0*256.0*256.0)*lightlevel_raw_3[3] + (256.0*256.0)*lightlevel_raw_3[2] + 256.0*lightlevel_raw_3[1] + lightlevel_raw_3[0])/65535;
        quality_3 = ((256.0*256.0*256.0)*quality_raw_3[3] + (256.0*256.0)*quality_raw_3[2] + 256.0*quality_raw_3[1] + quality_raw_3[0])/65535;

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_4);

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
        GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    }



    //
    // If something unexpected caused the interrupt, this would handle it.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CANclearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    InterruptclearACKGroup(INTERRUPT_ACK_GROUP9);
}
// ----- code for CAN end here -----
void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}
float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U

    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true

    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    float scale = (2.0 * PI )/(400.0 * 30.0);
    return (raw*-scale);
}

float readEncRight(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int

    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true

    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    float scale = (2.0 * PI )/(400.0 * 30.0);
    return (raw*scale);
}

void setEPWM2A(float controleffort) {
    if (controleffort > 10) { // Checks if controleffort is less than or equal to the absolute value of saturation_limit (-10 <= x <= 10)
        controleffort = 10;
    }// Sets to positive limit of controleffort
    if (controleffort < -10) {
        controleffort = -10;
    } // Sets to negative limit of controleffort

    EPwm2Regs.CMPA.bit.CMPA = (2500.0/20.0)*controleffort + 1250.0 ; // If controleffort is within the predefined range, then use this linearly changing value
}

// Function to control EPWM2B (right motor)
void setEPWM2B(float controleffort) {
    if (controleffort > 10) { // Checks if controleffort is less than or equal to the absolute value of saturation_limit (-10 <= x <= 10)
        controleffort = 10; // Sets to positive limit of controleffort
    }
    if (controleffort < -10) {
        controleffort = -10; // Sets to negative limit of controleffort
    }
    EPwm2Regs.CMPB.bit.CMPB = (2500.0/20.0)*controleffort + 1250.0; // If controleffort is within the predefined range, then use this linearly changing value
}

// Function to initialize EPWM9 for the buzzer
void InitEPwm9(void) {
    EALLOW;
    //EPWM9A
    EPwm9Regs.TBCTL.bit.CLKDIV = 1;
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm9Regs.TBCTL.bit.CTRMODE = 0;
    EPwm9Regs.TBCTL.bit.PHSEN = 0;
    EPwm9Regs.TBCTR = 0;
    EPwm9Regs.TBPRD = 2500;
    //EPwm9Regs.CMPA.bit.CMPA = 0;
    EPwm9Regs.AQCTLA.bit.CAU = 0;
    EPwm9Regs.AQCTLA.bit.ZRO = 3;
    EPwm9Regs.TBPHS.bit.TBPHS = 0;
    EDIS;
}
