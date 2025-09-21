// MSP430g2553 project Header File for EEL3926 Junior Design
//                             May 8, 2024
//                       Written by Arthur R. Weeks
//
// credit for the I2C part of the program is from
// https://forum.allaboutcircuits.com/ubs/msp430-i2c-lcd1602-interface.1822/
// the use of the program is for educational purpose only and
// not for commercial use
//
// SunFounder I2C LCD 1602
// MSP430F5529 I2C interface
// MrChips - 2023.07.29

// MSP430G2553 has one I2C interface, using pins P1_6 and P1_7
// P1.7 = SDA pin 15, P1.6 = SCL pin 14
// On P1.6 there is also an LED it must be removed to use the I2C

//---------------------------------------------------------------
// MSp430G2553 self contained programming pins SWBTDIO and SWBTCK
// SWBTDIO pin 16 20 pin Dip Pin 16 is also Reset not
// SWBTCK  pin 17 20 pin Dip
// GND, 3.3 Volts and 5 volts (either from programming board or
// from regulators
//---------------------------------------------------------------
// Need to remove the jumper for the LED on Port 1 bit 6 P1.6
// P1.6 is being used for I2C
//---------------------------------------------------------------

//---------------------------------------------------------------
// Port P1.1 used for pulse width modulation to drive the LED
// Port P1.1 has an LED connected to it and as the pulse
// width changes the brightness of the LED changes
//---------------------------------------------------------------

//---------------------------------------------------------------
// The ADC uses Channel 3 located on P1.3
// uses the 10 bits ADC with and output of
// 0v = 0 and 3.3 = 1023
//---------------------------------------------------------------

//---------------------------------------------------------------
// Ultrasonic Sensor
// Trigger is connected to Port 2 bit 3 P2.3 need a 10 usec pulse to start
// Ultrasonic sensor start Trigger operation
// delay by 10usec since we use a 10 usec
// pulse needed to start the ultrasonic trigger
//---------------------------------------------------------------

//---------------------------------------------------------------
// The pulse width is controlled by the variable Duty variable
// has a time period of 10 milliseconds
// the resolution of the pWM is 100 usec
// --------------------------------------------------------------
//---------------------------------------------------------------
// Echo Return is on port 2 bit 4 P2.4
// we have a rising edge of the echo
// return signal and followed by a falling edge (P2.4 goes to zero).
// the width of the pulse is the echo time
// which is the total time go to the object and back
//---------------------------------------------------------------

//-------------------------------------
// #includes Files here
//-------------------------------------
#include "msp430.h"
// Needed by the sprintf function
#include "stdio.h"

//-------------------------------------
// #defines
//-------------------------------------
// NULL Not defined in Code Composer Studio
#define NULL 0
// I2C defuat slave address for the LCD
#define SLAVE_ADDRESS 0x27
// Used to put  the LCD in command mode or character mode
#define LCD_RS BIT0
// Read or write to the LCD display
#define LCD_RW BIT1
// Use to strobe data to the LCD display
#define LCD_E  BIT2
// needed to turn on the LCD Back light
#define LCD_BACKLITE BIT3


// ---------------------------------------------------------------------------------
// Timer overflow value for the Timer A
// a count of 99 gives a 100 usec resolution or clock tick
// a count of 9 gives a 10 usec resolution or clock tick
// we start at zero and count up so we need to go to a count of 99
// TACCR0 =99;
// ---------------------------------------------------------------------------------
// should calibrate these values for each board
// For the MSP430G2553
// 99 give a timer interrupt of 100 usec which yields a ultrasonic
// double path sensor resolution of 34 mm
// #define TIMER_MAX_COUNT 99
// A 9 gives a timer interrupt of 10 usec which yields a ultrasonic
// double path sensor resolution of 3.4 mm
// with a 10 usec a timer resolution
// with a 100 usec timer yields 96 usec to 100 usec
// 4 usec jitter
#define TIMER_MAX_COUNT_A 9
//----------------------------------------------------------------------------
// This is the timer overflow count for the second timer running at
// 100 sec rate
// used by the pulse width modulator
#define TIMER_MAX_COUNT_B 99
//--------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// these defines are for the for loop Delay function with a MCLK (CPU) of 8 MHz
#define DELAY_100us    110
#define DELAY_200us    220
#define DELAY_500us    550
#define DELAY_1ms     1125
#define DELAY_10ms   11250
#define DELAY_50ms   56250
#define DELAY_1s   1125000

// These defines are for the Delay_Timer Function using the A_timer interrupt
#define DELAYT_100us      1
#define DELAYT_1ms        10
#define DELAYT_10ms       100
#define DELAYT_100ms      1000
#define DELAYT_500ms      5000
#define DELAYT_1000ms     10000
#define DELAYT_1s         10000
#define DELAYT_2s         20000
//--------------------------------------------------------------------------------

//-----------------------------------------------------------------
//Global variables go here
//-----------------------------------------------------------------
volatile static unsigned long int Delay_Count;
// These global variables are used by the utrasonic sensor
static unsigned long int Counter = 0;
static unsigned long int Elapsed_Time_Start;
static unsigned long int Elapsed_Time_Stop;
// This variable used by the Timer B timer
static unsigned long int Counter_B = 0;
// Duty cycle time for the PWM implemented in Timer A-1
// It can be controlled by the ADC between 0 to 1023 0 = 0% and 100 = 100 %
// directly given by the ADC
// The pulse width is controlled by the variable Duty variable
// has a time period of 10 milliseconds
// the resolution of the pWM is 100 usec
// Default the duty cyle to 100%
static unsigned long int Duty = 100;
// Define LED connected to port 1 bit 1
LED_P1_1 = BIT1;

// To start the trigger of the ultrasonnic sensor
static int Trigger1 = 0;
// used in the state machine to find the end of the echo
// return pulse
static int State_1 = 0;
// --------------------------------------------------------------------


//-------------------------------------
// Hardware and Initialization functions
//-------------------------------------
void Init_HW(void);
// default address is 0x27 for the LCD display
void LCD_I2C_Init(void);
void I2C_Init2(void);
void LCD_I2C_Send(unsigned int d);
// This timer used for the Timer_Delay function
// and the distance count used by the ultrasonic sensor
// 10 usec per clock tick
void Init_Timer_A(void);
// this is a second timer used by the system for the PWM to
// control the brightness of an LED
// 1000 usec per clock tick
void Init_Timer_B(void);
// used by the ADC converter
void Init_ADC(void);
unsigned int ADC_Read(void);
void Delay_Timer(volatile unsigned long int Time_Microsecond);
void Delay(volatile unsigned long int d);
void GPIO_Interrupt_Init_Port2(void);
void Ultrasonic_Trigger(void);
unsigned long int Ultrasonic_Echo_Read(void);


//---------------------------------------------------------------------------
// The LCD commands are based on the HITACHI LCD1602 Driver HD44780U (LCD-II)
// (Dot Matrix Liquid Crystal Display Controller/Driver) from its datasheet
//---------------------------------------------------------------------------
// Default address for the LCD1602 display is 0x27
// The datasheet for the LCD1602 display is Hitachi HD44780
//---------------------------------------------------------------------------
void LCD_I2C_Init(void);
void LCD_I2C_Send(unsigned int);
void LCD_Cmd(char ch);
void LCD_Character(char ch);
void LCD_Initialize(void);
void LCD_CLRscreen(void);
void LCD_Cursor_On_Blink(void);
void LCD_Cursor_On_No_Blink(void);
void LCD_Cursor_Off(void);
void LCD_Cursor_Postion(char Cursor_Position);
void LCD_String(char *String);

//---------------------------------------------------------------
// LCD Library functions
// --------------------------------------------------------------
// LCD_Initialization
// setup the the LCD Display
// This function sends commands to the display to clear the screen
// puts the display in dual 4 bit mode, cursor on / off, etc.
//---------------------------------------------------------------
void LCD_Initialize(void)
{
  LCD_Cmd(0x02);  // home
  LCD_Cmd(0x28);  // dual line, 4 bits
  LCD_Cmd(0x06);  // increment mode
  LCD_Cmd(0x0C);  // cursor turned off
  LCD_Cmd(0x10);  // cursor right
  LCD_Cmd(0x01);  // clear display
}

//-------------------------------------
// LCD_Cmd
// LCD Lower functions
// The command is passed as an 8 bit character
//--------------------------------------------
void LCD_Cmd(char ch)
{
  char d;
  d = (ch & 0xF0) + LCD_BACKLITE; // send upper nibble
  LCD_I2C_Send(d);
  LCD_I2C_Send(d + LCD_E);            // pulse E high-low
  LCD_I2C_Send(d);
  LCD_I2C_Send(LCD_BACKLITE);

  d = ((ch << 4) & 0xF0) + LCD_BACKLITE;  // send lower nibble
  LCD_I2C_Send(d);
  LCD_I2C_Send(d + LCD_E);    // pulse E high-low
  LCD_I2C_Send(d);
  LCD_I2C_Send(LCD_BACKLITE);
  Delay_Timer(DELAYT_1ms);
}

//-----------------------------------------------------------
// LCD_Character
// Displays a single character at the present cursor position
// passed via a character variable
//-----------------------------------------------------------
void LCD_Character(char ch)
{
  char d;
  d = (ch & 0xF0) + LCD_RS + LCD_BACKLITE; // send upper nybble
  LCD_I2C_Send(d);
  LCD_I2C_Send(d + LCD_E);                     // pulse E high-low
  LCD_I2C_Send(d);
  LCD_I2C_Send(LCD_RS + LCD_BACKLITE);

  d = ((ch << 4) & 0xF0) + LCD_RS + LCD_BACKLITE;  // send lower nybble
  LCD_I2C_Send(d);
  LCD_I2C_Send(d + LCD_E);              // pulse E high-low
  LCD_I2C_Send(d);
  LCD_I2C_Send(LCD_RS + LCD_BACKLITE);
//  Delay_Timer(DELAYT_1ms);
}


//-----------------------------------------------------------
// LCD_String
// Displays a Null Terminated Character String with
// no line feed (0x0a) or carriage return (0x0d) accepted.
// Prints the string starting at the present cursor position
// a pointer to the string that is to be printed is passed
// to the function
//-----------------------------------------------------------
void LCD_String(char *String)
{
  volatile static int i;
  i=0;
  while(*(String+i) != NULL){
      LCD_Character((char)*(String +i));
      i++;
  }
}

//------------------------------------------------------------------
// LCD_CLRscreen
// Clear the Screen and locate the cursor at top left position (home)
//------------------------------------------------------------------
void LCD_CLRscreen(void)
{
  LCD_Cmd(0x02);  // home
  LCD_Cmd(0x01);  // clear display
  Delay_Timer(DELAYT_1ms);
}

//------------------------------------------------------------
// LCD_Cursor_On_Blink
// Turn the cursor on and blinking
//------------------------------------------------------------
void LCD_Cursor_On_Blink(void)
{
  LCD_Cmd(0x0f);  // Cursor on and blinking
  Delay_Timer(DELAYT_1ms);
}

//------------------------------------------------------------
// LCD_Cursor_On_No_Blink
// Turn the cursor on and no blinking
//------------------------------------------------------------
void LCD_Cursor_On_No_Blink(void)
{
  LCD_Cmd(0x0E);  // Cursor on and blinking
  Delay_Timer(DELAYT_1ms);
}

//------------------------------------------------------------
// LCD_Cursor_Off
// Turn the cursor off
//------------------------------------------------------------
void LCD_Cursor_Off(void)
{
  LCD_Cmd(0xC);  // cursor turned off
  Delay_Timer(DELAYT_1ms);
}

//------------------------------------------------------------
// LCD_Cursor_Position
// for the LCD1602 LCD display 16 characters by 2 lines
// Sets the present Cursor position
// 0 - 15 for line one (0x00 - 0x0f)
// 64 - 79 for line two (0x40 - 0x4f)
//------------------------------------------------------------
void LCD_Cursor_Postion(char Cursor_Position)
{
  volatile char x;
  x=0x80 | Cursor_Position;
  LCD_Cmd(x);  // Cursor on and blinking
  Delay_Timer(DELAYT_1ms);
}

//-------------------------------------
// Delay functions
//-------------------------------------
//------------------------------------------------
// Delay
// This delay function is based on using a for loop
// This delay function is not interrupt based
// ------------------------------------------------
void Delay(volatile unsigned long int d)
{
 while (d>0){
 d--;
 }
}

//--------------------------------------------------------
// Delay_Timer
// This delay function uses the A_timer interrupt to
// determine the delay
// A timer is set to overflow once 10 microseconds
// d is in 10 microseconds increments desire
//--------------------------------------------------------
void Delay_Timer(volatile unsigned long int d)
{
// All of the count #define values in the beginning of the code
// given for a 10 usec A timer clock
// needed 10 usec for better resolution with the ultrasonic sensor
// 34 mm to 3.4mm resolution double path
// 1.7 mm single path resolution
// multiply the time count by 10 to use a 100 sec resolution for the
// Delay_Timer function
// the A_timer counts in 10 usec steps
// the passed value of d dives the number of 10 usec time steps
 while (Delay_Count < d*10){
     }
 Delay_Count = 0;
}

//-------------------------------------
// I2C functions
//-------------------------------------

//-------------------------------------
// LCD_I2C_Init
// This function performs the required I2C
// initialization need for the LCD
// the default LCDd I2C address is 0x27
//-------------------------------------
void LCD_I2C_Init(void)
{
  while (UCB0CTL1 & UCTXSTP);
  UCB0CTL1 |= UCSWRST;
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;
  UCB0CTL1 = UCSSEL_2 + UCSWRST;
  UCB0BR0 = 64;
  UCB0BR1 = 0;
  UCB0I2CSA = SLAVE_ADDRESS;
  UCB0CTL1 &= ~UCSWRST;        // clear software reset
}

//-------------------------------
// LCD_I2C_Send
// This I2C function sends 8 bits of data from
// the MSP430 to the I2C bus
// because of timing requirement is must use
// the non A_timer delay function
// The A_timer interrupt does not work for this
// function
//-------------------------------
void LCD_I2C_Send(unsigned int d)
{
  while (UCB0CTL1 & UCTXSTP);
// Start the I2C data stream
  UCB0CTL1 |= UCTR + UCTXSTT;
// Load the buffer with data
  UCB0TXBUF = d;
// Cannot use the Timer_delay function
// interrupts corrupt the I2C communication timing
  Delay(DELAY_200us);
// Send an I2C stop
  UCB0CTL1 |= UCTXSTP;
}

//-------------------------------------
// GPIO_Interrupt_Init_PORT2
// initialize Port 2 bit 4 to input and enable its pull down resistor
// It enables Port 2 Bit 3 as an output pin
// GPIO Port 2 (2.4) interrupt initialization
// This function sets bit 4 of port as an input port
// is also enables the resistor as a pull down resistor
// for Port 2 bit 3 it is enabled as an input pin
//-------------------------------------
void GPIO_Interrupt_Init_PORT2(void){
    P2DIR = 0x08;      // Port 2  all as input pins except bit 3 which is output
    P2OUT = 0;         // set all outputs to zero
    P2REN |= BIT4;     // Enable Pull/down resistor on P2.4
    P2OUT &= ~BIT4;    // Set P2.4 to pull low resistor
    P2IE  |= BIT4;     // Enable interrupts on P2.4
    P2IES &= ~BIT4;    // Trigger an interrupt from low to high on P2.4  need 0 in P2IES reg
    P2IFG &= ~BIT4;    // Clear P2.4 interrupt flag
    _BIS_SR(GIE);      // Enable the interrupts globally
}

//----------------------------------------------------------------
// Ultrasonic_Trigger
// This function sends out on port bit 3 a 10usec pulse to
// start the ultrasonic sensor to start / trigger an echo measurement
// Some time after this trigger pulse about 2.2 msec the
// echo pulse goes high.  It stays high proportional to the distance
// to the object / reflection.  Further away the longer the pulse stays high
//----------------------------------------------------------------
void Ultrasonic_Trigger(void){
// Set Port 2 bit 3 to a one to start
// Ultrasonic sensor start Trigger function
// The data sheet requires a 10 usec trigger pulse
// we need to delay by 10usec since we a 10 usec
// pulse requirement to start the ultrasonic trigger
// With a delay of 2 it measures a pulse width of 9.6 usec
// With a delay of 3 it measures a pulse width of 12.1 usec
//   P2OUT |= BIT3; // set Port 2 Bit 3 to a one
   P2OUT |=BIT3;
   Delay(2);      // delay be 10 uses
   P2OUT &= 0xf7; // leave port 2 Bits unchanged except Bit 3
   Trigger1 = 1;  // set the trigger flag to one for first trigger
                  // on the rising edge
   State_1 = 0;   //  Set the initial state of the state machine to 0
   Counter = 0;   // start the time counter to 0;
}

//----------------------------------------------------------------
// Ultrasonic_Echo_Read
// This function read the echo time from the ultrasonic sensor
// you must wait a minumum of 80 milliseconds before calling this function
// to read the echo time.
// the return value is the echo time in units on 10 usec
// start the ultrasonic sensor to start echo measurement
// range = high level time * velocity (340M/S) / 2 double path
// or (1,115.4ft/sec)
//----------------------------------------------------------------
unsigned long int Ultrasonic_Echo_Read(void){
    return(Elapsed_Time_Stop-Elapsed_Time_Start);
}


//------------------------------------------------------
// Init_HW
// Initialization
// this function sets the port direction for Port 1 to
// Blink the LED, Set the  MCLK frequency to 8 MHz for the correct
// time delay functions
//------------------------------------------------------
void Init_HW(void)
{
// Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
//------------------------------------------------
// Set the Master Clock freqeuncy to 8Mhz
//------------------------------------------------
// set DCO - Digitally-Controlled Oscillator frequency
// change 8 MHz  calibrated
    BCSCTL1 = CALBC1_8MHZ;
    DCOCTL  = CALDCO_8MHZ;
//-------------------------------------------------
// Setup Port 1
// Direction for Port 1 bit 0 to output for the LED
// Configure bits 6 and 7 for I2C
//--------------------------------------------------
// for the msp430g2553
// Each PxSEL and PxSEL2 bit is used to select the pin function
// - I/O port or peripheral module function.
// 00b = General-purpose I/O is selected
// 01b = Primary module function is selected
// 10b = Secondary module function is selected
// 11b = Tertiary module function is selected
//--------------------------------
// MSP430G2553 has two I2C interface pins, using pins P1_6 and P1_7
// Set P1.0 and P1.1 as output pins
// On the MSP430g2553ET board P1.0 has a default LED connected to it
// Adding an LED to Port P1.1
    P1DIR  = 0x03;
    P1OUT  = 0x00;
// assign P1.6 and P1.7 to I2C
    P1SEL  = BIT6 + BIT7;
    P1SEL2 = BIT6 + BIT7;

// ----------------------I2C Initialization -------------------------------
    LCD_I2C_Init();


//------------------------Timer Initialization ------------------------------
// Go initialize the A timer the first of two timers
   Init_Timer_A();
// In the MSP430g2553
// Go initialize the B timer the second of two timers
// In the MSP430g2553
   Init_Timer_B();
// let's wait for the timer to start up
   Delay(DELAY_1ms);
// enable the interrupts for the timer A-0 and A-1 interrupt functions
   _enable_interrupt();

// Let's wait for the timers to start otherwise we
// can not use the timer delay as the timer may not be started
   Delay(DELAY_1ms);

// Let's go initialize the ADC on port P1.3 (PORT 1 - BIT 3)
   Init_ADC();

// ----------------------GPIO Interrupt Initialization for Port 2 ---------
// Used by the ultrasonic sensor
   GPIO_Interrupt_Init_PORT2();
}


//--------------------------------------------------------
// Init_Timer_A The First of two timers in the MSP430g2553
// Timer A Initialization and Timer interrupts
//--------------------------------------------------------
//This Timer A routine used MSP430G2553 runs at 10 usec
//--------------------------------------------------------
void Init_Timer_A(void)
{
// Stop the timer so we can configure timer A
   TACCR0 = 0;
//Enable the timer A interrupt for CCR0
   TACCTL0 |= CCIE;
// Select the timer clock input to SMCLK SMClK divider by 1 and up mode.
// TASSEL_2 selects the SMCLK, ID_0 is divide by 1, MC_0 up count
   TACTL = TASSEL_2 + ID_0 + MC_1; //Select SMCLK, SMCLK/1 , Up Mode
// Set the compare VALUE TO value count of 10
// Need a count of 10 for a clock tick for 9-10 microsecond overflow of the counter
// We start at zero so we need to go to a count of 9
// Start Timer, Compare value for Up Mode to get 9-10us delay per loop
 // Total count = TACCR0 + 1. Hence we need to subtract 1.
// 100 ticks at  8 MHz gives a delay of 100us used by the timer delay function.
// used by the MSP 430G2553
   TACCR0=TIMER_MAX_COUNT_A*8; // A factor 8 is for the 8 MHz clock for MSP430G2553
}

//---------------------------------------------------------
// Init_Timer_A The Second  of two timers in the MSP430g2553
// Timer A Initialization and Timer interrupts
//---------------------------------------------------------
//This Timer A routine used MSP430G2553 runs at 100 usec
//---------------------------------------------------------
void Init_Timer_B(void)
{
// Stop the timer so we can configure timer A
   TA1CCR0 = 0;
//Enable the timer A interrupt for CCR0
   TA1CCTL0 |= CCIE;
// Select the timer clock input to SMCLK SMClK divider by 1 and up mode.
// TASSEL_2 selects the SMCLK, ID_0 is divide by 1, MC_0 up count
   TA1CTL = TASSEL_2 + ID_0 + MC_1; //Select SMCLK, SMCLK/1 , Up Mode
// Set the compare VALUE TO
// Need a count of 100 for a clock tick for 100 microsecond overflow of the counter
// We start at zero so we need to go to a count of 99
// TACCR0 =99;
// Start Timer, Compare value for Up Mode to get 100us delay per loop
// Total count = TACCR0 + 1. Hence we need to subtract 1.
// 100 ticks at  8 MHz gives a delay of 100us.
// used by the MSP 430G2553
    TA1CCR0=TIMER_MAX_COUNT_B*8; // A factor 8 is for the 8 MHz clock for MSP430G2553
}

//---------------------------------------------------------
// Setup the ADC on port 1 bit 3
// Bit 3 is only used for instead of I/O but
// for the ADC channel 3
//---------------------------------------------------------
void Init_ADC(void){
// Set the ADC reference to Vcc and Vss
// REF_2.5V = 2.5 volt reference
// ADC10DF = 2's complement otherwise unsign binary
   ADC10CTL0 = 0;
// SREF_0 ADC ref = Vcc
// SREF_1 = internal reference
// ADC10SHT_3 = set to 64 sample / hold clocks
// ADC10ON =1 turns ADC on hardware
   ADC10CTL0 = SREF_0 + + ADC10SHT_3 + ADC10ON;  // Set ADC module
   ADC10CTL1 = 0;
// INCH_3 set to channel P1.3
// SHS_0 set to 0 to ADC10SC bit (ADC Start Bit)
// ADC10SSEL_3 set ADC clock to SMclock
// CONSEQ_0 set to single A/D sampled
   ADC10CTL1 = SHS_0 + CONSEQ_0 + INCH_3  + ADC10SSEL_0;                        // Set ADC module
// Enable the ADC for Conversion gets the ADC ready for conversion
   ADC10CTL0 |= ENC;
// Turn on pin 3 for ADC input
   ADC10AE0 = BIT3;
}

//----------------------------------------------
// This function starts an ADC conversion and
// returns and unsigned integer number between 0
// and the Vref selected in digital numbers 0 to 1023
//---------------------------------------------
unsigned int ADC_Read(void){
//  Go tell the ADC to start a conversion
    ADC10CTL0 |= ADC10SC;
//  Wait for the ADC conversion to be finished
    while ((ADC10CTL1 & ADC10BUSY) == 1){
    }
    // ADC conversion so get the conversion result
    return((unsigned int)ADC10MEM);
}

//------------------------------------------------------------
// The Interrupt Service Function for Timer_A 0
// of the two timers in the MSP430g2553
// used by the Delay_Timer function
// and for the ultrasonic sensor
//------------------------------------------------------------
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A_CCR0_ISR(void)
{
// Used by the Delay_timer function
    Delay_Count++;
// If trigger = 1 then we need to start counting time to an
// echo is received by the ultrasonic sensor

    Counter ++;

// We get into this state after we have a rising edge of the echo
// return signal and followed by a falling edge (P2.4 goes to zero).
// we now need to store the final time into a variable
// Elasped_Time_Stop
    if(Trigger1==0 && State_1 == 1 && (P2IN & BIT4) == 0){
         State_1 = 0;
         Elapsed_Time_Stop = Counter;
         Counter = 0;
   }
}

//------------------------------------------------------------
// The Interrupt Service Function for Timer_A 1
// of the two timers in the MSP430g2553
// It also used as a Pulse Width Modulator for 
// PORT 1 Bit 1 P1.1
// Care must be used when accessing port one as
// P1.6 and P1.7 are used By I2C SCK and SDA
//------------------------------------------------------------
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A_CCR0_ISR(void)
{
// Counter_B set to 10 msec period
    Counter_B++;
    if(Counter_B == 100){
        Counter_B = 0;
    }
    if(Counter_B < Duty)
// Port P1.1 used fo pulse width modulation
           P1OUT |= BIT1;
        else
           P1OUT &= ~BIT1;
}

//-------------------------------------------------------------
// The ISR for Port 2 bit 4
// used for the ultrasonic sensor to start counting the echo time
// a rising edhe od P2.4 causes and interrupt ot occur which is handled
// by this function
//-------------------------------------------------------------
#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void){
// received the rising edge of echo signal from the ultrasonic sensor
// need to store the start time in a variable
// Elapsed_Time_Start
// set trigger1 to zero and set State_1 to 1 so we can start looking
// for the falling edge of the echo return signal
// range = high level time * velocity (340M/S) / 2 double path
// or (1,115.4ft/sec)
   if(Trigger1 == 1 && (P2IN && BIT4) == 1){
        Trigger1 = 0;
        Elapsed_Time_Start = Counter;
        State_1 = 1;
    }
 P2IFG &=~BIT4;  //Clear P.4 interrupt flag
}
