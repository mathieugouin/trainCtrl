/*************************************************************************
* Compiler :  CCS C - 16F876
* Programmer :  Mathieu Gouin
* Description : A program to time the duration of a drag race.
*
*************************************************************************/

/************************************************************************
* Description : Template
* Parameters :  void
* Return :      void
************************************************************************/

/************************************************************************/
/*                      PRE-PROCESSOR DIRECTIVES                        */
/************************************************************************/

#case // Case-sensitive compiler

#include <16F876.H>   // General library
// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>

#include "common.h"
#include "eeprom.h"
#include "i2c_lcd.h"

/* The PCM/PCW compilers now allow for 16 bit pointers for parts with
   RAM above 255.  Use a #device statement like the following to use
   16 bit pointers.  When 8 bit pointers are used the compiler will no
   longer use RAM above 255 except with #byte or read/write _ bank.
*/
//#device *=16

/* Configures the read_adc return size. For example, using
a PIC with a 10 bit A/D you can use 8 or 10 for xx- 8 will
return the most significant uint8_t, 10 will return the full A/D
reading of 10 bits. */
#device ADC=10

// Fuses : High Speed, NO WatchDog Timer, PowerUp Timer,
// NO code PROTECTion, NO Low Voltage Programmation.
#fuses HS,NOWDT,PUT,NOPROTECT,NOLVP

#use delay(clock=20000000)    // 20 MHz Clock

/************************************************************************/
/*                          TYPE DEFINITIONS                            */
/************************************************************************/

/************************************************************************/
/*                         MACROS DEFINITIONS                           */
/************************************************************************/

#define LED_PAUSE               (uint8_t)100   // time in ms to wait ON and OFF
#define BOOT_NB_ADDRESS         (uint8_t)0x00  // where the boot number is stored in EEPROM
#define DELAY_MESSAGE           2000           // delay in ms for the lcd message

// I2C IO Expander PCF8574 #1 address pins: A2:0, A1:0, A0:0
#define I2C_IO_EXP_1_R          (uint8_t)0x41  // Read
#define I2C_IO_EXP_1_W          (uint8_t)0x40  // Write

// I2C IO Expander PCF8574 #2 address pins: A2:0, A1:0, A0:1
#define I2C_IO_EXP_2_R          (uint8_t)0x43  // Read
#define I2C_IO_EXP_2_W          (uint8_t)0x42  // Write

// Symbolic names
// I2C LCD connected to IO Expander #1
#define I2C_LCD_R               I2C_IO_EXP_1_R
#define I2C_LCD_W               I2C_IO_EXP_1_W

// General Purpose I/O connected to IO Expander #2
#define I2C_GPIO_R              I2C_IO_EXP_2_R
#define I2C_GPIO_W              I2C_IO_EXP_2_W

// Microchip: 24FC512 I2C 64K x 8bit EEPROM
// Address pins: A2:0, A1:0, A0:0
#define I2C_EEPROM_R            (uint8_t)0xA1 // Read
#define I2C_EEPROM_W            (uint8_t)0xA0 // Write

// See w/ DIRECTION_PIN
#define DIRECTION_FWD           1
#define DIRECTION_REV           0

/************************************************************************/
/*                         PINS DEFINITIONS                             */
/************************************************************************/
#define LED_PIN     PIN_A5      // On-board test LED
#define BUTTON_PIN  PIN_B0      // Push button : normally open = HIGH
                                // pressed = low

// Serial 1 (S1) : Hardware UART
#define S1_RX       PIN_C7      // PIC <-- DB-9
#define S1_TX       PIN_C6      // PIC --> DB-9

// I2C Port
#define I2C_SCL     PIN_C3      // Serial Clock
#define I2C_SDA     PIN_C4      // Serial Data

// Motor Control (for backward compatibility with the same dev board)
#define PWM_PIN         PIN_C2  // CCP1 PWM Output
#define DIRECTION_PIN   PIN_C0  // H-Bridge direction input

// Race light sensor pins (active high)
#define RACE_START_PIN  PIN_B1  // Car crossed the start line beam
#define RACE_STOP_PIN   PIN_B2  // Car crossed the finish line beam

/************************************************************************/
/*                          PORTS DEFINITIONS                           */
/************************************************************************/

// Serial Port #1 using RS-232 to computer @ 9600 baud
#define SERIAL1_PORT    #use rs232(baud=9600, xmit=S1_TX, rcv=S1_RX, bits=8, parity=N)

// I2C port using hardware I2C.
#define I2C_PORT        #use i2c(MASTER, SCL=I2C_SCL, SDA=I2C_SDA, FAST)

/************************************************************************/
/*                          FUNCTION PROTOTYPES                         */
/************************************************************************/

void master_init(void);

void toggle_led(void);
void flash_led(uint8_t x);

uint8_t handle_boot_nb(void);

// Serial port 1
void put_serial1(uint8_t c);
uint8_t get_serial1(void);

void i2c_leds(uint8_t leds_on);

/************************************************************************/
/*                            GLOBAL VARIABLES                          */
/************************************************************************/
static bool g_race_started = false;

static uint8_t g_seconds = 0;
static uint8_t g_hundreth = 0;

/************************************************************************/
/*                              ISR                                     */
/************************************************************************/

// Interrupt when received data is available
#INT_RDA
void rs232_rx_isr(void)
{
}

// Interrupt when ADC conversion is complete
#INT_AD
void adc_done_isr(void)
{
}

// Interrupt when Timer0 overflows (also referd as RTCC)
#INT_RTCC
void timer0_isr(void)
{
}

#INT_TIMER1
void timer1_isr(void)
{
  toggle_led();
}

#INT_TIMER2
void timer2_isr(void)
{
  toggle_led();
}

#INT_CCP1
void ccp1_isr(void)
{
  //toggle_led();
  if (g_race_started)
  {
    g_hundreth++;
    if (g_hundreth == 100)
    {
      g_hundreth = 0;
      g_seconds++;
    }
  }
}

#INT_CCP2
void ccp2_isr(void)
{
}


/************************************************************************/
/*                              MAIN                                    */
/************************************************************************/
void main(void)
{
  uint8_t leds = 0;
  uint8_t i = 0;
  uint8_t c = 0;

  master_init();

  flash_led(5);   // reality check at boot

  // 0         1
  // 0123456789012345
  // Race Chrono
  // 20-FEB-15  B:123

  i2c_leds(0b00000000);

  i2c_lcd_init();
  printf(i2c_lcd_write_data, "Race Chrono");
  i2c_lcd_position_cursor(1, 0);
  i2c_lcd_write_data(__date__);
  printf(i2c_lcd_write_data, "  B:%u", handle_boot_nb());
  delay_ms(DELAY_MESSAGE);

  // Special debug mode at startup
  if (!input(BUTTON_PIN))
  {
    i2c_lcd_clear();
    i2c_lcd_position_cursor(0, 0);
    printf(i2c_lcd_write_data, "Sensor Pins:");
    i2c_lcd_position_cursor(1, 0);
    printf(i2c_lcd_write_data, "Beam=0, NoBeam=1");
    delay_ms(DELAY_MESSAGE);

    i2c_lcd_clear();
    i2c_lcd_position_cursor(0, 0);
    printf(i2c_lcd_write_data, "Start Pin:");
    i2c_lcd_position_cursor(1, 0);
    printf(i2c_lcd_write_data, "Stop Pin :");

    while(1) // reset to get out of debug mode...
    {
      i2c_lcd_position_cursor(0, 10);
      printf(i2c_lcd_write_data, "%u", input(RACE_START_PIN));
      i2c_lcd_position_cursor(1, 10);
      printf(i2c_lcd_write_data, "%u", input(RACE_STOP_PIN));
      //delay_ms(1);
    }
  }

  while(1)
  {
    g_seconds = 0;
    g_hundreth = 0;

    i2c_lcd_clear();

    // Sanity check before starting the race
    if (input(RACE_START_PIN) || input(RACE_STOP_PIN))
    {
      printf(i2c_lcd_write_data, "ERROR:");
      i2c_lcd_position_cursor(1, 0);
      printf(i2c_lcd_write_data, "Sensor Inputs");
      while(1) // Reset to get out
      {
      }
    }

    printf(i2c_lcd_write_data, "Ready...");
    // Wait for the car to cross the start beam
    while(!input(RACE_START_PIN))
    {
    }

    g_race_started = true;
    output_high(LED_PIN);
    i2c_lcd_clear();
    printf(i2c_lcd_write_data, "Race!");

    // Wait for the car to cross the stop beam
    while(!input(RACE_STOP_PIN))
    {
      i2c_lcd_position_cursor(1, 0);
      printf(i2c_lcd_write_data, "%u.%02u", g_seconds, g_hundreth);
    }

    g_race_started = false;
    output_low(LED_PIN);
    delay_ms(20); // to make sure the LCD displays the last time
    i2c_lcd_position_cursor(0, 0);
    printf(i2c_lcd_write_data, "Finish!");
    i2c_lcd_position_cursor(1, 0);
    printf(i2c_lcd_write_data, "%u.%02u", g_seconds, g_hundreth);

    while(input(BUTTON_PIN)) // wait for button press to start new race
    {
    }
  }

  i2c_lcd_clear();
  output_low(LED_PIN);
  i2c_leds(0b00000000);

  sleep();
} // main()

/************************************************************************/
/*                          FUNCTION DEFINITIONS                        */
/************************************************************************/

/**
 * Initialization of the PIC
 */
void master_init(void)
{
  // backward compatibility
  output_low(PWM_PIN);
  output_bit(DIRECTION_PIN, DIRECTION_FWD);

  // ----------------------------------------------------------
  // Timer1 Setup
  set_timer1(0);
  // Ftimer = Fc / 4 / Divider
  setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);

  // ----------------------------------------------------------
  // CCP1 Setup (configured as comparator with Timer1)
  // Fccp = Ftimer / (CCP + 1)
  // Fccp = 100 Hz (10 ms)
  CCP_1 = 6249;
  // Compare mode + special event: reset timer
  setup_ccp1(CCP_COMPARE_RESET_TIMER);
  enable_interrupts(INT_CCP1);

  enable_interrupts(GLOBAL);
}

/************************************************************************
* Description : Read EEPROM, increment value, write to EEPROM.
*
* Parameters :  void
* Return :      bood_nb, the value of the boot number currently stored
*               in eeprom.
************************************************************************/
uint8_t handle_boot_nb(void)
{
    uint8_t boot_nb;
    boot_nb = i2c_read_eeprom(BOOT_NB_ADDRESS);

    i2c_write_eeprom(BOOT_NB_ADDRESS, boot_nb + 1);
    return boot_nb;
}

/************************************************************************
* Description : Toggle the LED.
* Parameters :  void
* Return :      void
************************************************************************/
void toggle_led(void)
{
  static bool led_toggle = 1;
  output_bit(LED_PIN, led_toggle);
  led_toggle = !led_toggle;
}

/************************************************************************
* Description : Flash LED x number of times.
* Parameters :  x, the number of times the led will flash
* Return :      void
************************************************************************/
void flash_led(uint8_t x)
{
  uint8_t i;
  for (i = 0; i < x; ++i)
  {
    output_high(LED_PIN);
    delay_ms(LED_PAUSE);
    output_low(LED_PIN);
    delay_ms(LED_PAUSE);
  }
}

/* -------------------- I2C SERIAL PORT -------------------- */

// Definition of the I2C port settings.
I2C_PORT

#include "eeprom.c"
#include "i2c_lcd.c"

/************************************************************************
* Description : Turns ON or OFF the blue LED's on 1st PCF8574
* Parameters :  leds_on
* Return :      void
************************************************************************/
void i2c_leds(uint8_t leds_on)
{
  i2c_start();
  i2c_write(I2C_GPIO_W);
  i2c_write(~leds_on); // All pins high, low to ON a led.
  i2c_stop();
}

/* -------------------- SERIAL PORT 1 -------------------- */
// Functions using serial port 1 connected to the computer

#if 0

// define serial port 1 settings
SERIAL1_PORT

bool kbhit_serial1(void)
{
    return kbhit();
}

// Put a single char to serial port 1
void put_serial1(uint8_t c)
{
    putchar(c);
}

// Get a single char from serial port 1
uint8_t get_serial1(void)
{
    return getchar();
}

#endif // 1

