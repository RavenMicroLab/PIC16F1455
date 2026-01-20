/*
 * File:   nes_with_shift_registers.c
 * Author: cstep
 *
 * PIC16F1455 NES Controller with External Shift Registers
 *
 * Description:
 *   Uses the internal NES controller shift register (4021) for input
 *   and external CD4094BE shift register for output expansion.
 *   This allows reading 8 buttons and controlling 8+ LEDs/outputs
 *   using only 3 pins on the PIC16F1455.
 *
 * Device: PIC16F1455 (14-pin PDIP)
 * Compiler: XC8
 *
 * Pin Configuration:
 *   Pin 1 (VDD)   - +5V Power
 *   Pin 2 (RA5)   - Not Connected 
 *   Pin 3 (RA4)   - Not Connected 
 *   Pin 4 (RA3)   - Not Connected 
 *   Pin 5 (RC5)   - NES Controller Data (Input)
 *   Pin 6 (RC4)   - NES Controller Clock
 *   Pin 7 (RC3)   - NES Controller Latch
 *   Pin 8 (RC2)   - Strobe (Latch) to CD4094B
 *   Pin 9 (RC1)   - Clock to CD4094BE
 *   Pin 10 (RC0)  - Data Output to CD4094B
 *   Pin 11 (VUSB) - Not Connected 
 *   Pin 12 (RA1)  - Not Connected 
 *   Pin 13 (RA0)  - Not Connected 
 *   Pin 14 (VSS)  - Ground
 *
 * External Components:
 *   - NES Controller (4021 shift register inside)
 *   - CD4094BE 8-bit shift register for outputs
 *   - 8 LEDs with current limiting resistors
 *
 * Connections:
 *   Note: The pin numbers are the same for all NES controllers.
 *         Your colors may be different. Pin them out to be sure.
 *   NES Controller:
 *     Pin 1 (GND)   -> VSS -> blue
 *     Pin 2 (CLK)   -> RC4 -> white
 *     Pin 3 (LATCH) -> RC3 -> black
 *     Pin 4 (DATA)  -> RC5 -> green
 *     Pin 5 (+5V)   -> VDD -> red
 *
 *   CD4094BE Output Shift Register:    
 *     Pin 1  (STROBE) -> RC2
 *     Pin 2  (DATA)   -> RC0
 *     Pin 3  (CLOCK)  -> RC1 
 *     Pin 8  (VSS)    -> VSS (ground)
 *     Pin 9  (Qs)     -> Clock to Serial Output (not used)
 *     Pin 10 (Q`s)    -> Inverted Clock to Serial Output (not used)
 *     Pin 16 (VDD)    -> VDD (+5V power)
 *     Pin 15 (OE)     -> VDD (output enable - active HIGH)
 *     Outputs Q1-Q8 (pins 4-7, 11-14) -> LEDs with resistors
 */

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = CLKDIV6 // CPU System Clock Selection Bit (CPU system clock divided by 6)
#pragma config USBLSCLK = 48MHz // USB Low Speed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 3x     // PLL Multiplier Selection Bit (3x Output Frequency Selected)
#pragma config PLLEN = ENABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


#include <xc.h>

// Define functions
void shift_send (unsigned char b);
void shift_load (void);
unsigned char nes_read (void);
void nes_load (void);

#define _XTAL_FREQ 32000000UL // 32 MHz internal oscillator with PLL

#define SHIFT_DATA    RC0     // CD4094 DATA
#define SHIFT_CLOCK   RC1     // CD4094 CLOCK
#define SHIFT_STROBE  RC2     // CD4094 STROBE (Latch)

#define NES_DATA      RC5     // NES CONTROLLER DATA
#define NES_CLOCK     RC4     // NES CONTROLLER CLOCK
#define NES_LATCH     RC3     // NES CONTROLLER LATCH

// Input value from NES controller
unsigned char v1 = 0;

void main(void) {

    // Setup ports RC0:RC4 as outputs
    TRISC0 = 0;
    TRISC1 = 0; 
    TRISC2 = 0; 
    TRISC3 = 0;
    TRISC4 = 0;

    TRISC5 = 1; // NES DATA as input

    // Set ANSEL bits to digital I/O - disable analog functionality
    ANSC0 = 0; 
    ANSC1 = 0;
    ANSC2 = 0;
    ANSC3 = 0;

    while (1) {
        // Read NES controller
        nes_load();
        v1 = nes_read();

        // Send value to CD4094BE shift register
        // All LEDs OFF to start - Button press will light corresponding LED
        shift_send(~v1); // Invert bits: pressed = 0, released = 1
        shift_load();
    }
    return;
}

void shift_send (unsigned char b) {
    for (unsigned char i = 0; i < 8; i++) {
        SHIFT_DATA = (b >> (7 - i)) & 0x01; // Set data bit (MSB first)
        // Pulse clock
        SHIFT_CLOCK = 1;
        __delay_us(1);
        SHIFT_CLOCK = 0;
    }
}

unsigned char nes_read (void){
    unsigned char byte = 0;
    for (unsigned char i = 0; i < 8; i++) {
        // Read data bit
        byte |= (NES_DATA << i);
        // Pulse clock
        NES_CLOCK = 1;
        __delay_us(1);
        NES_CLOCK = 0;
    }
    return byte;
}

void shift_load (void) {
    // Pulse strobe to latch output on CD4094BE
    // The datasheet shows a minimum pulse width of 200 ns at 5 volts
    // Using 1 microsecond for safety (smallest unit without creating custom NOPs)
    SHIFT_STROBE = 1;
    __delay_us(1);
    SHIFT_STROBE = 0;
} 

void nes_load (void) {
    // Pulse latch to load NES controller data 
    // This is probably much longer than required,
    // but it attempts to ensure compatibility with all NES controller clones
    NES_LATCH = 1;
    __delay_us(12); // Hold latch high for at least 12 microseconds
    NES_LATCH = 0;
}
