;====================================================================
; File:   nes_with_shift_register.s
; Author: cstep
;
; PIC16F1455 and NES Controller with External Shift Register
;
; Description:
;   Uses the internal NES controller shift register (4021) for input
;   and external CD4094BE shift register for output expansion.
;   This allows reading 8 buttons and controlling 8+ LEDs/outputs.
;
; Device: PIC16F1455 (14-pin PDIP)
; Assembler: pic-as (v3.10+)
;
; Pin Configuration:
;   Pin 1 (VDD)   - +5V Power
;   Pin 5 (RC5)   - NES Controller Data (Input)
;   Pin 6 (RC4)   - NES Controller Clock
;   Pin 7 (RC3)   - NES Controller Latch
;   Pin 8 (RC2)   - Strobe (Latch) to CD4094BE
;   Pin 9 (RC1)   - Clock to CD4094BE
;   Pin 10 (RC0)  - Data Output to CD4094BE
;   Pin 14 (VSS)  - Ground
;
; External Components:
;   - NES Controller (4021 shift register inside)
;   - CD4094BE 8-bit shift register for outputs
;   - 8 LEDs with current limiting resistors
;
; Connections:
;   Note: The pin numbers are the same for all NES controllers.
;         Your colors may be different. Pin them out to be sure.
;   NES Controller:
;     Pin 1 (GND)   -> VSS -> blue
;     Pin 2 (CLK)   -> RC4 -> white
;     Pin 3 (LATCH) -> RC3 -> black
;     Pin 4 (DATA)  -> RC5 -> green
;     Pin 5 (+5V)   -> VDD -> red
;
;   CD4094BE Output Shift Register:    
;     Pin 1  (STROBE) -> RC2
;     Pin 2  (DATA)   -> RC0
;     Pin 3  (CLOCK)  -> RC1 
;     Pin 8  (VSS)    -> VSS (ground)
;     Pin 15 (OE)     -> VDD (output enable - active HIGH)
;     Pin 16 (VDD)    -> VDD (+5V power)
;     Outputs Q1-Q8 (pins 4-7, 11-14) -> LEDs with resistors
;====================================================================
    
    processor 16F1455
    #include <xc.inc>

;====================================================================
;  CONFIGURATION BITS
;====================================================================
; CONFIG1
    config FOSC = INTOSC    ; Internal oscillator
    config WDTE = OFF       ; Watchdog timer disabled
    config PWRTE = OFF      ; Power-up timer disabled  
    config MCLRE = OFF      ; MCLR pin is digital input
    config CP = OFF         ; Code protection disabled
    config BOREN = ON       ; Brown-out reset enabled
    config CLKOUTEN = OFF   ; CLKOUT function disabled
    config IESO = ON        ; Internal/External switchover enabled
    config FCMEN = ON       ; Fail-safe clock monitor enabled

; CONFIG2
    config WRT = OFF        ; Write protection off
    config CPUDIV = CLKDIV6 ; CPU clock divided by 6 (32MHz/6 ≈ 5.33MHz)
    config USBLSCLK = 48MHz ; USB clock 48MHz
    config PLLMULT = 3x     ; PLL 3x multiplier
    config PLLEN = ENABLED  ; PLL enabled
    config STVREN = ON      ; Stack overflow reset enabled
    config BORV = LO        ; Brown-out reset voltage low
    config LPBOR = OFF      ; Low-power brown-out reset disabled
    config LVP = ON         ; Low-voltage programming enabled

;====================================================================
;  CONSTANTS AND PIN DEFINITIONS
;====================================================================
; STATUS Register Bit Definitions (if not already defined)
#ifndef C
C       EQU 0               ; Carry bit
#endif

; CD4094BE Shift Register Pins
SHIFT_DATA_PIN   EQU 0      ; RC0 - Data to shift register
SHIFT_CLOCK_PIN  EQU 1      ; RC1 - Clock to shift register  
SHIFT_STROBE_PIN EQU 2      ; RC2 - Strobe (latch) to shift register

; NES Controller Pins
NES_DATA_PIN     EQU 5      ; RC5 - Data from NES controller
NES_CLOCK_PIN    EQU 4      ; RC4 - Clock to NES controller
NES_LATCH_PIN    EQU 3      ; RC3 - Latch to NES controller

;====================================================================
;  VARIABLE DEFINITIONS  
;====================================================================
    psect udata_shr
nes_data:       ds 1        ; NES controller button data
shift_data:     ds 1        ; Data to send to shift register
bit_counter:    ds 1        ; Loop counter for bit operations
delay_counter1: ds 1        ; Delay counter 1
delay_counter2: ds 1        ; Delay counter 2
temp:           ds 1        ; Temporary storage

;====================================================================
;  RESET VECTOR
;====================================================================
    psect resetVec, class=CODE, delta=2
resetVec:
    goto    main

;====================================================================
;  MAIN PROGRAM
;====================================================================
    psect code
main:
    ; Initialize oscillator: 8MHz × 3x PLL = 24MHz HF
    banksel OSCCON
    movlw   0b01111000  ; Set HFINTOSC at 8MHz
    movwf   OSCCON
    
    ; Wait for oscillator to stabilize
    btfss   OSCSTAT, 4     ; HFIOFS - High-Frequency Internal Oscillator Stable bit
    goto    $-1
    
    ; Configure GPIO pins
    call    init_gpio
    
    ; Main loop
main_loop:
    ; Read NES controller
    call    nes_latch_data
    call    nes_read_buttons
    
    ; Invert data (pressed=0, released=1 → pressed=1, released=0 for LEDs)
    comf    nes_data, w     ; Complement (invert) nes_data → W
    movwf   shift_data      ; Store inverted data
    
    ; Send data to shift register  
    call    shift_send_byte
    call    shift_latch_outputs
    
    ; Small delay before next read
    call    delay_1ms
    
    goto    main_loop

;====================================================================
;  GPIO INITIALIZATION
;====================================================================
init_gpio:
    ; Set data direction (0=output, 1=input)
    banksel TRISC
    movlw   0b00100000      ; RC5 input, RC0-RC4 outputs
    movwf   TRISC
    
    ; Disable analog inputs (set to digital)
    banksel ANSELC  
    clrf    ANSELC          ; All PORTC pins digital
    
    ; Initialize pin states
    banksel PORTC
    clrf    PORTC           ; All outputs low initially
    
    retlw   0

;====================================================================
;  NES CONTROLLER FUNCTIONS
;====================================================================

; Latch NES controller data (pulse latch pin)
nes_latch_data:
    banksel PORTC
    bsf     PORTC, NES_LATCH_PIN   ; Latch high
    call    delay_12us             ; Hold for 12µs
    bcf     PORTC, NES_LATCH_PIN   ; Latch low
    retlw   0

; Read 8 bits from NES controller
nes_read_buttons:
    banksel PORTC
    clrf    nes_data        ; Clear result
    movlw   8               ; Read 8 bits
    movwf   bit_counter
    
nes_read_loop:
    ; Read data bit into carry
    btfsc   PORTC, NES_DATA_PIN
    goto    nes_data_high
    ; Data is low (button pressed)
    bcf     STATUS, C       ; Clear carry
    goto    nes_shift_bit
nes_data_high:
    ; Data is high (button not pressed)  
    bsf     STATUS, C       ; Set carry
    
nes_shift_bit:
    ; Shift carry into nes_data (LSB first)
    rrf     nes_data, f
    
    ; Pulse clock
    bsf     PORTC, NES_CLOCK_PIN   ; Clock high
    call    delay_1us              ; Short delay
    bcf     PORTC, NES_CLOCK_PIN   ; Clock low
    
    ; Next bit
    decfsz  bit_counter, f
    goto    nes_read_loop
    
    retlw   0

;====================================================================
;  SHIFT REGISTER FUNCTIONS  
;====================================================================

; Send byte to CD4094BE shift register (MSB first)
shift_send_byte:
    banksel PORTC
    movlw   8               ; Send 8 bits
    movwf   bit_counter
    
shift_send_loop:
    ; Test MSB of shift_data
    btfss   shift_data, 7
    goto    shift_data_low
    ; MSB is 1
    bsf     PORTC, SHIFT_DATA_PIN
    goto    shift_clock_pulse
shift_data_low:
    ; MSB is 0
    bcf     PORTC, SHIFT_DATA_PIN
    
shift_clock_pulse:
    ; Pulse clock to shift bit
    bsf     PORTC, SHIFT_CLOCK_PIN  ; Clock high
    call    delay_1us               ; Short delay
    bcf     PORTC, SHIFT_CLOCK_PIN  ; Clock low
    
    ; Shift data left for next bit
    rlf     shift_data, f
    
    ; Next bit
    decfsz  bit_counter, f
    goto    shift_send_loop
    
    retlw   0

; Latch shift register outputs (pulse strobe)
shift_latch_outputs:
    banksel PORTC
    bsf     PORTC, SHIFT_STROBE_PIN ; Strobe high
    call    delay_1us               ; Hold for 1µs
    bcf     PORTC, SHIFT_STROBE_PIN ; Strobe low
    retlw   0

;====================================================================
;  DELAY FUNCTIONS
;====================================================================

; Clock: 8MHz×3(PLL)÷6(CPUDIV)÷4 = 1MHz instruction rate
; 1 microsecond delay (approximate at 1MHz instruction rate)
delay_1us:
    retlw   0       ; 2 cycles = 2µs at 1MHz instruction rate
                    ; Close enough for timing purposes

; 12 microsecond delay for NES latch
delay_12us:
    movlw   12
    movwf   delay_counter1
delay_12us_loop:
    call    delay_1us
    decfsz  delay_counter1, f
    goto    delay_12us_loop
    retlw   0

; 1 millisecond delay
delay_1ms:
    movlw   100
    movwf   delay_counter1
delay_1ms_outer:
    movlw   10
    movwf   delay_counter2
delay_1ms_inner:
    call    delay_1us
    decfsz  delay_counter2, f
    goto    delay_1ms_inner
    decfsz  delay_counter1, f
    goto    delay_1ms_outer
    retlw   0

;====================================================================
;  END OF PROGRAM
;====================================================================
    end     resetVec
