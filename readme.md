# PIC16F1455 Projects

## Overview

The PIC16F1455 is a powerful 14-pin enhanced mid-range 8-bit microcontroller with advanced features for embedded applications.

## Key Features

- **Enhanced Mid-Range Core**: 49 instructions with enhanced capabilities
- **Memory**: 8K words Flash program memory, 1K bytes SRAM
- **Operating Frequency**: Up to 48MHz with internal oscillator and 4x PLL
- **USB Capability**: Full-speed USB 2.0 device interface
- **Pins**: 14-pin PDIP package with 11 I/O pins
- **Peripherals**: Timers, PWM, ADC, UART, I2C, SPI
- **Operating Voltage**: 2.3V to 5.5V

## Pin Configuration (14-pin PDIP)

```
     PIC16F1455
     +----u----+
VDD -|1      14|- VSS
RA5 -|2      13|- RA0
RA4 -|3      12|- RA1  
RA3 -|4      11|- VUSB
RC5 -|5      10|- RC0
RC4 -|6       9|- RC1
RC3 -|7       8|- RC2
     +---------+
```

## Projects

### 1. NES Controller Interface (Assembly)
**File**: `nes_with_shift_register.s`

Assembly implementation of NES controller reader with shift register LED output.

#### Features:
- Reads 8-button NES controller via 4021 shift register interface
- Controls 8 LEDs using CD4094BE shift register
- Real-time button state mirroring
- Precise timing with 1MHz instruction rate

#### Pin Connections:
| Pin | Function | Connection |
|-----|----------|------------|
| RC0 | Data Out | CD4094BE Data |
| RC1 | Clock Out | CD4094BE Clock |
| RC2 | Strobe | CD4094BE Strobe |
| RC3 | NES Latch | NES Controller Latch |
| RC4 | NES Clock | NES Controller Clock |
| RC5 | NES Data | NES Controller Data |

#### NES Controller Wiring:
```
NES Controller:
Pin 1 (GND)   -> VSS
Pin 2 (CLK)   -> RC4
Pin 3 (LATCH) -> RC3
Pin 4 (DATA)  -> RC5
Pin 5 (+5V)   -> VDD
```

#### CD4094BE Shift Register:
```
CD4094BE:
Pin 1  (STROBE) -> RC2
Pin 2  (DATA)   -> RC0
Pin 3  (CLOCK)  -> RC1
Pin 8  (VSS)    -> Ground
Pin 15 (OE)     -> +5V (Output Enable)
Pin 16 (VDD)    -> +5V
Pins 4-7, 11-14 -> LEDs with resistors
```

#### Button Mapping:
- **Bit 0**: A Button
- **Bit 1**: B Button
- **Bit 2**: Select
- **Bit 3**: Start
- **Bit 4**: Up
- **Bit 5**: Down
- **Bit 6**: Left
- **Bit 7**: Right

### 2. NES Controller Interface (C)
**File**: `nes_with_shift_register.c`

C implementation of the same NES controller functionality.

## Development Environment

- **IDE**: MPLAB X
- **Compiler**: XC8 v3.10+
- **Assembler**: pic-as v3.10+
- **Device Family Pack**: PIC12-16F1xxx_DFP v1.8.254+

## Getting Started

### Prerequisites
1. MPLAB X IDE
2. XC8 Compiler
3. PIC16F1455 microcontroller
4. PICkit 4/5 programmer
5. Required external components (NES controller, CD4094BE, LEDs, resistors)

### Hardware Setup
1. Connect PICkit programmer to ICSP pins
2. Wire external components per project schematics
3. Apply 5V power
4. Program and test

### Power Requirements
- Operating voltage: 5V (for NES controller compatibility)
- Current consumption: <50mA typical

## Resources

- [PIC16F1455 Datasheet](https://www.microchip.com/en-us/product/PIC16F1455)
- [NES Controller Protocol](https://www.nesdev.org/wiki/Controller_reading_code)
- [CD4094BE Datasheet](https://www.ti.com/product/CD4094B)
- [MPLAB X IDE](https://www.microchip.com/mplab/mplab-x-ide)

![nes_controller_project](https://github.com/user-attachments/assets/c4c33a18-e18a-452d-b1eb-1e87a6a5ad3c)
