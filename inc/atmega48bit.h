/******************************************************************************
*                                                                             *
*   file name      :    atmega48bit.h                                         *
*   file type      :    Header                                                *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    19.08.2008 / A.Weick                                  *
*                                                                             *
*   version        :    REV 0.01                                              *
*                                                                             *
*   Copyright (c) 2008 Andreas Weick                                          *
*                      Alle Rechte vorbehalten.                               *
*                     - Confidential -                                        *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    BIT's Makrodefinitionen für ATtiny48                  *
*                                                                             *
*   specials       :    <description> (more lines posiple)                    *
*                                                                             *
******************************************************************************/

#ifndef __ATMEGA48BIT_H                 
#define __ATMEGA48BIT_H              

/*============================== include files ==============================*/
/* nothing */

/*================================== Makros =================================*/


//--------------------  Gerneral Bit definitions
#define  BIT7   (7)
#define  BIT6   (6)
#define  BIT5   (5)
#define  BIT4   (4)
#define  BIT3   (3)
#define  BIT2   (2) 
#define  BIT1   (1)
#define  BIT0   (0)
//---------------------


// Hinweis Adresse in (...) ist die IO-Adresse  für die Dataspace Adresse muss noch 0x20 dazu addiert werden
// Beispiel ADCL  I/0 Adresse = 0x04,  Dataspace Adresse = 0x24



/* 0x00 - 0x02  reserved */



//########################################################
//#  PINB (0x03)          Input Pins, Port B 

#define PINB7  (7)      // Data Bit
#define PINB6  (6)      // Data Bit
#define PINB5  (5)      // Data Bit
#define PINB4  (4)      // Data Bit
#define PINB3  (3)      // Data Bit
#define PINB2  (2)      // Data Bit
#define PINB1  (1)      // Data Bit
#define PINB0  (0)      // Data Bit


//########################################################
//# DDRB (0x04)          Data Direction Register, Port B 

#define DDB7  (7)       // Data Bit
#define DDB6  (6)       // Data Bit
#define DDB5  (5)       // Data Bit
#define DDB4  (4)       // Data Bit
#define DDB3  (3)       // Data Bit
#define DDB2  (2)       // Data Bit
#define DDB1  (1)       // Data Bit
#define DDB0  (0)       // Data Bit


//########################################################
//# PORTB (0x05)         Data Register, Port B 

#define PB7  (7)       // Data Bit
#define PB6  (6)       // Data Bit
#define PB5  (5)       // Data Bit
#define PB4  (4)       // Data Bit
#define PB3  (3)       // Data Bit
#define PB2  (2)       // Data Bit
#define PB1  (1)       // Data Bit
#define PB0  (0)       // Data Bit
//########################################################




//########################################################
//#  PINC (0x06)          Input Pins, Port C 

#define PINC7  (7)      // Data Bit
#define PINC6  (6)      // Data Bit
#define PINC5  (5)      // Data Bit
#define PINC4  (4)      // Data Bit
#define PINC3  (3)      // Data Bit
#define PINC2  (2)      // Data Bit
#define PINC1  (1)      // Data Bit
#define PINC0  (0)      // Data Bit


//########################################################
//# DDRC (0x07)          Data Direction Register, Port C 

#define DDC7  (7)       // Data Bit
#define DDC6  (6)       // Data Bit
#define DDC5  (5)       // Data Bit
#define DDC4  (4)       // Data Bit
#define DDC3  (3)       // Data Bit
#define DDC2  (2)       // Data Bit
#define DDC1  (1)       // Data Bit
#define DDC0  (0)       // Data Bit


//########################################################
//# PORTC(0x08)         Data Register, Port C 

#define PC7  (7)       // Data Bit
#define PC6  (6)       // Data Bit
#define PC5  (5)       // Data Bit
#define PC4  (4)       // Data Bit
#define PC3  (3)       // Data Bit
#define PC2  (2)       // Data Bit
#define PC1  (1)       // Data Bit
#define PC0  (0)       // Data Bit
//########################################################




//########################################################
//#  PIND (0x09)          Input Pins, Port D 

#define PIND7  (7)      // Data Bit
#define PIND6  (6)      // Data Bit
#define PIND5  (5)      // Data Bit
#define PIND4  (4)      // Data Bit
#define PIND3  (3)      // Data Bit
#define PIND2  (2)      // Data Bit
#define PIND1  (1)      // Data Bit
#define PIND0  (0)      // Data Bit


//########################################################
//# DDRD (0x0A)          Data Direction Register, Port D

#define DDD7  (7)       // Data Bit
#define DDD6  (6)       // Data Bit
#define DDD5  (5)       // Data Bit
#define DDD4  (4)       // Data Bit
#define DDD3  (3)       // Data Bit
#define DDD2  (2)       // Data Bit
#define DDD1  (1)       // Data Bit
#define DDD0  (0)       // Data Bit


//########################################################
//# PORTD(0x0B)         Data Register, Port D 

#define PD7  (7)       // Data Bit
#define PD6  (6)       // Data Bit
#define PD5  (5)       // Data Bit
#define PD4  (4)       // Data Bit
#define PD3  (3)       // Data Bit
#define PD2  (2)       // Data Bit
#define PD1  (1)       // Data Bit
#define PD0  (0)       // Data Bit
//########################################################


//########################## 
//# 0x0C - 0x14  reserved  #
//##########################



//############################################             
//# TIFR0  (0x15)   TIFR0 – Timer/Counter 0 Interrupt Flag Register 

/* bits 7-3 reserved , read as zero*/

#define OCF0B    (2)     // Timer/Counter 0 Output Compare B Match Flag
#define OCF0A    (1)     // Timer/Counter 0 Output Compare A Match Flag
#define TOV0     (0)     // Timer/Counter0 Overflow Flag
        

//############################################             
//# TIFR1  (0x16)   TIFR1 – Timer/Counter1 Interrupt Flag Register 

/* bits 7-6 reserved , read as zero*/

#define ICF1     (5)     // Timer/Counter1, Input Capture Flag
/* bits 4-3 reserved , read as zero*/
#define OCF1B    (2)     // Timer/Counter1, Output Compare B Match Flag
#define OCF1A    (1)     // Timer/Counter1, Output Compare A Match Flag
#define TOV1     (0)     // Timer/Counter1, Overflow Flag


//############################################             
//# TIFR2  (0x17)   TIFR2 – Timer/Counter2 Interrupt Flag Register 

/* bits 7-3 reserved , read as zero*/
#define OCF2B    (2)     // Output Compare Flag 2 B
#define OCF2A    (1)     // Output Compare Flag 2 A
#define TOV2     (0)     // Timer/Counter2 Overflow Flag



//########################## 
//# 0x18 - 0x1A  reserved  #
//##########################



//############################################             
//# PCIFR  (0x1B)   PCIFR – Pin Change Interrupt Flag Register 

/* bits 7-3 reserved , read as zero*/
#define PCIF2    (2)     // Pin Change Interrupt Flag 2
#define PCIF1    (1)     // Pin Change Interrupt Flag 1
#define PCIF0    (0)     // Pin Change Interrupt Flag 0



//############################################             
//# EIFR  (0x1C)   EIFR – External Interrupt Flag Register 

/* bits 7-2 reserved , read as zero*/
#define INTF1    (1)     // External Interrupt Flag 1
#define INTF0    (0)     // External Interrupt Flag 1



//############################################             
//# EIMSK  (0x1D)   EIMSK – External Interrupt Mask Register 

/* bits 7-2 reserved , read as zero*/
#define INT1    (1)     // External Interrupt Request 1 Enable
#define INT0    (0)     // External Interrupt Request 0 Enable



//############################################             
//# GPIOR0  (0x1E)   GPIOR0 – General Purpose I/O Register 0 
//# Bit gelten auch für GPIOR1 und GPIOR2

#define GPIO7    (7)     // Data Bit
#define GPIO6    (6)     // Data Bit
#define GPIO5    (5)     // Data Bit
#define GPIO4    (4)     // Data Bit
#define GPIO3    (3)     // Data Bit
#define GPIO2    (2)     // Data Bit
#define GPIO1    (1)     // Data Bit
#define GPIO0    (0)     // Data Bit


//############################################
//#  EECR (0x1F)    EEPROM Control Register  # 

/*             (7)
                ..
               (6)    resserved  will read as zero
*/
#define EEPM1   (5)     // EEPROM Programming Mode Bit 1
#define EEPM0   (4)     // EEPROM Programming Mode Bit 0
#define EERIE   (3)     // EEPROM Ready Interrupt Enable
#define EEMPE   (2)     // EEPROM Master Write Enable
#define EEPE    (1)     // EEPROM Write Enable
#define EERE    (0)     // EEPROM Read Enable


//############################################
//# EEDR (0x20)      EEPROM Data Register (8-Bit) 

#define EEDR7   (7)     // Data Bit
#define EEDR6   (6)     // Data Bit
#define EEDR5   (5)     // Data Bit
#define EEDR4   (4)     // Data Bit
#define EEDR3   (3)     // Data Bit
#define EEDR2   (2)     // Data Bit
#define EEDR1   (1)     // Data Bit
#define EEDR0   (0)     // Data Bit






//#########################################################
//# EEARL  (0x21)    The EEPROM Address Register Low Byte #

#define EEAR7   (7)     // EEPROM Address Bits (low Byte)
#define EEAR6   (6)     // '
#define EEAR5   (5)     // '
#define EEAR4   (4)     // '
#define EEAR3   (3)     // '
#define EEAR2   (2)     // '
#define EEAR1   (1)     // '
#define EEAR0   (0)     // '

//#########################################################
//# EEARH  (0x22)    The EEPROM Address Register Low Byte #

/* reserved read as zero
#define EEAR15   (7)     // EEPROM Address Bits (low Byte)
#define EEAR14   (6)     // '
#define EEAR13   (5)     // '
#define EEAR12   (4)     // '
#define EEAR11   (3)     // '
#define EEAR10   (2)     // '
#define EEAR9    (1)     // '
*/
#define EEAR8    (0)     // '   EEAR8 is an unused bit in ATmega48 and must always be written to zero.


//############################################             
//# GTCCR  (0x23)   GTCCR – General Timer/Counter Control Register 

#define TSM      (7)     // Timer/Counter Synchronization Mode
/* bits 6-2 reserved , read as zero*/
#define PSRASY   (1)     // Prescaler Reset Timer/Counter2
#define PSRSYNC  (0)     // Prescaler Reset



//############################################             
//# TCCR0A  (0x24)   TCCR0A – Timer/Counter Control Register A 

#define COM0A1   (7)     // Compare Match Output A Mode Bits
#define COM0A0   (6)     // '
#define COM0B1   (5)     // Compare Match Output B Mode Bits
#define COM0B0   (4)     // '
/* bits 3-2 reserved , read as zero*/
#define WGM01    (1)     // Waveform Generation Mode Bits
#define WGM00    (0)     // '


//############################################             
//# TCCR0B  (0x25)   TCCR0B – Timer/Counter Control Register B 

#define FOC0A   (7)     // Force Output Compare A
#define FOC0B   (6)     // Force Output Compare B
/* bits 5-4 reserved , read as zero*/
#define WGM02   (3)     // Waveform Generation Mode Bit2 (restliche Bits siehe oben)
#define CS02    (2)     // Clock Select Bits
#define CS01    (1)     // '
#define CS00    (0)     // '


//############################################
//# TCNT0 (0x26)         Timer/Counter0 (8-Bit)  Timer/Counter1 is realized as an up counter 

#define TCNT07  (7)     // Data Bit
#define TCNT06  (6)     // Data Bit
#define TCNT05  (5)     // Data Bit
#define TCNT04  (4)     // Data Bit
#define TCNT03  (3)     // Data Bit
#define TCNT02  (2)     // Data Bit
#define TCNT01  (1)     // Data Bit
#define TCNT00  (0)     // Data Bit




//############################################
//# OCR0A  (0x27)        OCR0A – Timer/Counter0 Output Compare Register A (8-Bit) 

#define OCR0A7  (7)     // Data Bit
#define OCR0A6  (6)     // Data Bit
#define OCR0A5  (5)     // Data Bit
#define OCR0A4  (4)     // Data Bit
#define OCR0A3  (3)     // Data Bit
#define OCR0A2  (2)     // Data Bit
#define OCR0A1  (1)     // Data Bit
#define OCR0A0  (0)     // Data Bit


//############################################
//# OCR0B  (0x28)        OCR0B – Timer/Counter0 Output Compare Register B (8-Bit) 

#define OCR0B7  (7)     // Data Bit
#define OCR0B6  (6)     // Data Bit
#define OCR0B5  (5)     // Data Bit
#define OCR0B4  (4)     // Data Bit
#define OCR0B3  (3)     // Data Bit
#define OCR0B2  (2)     // Data Bit
#define OCR0B1  (1)     // Data Bit
#define OCR0B0  (0)     // Data Bit



//########################## 
//# 0x29   reserved        #
//##########################


//############################################################             
//# GPIOR1  (0x2A)   GPIOR1 – General Purpose I/O Register 1 #
//# GPIOR2  (0x2B)   GPIOR2 – General Purpose I/O Register 2 #
//# gleiche Bitdefinitionen wie oben bei GPIOR0              #
//############################################################



//############################################
//# SPCR  (0x2C)        SPCR – SPI Control Register 

#define SPIE  (7)     // SPI Interrupt Enable
#define SPE   (6)     // SPI Enable
#define DORD  (5)     // Data Order
#define MSTR  (4)     // Master/Slave Select
#define CPOL  (3)     // Clock Polarity
#define CPHA  (2)     // Clock Phase
#define SPR1  (1)     // SPI Clock Rate Select Bits
#define SPR0  (0)     // '


//############################################             
//# SPSR  (0x2D)        SPSR – SPI Status Register 

#define SPIF   (7)     // SPI Interrupt Flag
#define WCOL   (6)     // Write COLlision Flag
/* bits 5-1 reserved , read as zero*/
#define SPI2X    (0)     // Double SPI Speed Bit


//############################################
//# SPDR  (0x2E)        SPDR – SPI Data Register 

#define SPDR7  (7)     // Data Bit
#define SPDR6  (6)     // Data Bit
#define SPDR5  (5)     // Data Bit
#define SPDR4  (4)     // Data Bit
#define SPDR3  (3)     // Data Bit
#define SPDR2  (2)     // Data Bit
#define SPDR1  (1)     // Data Bit
#define SPDR0  (0)     // Data Bit


//########################## 
//# 0x2F   reserved        #
//##########################


//######################################################################
//# ACSR (0x30)          Analog Comparator Control and Status Register 

#define ACD     (7)     // Analog Comparator Disable
#define ACBG    (6)     // Analog Comparator Bandgap Select
#define ACO     (5)     // Analog Comparator Output
#define ACI     (4)     // Analog Comparator Interrupt Flag
#define ACIE    (3)     // Analog Comparator Interrupt Enable
#define ACIC    (2)     // Analog Comparator Input Capture Enable
#define ACIS1   (1)     // Analog Comparator Interrupt Mode Select Bits
#define ACIS0   (0)     // '


//########################## 
//# 0x31  - 0x32  reserved #
//##########################


//############################################             
//# SMCR  (0x33)   SMCR – Sleep Mode Control Register 

/* bits 7-4 reserved , read as zero*/
#define SM2   (3)     // Sleep Mode Select Bits
#define SM1   (2)     // '
#define SM0   (1)     // '
#define SE    (0)     // Sleep Enable


//############################################             
//# MCUSR (0x34)    MCUSR – MCU Status Register 

// bit 7-4   reserved , read as zero
#define WDRF    (3)     // Watchdog System Reset Flag
#define BORF    (2)     // Brown-out Reset Flag
#define EXTRF   (1)     // External Reset Flag
#define PORF    (0)     // Power-on Reset Flag   



//############################################             
//# MCUCR   (0x35))  MCU Control Register

// bit 7 - 5   reserved , read as zero
#define PUD     (4)     // Pull-up Disable            
// bit 3-2 reserved , read as zero
#define IVSEL   (1)     // Interrupt Vector Select
#define IVCE    (0)     // Interrupt Vector Change Enable



//########################## 
//# 0x36   reserved        #
//##########################



//######################################################################
//# SPMCSR (0x37)          SPMCSR – Store Program Memory Control and Status Register 

#define SPMIE     (7)     // SPM Interrupt Enable
#define RWWSB     (6)     // Read-While-Write Section Busy  (It will always read as zero in ATmega48)
// bit  5   reserved , read as zero
#define RWWSRE    (4)     // Read-While-Write Section Read Enable (für Atmega48 Datasheet beachten)
#define BLBSET    (3)     // Boot Lock Bit Set
#define PGWRT     (2)     // Page Write
#define PGERS     (1)     // Page Erase
#define SELFPRGEN (0)     // Self Programming Enable


//########################## 
//# 0x38  - 0x3C  reserved #
//##########################




//######################################################################
//# SPL   (0x3D) Stack Pointer Low Register        Initial Value= RAMEND
 
#define SP7  (7)     // Data Bit
#define SP6  (6)     // Data Bit
#define SP5  (5)     // Data Bit
#define SP4  (4)     // Data Bit
#define SP3  (3)     // Data Bit
#define SP2  (2)     // Data Bit
#define SP1  (1)     // Data Bit
#define SP0  (0)     // Data Bit

//######################################################################
//# SPH   (0x3E) Stack Pointer High Register     Initial Value= RAMEND

/* reserved read as zero 
#define SP15  (7)     // Data Bit
#define SP14  (6)     // Data Bit
#define SP13  (5)     // Data Bit
#define SP12  (4)     // Data Bit
#define SP11  (3)     // Data Bit
*/
#define SP10  (2)     // Data Bit   (nicht für ATMEGA 48 ==> read as 0)
#define SP9   (1)     // Data Bit
#define SP8   (0)     // Data Bit


//############################################             
//# SREG   (0x3F) Status Register     

#define I   (7)     // Global Interrupt Enable
#define T   (6)     // Bit Copy Storage
#define H   (5)     // Half Carry Flag
#define S   (4)     // Sign Bit, S = N . V
#define V   (3)     // Two’s Complement Overflow Flag
#define N   (2)     // Negative Flag
#define Z   (1)     // Zero Flag
#define C   (0)     // Carry Flag


//################## ab hier extende IO-Space   ab SRAM - Adresse 0x60 ###########################
//For the Extended I/O space from 0x60 - 0xFF in SRAM, only the ST/STS/STD and LD/LDS/LDD
//instructions can be used.

// Hinweis: ab hier ist die  Adresse in (...) jetzt die Dataspace-Adresse   



//############################################
//# WDTCSR   (0x60) SRAM       WDTCSR – Watchdog Timer Control Register 

#define WDIF   (7)     // Watchdog Interrupt Flag
#define WDIE   (6)     // Watchdog Interrupt Enable
#define WDP3   (5)     // Watchdog Timer Prescaler Bit3
#define WDCE   (4)     // Watchdog Change Enable
#define WDE    (3)     // Watchdog System Reset Enable
#define WDP2   (2)     // Watchdog Timer Prescaler Bits
#define WDP1   (1)     // '
#define WDP0   (0)     // '


//######################################################################
//# CLKPR (0x61) SRAM    CLKPR – Clock Prescale Register 

#define CLKPCE     (7)     // Clock Prescaler Change Enable
// bit 6-4   reserved , read as zero
#define CLKPS3     (3)     // Clock Prescaler Select Bits
#define CLKPS2     (2)     // '
#define CLKPS1     (1)     // '
#define CLKPS0     (0)     // '


//################################# 
//# 0x62  - 0x63 (SRAM)  reserved #
//#################################


//######################################################################
//# PRR (0x64) SRAM    PRR – Power Reduction Register 

#define PRTWI     (7)     // Power Reduction TWI
#define PRTIM2    (6)     // Power Reduction Timer/Counter2
#define PRTIM0    (5)     // Power Reduction Timer/Counter0
// bit 4   reserved , read as zero
#define PRTIM1    (3)     // Power Reduction Timer/Counter1
#define PRSPI     (2)     // Power Reduction Serial Peripheral Interface
#define PRUSART0  (1)     // Power Reduction USART0
#define PRADC     (0)     // Power Reduction ADC


//######################## 
//# 0x65 (SRAM)  reserved #
//########################



//############################################
//# OSCCAL (0x66) SRAM  Oscillator Calibration Register  

#define CAL7    (7)     // Data Bit
#define CAL6    (6)     // Data Bit
#define CAL5    (5)     // Data Bit
#define CAL4    (4)     // Data Bit
#define CAL3    (3)     // Data Bit
#define CAL2    (2)     // Data Bit
#define CAL1    (1)     // Data Bit
#define CAL0    (0)     // Data Bit


//######################### 
//# 0x67 (SRAM)  reserved #
//#########################


//######################################################################
//# PCICR (0x68) SRAM  PCICR – Pin Change Interrupt Control Register 

// bit 7- 3   reserved , read as zero
#define PCIE2     (2)     // Pin Change Interrupt Enable 2
#define PCIE1     (1)     // Pin Change Interrupt Enable 1
#define PCIE0     (0)     // Pin Change Interrupt Enable 0



//######################################################################
//# EICRA (0x69) SRAM  EICRA – External Interrupt Control Register A 

// bit 7- 4   reserved , read as zero
#define ISC11     (3)     // Interrupt Sense Control 1 Bits
#define ISC10     (2)     // '
#define ISC01     (1)     // Interrupt Sense Control 0 Bits
#define ISC00     (0)     // '


//######################### 
//# 0x6A (SRAM)  reserved #
//#########################


//############################################
//# PCMSK0 (0x6B) SRAM  PCMSK0 – Pin Change Mask Register 0  

#define M_PCINT7    (7)     // Pin Change Enable Mask Bits
#define M_PCINT6    (6)     // '
#define M_PCINT5    (5)     // '
#define M_PCINT4    (4)     // '
#define M_PCINT3    (3)     // '
#define M_PCINT2    (2)     // '
#define M_PCINT1    (1)     // '
#define M_PCINT0    (0)     // '


//############################################
//# PCMSK1 (0x6C) SRAM  PCMSK1 – Pin Change Mask Register 1  

// bit 7   reserved , read as zero
#define PCINT14    (6)     // Pin Change Enable Mask Bits
#define PCINT13    (5)     // '
#define PCINT12    (4)     // '
#define PCINT11    (3)     // '
#define PCINT10    (2)     // '
#define PCINT9     (1)     // '
#define PCINT8     (0)     // '


//############################################
//# PCMSK2 (0x6D) SRAM  PCMSK2 – Pin Change Mask Register 2  

#define PCINT23    (7)     // Pin Change Enable Mask Bits
#define PCINT22    (6)     // '
#define PCINT21    (5)     // '
#define PCINT20    (4)     // '
#define PCINT19    (3)     // '
#define PCINT18    (2)     // '
#define PCINT17    (1)     // '
#define PCINT16    (0)     // '


//######################################################################
//# TIMSK0 (0x6E) SRAM  TIMSK0 – Timer/Counter0 Interrupt Mask Register 

// bit 7- 3   reserved , read as zero
#define OCIE0B     (2)     // Timer/Counter0 Output Compare Match B Interrupt Enable
#define OCIE0A     (1)     // Timer/Counter0 Output Compare Match A Interrupt Enable
#define TOIE0      (0)     // Timer/Counter0 Overflow Interrupt Enable



//######################################################################
//# TIMSK1 (0x6F) SRAM  TIMSK1 – Timer/Counter0 Interrupt Mask Register 

// bit 7- 6   reserved , read as zero
#define ICIE1     (5)     // Timer/Counter1, Input Capture Interrupt Enable
// bit 4- 3   reserved , read as zero
#define OCIE1B     (2)     // Timer/Counter1, Output Compare B Match Interrupt Enable
#define OCIE1A     (1)     // Timer/Counter1, Output Compare A Match Interrupt Enable
#define TOIE1      (0)     // Timer/Counter1, Overflow Interrupt Enable



//######################################################################
//# TIMSK2 (0x70) SRAM  TIMSK2 – Timer/Counter2 Interrupt Mask Register 

// bit 7- 3   reserved , read as zero
#define OCIE2B     (2)     // Timer/Counter2 Output Compare Match B Interrupt Enable
#define OCIE2A     (1)     // Timer/Counter2 Output Compare Match A Interrupt Enable
#define TOIE2      (0)     // Timer/Counter2 Overflow Interrupt Enable


//################################# 
//# 0x71  - 0x77 (SRAM)  reserved #
//#################################



//########################################################
//# ADCL   (0x78) SRAM     ADC Data Register Low Byte*/ 

//# Hinweis: - Definition sind nur gültig für ADLAR=0   Default, Ergebnis rechtsausgerichtet 
//#          - Immer zuerst ADCL lesen, bevor ADCH gelesen wird 
//#
        #define  ADC7   (7)
        #define  ADC6   (6)
        #define  ADC5   (5)
        #define  ADC4   (4)
        #define  ADC3   (3)
        #define  ADC2   (2) 
        #define  ADC1   (1)
        #define  ADC0   (0)
//########################################################



//########################################################
//#   ADCH   (0x79)  SRAM     ADC Data Register High Byte */ 

//   Hinweis: Definition sind nur gültig für ADLAR=0   Default, Ergebnis rechtsausgerichtet 
//            - Immer zuerst ADCL lesen, bevor ADCH gelesen wird 


/* bits 7-2 reserved , read as zero*/

#define  ADC9   (1)     
#define  ADC8   (0)     




//########################################################
//# ADCSRA (0x7A)  SRAM        ADC Control and Status Register A 

#define ADEN    (7)     // ADC Enable
#define ADSC    (6)     // ADC Start Conversion
#define ADATE   (5)     // ADC Auto Trigger Enable
#define ADIF    (4)     // ADC Interrupt Flag
#define ADIE    (3)     // ADC Interrupt Enable
#define ADPS2   (2)     // ADC Prescaler Select Bits
#define ADPS1   (1)     //  '
#define ADPS0   (0)     //  '



//########################################################
//# ADCSRB (0x7B)  SRAM        ADC Control and Status Register B 

/* bit 7 reserved , read as zero*/
#define ACME    (6)     // Analog Comparator Multiplexer Enable

/* bits 5-3  reserved , read as zero*/
#define ADTS2   (2)     // ADC Auto Trigger Source Bits
#define ADTS1   (1)     //  '
#define ADTS0   (0)     //  '



//#############################################################
// ADMUX  (0x7C)  SRAM      ADC Multiplexer Selection Register 

#define REFS1   (7)     // Reference Selection Bits
#define REFS0   (6)     // '
#define ADLAR   (5)     // ADC Left Adjust Result
/* bit 4 reserved , read as zero*/
#define MUX3    (3)     // Analog Channel and Gain Selection Bits
#define MUX2    (2)     // '
#define MUX1    (1)     // '
#define MUX0    (0)     // '


//######################### 
//# 0x7D (SRAM)  reserved #
//#########################



//########################################################
//# DIDR0 (0x7E)  SRAM       DIDR0 – Digital Input Disable Register 0 

/* bits 7-6  reserved , read as zero*/
#define ADC5D   (5)     // ADC5..0 Bits  Digital Input Disable
#define ADC4D   (4)     // '
#define ADC3D   (3)     // '
#define ADC2D   (2)     // '
#define ADC1D   (1)     // '
#define ADC0D   (0)     // '



//########################################################
//# DIDR1 (0x7F)  SRAM       DIDR1 – Digital Input Disable Register 1 

/* bits 7-2  reserved , read as zero*/
#define AIN1D   (1)     // Digital Input Disable Bit 1
#define AIN0D   (0)     // Digital Input Disable Bit 0



//############################################
//# TCCR1A  (0x80) SRAM    Timer/Counter1 Control Register A  

#define COM1A1  (7)     // Compare Output Mode for Channel A, Bit 1 
#define COM1A0  (6)     // Compare Output Mode for Channel A, Bit 0
#define COM1B1  (5)     // Compare Output Mode for Channel B, Bit 1
#define COM1B0  (4)     // Compare Output Mode for Channel B, Bit 0
/* bits 3-2  reserved , read as zero*/
#define WGM11   (1)     // Waveform Generation Mode Bit 1
#define WGM10   (0)     // Waveform Generation Mode Bit 0  


//############################################
//# TCCR1B  (0x81) SRAM       Timer/Counter1 Control Register B 

#define ICNC1    (7)     // Input Capture Noise Canceler
#define ICES1    (6)     // Input Capture Edge Select ICP1
// bit 5 reserved , read as zero, write as zero
#define WGM13    (4)     // Waveform Generation Mode Bits
#define WGM12    (3)     // '
#define CS12     (2)     // Clock Select Bits
#define CS11     (1)     // '
#define CS10     (0)     // '


//############################################
//# TCCR1C  (0x82) SRAM       Timer/Counter1 Control Register C 

#define FOC1A    (7)     // Input Capture Noise Canceler
#define FOC1B    (6)     // Input Capture Edge Select ICP1
// bits 5- 0 reserved , read as zero, write as zero



//######################### 
//# 0x83 (SRAM)  reserved #
//#########################



//############################################
//# TCNT1L   (0x84) SRAM     Timer/Counter1 Low Byte 

#define TCNT17  (7)     // Data Bit
#define TCNT16  (6)     // Data Bit
#define TCNT15  (5)     // Data Bit
#define TCNT14  (4)     // Data Bit
#define TCNT13  (3)     // Data Bit
#define TCNT12  (2)     // Data Bit
#define TCNT11  (1)     // Data Bit
#define TCNT10  (0)     // Data Bit


//############################################
//# TCNT1H   (0x85) SRAM     Timer/Counter1 High Byte 

#define TCNT115  (7)     // Data Bit
#define TCNT114  (6)     // Data Bit
#define TCNT113  (5)     // Data Bit
#define TCNT112  (4)     // Data Bit
#define TCNT111  (3)     // Data Bit
#define TCNT110  (2)     // Data Bit
#define TCNT19   (1)     // Data Bit
#define TCNT18   (0)     // Data Bit




//############################################
//# ICR1L   (0x86) SRAM    Input Capture Register 1  Low Byte 

#define ICR17  (7)     // Data Bit
#define ICR16  (6)     // Data Bit
#define ICR15  (5)     // Data Bit
#define ICR14  (4)     // Data Bit
#define ICR13  (3)     // Data Bit
#define ICR12  (2)     // Data Bit
#define ICR11  (1)     // Data Bit
#define ICR10  (0)     // Data Bit



//############################################
//# ICR1H   (0x87) SRAM    Input Capture Register 1  High Byte 

#define ICR115  (7)     // Data Bit
#define ICRT114  (6)     // Data Bit
#define ICRT113  (5)     // Data Bit
#define ICRT112  (4)     // Data Bit
#define ICRT111  (3)     // Data Bit
#define ICRT110  (2)     // Data Bit
#define ICRT19   (1)     // Data Bit
#define ICRT18   (0)     // Data Bit



//############################################
//# OCR1AL  (0x88)  SRAM       Output Compare Register 1 A  Low Byte

#define OCR1A7  (7)     // Data Bit
#define OCR1A6  (6)     // Data Bit
#define OCR1A5  (5)     // Data Bit
#define OCR1A4  (4)     // Data Bit
#define OCR1A3  (3)     // Data Bit
#define OCR1A2  (2)     // Data Bit
#define OCR1A1  (1)     // Data Bit
#define OCR1A0  (0)     // Data Bit



//############################################
//# OCR1AH  (0x89)  SRAM       Output Compare Register 1 A High Byte 

#define OCR1A15  (7)     // Data Bit
#define OCR1A14  (6)     // Data Bit
#define OCR1A13  (5)     // Data Bit
#define OCR1A12  (4)     // Data Bit
#define OCR1A11  (3)     // Data Bit
#define OCR1A10  (2)     // Data Bit
#define OCR1A9   (1)     // Data Bit
#define OCR1A8   (0)     // Data Bit



//############################################
//# OCR1BL  (0x8A) (SRAM)       Timer/Counter1 Output Compare Register B Low Byte 

#define OCR1B7  (7)     // Data Bit
#define OCR1B6  (6)     // Data Bit
#define OCR1B5  (5)     // Data Bit
#define OCR1B4  (4)     // Data Bit
#define OCR1B3  (3)     // Data Bit
#define OCR1B2  (2)     // Data Bit
#define OCR1B1  (1)     // Data Bit
#define OCR1B0  (0)     // Data Bit




//############################################
//# OCR1BH  (0x8B)   (SRAM)     Timer/Counter1 Output Compare Register B High Byte 

#define OCR1B15  (7)     // Data Bit
#define OCR1B14  (6)     // Data Bit1
#define OCR1B13  (5)     // Data Bit
#define OCR1B12  (4)     // Data Bit
#define OCR1B11  (3)     // Data Bit
#define OCR1B10  (2)     // Data Bit
#define OCR1B9   (1)     // Data Bit
#define OCR1B8   (0)     // Data Bit



//################################# 
//# 0x8C  - 0xAF (SRAM)  reserved #
//#################################



//############################################
//# TCCR2A  (0xB0) SRAM    Timer/Counter 2 Control Register A  

#define COM2A1  (7)     // Compare Output Mode for Channel A, Bit 1 
#define COM2A0  (6)     // Compare Output Mode for Channel A, Bit 0
#define COM2B1  (5)     // Compare Output Mode for Channel B, Bit 1
#define COM2B0  (4)     // Compare Output Mode for Channel B, Bit 0
/* bits 3-2  reserved , read as zero*/
#define WGM21   (1)     // Waveform Generation Mode Bit 1
#define WGM20   (0)     // Waveform Generation Mode Bit 0  


//############################################
//# TCCR2B  (0xB1) SRAM       Timer/Counter2 Control Register B 

#define FOC2A    (7)     // Force Output Compare A
#define FOC2B    (6)     // Force Output Compare B
// bits 5 - 4 reserved , read as zero, write as zero
#define WGM22    (3)     // Waveform Generation Mode
#define CS22     (2)     // Clock Select Bits
#define CS21     (1)     // '
#define CS20     (0)     // '



//############################################
//# TCNT2 (0xB2)   (SRAM)      Timer/Counter2 (8-Bit)  Timer/Counter2 is realized as an up counter 

#define TCNT27  (7)     // Data Bit
#define TCNT26  (6)     // Data Bit
#define TCNT25  (5)     // Data Bit
#define TCNT24  (4)     // Data Bit
#define TCNT23  (3)     // Data Bit
#define TCNT22  (2)     // Data Bit
#define TCNT21  (1)     // Data Bit
#define TCNT20  (0)     // Data Bit


//############################################
//# OCR2A  (0xB3)  (SRAM)      Timer/Counter2 Output Compare Register A (8-Bit) 

#define OCR2A7  (7)     // Data Bit
#define OCR2A6  (6)     // Data Bit
#define OCR2A5  (5)     // Data Bit
#define OCR2A4  (4)     // Data Bit
#define OCR2A3  (3)     // Data Bit
#define OCR2A2  (2)     // Data Bit
#define OCR2A1  (1)     // Data Bit
#define OCR2A0  (0)     // Data Bit


//############################################
//# OCR2B  (0xB4)  (SRAM)      Timer/Counter2 Output Compare Register B (8-Bit) 

#define OCR2B7  (7)     // Data Bit
#define OCR2B6  (6)     // Data Bit
#define OCR2B5  (5)     // Data Bit
#define OCR2B4  (4)     // Data Bit
#define OCR2B3  (3)     // Data Bit
#define OCR2B2  (2)     // Data Bit
#define OCR2B1  (1)     // Data Bit
#define OCR2B0  (0)     // Data Bit




//######################### 
//# 0xB5 (SRAM)  reserved #
//#########################


//############################################
//# ASSR  (0xB6)   (SRAM)     ASSR – Asynchronous Status Register 

// bit 7  reserved , read as zero, write as zero
#define EXCLK    (6)     // Enable External Clock Input
#define AS2      (5)     // Asynchronous Timer/Counter2
#define TCN2UB   (4)     // Timer/Counter2 Update Busy
#define OCR2AUB  (3)     // Output Compare Register2 Update Busy
#define OCR2BUB  (2)     // Output Compare Register2 Update Busy
#define TCR2AUB  (1)     // Timer/Counter Control Register2 Update Busy
#define TCR2BUB  (0)     // Timer/Counter Control Register2 Update Busy



//######################### 
//# 0xB7 (SRAM)  reserved #
//#########################


//############################################
//# TWBR  (0xB8)   (SRAM)     2-wire Serial Interface Bit Rate Register TWI

#define TWBR7  (7)     // Data Bit
#define TWBR6  (6)     // Data Bit
#define TWBR5  (5)     // Data Bit
#define TWBR4  (4)     // Data Bit
#define TWBR3  (3)     // Data Bit
#define TWBR2  (2)     // Data Bit
#define TWBR1  (1)     // Data Bit
#define TWBR0  (0)     // Data Bit



//############################################
//# TWSR  (0xB9) (SRAM)      TWSR – TWI Status Register

#define TWS7  (7)     // TWI Status Bits
#define TWS6  (6)     // '
#define TWS5  (5)     // '
#define TWS4  (4)     // '
#define TWS3  (3)     // '
// bit 2  reserved , read as zero, write as zero
#define TWPS1  (1)     // TWI Prescaler Bits
#define TWPS0  (0)     // '


//############################################
//# TWAR  (0xBA) (SRAM)     TWAR – TWI (Slave) Address Register

#define TWA6  (7)     // TWI (Slave) Address Register
#define TWA5  (6)     // '
#define TWA4  (5)     // '
#define TWA3  (4)     // '
#define TWA2  (3)     // '
#define TWA1  (2)     // '
#define TWA0  (1)     // '
#define TWGCE  (0)    // TWI General Call Recognition Enable Bit



//############################################
//# TWDR  (0xBB)   (SRAM)     2-wire Serial Interface Data Register ( TWI)

#define TWD7  (7)     // Data Bit
#define TWD6  (6)     // Data Bit
#define TWD5  (5)     // Data Bit
#define TWD4  (4)     // Data Bit
#define TWD3  (3)     // Data Bit
#define TWD2  (2)     // Data Bit
#define TWD1  (1)     // Data Bit
#define TWD0  (0)     // Data Bit


//############################################
//# TWCR  (0xBC)  (SRAM)    TWCR – TWI Control Register

#define TWINT  (7)     // TWI Interrupt Flag
#define TWEA   (6)     // TWI Enable Acknowledge Bit
#define TWSTA  (5)     // TWI START Condition Bit
#define TWSTO  (4)     // TWI STOP Condition Bit
#define TWWC   (3)     // TWI Write Collision Flag
#define TWEN   (2)     // TWI Enable Bit
// bit 1  reserved , read as zero, write as zero
#define TWIE   (0)     // TWI Interrupt Enable


//############################################
//# TWAMR  (0xBD)  (SRAM)      TWI (Slave) Address Mask Register

#define TWAM6  (7)     // TWI Address Mask Bits
#define TWAM5  (6)     // '
#define TWAM4  (5)     // '
#define TWAM3  (4)     // '
#define TWAM2  (3)     // '
#define TWAM1  (2)     // '
#define TWAM0  (1)     // '
// bit 0  reserved , read as zero, write as zero


//################################# 
//# 0xBE  - 0xBF (SRAM)  reserved #
//#################################


//############################################
//# UCSR0A  (0xC0)   (SRAM)   UCSR0A-  USART Control and Status Register 0 A

#define RXC0   (7)     // USART Receive Complete
#define TXC0   (6)     // USART Transmit Complete
#define UDRE0  (5)     // USART Data Register Empty
#define FE0    (4)     // Frame Error
#define DOR0   (3)     // Data OverRun
#define UPE0   (2)     // USART Parity Error
#define U2X0   (1)     // Double the USART Transmission Speed
#define MPCM0  (0)     // Multi-processor Communication Mode



//############################################
//# UCSR0B  (0xC1)   (SRAM)   UCSR0B-  USART Control and Status Register 0 B

#define RXCIE0  (7)     // RX Complete Interrupt Enable 0
#define TXCIE0  (6)     // TX Complete Interrupt Enable 0
#define UDRIE0  (5)     // USART Data Register Empty Interrupt Enable 0
#define RXEN0   (4)     // Receiver Enable 0
#define TXEN0   (3)     // Transmitter Enable 0
#define UCSZ02  (2)     // Character Size 0
#define RXB80   (1)     // Receive Data Bit 8 0
#define TXB80   (0)     // Transmit Data Bit 8 0


//############################################
//# UCSR0C  (0xC2)   (SRAM)   UCSR0C-  USART Control and Status Register 0 C

#define UMSEL01  (7)     // USART Mode Select Bits
#define UMSEL00  (6)     // '
#define UPM01    (5)     // Parity Mode Bits
#define UPM00    (4)     // '
#define USBS0    (3)     // Stop Bit Select

#define UCSZ01   (2)     // Character Size Bits
#define UCSZ00   (1)     // '
#define UDORD0   (2)     // Data Order
#define UCPHA0   (1)     // Clock Phase

#define UCPOL0   (0)     // Clock Polarity



//######################### 
//# 0xC3 (SRAM)  reserved #
//#########################


//############################################
//# UBRR0L  (0xC4)   (SRAM)    USART 0  Baud Rate Register Low Byte

#define UBRR07  (7)     // Data Bit
#define UBRR06  (6)     // Data Bit
#define UBRR05  (5)     // Data Bit
#define UBRR04  (4)     // Data Bit
#define UBRR03  (3)     // Data Bit
#define UBRR02  (2)     // Data Bit
#define UBRR01  (1)     // Data Bit
#define UBRR00  (0)     // Data Bit



//############################################
//# UBRR0H  (0xC5)   (SRAM)    USART 0  Baud Rate Register High Byte

// bits 7 - 4 reserved , read as zero, write as zero
#define UBRR011  (3)     // Data Bit
#define UBRR010  (2)     // Data Bit
#define UBRR09   (1)     // Data Bit
#define UBRR08   (0)     // Data Bit



//############################################
//# UDR0  (0xC6)   (SRAM)    USART 0 I/O Data Register

#define UDR07  (7)     // Data Bit
#define UDR06  (6)     // Data Bit
#define UDR05  (5)     // Data Bit
#define UDR04  (4)     // Data Bit
#define UDR03  (3)     // Data Bit
#define UDR02  (2)     // Data Bit
#define UDR01  (1)     // Data Bit
#define UDR00  (0)     // Data Bit


//################################# 
//# 0xC7  - 0xFF (SRAM)  reserved #
//#################################



//###########################
// Constants 
#define RAMEND      0x2FF  // atmega 48      

//#define RAMEND      0x4FF // atmega 88      
//#define RAMEND      0x4FF // atmega 168     


//#define E2END       0x3FF
//#define FLASHEND    0x7FFF

#define sei()  #asm("sei")
#define cli()  #asm("cli")
/*================================= Typedefs ================================*/
/* nothing */

/*============================ global variables =============================*/
/* nothing */

/*======================= prototyps of global functions =====================*/
/* nothing */

#endif /* #ifndef __ATMEGA48BIT_H */
