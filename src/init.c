/******************************************************************************
*                                                                             *
*   Modul-Name :        Init Modul                                            *
*   Datei-Name :        init.c                                              *
*   Datei-Typ :         Sub                                                   *
*                                                                             *
*   Autor :             A.Weick                                               *
*   Letzte Aenderung :  27.11.2007 / A.Weick                                  *
*                       19.08.2008 / atmega48 eingefügt - weick   (mod-001)   *
*                       27.09.2008 / atmega168 eingefügt - weick  (mod-002)   *
*                                                                             *
*   Version :           REV 0.03                                              *
*                                                                             *
*   Copyright (c) 2008 Andreas Weick                                          *
*                      Alle Rechte vorbehalten.                               *
*                     - Confidential -                                        *
*                                                                             *
*=============================================================================*
*                                                                             *
*   Beschreibung :      Initialisierung des Atmel Prozoessor und den          *
*                       Watchdog aktivieren                                   *
*   Besonderheiten :    Keine                                                 *
*                                                                             *
*   Portabilitaet :     ANSI-C                                                *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   Definitions-Datei :  init.h                                               *
*                                                                             *
*   Exportschnittstelle des Moduls :                                          *
*      void vInit(void)                                                       *
*                                                                             *
*   Benoetigte Bibliotheken und Module :                                      *
*     - ANSI-C-Standardbibliothek                                             *
*     - makros.h                                                              *
*     - atmegaxx8.h                                                           *
*                                                                             *
******************************************************************************/
/*============================== Modul-History ==============================*/
/*
REV 0.01 27.11.2007 -AW-:
	 Created

REV 0.02 19.08.2008  -AW-:
 Makros für  atmega48 eingefügt - weick   (mod-001)

REV 0.03 27.09.2008  -AW-:
 Makros für  atmega168 eingefügt - weick   (mod-002)


*/
/*=========================== Definitions-Dateien ===========================*/

#ifdef _CHIP_ATMEGA48_
  #include <mega48.h>
  #include "inc/atmega48bit.h"
#else
  #ifdef   _CHIP_ATMEGA168_
    #include <mega168.h>
    #include "inc/atmega168bit.h"
 #else
   #include "../inc/attiny26bit.h"
   #include <tiny26.h>
 #endif
#endif

#include "../inc/init.h"


#include "../inc/makros.h"

/*================================= Makros ==================================*/
#define __INIT_C__
/*================================ Typedefs =================================*/

/*============================ Globale Variablen ============================*/

/*================== Prototypen der dateiglobalen Funktionen ================*/


/*================================ Datei-Code ===============================*/




/******************************************************************************
*                                                                             *
*   funktion       :    vInit()                                               *
*                          Register initialisieren                            *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    27.11.2007 / A.Weick                                  *
*                       19.08.2008 / atmega48 eingefügt - weick   (mod-001)   *
*                       27.09.2008 / atmega168 eingefügt - weick  (mod-002)   *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    Intitialsierung                                       *
*                                                                             *
*   specials       :    none                                                  *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    none                                                  *
*                          -                                                  *
*                                                                             *
*   return value   :    none                                                  *
*                          -                                                  *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*                                                                             *
*                                                                             *
******************************************************************************/

void vInit(void) {

// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=0x80;
CLKPR=0x00;
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Input/Output Ports initialization
// Port B initialization
// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
PORTB=0x00;
DDRB=0x08;  // Licht Ausgang

// Port C initialization
// Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
// State6=T State5=T State4=T State3=T State2=T State1=T State0=T
PORTC=0x00;
DDRC=0x00;

// Port D initialization
// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
PORTD= (0x00 |  (1<<PD6));
DDRD= ( 0x00 |  (1<<PD6) | (1<<PD7) );

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 31,250 kHz   T= 32 µs
// Mode: Normal top=FFh
// OC0A output: Disconnected
// OC0B output: Disconnected
TCCR0A=0x00;
TCCR0B=0x04;
TCNT0= TIMER0_CNT;    // 96µs       //0xFD;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: Timer 1 Stopped
// Mode: Normal top=FFFFh
// OC1A output: Discon.
// OC1B output: Discon.
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer 1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=0x00;
TCCR1B=0x00;
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 7,813 kHz
// Mode: Normal top=FFh
// OC2A output: Disconnected
// OC2B output: Disconnected
ASSR=0x00;
TCCR2A=0x00;
TCCR2B=0x07;
TCNT2=0x3D;
OCR2A=0x00;
OCR2B=0x00;

// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Falling Edge
// INT1: Off
// Interrupt on any change on pins PCINT0-7: Off
// Interrupt on any change on pins PCINT8-14: Off
// Interrupt on any change on pins PCINT16-23: Off
EICRA=0x02;
EIMSK=0x01;
EIFR=0x01;
PCICR=0x00;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=0x00;    // erst mal stoppen, wird dann im int0 Interupt gesstartet                  0x01;
// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=0x00;
// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=0x01;                // für en delay takt starten

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On change mod-001   alt:  Off
// USART Transmitter: On
// USART0 Mode: Asynchronous
// USART Baud Rate: 19200
UCSR0A=0x00;
UCSR0B=0xD8;  //change mod-001 alt: UCSR0B=0x48;
UCSR0C=0x06;
UBRR0H=0x00;
UBRR0L=0x19;

// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
ADCSRB=0x00;


// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/256k
// Watchdog Timer interrupt: Off
#pragma optsize-
#asm("wdr")
WDTCSR=0x1F;
WDTCSR=0x0F;
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif
} // end vInit()




