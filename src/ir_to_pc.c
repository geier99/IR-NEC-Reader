/*****************************************************
This program was produced by the
CodeWizardAVR V1.25.9 Standard
Automatic Program Generator
© Copyright 1998-2008 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : Atmega48  NEC IR Protokoll lesen und über RS232 ausgeben
Version : 1.0.0
Date    : 19.08.2008
Author  : Andreas Weick
Company :
Comments: Liest infrarot codes und sendet diese an den PC in 4 verschiedenen Formatvarianten
          UART-Settings: 19200 8N1

Revision: 1.0.1     // mod-003
Date:   : 28.09.2008
Comments: Jetzt auch die Invertierten Datenbytes mit den eingelesenen Werten ausgeben,
          und nicht einfach das Empfange Datenbyte bei der Ausgabe invertieren

Chip type           : ATmega48  bzw. ATmega168 (empfohlen)
Clock frequency     : 8,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 128
*****************************************************/

//############################################################################################
//                                                                                           #
//   W I C H T I G  !!!!!!!!!!!!                                                             #
//                                                                                           #
// Nicht vergessen das Calibration Byte an Adresse 0x0000 beim EEPROM zu schreiben           #
//                                                                                           #
//############################################################################################



//############################################################################################
//                                                                                           #
//  Prozessorauswahl erfolgt nun ausschliesslich über Project-->Configure-->Compiler--> CHIP #
//                                                                                           #
//  zulässig sind ATMega48 und ATMega168                                                     #
//  (wenn bei Atmega48 der Speicher knapp wird, einfach ein paar Print Befehle wegelassen    #
//############################################################################################



#ifdef  _CHIP_ATMEGA48_
    #include <mega48.h>
    #include "inc/atmega48bit.h"
    #warning "[ir_to_pc.c]: ATMega48  gewählt"
#endif

#ifdef  _CHIP_ATMEGA168_
    #include <mega168.h>
    #include "inc/atmega168bit.h"
    #warning "[ir_to_pc.c]: ATMega168  gewählt"
#endif

#include <delay.h>

#include "inc\makros.h"
#include "inc\eeprom.h"
#include "inc\ir_nec.h"
#include "inc\init.h"
#include "inc\lcd.h"

#define DELAY_100MS 4     // Anzahl der 25ms Ticks um auf 100ms zu kommen
#define MAXBUFFER   20    // RAM Buffer für Strings  19-Zeichenstring

static void printHelp(void);

void vTimeHdl(void);
void WDT_off(void);
void WDT_on(void);
void delay_100ms(void);       // ca. 100 ms Verzögerung
void delay_500ms(void);
void delay_1s(void);

volatile unsigned char uc25MSEC_CNT=0;
volatile unsigned char ucSec_Tick=0, uc250ms_Tick=0;
volatile unsigned char uc100ms_Tick=0,uc25ms_Tick=0;


#define RXB8 1
#define TXB8 0
#define UPE 2
#define OVR 3
#define FE 4
#define UDRE 5
#define RXC 7

#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<OVR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)



// insert mod-001
// USART Receiver buffer
#define RX_BUFFER_SIZE0 8
char rx_buffer0[RX_BUFFER_SIZE0];

#if RX_BUFFER_SIZE0<256
unsigned char rx_wr_index0,rx_rd_index0,rx_counter0;
#else
unsigned int rx_wr_index0,rx_rd_index0,rx_counter0;
#endif

// This flag is set on USART Receiver buffer overflow
bit rx_buffer_overflow0;

// USART Receiver interrupt service routine
interrupt [USART_RXC] void usart_rx_isr(void)
{
char status,data;
status=UCSR0A;
data=UDR0;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
   rx_buffer0[rx_wr_index0]=data;
   if (++rx_wr_index0 == RX_BUFFER_SIZE0) rx_wr_index0=0;
   if (++rx_counter0 == RX_BUFFER_SIZE0)
      {
      rx_counter0=0;
      rx_buffer_overflow0=1;
      };
   };
}

#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used+
char getchar(void)
{
char data;
if (rx_counter0==0) {    // mod-001  by aw  alt: while (rx_counter0==0);
   return 0;  // kein zeichne empfangen
}

data=rx_buffer0[rx_rd_index0];
if (++rx_rd_index0 == RX_BUFFER_SIZE0) rx_rd_index0=0;
#asm("cli")
--rx_counter0;
#asm("sei")
return data;
}
#pragma used-
#endif

// end insert mod-001


// USART Transmitter buffer
#define TX_BUFFER_SIZE0 32
char tx_buffer0[TX_BUFFER_SIZE0];

#if TX_BUFFER_SIZE0<256
unsigned char tx_wr_index0,tx_rd_index0,tx_counter0;
#else
unsigned int tx_wr_index0,tx_rd_index0,tx_counter0;
#endif

// USART Transmitter interrupt service routine
interrupt [USART_TXC] void usart_tx_isr(void)
{
if (tx_counter0)
   {
   --tx_counter0;
   UDR0=tx_buffer0[tx_rd_index0];
   if (++tx_rd_index0 == TX_BUFFER_SIZE0) tx_rd_index0=0;
   };
}

#ifndef _DEBUG_TERMINAL_IO_
// Write a character to the USART Transmitter buffer
#define _ALTERNATE_PUTCHAR_
#pragma used+
void putchar(char c)
{
while (tx_counter0 == TX_BUFFER_SIZE0);
#asm("cli")
if (tx_counter0 || ((UCSR0A & DATA_REGISTER_EMPTY)==0))
   {
   tx_buffer0[tx_wr_index0]=c;
   if (++tx_wr_index0 == TX_BUFFER_SIZE0) tx_wr_index0=0;
   ++tx_counter0;
   }
else
   UDR0=c;
#asm("sei")
}
#pragma used-
#endif

// Standard Input/Output functions
#include <stdio.h>



// Timer 2 overflow interrupt service routine   (25ms Tick für die Zeitticks verwendne)
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
    // Reinitialize Timer 2 value
    TCNT2=0x3D;                                       // Timer2 für den 25ms Tick intialisieren
    // Place your code here
    uc25ms_Tick=1;
    if (uc25MSEC_CNT<=255)  uc25MSEC_CNT++;           // Zähler für die 25ms Ticks erhöhen
    if (uc100ms_Tick>0) uc100ms_Tick--;               // Zähler für den 100ms Tick dekrementiren
}





// Declare your global variables here

unsigned char i,ch;
unsigned char buffer[MAXBUFFER];

volatile enum printmodus {
   normal          //  Adresse + Command in Klartext
,  komplett        //  wie normal + die zusätzlich invertierte Bytes
,  dump2            //  Hexdump Adresse und Command
,  dump4            //  4 Byte Hex-Dump  adresse, /adresse  command, /command
} enDisplayModus;

void main(void){
    // Declare your local variables here

    vInit();
    vInitLcd();
    vGotoXY_LCD(0,0);
    vPrintStringF_LCD("IR-NEC Analyzer");
    vGotoXY_LCD(0,1);
    vPrintStringF_LCD("(c) 2008 by Andi");

    PORTB |= 0x08;  // LCD Licht an
    printHelp();

    enDisplayModus=normal ;

    delay_ms(100);                   // warte

    // Global enable interrupts
    #asm("sei")



     i=bEepromReadByte(0x0000);       // Kalibrierbyte vom EPROM lesen (muss beim programmieren an die EEPROM-Adresse programmiert werden)
     OSCCAL= i;                       // abspeichern

    // vEepromWriteByte(0x0001, i);   // test für eeprom writefuktion und ob vorher richtige EERPom ausgelesen wurde.

    delay_1s();
    newData_flag=0;

    while (1)  {
        // Place your code here
        #asm("wdr")    // watchdog reeset

        vTimeHdl();

        ch=getchar();
        if ( ch != 0) {
            switch (ch) {
            case '?':
            case 'h':
                printHelp();
                break;

            case 'R':
                printf("\033[2J\033[1;1");     //Screen löschen und Cursor oben positionieren
                delay_100ms();
                printf("\n\rWatchdog Reset!\n\r");
                while (1);
                break;

            case 'c':
                printf("\033[2J\033[1;1");
                break;

            case 'l':
                printf("Toggle LCD - Licht\n\r\n\r");
                PORTB ^= 0x08;
                break;


            case '0':
                    printf("\n\r\n\rAnzeigemodus: normal\n\r\n\r");
                    enDisplayModus=normal;
                break;

            case '1':
                    printf("\n\r\n\rAnzeigemodus: komplett\n\r\n\r");
                    enDisplayModus=komplett;
                break;
            case '2':
                    printf("\n\r\n\rAnzeigemodus: 2 Byte Hexdump ( Adresse, Command )\n\r\n\r");
                    enDisplayModus=dump2;
                break;
            case '3':
                    printf("\n\r\n\rAnzeigemodus: 4 Byte Hexdump (Adr, /Adr, Com, /Com)\n\r\n\r");
                    enDisplayModus=dump4;
                break;
            default:
                   printf("\n\r\n\r unbekannter Befehl:%c\n\r",ch);

            } // end switch
        }  // end ch
   } // end while

} // end main





void vTimeHdl(void){
    if (uc25MSEC_CNT>=10) {                    // 250ms
        uc25MSEC_CNT=0;

        if(uc250ms_Tick<255) uc250ms_Tick++;

        if(uc250ms_Tick>=4) {                  // 1s
         uc250ms_Tick=0;
         if(ucSec_Tick<255) ucSec_Tick++;
        }
    }

    if (uc25ms_Tick) { // alle 25ms ausführen
        if( newData_flag == 1 ) {
            sprintf(buffer,"%02X %02X | %02X %02X   ",Address,Address_Sec,Command, Command_Sec);
            vGotoXY_LCD(0,1);
            vPrintString_LCD(buffer);

            switch (enDisplayModus) {
                case komplett:
                    printf("\n\rAdresse: 0x%02X , 0x%02X   Command: 0x%02X , 0x%02X   ",Address,Address_Sec,Command, Command_Sec);
                    break;

                case dump2:
                    printf("\n\r 0x%02X 0x%02X",Address,Command);
                    break;

                case dump4:
                    printf("\n\r 0x%02X 0x%02X 0x%02X 0x%02X",Address,Address_Sec,Command, Command_Sec);
                    break;
                default:
                    printf("\n\rAdresse: 0x%02X   Command: 0x%02X   ",Address,Command);
                    break;
            }// end switch enDisplayModus
            newData_flag=0;  // Flag
        }
       uc25ms_Tick=0;
    }

    if (  ucSec_Tick) {  // alle Sekunde ausführen

        if ( (enDisplayModus==normal) || (enDisplayModus==komplett) ) {
            if(Repeat_flag==1) {
                printf("*");
            }
            else {
                printf(".");
            }
        } // end enDisplayModus
        ucSec_Tick=0;
    }
}




void delay_100ms(void) {       // ca. 100 ms Verzögerung
    uc100ms_Tick =   DELAY_100MS ;
    do {
        #asm("wdr")  ;
    } while (uc100ms_Tick);
}

void delay_500ms(void) {  // ca. 0,5s
    unsigned char i;
    for (i=0;i<5;i++) {
    delay_100ms();
    }
}

void delay_1s(void) {
    delay_500ms();
    delay_500ms();
}


void WDT_off(void)   // für atmega48
{
    #asm("cli")     // Interrupt sperren
    #asm("wdr")    // Reset Watchdog

    MCUSR &= ~(1<<WDRF);   //  Clear WDRF  Watch dog System Reset Flag

    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional time-out */
    WDTCSR |= (1<<WDCE) | (1<<WDE);

    /* Turn off WDT */
    WDTCSR = 0x00;

    #asm("sei")
}



void WDT_on(void) {
    #asm("cli")     // Interrupt sperren
    #asm("wdr")    // Reset Watchdog

    /* Start timed equence */
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = (1<<WDE) | (1<<WDP2)| (1<<WDP1) | (1<<WDP0);   /* Set new prescaler(time-out) value = 256K cycles (~2 s) */

    #asm("sei")
}

static void printHelp(void) {

    printf("\n\rIR-NEC-Code Analyse Testprogramm von Andreas Weick (c)2008-2017  Version: 1.0.2\n\r\n\r");
    printf("Commands:  'R' => Watchdogreset\n\r");
    printf("           'c' => Bildschirm loeschen\n\r");
    printf("           0-3 => Displaymodus\n\r\n\r");
    printf("           'l' => LCD Light\n\r");
    printf("         '?,h' => diesen Hilfe Text\n\r\n\r");
    printf("\n\rLegende: . :kein IR Signal,  * :FB-Taste gehalten\n\r\n\r");
    printf("NEC Protokoll:\n\n\rWarte auf IR-Signal:\n\r");
}
