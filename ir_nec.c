/******************************************************************************
*                                                                             *
*   Modul-Name :        IR NEC Remote                                         *
*   Datei-Name :        ir_nec.c                                              *
*   Datei-Typ :         Sub                                                   *
*                                                                             *
*   Autor :             A.Weick                                               *
*   Letzte Aenderung :  15.08.2008 / A.Weick                                  *
*                                                                             *
*   Version :           REV 0.00                                              *
*                                                                             *
*   Copyright (c) 2008 Andreas Weick.                                         *
*                      Alle Rechte vorbehalten.                               *
*                                                                             *
*=============================================================================*
*                                                                             *
*   Beschreibung :      Funktionen für das IR REMOTE Protokoll NEC            *
*                                                                             *
*   Besonderheiten :    Keine                                                 *
*                                                                             *
*   Portabilitaet :     ANSI-C                                                *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   Definitions-Datei :  ir_nec.h                                             *
*                                                                             *
*   Exportschnittstelle des Moduls :                                          *
*      interrupt [TIM0_OVF] void timer0_ovf_isr(void);                        *
*      interrupt [EXT_INT0] void ext_int0_isr(void);                          *
*                                                                             *
*   Globale Variablen des Moduls:                                             *
*      extern bit Repeat_flag;                                                *
*      extern bit newData_flag;                                               *
*      extern unsigned char Address;                                          *
*      extern unsigned char Command;                                          *
*      extern unsigned char Address_Sec;                                      *
*      extern unsigned char Command_Sec;                                      *
*                                                                             *
*   Benoetigte Bibliotheken und Module :                                      *
*     - ANSI-C-Standardbibliothek                                             *
*     - makros.h                                                              *
*     - mega48.h                                                              *
*     - atmega48bit.h                                                         *
*                                                                             *
******************************************************************************/ 
/*
Die Grundidee für die Infrarotauswerte Routine stammt von:
****************************************************************************
* Filename    : IR NEC by http://www.sbprojects.com/knowledge/ir/index.php *
* Revision    : 1.0                                                        *
* Controller  : ATMEGA8                                                    *
* Compiler    : BASCOM-AVR 1.11.8.3 DEMO                                   *
* Author      : Rubashka Vasiliy , Ukraine , 2007                          *
* WWW         : http://ledeffects.net                                      *
* Mail        : info@ledeffects.net                                        *
****************************************************************************

Von mir wurde das Bascom - Konzept übernommen und nach C portiert.

*/
/*============================== Modul-History ==============================*/
/*
REV 0.01 27.11.2007 -AW-:
	 Created

*/

/*=========================== Definitions-Dateien ===========================*/
#include "/inc/ir_nec.h" 
#include "/inc/makros.h" 


#ifdef  _CHIP_ATMEGA48_
  #include <mega48.h>
  #include "inc/atmega48bit.h"
  #warning "[ir_nec.c]: ATMega48  gewählt"
#endif

#ifdef  _CHIP_ATMEGA168_
  #include <mega168.h>
  #include "inc/atmega168bit.h"
  #warning "[ir_nec.c]: ATMega168  gewählt"
#endif



/*================================= Makros ==================================*/
#define __IR_NEC_C__
/*================================ Typedefs =================================*/

/*============================ Globale Variablen ============================*/


unsigned int Tik;           //' Zähler für die 96µs Ticks
unsigned char Byt;          // 'counter accepted bit

bit Repeat_flag;            // 'flag of repetition
bit Start_flag;             // 'flag of start condition
bit newData_flag;           // Wird gesetzt, sobald neue Daten eingelesen wurden und zur Ausgabe bereit stehen

unsigned char Address;      // 'byte of address  
unsigned char Address_Sec;  // 2. Adressbyte (= ~Adress)        // insert mod-003 :  
unsigned char Command;      // 'byte of command
unsigned char Command_Sec;  // 2. Commandbyte (= ~Command)      // insert mod-003:

unsigned char Address_1;    // 'direct byte of address
unsigned char Command_1;    // 'direct byte of command

unsigned char Address_0;    //'indirect byte of address
unsigned char Command_0;    //'indirect byte of command
//unsigned int  Summa;        // wird vorerst noch nicht verwendet




 
/*================== Prototypen der dateiglobalen Funktionen ================*/
void vStartTimer0(void);
void vStopTimer0(void);
/*================================ Datei-Code ===============================*/



// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
// Reinitialize Timer 0 value
TCNT0=TIMER0_CNT;                         //0xFD;
// Place your code here

  Tik++;
  if (Tik >= 1200) { // 'if 1200 teaks, have thrown all in source condition  >=115ms  ==> somit keine Daten, Messung zurücksetzen
        Tik = 0;
        Repeat_flag = 0;
        Start_flag = 0;
        Address_1 = 0;
        Command_1 = 0;
        Address_0 = 0;
        Command_0 = 0;
        Address = 0;
        Command = 0; 

        Address_Sec=0;    // insert mod-003
        Command_Sec=0;    // insert mod-003

        TIMSK0= 0x00;        //  Stop Timer0              // Timer anhalten, da die nächste Messung erst wieder mit einer negativen Flange erfolgt
   }
} // end Timer0 ISR




// External Interrupt 0 service routine
interrupt [EXT_INT0] void ext_int0_isr(void)
{
    // Place your code here

    TCNT0= TIMER0_CNT ;
    TIMSK0= (1<<TOIE0);        //    Timer0 Interrupt freigeben




    if ( (Tik >= 139) && (Tik < 150)) { // Then 'if has happenned from 139 before 150 teaks - "START"
       // Address = 1;    fehler in der Vorlage , deswegen auskommentiert
        Repeat_flag = 0;
        Start_flag = 1;
        Address_1 = 0;
        Command_1 = 0;
        Address_0 = 0;
        Command_0 = 0;
    }

    if ( (Tik >= 116) && (Tik < 139) ) {    // Then 'if has happenned from 116 before 138 teaks - "REPETITION"
        //Address = 2;    fehler in der Vorlage auskommentiert
        Repeat_flag = 1;
        Start_flag = 0;
    }

    if ( (Tik >= 22) && (Tik < 116) && (Start_flag == 1) ) { // Then 'if has happenned from 22 before 115 teaks - have taken "1"
        Byt++;

        if (Byt < 9) {
            Address_1<<=1;
            Address_1 += 1;
        }

        if( (Byt >= 9) && (Byt < 17) ) {
            Address_0 <<= 1;
            Address_0 +=  1;
        }

        if( (Byt >= 17) && (Byt < 25) ) {
            Command_1 <<= 1;
            Command_1 +=  1;
        }

        if (Byt >= 25) {
            Command_0 <<= 1;
            Command_0 +=  1;
        }
 
    } // End ' Tik >= 22 And Tik < 116 And Start_flag = 1

    if ( (Tik >= 10) && (Tik < 22) && (Start_flag == 1) ) { // Then 'if has happenned from 10 before 21 teaks - have taken "0"
        Byt++;

        if (Byt < 9) {
            Address_1 <<= 1;
        }

        if ((Byt >= 9) && (Byt < 17) ){
            Address_0 <<=1;
        }

        if ( (Byt >= 17) && (Byt < 25) ) {
            Command_1 <<=1;
        }

        if (Byt >= 25) {
            Command_0 <<= 1;
        }
    }

    Tik = 0;



    if (Byt == 32) {  // Then 'if have taken 4 bytes, check correctness a receiving a command
 //'if address or command 16-bit, check does not pass
// 'Summa = Address_0 + Address_1
 //'If Summa = 255 Then
        Address = Address_1;  
        Address_Sec = Address_0;  // insert mod-003
        newData_flag= 1;    
 //'Else
 //'Address = 0
 //'End If

 //'Summa = Command_0 + Command_1
 //'If Summa = 255 Then
        Command = Command_1; 
        Command_Sec = Command_0;  // insert mod-003
 //'Else
 //'Command = 0
 //'End If
        Byt = 0;
        Repeat_flag = 0;
        Start_flag = 0;
        TIMSK0= 0x00;        //    Timer0 Interrupt sperren
    }

 } // end int2 isr


static void vStartTimer0(void) { // wird zur zeit nicht verwendet, um Platz zu sparen
    TCNT0= TIMER0_CNT ;
    TIMSK0= (1<<TOIE0);        //    Timer0 Interrupt freigeben
}

static void vStopTimer0(void) { // wird zur zeit nicht verwendet, um Platz zu sparen
    TIMSK0= 0x00;        //    Timer0 Interrupt sperren                     
}








