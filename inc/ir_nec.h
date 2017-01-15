/******************************************************************************
*                                                                             *
*   Datei-Name :        <ir_nec.h>                                            *
*   Datei-Typ :         Header                                                *
*                                                                             *
*   Autor :             Andreas WEick                                         *
*   Letzte Aenderung :  15.08.2002 / <aendrg.-autor>                          *
*                                                                             *
*   Version :           REV 0.00                                              *
*                                                                             *
*   Copyright (c) 2008 Andreas Weick                    .                     *
*                      Alle Rechte vorbehalten.                               *
*                     - Confidential -                                        *
*                                                                             *
*=============================================================================*
*                                                                             *
*   Beschreibung :      Routinen für für das IR-NEC Remoteprotokoll (senden)  *
*                                                                             *
*   Besonderheiten :    <beschreibung> (Mehrere Zeilen möglich)               *
*                                                                             *
******************************************************************************/

#ifndef __IR_NEC_H                    /* Falls noch nicht vorhanden, ...     */
#define __IR_NEC_H                    /* ... Modulkennung definieren         */

/*=========================== Definitions-Dateien============================*/
/* keine */

/*================================== Makros =================================*/

// Makrodefinitionen für Bitoperationen:
//
/* keine */


/*================================= Typedefs ================================*/

/*============================= Globale Variablen ===========================*/
/* del mod-003
extern unsigned int Tik;  //' Zähler für die 96µs Ticks
extern unsigned char Byt;  // 'counter accepted bit

extern bit Repeat_flag;   // 'flag of repetition
extern bit Start_flag;    // 'flag of start condition
*/

extern bit Repeat_flag;   // 'flag of repetition
extern bit newData_flag;

extern unsigned char Address;  // 'byte of address
extern unsigned char Command;  // 'byte of command

// begin of insert mod-003
extern unsigned char Address_Sec;  // 'byte of address
extern unsigned char Command_Sec;  // 'byte of command
// end of insert mod-003

/* begin of del mod-003
extern unsigned char Address_1; // 'direct byte of address
extern unsigned char Command_1; // 'direct byte of command

extern unsigned char Address_0; //'indirect byte of address
extern unsigned char Command_0; //'indirect byte of command
extern unsigned int  Summa;
// end of del mod-003*/

/*============================ Funktions-Prototypen =========================*/

#endif /* #ifndef __IR_NEC_H */






