/******************************************************************************
*                                                                             *
*   Datei-Name :        <eeprom.h>                                            *
*   Datei-Typ :         Header                                                *
*                                                                             *
*   Autor :             Andreas WEick                                         *
*   Letzte Aenderung :  27.11.2007 / <aendrg.-autor>                          *   
*                                                                             *
*   Version :           REV 0.01                                              *
*                                                                             *
*   Copyright (c) 2007 Andreas Weick.                                         *
*                      Alle Rechte vorbehalten.                               *
*                     - Confidential -                                        *
*                                                                             *
*=============================================================================*
*                                                                             *
*   Beschreibung :      Routinen für den EEPROM zugriff                       *
*                                                                             *
*   Besonderheiten :    <beschreibung> (Mehrere Zeilen möglich)               *
*                                                                             *
******************************************************************************/

#ifndef __EEPROM_H                    /* Falls noch nicht vorhanden, ...     */
#define __EEPROM_H                    /* ... Modulkennung definieren         */

/*=========================== Definitions-Dateien============================*/
/* keine */

/*================================== Makros =================================*/

// Makrodefinitionen für Bitoperationen:
//
/* keine */

        
/*================================= Typedefs ================================*/
#ifndef _BYTE_
#define _BYTE_
    typedef unsigned char byte;   
#endif

#ifndef _WORD_
#define _WORD_
    typedef unsigned int word;   
#endif

/*============================= Globale Variablen ===========================*/
/* keine */

/*============================ Funktions-Prototypen =========================*/
#pragma used+
void vEepromWriteByte(word wAdresse, byte bData);
byte  bEepromReadByte(word wAdresse);
#pragma used-

#endif /* #ifndef __EEPROM_H */






