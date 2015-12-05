/******************************************************************************
*                                                                             *
*   Modul-Name :        EEPROM  modul                                         *
*   Datei-Name :        eeprom.c                                              *
*   Datei-Typ :         Sub                                                   *
*                                                                             *
*   Autor :             A.Weick                                               *
*   Letzte Aenderung :  27.11.2007 / A.Weick                                  *
*                       19.08.2008 / atmega48 eingefügt - weick   (mod-001)   *
*                       27.09.2008 / atmega168 eingefügt - weick  (mod-002)   *
*                                                                             *
*   Version :           REV 0.03                                              *
*                                                                             *
*   Copyright (c) 2007 Andreas Weick.                                         *
*                      Alle Rechte vorbehalten.                               *
*                     - Confidential -                                        *
*                                                                             *
*=============================================================================*
*                                                                             *
*   Beschreibung :      Funktionen für den EEPROM Zugriff                     *
*                                                                             *
*   Besonderheiten :    Keine                                                 *
*                                                                             *
*   Portabilitaet :     ANSI-C                                                *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   Definitions-Datei :  eeprom.h                                             *
*                                                                             *
*   Exportschnittstelle des Moduls :                                          *
*      <funktionsliste>                                                       *
*                                                                             *
*   Benoetigte Bibliotheken und Module :                                      *
*     - ANSI-C-Standardbibliothek                                             *
*     - makros.h 
*     - atmegaxx8.h                                                         *
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
#include "/inc/eeprom.h"  
#include "/inc/makros.h" 

#ifdef _CHIP_ATMEGA48_
  #include <mega48.h>
  #include "inc/atmega48bit.h"
#else
  #ifdef   _CHIP_ATMEGA168_
    #include <mega168.h>
    #include "inc/atmega168bit.h"
 #else
   #include "/inc/attiny26bit.h"
   #include <tiny26.h>  
 #endif
#endif


/*================================= Makros ==================================*/
#define __EEPROM_C__
/*================================ Typedefs =================================*/

/*============================ Globale Variablen ============================*/

/*================== Prototypen der dateiglobalen Funktionen ================*/


/*================================ Datei-Code ===============================*/

/******************************************************************************
*                                                                             *
*   funktion       :    vEepromWriteByte                                      *
*                          Byte in EEProm-Adresse ablegen                     *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    27.11.2007 / A.Weick                                  *  
*                       19.08.2008 / atmega48 eingefügt - weick   (mod-001)   *
*                       27.09.2008 / atmega168 eingefügt - weick  (mod-002)   *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    Datenbyte an EEPROM-Adresse speichern                 *
*                                                                             *
*   specials       :    none                                                  *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    word wAdresse                                         *
*                          EEPROM-Adresse für das Datenbyte                   *
*                       byte bData                                            *
*                          Datenbyte welches im EEPROM abgelegt werden soll   *
*                                                                             *
*   return value   :    none                                                  *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*                                                                             *
*                                                                             *
******************************************************************************/


 void vEepromWriteByte(word wAdresse, byte bData) {
#ifdef _CHIP_ATMEGA48_
    while (CHECKBIT(EECR,EEPE)) ;       // Warten bis 0 => es steht kein Write an, somit darf gelesen werden
    EEAR=wAdresse;
    EEDR=bData;
    #asm("cli")
    SETBIT(EECR,EEMPE);
    SETBIT(EECR,EEPE);
    #asm("sei")

#else
  #ifdef _CHIP_ATMEGA168_
    while (CHECKBIT(EECR,EEPE)) ;       // Warten bis 0 => es steht kein Write an, somit darf gelesen werden
    EEAR=wAdresse;
    EEDR=bData;
    #asm("cli")
    SETBIT(EECR,EEMPE);
    SETBIT(EECR,EEPE);
    #asm("sei")

  #else
    while (CHECKBIT(EECR,EEWE)) ;       // Warten bis 0 ==> Bereit zum Daten schreiben

    EEAR=wAdresse;
    EEDR=bData;
    #asm("cli")
    SETBIT(EECR,EEMWE);
    SETBIT(EECR,EEWE);
    #asm("sei")
  #endif    
#endif
}





/******************************************************************************
*                                                                             *
*   funktion       :    bEepromReadByte                                       *
*                          Byte aus dem EEPROM auslesen                       *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    27.11.2007 / A.Weick                                  *
*                       19.08.2008 / atmega48 eingefügt - weick   (mod-001)   *
*                       27.09.2008 / atmega168 eingefügt - weick  (mod-002)   *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    Datenbyte aus EEPROM-Adresse auslesen                 *
*                                                                             *
*   specials       :    none                                                  *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    word wAdresse                                         *
*                          EEPROM-Adresse für das Datenbyte                   *
*                                                                             *
*   return value   :    byte                                                  *
*                          ausgelesenes Datenbyte aus EEPROM                  *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*                                                                             *
*                                                                             *
******************************************************************************/

byte  bEepromReadByte(word wAdresse) {    

#ifdef _CHIP_ATMEGA48_ 
    while (CHECKBIT(EECR,EEPE)) ;       // Warten bis 0 => es steht kein Write an, somit darf gelesen werden
#else
  #ifdef _CHIP_ATMEGA168_ 
    while (CHECKBIT(EECR,EEPE)) ;       // Warten bis 0 => es steht kein Write an, somit darf gelesen werden
  #else
    while (CHECKBIT(EECR,EEWE)) ;       // Warten bis 0 => es steht kein Write an, somit darf gelesen werden
  #endif
#endif

    EEAR=wAdresse;
    SETBIT(EECR,EERE);                  // Byte lesen

    return  EEDR;                     
}





