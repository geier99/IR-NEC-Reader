/******************************************************************************
*                                                                             *
*   Datei-Name :        makros.h                                              *
*   Datei-Typ :         Header                                                *
*                                                                             *
*   Autor :             A.Weick                                               *
*   Letzte Aenderung :  20.07.2007 / <aendrg.-autor>                          *
*                                                                             *
*   Version :           REV 0.01                                              *
*                                                                             *
*   Copyright (c) 2007 Andreas Weick.                                         *
*                      Alle Rechte vorbehalten.                               *
*                     - Confidential -                                        *
*                                                                             *
*=============================================================================*
*                                                                             *
*   Beschreibung :      <beschreibung> (Mehrere Zeilen möglich)               *
*                                                                             *
*   Besonderheiten :    <beschreibung> (Mehrere Zeilen möglich)               *
*                                                                             *
******************************************************************************/

#ifndef __MAKROS_H                    /* Falls noch nicht vorhanden, ...     */
#define __MAKROS_H                    /* ... Modulkennung definieren         */

/*=========================== Definitions-Dateien============================*/
/* keine */

/*================================== Makros =================================*/

// Makrodefinitionen für Bitoperationen:
//
#define SETBIT(ADRESS,BIT)    (ADRESS  |=  (1<<BIT))       
#define CLRBIT(ADRESS,BIT)    (ADRESS  &=  ~(1<<BIT))
#define COMPBIT(ADRESS,BIT)   (ADRESS  ^=  (1<<BIT))
#define CHECKBIT(ADRESS,BIT)  (ADRESS  &   (1<<BIT))            // Ergebnis ist das zu prüfende Bit   !!!
#define CHECKBIT2(ADRESS,BIT) ((ADRESS &   (1<<BIT)) >> BIT )   // 0x01 wenn Bit gesetzt war, 




/*============================= Globale Variablen ===========================*/

//#define TRUE  1
//#define FALSE 0

// #define TRUE (1==1)
// #define FALSE (1!=1)

#define CAR_TYPE_MERCEDES   1  // für die CarType Erkennung wird der Eingang EXT01 verwendet, im Merceds Kabelsatzt wird der Eingang auf +12V gelegt
#define CAR_TYPE_BMW        0  // im BMW Kabelsatzt wird der Eingang EXT01 auf Masse gelegt

#define TIMER0_CNT  (0xFD)    // == 96µs

        
/*================================= Typedefs ================================*/

#ifndef _BYTE_
#define _BYTE_
    typedef unsigned char byte;   
#endif

#ifndef _WORD_
#define _WORD_
    typedef unsigned int word;   
#endif



#ifndef _BOOL_
#define _BOOL_
    typedef enum {
                     FALSE=0
                    ,TRUE
                 } boolean_t;

#endif



typedef enum {
                BMW=0
,               MERCEDES
             } CAR_TYPE;




/*============================ Funktions-Prototypen =========================*/
/* keine */      


                                      





//------------------- USART-------------------------------


// Makros für den Schreibzugriff auf UCSRC und UBRRH, da beide Register die gleiche IO-Adresse haben
//
#define  SET_UCSRC(DATA)  ( UCSRC= (DATA) | 0x80)
#define  SET_UBRRH(DATA)  ( UBRRH= (DATA) & 0x7F)


// Makros zum Lesen von UCSRC und UBRHH
// noch nicht umgesetzt, da dies selten benötigt wird. s. Datenblatt Seite 157











#endif /* #ifndef __MAKROS_H*/


        




