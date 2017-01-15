/******************************************************************************
*                                                                             *
*   file name      :    lcd.h                                                 *
*   file type      :    Header                                                *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    07.10.2008 / A.Weick                                  *
*                                                                             *
*   version        :    REV 0.03                                              *
*                                                                             *
*   Copyright (c) 2008 Andreas Weick                                          *
*                      Alle Rechte vorbehalten.                               *
*                     - Confidential -                                        *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    Bits Definitionen für LCD Routinen                    *
*                                                                             *
*   specials       :    Für die Ansteuerung des LCD's werden nur Bits von     *
*                       einem Port verwendet.                                 *
*                                                                             *
******************************************************************************/


// Hinweis: nur im HalfModde können die Pins wahlweise zugeordnet werden .
// Beim Full mode nicht

#ifndef __LCD_H
    #define __LCD_H

/*============================== include files ==============================*/
/* nothing */

/*================================== Makros =================================*/

//#############################################################################
//------------- ab hier editieren --------------------------------------------#

    #define LCD_PORT_MODE_FULL 0         // LCD Pins liegen alle komplett an einem Port
    #define LCD_PORT_MODE_HALF 1         // Datenbits und Kontrollbits liegen auf unterschiedlichen Ports
                                         // Wenn möglich FULL bevorzugen, da dies Code spart  (112 Bytes)

    #if  (LCD_PORT_MODE_FULL==1)
        #define LCD_PORT          (PORTD)  // <==== EDIT        //  Hier den Port für den LCD-Anschluss festlegen
        #define LCD_PORTDR         (DDRD)  // <==== EDIT
        #define LCD_PORTIN         (PIND)  // <==== EDIT
    #else
        #define LCD_PORT_CONTROL  (PORTB)  // <==== EDIT        // Port für die Kontrollbits vom LCD Anschluss    //insert mod-001
        #define LCD_PORTDR_CONTROL (DDRB)  // <==== EDIT
        #define LCD_PORTIN_CONTROL (PINB)  // <==== EDIT

        #define LCD_PORT_DATA     (PORTD)  // <==== EDIT        // Port für die Daten bits vom LCD Anschluss  // insert mod-001
        #define LCD_PORTDR_DATA    (DDRD)  // <==== EDIT
        #define LCD_PORTIN_DATA    (PIND)  // <==== EDIT
    #endif
                                          // Bits Definitionen vom LCD Port

    // Kontroll Bits LCD
    #define LCD_RS  (0)                   // Register Select ( Befehlsregister / Datenregister)
    #define LCD_RW  (1)                   // READ/WRITE Bit  ( 0=Lesen  , 1=schreiben)
    #define LCD_EN  (2)                   // Enable Bit   (LCD übernimmte die Daten bei positiver Flanke)

    // Datenbits LCD  festlgegen (entsprechen Ihrer Positionen im Port-Register
    //--- DB0 - DB3 sind fest von der Hardware auf 0 zugelegen ( wegen 4 Bit Modus)
    #define DB0    (0)                    // Datenbit 0
    #define DB1    (1)                    // Datenbit 1
    #define DB2    (2)                    // Datenbit 2
    #define DB3    (3)                    // Datenbit 3

    #define DB4    (4)                    // Datenbit 4
    #define DB5    (5)                    // Datenbit 5
    #define DB6    (6)                    // Datenbit 6
    #define DB7    (7)                    // Datenbit 7

//------------- ab hier nichts mehr ändern ------------------------------------#
//##############################################################################


    #if  (LCD_PORT_MODE_FULL==1) &&  (LCD_PORT_MODE_HALF==1) ||  (LCD_PORT_MODE_FULL==0) &&  (LCD_PORT_MODE_HALF==0)
        #error "LCD-Port Modus festelegen: FULL oder HALF!"
    #endif


/*================================= Typedefs ================================*/
/* nothing */

/*============================ global variables =============================*/
/* nothing */

/*======================= prototyps of global functions =====================*/
    void lcd_command(unsigned char data);                       // LCD Command im 4 Bit Modus an LCD übertragen
    void lcd_data(unsigned char data);                          // LCD Datenbyte im 4 Bit Modus an LCD übertragen
    void vInitLcd(void);                                        // LCD Display intialisieren (4 Bit Modus, 2 Zeile 7x5 Pixel)

    void vClearLCD(void);                                       // LCD löschen und Cursur auf Home setzen
    void vGotoXY_LCD(unsigned char ucX, unsigned char ucY);     // Cursor positionieren

    void vPrintStringF_LCD(flash unsigned char *ptrString);     // Zeichekette die im Flash abgelegt ist ausgeben
    void vPrintString_LCD( unsigned char *ptrString);           // Zeichekette die im RAM abgelegt ist ausgeben


#endif /* #ifndef __LCD_H */
