/******************************************************************************
*                                                                             *
*   Modul-Name :        LCD Modul                                             *
*   Datei-Name :        lcd.c                                                 *
*   Datei-Typ :         Sub                                                   *
*                                                                             *
*   Autor :             A.Weick                                               *
*   Letzte Aenderung :  22.09.2008 / A.Weick                                  *
*                                                                             *
*   Version :           REV 0.03                                              *
*                                                                             *
*   Copyright (c) 2008  A.Weick                                               *
*                       Alle Rechte vorbehalten.                              *
*                      - Confidential -                                       *
*                                                                             *
*=============================================================================*
*                                                                             *
*   Beschreibung :      LCD - Display (Text 2x16 Zeilen ansteuern             *
*                                                                             *
*   Besonderheiten :    Keine                                                 *
*                                                                             *
*   Portabilitaet :     ANSI-C                                                *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   Definitions-Datei :  lcd.h                                                *
*                                                                             *
*   Exportschnittstelle des Moduls :                                          *
*                        void lcd_command(unsigned char data);                *
*                        void lcd_data(unsigned char data);                   *
*                                                                             *
*   Benoetigte Bibliotheken und Module :                                      *
*     - ANSI-C-Standardbibliothek                                             *
*     - mega48.h                                                              *
*     - lcd.h                                                                 *
*     - delay.h                                                               *
*                                                                             *
******************************************************************************/
/*============================== Modul-History ==============================*/
/*
REV 0.01 22.09.2008 -AW-:
	 Created

todo: Port Initialsierung mit in die Init Routine packen

REV 6.10.2008  Fehler bei Halfmode behoben (RS mit explizit löschen beim Command befehlt) -AW-
REV 7.10.2008  weitere Funktionen hinzugefügt   - AW -

*/

/*=========================== Definitions-Dateien ===========================*/
#ifdef  _CHIP_ATMEGA48_
  #include <mega48.h>
  #include "inc/atmega48bit.h"
//  #warning "[ir_to_pc.c]: ATMega48  gewählt"
#endif

#ifdef  _CHIP_ATMEGA168_
  #include <mega168.h>
  #include "inc/atmega168bit.h"
//  #warning "[ir_to_pc.c]: ATMega168  gewählt"
#endif


#include <delay.h>
#include "inc/lcd.h"

/*================================= Makros ==================================*/
#define __LCD_C__
/*================================ Typedefs =================================*/

/*============================ Globale Variablen ============================*/
static unsigned char _base_y[4]={0x80,0xc0};   // Adresse für 1. und 2. Zeille rest wird nicht verwendet
static unsigned char _lcd_x=0,_lcd_y=0,_lcd_maxx;



/*================== Prototypen der dateiglobalen Funktionen ================*/
static void lcd_sendingSignal(void);                        // Strobe Signal für LCD

#if  (LCD_PORT_MODE_HALF==1)
    static unsigned char ucSetDataBits(unsigned char ucData);   // Datenbits auf den Datenausgabe Port mappen
#endif


/*================================ Datei-Code ===============================*/

/******************************************************************************
*                                                                             *
*   funktion       :    lcd_sendingSignal                                     *
*                          Strobe Signal für das LCD um die Daten zu nehmen   *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    22.09.2008 / A.Weick                                  *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    Erzeugt eine postive Flanke am LCD Enable Pin         *
*                       damit das LCD die Daten übernimmt                     *
*                                                                             *
*   specials       :    keine                                                 *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    keine                                                 *
*                                                                             *
*   return value   :    keine                                                 *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*      (M:R) -                                                                *
*                                                                             *
******************************************************************************/
static void lcd_sendingSignal(void){
    #if  (LCD_PORT_MODE_FULL==1)
          LCD_PORT         |= ( 1<< LCD_EN );     // Enable kurz einschalten     (Display übernimmt die Daten bei positiver Flanke )
    #else
          LCD_PORT_CONTROL |= ( 1<< LCD_EN );     // Enable kurz einschalten     (Display übernimmt die Daten bei positiver Flanke )
    #endif

    #asm
        nop
        nop
        nop
        nop
    #endasm

    #if  (LCD_PORT_MODE_FULL==1)
        LCD_PORT         &= ~( 1<<LCD_EN );     // Bit wieder zurücksetzen
    #else
        LCD_PORT_CONTROL &= ~( 1<<LCD_EN );     // Bit wieder zurücksetzen
    #endif

    delay_us(40);                   // Wartezeit, die ein normaler Befehl benötigt (ausser Clear Display und Return Home)
}


/******************************************************************************
*                                                                             *
*   function       :    lcd_command                                           *
*                       Befehlsbyte an LCD im 4-Bit Modus übertrage           *
*                       High Nibble zuerst, dann Low Nibble                   *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    22.09.2008 / A.Weick                                  *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    Überträgt ein Befehls Byte im 4 Bit Modus an das LCD. *
*                       (RS-Bit vom LCD ist zurückgesetzt)                    *
*                       Die Verzögerungszeit nach der Datenübertragung beträgt*
*                       40 µs.                                                *
*                                                                             *
*   specials       :    keine                                                 *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    unsigned char data                                    *
*                             - Befehlsbyte für das Display                   *                                                                            *
*   return value   :    keine                                                 *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*      (M:R) -                                                                *
*                                                                             *
******************************************************************************/

void lcd_command(unsigned char data){               // im 4-Bit Modus wird zuerst das Highnibble gesendet
    unsigned char ucLow;                            // Low Nibble vom Parameter als Highnibble übertragen

    ucLow = ( data<<4 );                            // 4 Bit Modus verwendet nur das Highnibble für die Datenübertragung
    data &= 0xF0;                                   // RS bleibt gelöscht ==> Command Instruction

    #if  (LCD_PORT_MODE_FULL==1)
        LCD_PORT=data;                              // High Nibble zuerst übertragen
    #else
        LCD_PORT_DATA  &=  ~( (1<<DB7) | (1<<DB6) |(1<<DB5) |(1<<DB4) );
        LCD_PORT_DATA  += ucSetDataBits(data);      // High Nibble zuerst übertragen
        LCD_PORT_CONTROL &= ~(1<<LCD_RS);           // mit gelöschtem RS-Bit (Daten)
    #endif

    lcd_sendingSignal();

    #if  (LCD_PORT_MODE_FULL==1)
        LCD_PORT=ucLow;                             // dann das Low Nibble übertragen
    #else
        LCD_PORT_DATA    &=  ~( (1<<DB7) | (1<<DB6) |(1<<DB5) |(1<<DB4) );
        LCD_PORT_DATA    += ucSetDataBits(ucLow); // High Nibble zuerst übertragen
        LCD_PORT_CONTROL &= ~(1<<LCD_RS);          // mit gelöschtem RS-Bit (Daten)
    #endif

    lcd_sendingSignal();
}


/******************************************************************************
*                                                                             *
*   function       :    lcd_data                                              *
*                       Datenbyte an LCD im 4-Bit Modus übertragen            *
*                       High Nibble zuerst, dann Low Nibble                   *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    22.09.2008 / A.Weick                                  *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    Überträgt ein Daten Byte im 4 Bit Modus an das LCD.   *
*                       (RS-Bit vom LCD ist gesetzt)                          *
*                       Die Verzögerungszeit nach der Datenübertragung beträgt*
*                       40 µs.                                                *
*                                                                             *
*   specials       :    keine                                                 *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    unsigned char data                                    *
*                             - Datenbyte für das Display                     *                                                                            *
*   return value   :    keine                                                 *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*      (M:R) -                                                                *
*                                                                             *
******************************************************************************/

void lcd_data(unsigned char data){                      // im 4-Bit Modus wird zuerst das Highnibble gesendet
    unsigned char ucLow;

    ucLow = (data<<4);                                  // Low Nibble nach High-Nibble zum senden maskieren
    data &= 0xF0;                                       // High Nibble maskieren

    #if  (LCD_PORT_MODE_FULL==1)
        LCD_PORT  = data;                               // High Nibble zuerst übertragen
        LCD_PORT |= (1<<LCD_RS);                        // mit gesetzem RS-Bit (Daten)
    #else
        LCD_PORT_DATA    &= ~( (1<<DB7) | (1<<DB6) |(1<<DB5) |(1<<DB4) );
        LCD_PORT_DATA    += ucSetDataBits(data);        // High Nibble zuerst übertragen
        LCD_PORT_CONTROL |= (1<<LCD_RS);                // mit gesetzem RS-Bit (Daten)
    #endif

    lcd_sendingSignal();


    #if  (LCD_PORT_MODE_FULL==1)
        LCD_PORT  = ucLow;                              // Nun das Low Nibble übertragen
        LCD_PORT |=  (1<<LCD_RS);                       // mit gesetzem RS-Bit (Daten)
    #else
        LCD_PORT_DATA    &= ~( (1<<DB7) | (1<<DB6) |(1<<DB5) |(1<<DB4) );
        LCD_PORT_DATA    += ucSetDataBits(ucLow);       // Nun das Low Nibble übertragen
        LCD_PORT_CONTROL |= (1<<LCD_RS);                // mit gesetzem RS-Bit (Daten)
    #endif
    lcd_sendingSignal();
}



/******************************************************************************
*                                                                             *
*   function       :    vInitLcd                                              *
*                       LCD Display initialisieren                            *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    22.09.2008 / A.Weick                                  *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    Initialisiert das Display mit folgenden Parameter:    *
*                       4 Bit Modus (nur DB7 - DB4 für die Datenübertragung)  *
*                       2 Zeilen sind aktiv                                   *
*                       Pixel Matrix = 7x5 Pixel                              *
*                       Cursur Position auf 0,0 setzen (blinkend)             *
*                       Entry Mode = Inkremet                                 *
*                       Display löschen und Einschalten                       *
*                                                                             *
*   specials       :    konfiguriert die Port Pins                            *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    keine                                                 *
*                                                                             *                                                                            *
*   return value   :    keine                                                 *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*      (M:R) -                                                                *
*                                                                             *
******************************************************************************/

void vInitLcd(void) {  // LCD Display intialisieren (4 Bit Modus, 2 Zeile 7x5 Pixel)
    unsigned char i=0;

    // Portpins konfigurieren
    #if  (LCD_PORT_MODE_FULL==1)
        LCD_PORT   = 0x00;
        LCD_PORTDR = ( (1<<DB7) | (1<<DB6) | (1<<DB5) | (1<<DB4) | (1<<LCD_EN) | (1<<LCD_RW) | (1<<LCD_RS)   );
    #else
        LCD_PORT_CONTROL   &= ~((1<<LCD_EN) | (1<<LCD_RW) | (1<<LCD_RS));
        LCD_PORTDR_CONTROL |=  ((1<<LCD_EN) | (1<<LCD_RW) | (1<<LCD_RS));

        LCD_PORT_DATA   &= ~((1<<DB7) | (1<<DB6) | (1<<DB5) | (1<<DB4));
        LCD_PORTDR_DATA |= ((1<<DB7) | (1<<DB6) | (1<<DB5) | (1<<DB4));
    #endif


    delay_ms(100);                                      // warten Powerup >15ms   (lieber mal etwas mehr:-)

    while(i++<3) {
        #if  (LCD_PORT_MODE_FULL==1)
            LCD_PORT = ( (1<<DB5)|(1<<DB4) );           // erstmal Init Sequenz mit 8 Bit Modus , 1 Zeile 3 mal schicken
        #else
            LCD_PORT_DATA &= ~( (1<<DB7) | (1<<DB6) |(1<<DB5) |(1<<DB4) );
            LCD_PORT_DATA +=  ( (1<<DB5) | (1<<DB4) );  // erstmal Init Sequenz mit 8 Bit Modus , 1 Zeile 3 mal schicken
        #endif

        lcd_sendingSignal();                            // es geht nur obige Konfiguration, da DB0-DB3 fest auf Masse gelegt sind.
        delay_ms(5);
    }
                                                        // Display ist nun initialisiert und kann Befehle annehmen

    #if  (LCD_PORT_MODE_FULL==1)
        LCD_PORT = (1<<DB5);                            // 4 Bit Modus aktivieren  ( 4bit Modus, 1 Zeile )
    #else
        LCD_PORT_DATA &= ~( (1<<DB7) | (1<<DB6) |(1<<DB5) |(1<<DB4) );
        LCD_PORT_DATA +=    (1<<DB5);                   // 4 Bit Modus aktivieren  ( 4bit Modus, 1 Zeile )
    #endif

    lcd_sendingSignal();
    delay_ms(5);                                        // 4 Bit Modus und 1.Zeile aktiv  ==> ab jetzt müssen alle Daten in Nibbles an das Display gesendet werden

                                                        // jetzt kann die restliche Konfiguration erfolgen (da jetzt 4 Bit Modus aktiv ist, können jetzt auch die Datenbits DB3-DB0 angesprochen werden)

    lcd_command( (1<<DB5)|(1<<DB3) );                   // Funktions-Set: 2 Zeilen, 5x7 Matrix, 4 Bit
    lcd_command( 1<<DB3 );                              // LCD off  (Cursur aus, Blinken aus)

    lcd_command(1<<DB0);                                // Clear Display
    delay_ms(2);                                        // warte bis Display gelöscht ist.

    lcd_command( (1<<DB2)|(1<<DB1) );                   // Entry Mode - Inkrement und kein Displayshift
    lcd_command( (1<<DB3)|(1<<DB2)|(1<<DB1)|(1<<DB0));  // LCD on , blinking, cursor
}




/******************************************************************************
*                                                                             *
*   function       :    vClearLCD                                             *
*                       LCD Display  löschen                                  *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    04.10.2008 / A.Weick                                  *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    Display löschen und Cursor auf Home setzen            *
*                                                                             *
*   specials       :    keine                                                 *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    keine                                                 *
*                                                                             *                                                                            *
*   return value   :    keine                                                 *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*      (M:R) -                                                                *
*                                                                             *
******************************************************************************/

void vClearLCD(void) {          // LCD löschen und Cursur auf Home setzen
    lcd_command(0x01);          // Clear Display
    delay_ms(2);                // warte bis Display gelöscht ist.
}


/******************************************************************************
*                                                                             *
*   function       :    vGotoXY_LCD                                           *
*                        Cursor an Position x,y  (max= 15, 1)                 *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    04.10.2008 / A.Weick                                  *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    Cursor positionieren, ohne Inhalte zu löschen         *
*                                                                             *
*   specials       :    keine                                                 *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    keine                                                 *
*                                                                             *
*   return value   :    keine                                                 *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*      (M:R) -                                                                *
*                                                                             *
******************************************************************************/

void vGotoXY_LCD(unsigned char ucX, unsigned char ucY) {  // Cursor positionieren,  DB6-DB0=Adresse
    lcd_command(_base_y[ucY]+ucX);
    _lcd_x=ucX; // dummy
    _lcd_y=ucY; //dummy
    delay_ms(2);
}


/******************************************************************************
*                                                                             *
*   function       :    vPrintStringF_LCD                                     *
*                        String aus dem Flash-Bereich auf LCD ausgeben        *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    04.10.2008 / A.Weick                                  *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    String aus dem Flash-Bereich auf LCD ausgeben         *
*                                                                             *
*   specials       :    keine                                                 *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    keine                                                 *
*                                                                             *
*   return value   :    keine                                                 *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*      (M:R) -                                                                *
*                                                                             *
******************************************************************************/

void vPrintStringF_LCD( unsigned char flash *ptrString) {
    while(*ptrString) {
        lcd_data(*ptrString++);
    }
}

/******************************************************************************
*                                                                             *
*   function       :    vPrintString_LCD                                      *
*                        String aus dem RAM auf LCD ausgeben                  *
*                                                                             *
*   author         :    A.Weick                                               *
*   last change    :    07.10.2008 / A.Weick                                  *
*                                                                             *
*=============================================================================*
*                                                                             *
*   description    :    String aus dem RAM-Bereich auf LCD ausgeben           *
*                                                                             *
*   specials       :    keine                                                 *
*                                                                             *
*============================== Schnittstellen ===============================*
*                                                                             *
*   parameters     :    keine                                                 *
*                                                                             *
*   return value   :    keine                                                 *
*                                                                             *
*   used global varaibles (M/G:R/W) :                                         *
*      (M:R) -                                                                *
*                                                                             *
******************************************************************************/

void vPrintString_LCD( unsigned char *ptrString) {
    while(*ptrString) {
        lcd_data(*ptrString++);
    }
}


#if  (LCD_PORT_MODE_HALF==1)
    /******************************************************************************
    *                                                                             *
    *   function       :    ucSetDataBits                                         *
    *                       Datenbits auf die Portpins vom Datenport mappen       *
    *                       (DB7--DB4)                                            *
    *                                                                             *
    *   author         :    A.Weick                                               *
    *   last change    :    04.10.2008 / A.Weick                                  *
    *                                                                             *
    *=============================================================================*
    *                                                                             *
    *   description    :    Mit Hilfe der Port-Pins Defines wird hier das         *
    *                       Daten-Nibble (DB7 - DB4) auf die entsprechenden       *
    *                       Portpins vom Datenport gemappt                        *
    *                                                                             *
    *   specials       :    keine                                                 *
    *                                                                             *
    *============================== Schnittstellen ===============================*
    *                                                                             *
    *   parameters     :    unsigned char data                                    *
    *                             - Datennibble für das Display                   *                                                                            *
    *   return value   :    unsigned char DB7-DB4 Nibble                          *
    *                                                                             *
    *   used global varaibles (M/G:R/W) :                                         *
    *      (M:R) - M                                                                *
    *                                                                             *
    ******************************************************************************/

    static unsigned char ucSetDataBits(unsigned char ucData) {
        unsigned char ucTemp;

        ucTemp=0;
        if (ucData & 0x80) ucTemp |= (1<<DB7);  // Daten Bits Zuordnen  Bit7
        if (ucData & 0x40) ucTemp |= (1<<DB6);  // Daten Bits Zuordnen  Bit6
        if (ucData & 0x20) ucTemp |= (1<<DB5);  // Daten Bits Zuordnen  Bit5
        if (ucData & 0x10) ucTemp |= (1<<DB4);  // Daten Bits Zuordnen  Bit4

        return(ucTemp);
    }
#endif