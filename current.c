//***************************************************************************

// Im Hauptprogramm einfügen:
 
// current
volatile static uint32_t             currentcount=0;     // Anzahl steps von timer2 zwischen 2 Impulsen. Schritt 10uS
volatile uint32_t                    totalimpulscount=0;     // Anzahl Impulse vom Stromzaehler fortlaufend seit startup, Energiezaehler


volatile static uint32_t            impulszeit=0;  // anzahl steps in INT1, wird nach div durch ANZAHLWERTE zu impulszeitsumme addiert.uebernommen
volatile  uint32_t                  integerimpulszeit=0;  // anzahl steps in INT1, fortlaufend addiert waehrend ANZAHLWERTE

volatile static float               impulszeitsumme=0;
volatile static float               impulsmittelwert=0; // Mittelwertt der Impulszeiten in einem Messintervall

// werte fuer SPI

volatile uint8_t                    impulsmittelwerthh=0; // > 16 bit
volatile uint8_t                    impulsmittelwerth=0; // hibyte
volatile uint8_t                    impulsmittelwertl=0; // lobyte

volatile uint16_t                   filtercount = 0;        // counter fuer messwerte der Filterung
volatile static float               filtermittelwert = 0;   // gleitender Mittelwert
volatile uint8_t                    filterfaktor = 4;       // Filterlaenge

volatile uint8_t                    currentstatus=0; // Byte fuer Status der Strommessung
volatile uint8_t                    webstatus =0;     // Byte fuer Ablauf der Messung/Uebertragung
volatile uint8_t                    errstatus =0;     // Byte fuer errors der Messung/Uebertragung

volatile uint8_t                    sendWebCount=0;	// Zahler fuer Anzahl TWI-Events,
                                                      // nach denen Daten des Clients gesendet werden sollen


volatile uint16_t messungcounter;


#define TEST   0

#include <avr/io.h>
#include "lcd.h"

// Endwert fuer Compare-Match von Timer2
#define TIMER2_ENDWERT12					 136; // 10 us

//#define TIMER2_ENDWERT8					 220; // 20 us
#define TIMER2_ENDWERT8                109; // 10 us

//#define TIMER2_ENDWERT  250


//#define TIMER2_ENDWERT					 203; // 10 us

#define IMPULSBIT                   4 // gesetzt wenn Interrupt0 eintrifft. Nach Auswertung im Hauptprogramm wieder zurueckgesetzt
#define NEWBIT                      5  // Gesetzt wenn SPI-Uebertragung fertig. reset wenn anzahlwerte erreicht.
#define COUNTBIT                    3 // Messung ist im gang, noch nicht ANZAHLWERTE gesammelt

#define ANZAHLWERTE                 2 // Anzahl Werte fuer Mittelwertbildung

#define ANZAHLPAKETE                8 // Anzahl Pakete bis zur Uebertragung

#define TIMERINTERVALL              10 // Takt des Zaehlers in ms

// Impulszaehler fuer Zaehlen der Impulse bei hoher frequenz(Leistung). Ev genauer als Zeitmessung zwischen Impulsen
#define INTERVALL 100000 // 100ms Intervall fuer das Zaehlen der Impulse, 36 mWh
// 

volatile static uint32_t intervallzeit=0; // in ISR von timer2 gezaehlt


volatile static uint32_t sendintervallzeit=0; // in ISR  gezaehlt, Intervall fuer Send Data

volatile uint16_t stromimpulscounter= 0; // Anz Stromimpulse im Intervall

volatile static uint32_t anzahlimpulsmittelwert=0;

//#define SENDINTERVALLCOUNT 60 // 1min

#define SENDINTERVALLCOUNT 20 // 20s

#define SENDINTERVALL 2* SENDINTERVALLCOUNT//


// mittelwerte aufsummiert
volatile static uint16_t stromimpulsmittelwertarray[4]={};
volatile uint8_t stromimpulsindex=0;


#define OSZICPORT		PORTD
#define OSZICDDR	DDRD

#define PULSC			7

#define OSZICLO OSZICPORT &= ~(1<<PULSC)
#define OSZICHI OSZICPORT |= (1<<PULSC)
#define OSZICTOGG OSZICPORT ^= (1<<PULSC)

#define CURRENTSEND                 0     // Bit fuer: Daten an Server senden
#define CURRENTSTOP                 1     // Bit fuer: Impulse ignorieren
#define CURRENTWAIT                 2     // Bit fuer: Impulse wieder bearbeiten

#define CALLBACKWAIT                7     // Bit fuer: warten auf callback
#define	IMPULSPIN                  0    // Pin fuer Anzeige Impuls

// Fehlerbits
#define  CALLBACKERR                0

//volatile uint8_t timer2startwert=TIMER2_ENDWERT;


#define DATALOOP                    7     // wird von loopcount1 gesetzt, startet senden


// SPI

//**************************************************************************
#include <avr/io.h>
#include "lcd.h"


/*
 TIMSK2=(1<<OCIE2A); // compare match on OCR2A
 TCNT2=0;  // init counter
 OCR2A=244; // value to compare against
 TCCR2A=(1<<WGM21); // do not change any output pin, clear at compare match

 */

// Timer2 fuer Atmega328

void timer2(void) // Takt fuer Strommessung
{ 
   //lcd_gotoxy(10,1);
	//lcd_puts("Tim2 ini\0");
   TCCR2A=0;
   //PRR&=~(1<<PRTIM2); // write power reduction register to zero
  
   // ***: Setting gemaess  https://withinspecifications.30ohm.com/2014/02/20/Fast-PWM-on-AtMega328/
   TIMSK2 |= (1<<OCIE2A);                 // CTC Interrupt Enable

   TCCR2A |= (1<<WGM21);                  // set OC2A on bottom, clear on comp match
   TCCR2A |= (1<<WGM20);                  // ***
   
   TCCR2A |= (1<<COM2A1);                  // CTC
   TCCR2A |= (1<<COM2B1);                  // ***
   
   /*
    CS22	CS21	CS20	Description
    0    0     0     No clock source
    0    0     1     clk/1
    0    1     0     clk/8
    0    1     1     clk/32
    1    0     0     clk/64
    1    0     1     clk/128
    1    1     0     clk/256
    1    1     1     clk/1024
    */

   //TCCR2B |= (1<<CS22); // 
   //TCCR2B |= (1<<CS21); //
   
   if(F_CPU == 8000000)
   {
     TCCR2B |= (1<<CS20);    // kein Teiler
    //TCCR2B |= (1<<CS20) | (1<<CS21) | (1<<CS22);
   }
   else
   {
      TCCR2B |= (1<<CS21);    // Teiler 8
   }
   
   TCCR2B |= (1<<WGM22);   // ***
	
   
   TIFR2 |= (1<<TOV2);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	
   //TIMSK2 |= (1<<TOIE2);						//Overflow Interrupt aktivieren
   TCNT2 = 0;                             //Rücksetzen des Timers
	//OSZILO;
   
   if(F_CPU == 8000000) // 40 kHz
   {
      OCR2A = TIMER2_ENDWERT8;   // Frequenz
   }
   else
   {
      OCR2A = TIMER2_ENDWERT12;   // Frequenz
   }
   
   OCR2B = 5; // Pulsweite
}


#pragma mark ISR TIMER2_COMPA

ISR(TIMER2_COMPA_vect) // CTC Timer2
{
   
   OSZICLO;
   currentcount++; // Zaehlimpuls
   
   //PORTB ^= (1<<0);
   
   // Zeitfenster fuer Impulszaehlung bei hohen Frequenzen 100 ms
   // Zeit messen fuer Intervall
  /*
   intervallzeit++;
   if (intervallzeit == INTERVALL) // 100ms Messintervall abgelaufen, Anzahl zum schleppenden Mittelwert anfuegen. 
   {
      intervallzeit = 0;
      stromimpulsmittelwertarray[stromimpulsindex++] = stromimpulscounter; // messung im Array fuer mittelwert einfuegen
      stromimpulscounter=0;
      stromimpulsindex &= 0x03; // :4 (Ringzaehler bis 4)
      
      sendintervallzeit++; // intervall fuer send data

      if (sendintervallzeit == SENDINTERVALL)
      {
         sendintervallzeit=0;
         webstatus |= (1<<DATALOOP);
      }
   }
   */
   OSZICHI;
}

// ISR fuer Atmega328

#pragma mark ISR INT1

ISR(INT1_vect) // Neuer Impuls vom Zaehler ist angekommen. Entspricht 1000 mWh  (vorher 360 mWh)
{
   if (TEST)
   {
      //lcd_gotoxy(0,2);
      //lcd_puts("T");
      //lcd_putint(stromimpulscounter);
      
     // webstatus |= (1<<CURRENTWAIT);
      currentstatus |= (1<<NEWBIT);
      currentstatus |= (1<<IMPULSBIT);
      
   }
   //OSZILO;
   stromimpulscounter++; // Anzahl Impulse der laufenden Messung
   
//   volatile uint8_t messungcounter;
   
   if (webstatus & (1<<CURRENTSTOP)) // Webevent im Gang, Impulse ignorieren (nicht verwendet)
   {
      //lcd_puts("st\0");
      //return;
   }
   
    // In strom_browserresult_callback gesetzt: Uebertragung ist fertig, beim naechsten Impuls Messungen wieder starten
   if (webstatus & (1<<CURRENTWAIT)) // Webevent fertig, neue Serie starten
   {
      if (TEST)
      {
       //  lcd_gotoxy(6,2);
       //  lcd_putc('i');
       //  lcd_putint16(currentcount);
      }
      webstatus &= ~(1<<CURRENTWAIT);
      TCCR2B |= (1<<CS20); // Timer wieder starten, Impuls ist Startimpuls, nicht auswerten
      currentcount =0;
      
      filtercount = 0;
      return;
   }
   
   totalimpulscount++; // Gesamtzahl der Impulse vom Zaehler fortlaufend  seit startup: Energiezaehler
   
   currentstatus |= (1<<IMPULSBIT);// Abstand bis zum naechsten Impuls messen: Bit bearbeiten in WebServer.
   
   // Zaehler fuer timerimpulse zuruecksetzen, alten Wert speichern fuer Mittelwertbildung in webserver.cb
   {
      //OSZILO;
      
      impulszeit = currentcount;
      currentcount =0;  // wird in TIMER2_COMPA_vect inkrementiert
      
      //PORTB ^= (1<<IMPULSPIN);
   }
   //OSZIHI;
}	// ISR

void InitCurrent(void) // INT1
{ 
   OSZICDDR |= (1<<PULSC);
   OSZICPORT |= (1<<PULSC);
    /*
	// interrupt on INT0 pin falling edge (sensor triggered) 
	EICRA = (1<<ISC01) | (0<<ISC00);
	// turn on interrupts!
	EIMSK  |= (1<<INT0);
*/
   // interrupt on INT1 pin falling edge 
   EICRA |= (1<<ISC11) ;//| (0<<ISC10);
	
   // turn on interrupts!
	EIMSK  |= (1<<INT1);


	//lcd_gotoxy(0,0);
	//lcd_puts("I1\0");
   
   
} 


