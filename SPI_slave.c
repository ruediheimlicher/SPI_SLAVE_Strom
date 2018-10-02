//***************************************************************************

// Im Hauptprogramm einfügen:
 
// SPI
 volatile uint8_t						spistatus=0;
 
#define TIMER0_STARTWERT					0x80
#define SPI_BUFSIZE							8

/*

// declared in SPI_slave
extern volatile uint8_t out_startdaten;				// Startdaten des Slave, bestimmen Art der Daten
extern volatile uint8_t outbuffer[SPI_BUFSIZE];		// Der SPI-buffer, der vom Master ausgelesen werden kann.
extern volatile uint8_t out_enddaten;					// Enddaten des Slave, Quittung fuer vollstaendige Uebertragung

extern volatile uint8_t in_startdaten;					// Startdaten vom Master, bestimmen Art der Daten
extern volatile uint8_t inbuffer[SPI_BUFSIZE];		// In-Buffer fuer die Daten vom Master
extern volatile uint8_t in_enddaten;					// Enddaten vom Master, Anzeige fuer vollstaendige Uebertragung
*/





 // 

// defines fuer webspistatus

#define SEND_SERIE_BIT			0	// unused
#define WRITE_CONFIRM_BIT		1
#define SEND_REQUEST_BIT		2
#define DATA_RECEIVE_BIT		3
#define TWI_WAIT_BIT          4 // stoppt den TWI waehrend status 0
//#define SPI_ERR_BIT				5 // deakt, in spistatus verwendet als 6
#define SPI_DATA_BIT				6 // unused
#define SPI_DATA_READY_BIT		7

#define TIMER2_WERT				0xFF

 
// Startwert, auf den Timer0 bei Overflow gesetzt wird. Je hoeher, desto kuerzer ist die Timer-Laufzeit.
// Der Wert wird bei jedem zu ladenden Byte gesetzt. Die ISR raeumt am Schluss auf, wenn etwas schief gegangen ist.

volatile uint8_t timer0startwert=TIMER0_STARTWERT;



// SPI





//**************************************************************************
#include <avr/io.h>
#include "lcd.h"




volatile uint8_t								out_startdaten;
volatile uint8_t								outbuffer[SPI_BUFSIZE];
volatile uint8_t								out_enddaten;
volatile uint8_t								out_hbdaten;
volatile uint8_t								out_lbdaten;

//extern volatile uint8_t						timer0startwert;

static volatile uint8_t						ByteCounter=0xFF;
volatile uint8_t								errCounter=0;
volatile uint8_t								TWI_errCounter=0;

static volatile uint8_t						ONCounter=0x00;
static volatile uint8_t						OFFCounter=0x00;
static volatile uint8_t						OutCounter=0x00;
static volatile uint8_t						SendOKCounter=0x00;
static volatile uint8_t						SendErrCounter=0x00;
static volatile uint8_t						IncompleteCounter=0x00;

static volatile uint8_t								in_startdaten;
volatile uint8_t								inbuffer[SPI_BUFSIZE];
volatile uint8_t								in_enddaten;
volatile uint8_t								in_hbdaten;
volatile uint8_t								in_lbdaten;

volatile uint8_t								complement=0;		//Zweiercomplement
volatile uint8_t								spi_bitcontrol;
volatile uint8_t								spi_errcount;

static volatile uint8_t						bitpos=0xFF;
static volatile uint8_t						startbitpos=0;

//

// OSZI
#ifndef OSZIAPORT
#define OSZIAPORT PORTD
#endif
#ifndef OSZIADDR
#define OSZIADDR DDRD
#endif
#ifndef PULSA
#define PULSA 6
#endif


#ifndef OSZIALO
#define OSZIALO OSZIAPORT &= ~(1<<PULSA)
#endif
#ifndef OSZIAHI
#define OSZIAHI OSZIAPORT |= (1<<PULSA)
#endif
#ifndef OSZIATOG
#define OSZIATOG OSZIAPORT ^= (1<<PULSA)
#endif

#define SPI_CONTROL_DDR			DDRD
#define SPI_CONTROL_PORT		PORTD
#define SPI_CONTROL_PORTPIN	PIND

#define SPI_CONTROL_MOSI		PORTD0	// Eingang fuer Daten vom Master
#define SPI_CONTROL_MISO		PORTD1	// Ausgang fuer Daten zum Master
#define SPI_CONTROL_SCK			PORTD2	// INT0 als Eingang fuer CLK
#define SPI_CONTROL_CS_HC		PORTD4	// CS fuer HomeCentral Master

#define SPI_INT0_DDR				DDRD
#define SPI_INT0_PORT			PORTD


// Makros:
#define IS_CS_HC_ACTIVE			SPI_CONTROL_PORTPIN & ~(1<<SPI_CONTROL_CS_HC)	// ist CS fuer HC  LO?



#define SPI_PULSAE_DELAY		70 // delay in ns
//#define DATENBREITE			8	// Anzahl Bytes in einer Serie


// defines fuer spistatus
#define ACTIVE_BIT				0
#define STARTDATEN_BIT			1
#define ENDDATEN_BIT				2
#define SUCCESS_BIT				3
#define LB_BIT						4
#define HB_BIT						5
//#define SPI_ERR_BIT				6 -> Bit 6 ist SPI_SHIFT_IN_OK_BIT
#define TWI_ERR_BIT				7

/*
Timer0 fuer Atmega8
void timer0 (void) 
{ 
// Timer fuer Exp
	//TCCR0 |= (1<<CS01);						// clock	/8
	//TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
	TCCR0 |= (1<<CS02);							// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	
	TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = TIMER0_STARTWERT;					//Rücksetzen des Timers
	OSZIALO;
}

ISR (OVERFLOW0)
{	
	spistatus &= ~(1<<ACTIVE_BIT);		// Ende der Uebertragung, Bit zuruecksetzen
	spistatus &= ~(1<<STARTDATEN_BIT);	// Ende der Uebertragung, Bit fuer Startdaten sicher zuruecksetzen
	spistatus &= ~(1<<ENDDATEN_BIT);		// Ende der Uebertragung, Bit fuer Enddaten sicher zuruecksetzen

	
	OSZIAHI ;
	TCCR0=0;
	SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
	//lcd_clr_line(1);
	//lcd_gotoxy(10,1);
	//lcd_puts("Tim0\0");
}

*/

/*
// Timer0 fuer Atmega328, in Webserver verwendet
void timer0 (void) 
{ 
// Timer fuer Exp
	//TCCR0 |= (1<<CS01);						// clock	/8
	//TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
	TCCR0A |= (1<<CS02);							// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	
	TIFR0 |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = TIMER0_STARTWERT;					//Rücksetzen des Timers
	OSZIALO;
}
*/



/*
// ISR fuer Atmega328, in Webserver verwendet
ISR (TIMER0_OVF_vect)
{	
	spistatus &= ~(1<<ACTIVE_BIT);		// Ende der Uebertragung, Bit zuruecksetzen
	spistatus &= ~(1<<STARTDATEN_BIT);	// Ende der Uebertragung, Bit fuer Startdaten sicher zuruecksetzen
	spistatus &= ~(1<<ENDDATEN_BIT);		// Ende der Uebertragung, Bit fuer Enddaten sicher zuruecksetzen

	
	OSZIAHI ;
	TCCR0A=0;
	SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
	//lcd_clr_line(1);
	//lcd_gotoxy(10,1);
	//lcd_puts("Tim0\0");
}
*/



void InitSPI_Slave(void) 
{ 

	SPI_CONTROL_DDR   |= (1<<SPI_CONTROL_MISO);		// MISO als Output
	SPI_CONTROL_PORT	|= (1<<SPI_CONTROL_MISO);		// HI
	SPI_CONTROL_DDR	&= ~(1<<SPI_CONTROL_CS_HC);	// Chip Select als Eingang
	SPI_CONTROL_PORT	|= (1<<SPI_CONTROL_CS_HC);		// HI

	SPI_CONTROL_DDR	&= ~(1<<SPI_CONTROL_MOSI);		// MOSI als Eingang
	SPI_CONTROL_PORT	|= (1<<SPI_CONTROL_MOSI);		// HI
	
	SPI_INT0_DDR &= ~(1<<SPI_CONTROL_SCK);				// INT0 als SCK Eingang
	SPI_INT0_PORT |=(1<<SPI_CONTROL_SCK);				// HI
   
   OSZIADDR |=(1<<PULSA);
   OSZIAPORT |= (1<<PULSA);

	
//	PCICR |= (1<<PCIE0);
//	PCMSK0 = (1<<PCINT0);
	_delay_us(5);
 
  
   //
	// interrupt on INT0 pin falling edge (sensor triggered) 
   EICRA |= (1<<ISC01) ;//| (0<<ISC00);
	
	// turn on interrupts!
	EIMSK  |= (1<<INT0);
	_delay_us(5);

	sei(); // Enable global interrupts

} 



// Interrupt Routine Slave Mode (interrupt controlled)
// Aufgerufen bei fallender Flanke an INT0

ISR( INT0_vect ) // Clk fuer SPI-Datenaustausch auf INT0
{
//   cli();
   //OSZILO;
   _delay_us(10);
   if (spistatus & (1<<ACTIVE_BIT))									// CS ist LO, Interrupt ist OK
   {
       if (spistatus & (1<<STARTDATEN_BIT))						// out_startdaten senden, in_startdaten laden
      {
         if (SPI_CONTROL_PORTPIN & (1<<SPI_CONTROL_MOSI))	// bit ist HI
         {
            in_startdaten |= (1<<(7-bitpos));
         }
         else																// bit ist LO
         {
            in_startdaten |= (0<<(7-bitpos));
          }
         
         // Output laden
         if (out_startdaten & (1<<(7-bitpos)))
         {
            SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
         }
         else
         {
            SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
         }
         
         
         bitpos++;
         
         if (bitpos>=8) // Byte fertig
         {
            complement=	~in_startdaten;		// Zweiercomplement aus Startdaten bestimmen fuer Enddaten
            
            spistatus &= ~(1<<STARTDATEN_BIT);					// Bit fuer Startdaten zuruecksetzen
            spistatus |= (1<<LB_BIT);								// Bit fuer lb setzen
            
            bitpos=0;
            
         }
         
      }
      
      
      //	LB
      else if (spistatus & (1<<LB_BIT))					// out_lbdaten senden, in_lbdaten laden
      {
         
         if (SPI_CONTROL_PORTPIN & (1<<SPI_CONTROL_MOSI))	// bit ist HI
         {
            in_lbdaten |= (1<<(7-bitpos));
            
            // Echo laden
            //SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
         }
         else																// bit ist LO
         {
            in_lbdaten |= (0<<(7-bitpos));
            
            // Echo laden
            //SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
         }
         
         
         // Output laden
         if (out_lbdaten & (1<<(7-bitpos)))						// bit ist HI
         {
            SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
         }
         else																// bit ist LO
         {
            SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
         }
         
         bitpos++;
         
         
         if (bitpos>=8)	// Byte fertig
         {
            spistatus &= ~(1<<LB_BIT);						// Bit fuer lb zuruecksetzen
            spistatus |= (1<<HB_BIT);						// Bit fuer hb setzen
            bitpos=0;
            spi_bitcontrol |= (1<<LB_BIT);
         }
         
      }
      //LB end
      
      //	HB
      else if (spistatus & (1<<HB_BIT))					// out_hbdaten senden, in_hbdaten laden
      {
         
         if (SPI_CONTROL_PORTPIN & (1<<SPI_CONTROL_MOSI))	// bit ist HI
         {
            in_hbdaten |= (1<<(7-bitpos));
            
            // Echo laden
            //SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
         }
         else																// bit ist LO
         {
            in_hbdaten |= (0<<(7-bitpos));
            
            // Echo laden
            //SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
         }
         
         
         // Output laden
         if (out_hbdaten & (1<<(7-bitpos)))						// bit ist HI
         {
            SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
         }
         else																// bit ist LO
         {
            SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
         }
         
         bitpos++;
         
         if (bitpos>=8)	// Byte fertig
         {
            spistatus &= ~(1<<HB_BIT);						// Bit fuer hb zuruecksetzen, anschliessend kommt data-array
            
            bitpos=0;
            spi_bitcontrol |= (1<<HB_BIT);
         }
         
      }
      //HB end
      
      
      
      
      else if (spistatus & (1<<ENDDATEN_BIT))					// out_enddaten senden, in_enddaten laden
      {
         if (SPI_CONTROL_PORTPIN & (1<<SPI_CONTROL_MOSI))	// bit ist HI
         {
            in_enddaten |= (1<<(7-bitpos));
            
            // Echo laden
            //SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
         }
         else																// bit ist LO
         {
            in_enddaten |= (0<<(7-bitpos));
            
            // Echo laden
            //SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
         }
         
         
         // Output laden
         //			if (out_enddaten & (1<<(7-bitpos)))						// bit ist HI
         if (complement & (1<<(7-bitpos)))						// bit ist HI
         {
            SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
         }
         else																// bit ist LO
         {
            SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
         }
         
         
         bitpos++;
         
         
         if (bitpos>=8)	// Byte fertig
         {
            bitpos=0;
            if (out_startdaten + in_enddaten==0xFF)
            {
               //lcd_putc('+');
               //spistatus |= (1<<SUCCESS_BIT);					// Datenserie korrekt geladen
               
            }
            else
            {
               //lcd_putc('-');
               spistatus &= ~(1<<SUCCESS_BIT);					// Datenserie nicht korrekt geladen
               errCounter++;
            }
            spi_bitcontrol |= (1<<ENDDATEN_BIT);
            
            // 24.6.2010
            //				out_startdaten=0xC0; ergab nicht korrekte Pruefsumme mit in_enddaten
            
            //18.7.10
            //				out_hbdaten=0;
            //				out_lbdaten=0;
            
         }
         
      }
      
      else			// Datenarray in inbuffer laden, Daten von outbuffer senden
         
      {
         
         
         if (SPI_CONTROL_PORTPIN & (1<<SPI_CONTROL_MOSI))	// bit ist HI
         {
            
            inbuffer[ByteCounter] |= (1<<(7-bitpos));
            
            // Echo laden
            //SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
         }
         else																// bit ist LO
         {
            inbuffer[ByteCounter] |= (0<<(7-bitpos));
            
            // Echo laden
            //SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
         }
         
         
         // Output laden
         if (outbuffer[ByteCounter] & (1<<(7-bitpos)))		// bit ist HI
         {
            SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
         }
         else																// bit ist LO
         {
            SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
         }
         
         
         bitpos++;
         
         if (bitpos>=8) // Byte fertig
         {
            if (ByteCounter<SPI_BUFSIZE -1 )						// zweitletztes Byte
            {
               ByteCounter++;
            }
            else 
            {
               spistatus |= (1<<ENDDATEN_BIT);					// array fertig, Bit fuer Enddaten setzen
            
               // Array fertig
               spi_bitcontrol |= (1<<SPI_DATA_BIT);
            }
            bitpos=0;
            // control
            
         }	// if bitpos
      }						//  Datenarray in inbuffer laden, Daten von outbuffer senden
   
   }						// if (spistatus & (1<<ACTIVE_BIT))
   sei();
   //OSZIHI;
}		// ISR



