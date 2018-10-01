//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <string.h>
#include <ctype.h>

#include "lcd.c"
#include "current.c"
#include "adc.c"
#include "SPI_slave.c"
//***********************************
//Strom                    *
//									*
//***********************************



#define STARTDELAYBIT	0
#define HICOUNTBIT		1

#define WDTBIT			7


#define STROMPORT	PORTD		// Eingang fuer Stromimpuls
#define STROMDDR	DDRD

// Definitionen Slave Strom


#define LOOPLEDPORT		PORTB
#define LOOPLEDDDR		DDRB

// Define fuer Slave:
#define LOOPLED			0


#define TASTE1		38
#define TASTE2		46
#define TASTE3		54
#define TASTE4		72
#define TASTE5		95
#define TASTE6		115
#define TASTE7		155
#define TASTE8		186
#define TASTE9		205
#define TASTEL		225
#define TASTE0		235
#define TASTER		245
#define TASTATURPORT PORTC

#define TASTATURPIN        3
#define POTPIN             0
#define BUZZERPIN          0
#define ECHOPIN            5           //	Ausgang fuer Impulsanzeige

//#define buffer_size        8

uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset


//volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[buffer_size];


static volatile uint8_t SlaveStatus=0x00; //status


void delay_ms(unsigned int ms);

static volatile uint8_t paketcounter=0;
#define TIMERIMPULSDAUER            10    // us

#define CURRENTSEND                 0     // Daten an server senden
#define DATASEND                    4     // Bit fuer: Daten enden
#define DATAOK                      5     // Data kan gesendet weren
#define DATAPEND                    6     // Senden ist pendent
#define DATALOOP                    7     // wird von loopcount1 gesetzt, startet senden

volatile uint16_t                   wattstunden=0;
volatile uint16_t                   kilowattstunden=0;
volatile uint32_t                   webleistung=0;

volatile float leistung =1;
float lastleistung =1;
uint8_t lastcounter=0;
volatile uint8_t  anzeigewert =0;

static char stromstring[10];
//static char CurrentDataString[64];

// defines fuer BUS-Status
#define SPI_SHIFT_IN_OK_BIT	6


#define SPI_SENDBIT				0

#define TCR1A_SET  TCCR1B |= (1<<CS10)
#define TCR1A_STOP  TCCR1B &= ~(1<<CS10)

void delay_ms(unsigned int ms);

uint8_t OG1status=0x00;


void RingD2(uint8_t anz)
{
	uint8_t k=0;
	for (k=0;k<2*anz;k++)
	{
		PORTD |=(1<<BUZZERPIN);
		_delay_ms(2);
		PORTD &=~(1<<BUZZERPIN);
		_delay_ms(2);
		
	}
	PORTD &=~(1<<BUZZERPIN);
}


uint8_t Tastenwahl(uint8_t Tastaturwert)
{
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}



void SPI_slaveinit(void)
{
	
   // LED
   LOOPLEDDDR |=(1<<LOOPLED);
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD

	SlaveStatus=0;

   // SPI
   // interrupt on INT1 pin falling edge
 //  EICRA |= (1<<ISC01) ;//| (0<<ISC10);
   
   // turn on interrupts!
 //  EIMSK  |= (1<<INT0);
   
   
   /*	
	DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC0); //Pull-up
	DDRC &= ~(1<<DDC1);	//Pin 1 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC1); //Pull-up
	DDRC &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC3); //Pull-up
 */
	DDRC &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
//	PORTC |= (1<<DDC3); //Pull-up


	
	
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

void timer0 (void) 
{ 
// Timer fuer Exp
//	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
//	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	
//Timer fuer Servo	
	TCCR0B |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
	
	TIFR0 |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);			//Overflow Interrupt aktivieren
	TCNT0 = 0x00;					//RŸcksetzen des Timers
	
}

void timer1(void) // Instrument-Anzeige PWM
{
   TCCR1A |= (1<<COM1A1);
   //TCCR1A |= (1<<COM1B1);
//   TCCR1A |=  (1<<WGM10);
   TCCR1A |=  (1<<WGM11);
   
    TCCR1B |=  (1<<WGM12);
    TCCR1B |=  (1<<WGM13);
   
//   TCCR1B |= (1<<CS10);
//   TCCR1B |= (1<<CS12);
   TCR1A_SET;
   
   ICR1 = 0xF000;
   OCR1A = 0x01;
   
   DDRB |= (1<<1);
 //  TIMSK1 |= OCIE1A;
}


ISR(TIMER1_COMPA_vect)
{
   //OCR1A = 0xFFFF - (uint16_t)(impulsmittelwert+0.5); // float to uint16
   OCR1A = webleistung;
   //OCR1A = 0xAFFF;
}


void timer1_set(int on)
{
   if (on)
   {
      TCR1A_SET;
   }
   else
   {
      TCR1A_STOP;
   }
}


void str_cpy(char *ziel,char *quelle)
{
   uint8_t lz=strlen(ziel); //startpos fuer cat
   //printf("Quelle: %s Ziellaenge: %d\n",quelle,lz);
   uint8_t lq=strlen(quelle);
   //printf("Quelle: %s Quelllaenge: %d\n",quelle,lq);
   uint8_t i;
   for(i=0;i<lq;i++)
   {
      //printf("i: %d quelle[i]: %c\n",i,quelle[i]);
      ziel[i]=quelle[i];
   }
   lz=strlen(ziel);
   ziel[lz]='\0';
}

void str_cat(char *ziel,char *quelle)
{
   uint8_t lz=strlen(ziel); //startpos fuer cat
   //printf("Quelle: %s Ziellaenge: %d\n",quelle,lz);
   uint8_t lq=strlen(quelle);
   //printf("Quelle: %s Quelllaenge: %d\n",quelle,lq);
   uint8_t i;
   for(i=0;i<lq;i++)
   {
      //printf("i: %d quelle[i]: %c\n",i,quelle[i]);
      ziel[lz+i]=quelle[i];
      
   }
   //printf("ziel: %s\n",ziel);
   lz=strlen(ziel);
   ziel[lz]='\0';
}


// http://stackoverflow.com/questions/122616/how-do-i-trim-leading-trailing-whitespace-in-a-standard-way
char *trimwhitespace(char *str)
{
   char *end;
   
   // Trim leading space
   while(isspace(*str)) str++;
   
   if(*str == 0)  // All spaces?
      return str;
   
   // Trim trailing space
   end = str + strlen(str) - 1;
   while(end > str && isspace(*end)) end--;
   
   // Write new null terminator
   *(end+1) = 0;
   
   return str;
}


/*
 http://www.lothar-miller.de/s9y/archives/25-Filter-in-C.html
 Filter in C
 
 Ein Filter in der Art eines RC-Filters (PT1-Glied) kann in einem uC relativ leicht implementiert werden. Dazu bedarf es, wie beim RC-Glied eines "Summenspeichers" (der Kondensator) und einer Gewichtung (Widerstand bzw. Zeitkonstante).
 */
/*
 unsigned long mittelwert(unsigned long newval)
 {
 static unsigned long avgsum = 0;
 // FilterlŠngen in 2er-Potenzen --> Compiler optimiert
 avgsum -= avgsum/128;
 avgsum += newval;
 return avgsum/128;
 }
 */
/*
 Etwas spannender wird es, wenn die Initialisierung des Startwertes nicht so lange dauern soll. Wenn ich beispielsweise nach dem 1. Messwert diesen Wert und ab dem 2. Messwert bis zur Filterbreite den Mittelwert aus allen Messungen haben mšchte, dann ist eine andere Behandlung der Initialisierung nštig.
 */
unsigned long mittelwert(unsigned long newval,int faktor)
{
   static short n = 0;
   static unsigned long avgsum = 0;
   if (n<faktor)
   {
      n++;
      avgsum += newval;
      return avgsum/n;
   }
   else
   {
      // Konstanten kann der Compiler besser optimieren
      avgsum -= avgsum/faktor;
      avgsum += newval;
      return avgsum/faktor;
   }
}

unsigned long gleitmittelwert(unsigned long newval,int faktor)
{
   static short n = 0;
   static unsigned long avgsum = 0;
   if (n<faktor)
   {
      n++;
      avgsum += newval;
      return avgsum/n;
   }
   else
   {
      // Konstanten kann der Compiler besser optimieren
      avgsum -= avgsum/faktor;
      avgsum += newval;
      return avgsum/faktor;
      
   }
}

volatile uint8_t testwert=0;

int main (void)
{
   /*
    in Start-loop in while
    init_twi_slave (SLAVE_ADRESSE);
    sei();
    */
   
   wdt_disable();
   MCUSR &= ~(1<<WDRF);
   //WDTCR |= (1<<WDCE) | (1<<WDE);
   WDTCSR = 0x00;
   
   SPI_slaveinit(); // SPI mit TWI_Slave WS
   //PORT2 |=(1<<PC4);
   //PORTC |=(1<<PC5);
   
   /* initialize the LCD */
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   
   lcd_puts("Guten Tag\0");
   delay_ms(1000);
   lcd_cls();
   
   
   delay_ms(1000);
   
   lcd_clr_line(0);
   lcd_puts("Strom");
   uint8_t Tastenwert=0;
   uint8_t TastaturCount=0;
   /*
    uint8_t Servowert=0;
    uint8_t Servorichtung=1;
    
    uint16_t TastenStatus=0;
    uint16_t Tastencount=0;
    uint16_t Tastenprellen=0x01F;
    uint8_t Schalterposition=0;
    */
   //timer0();
   
   //initADC(TASTATURPIN);
   //wdt_enable(WDTO_2S);
   
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;
   uint16_t sekunde=0;
   uint16_t startdelay0=0x01FF;
   //uint16_t startdelay1=0;
   
   uint8_t SPI_Call_count0=0;
   
   //uint8_t twierrcount=0;
   LOOPLEDDDR |=(1<<LOOPLED);
   
   delay_ms(800);
   //eeprom_write_byte(&WDT_ErrCount0,0);
   //	uint8_t eepromWDT_Count0=eeprom_read_byte(&WDT_ErrCount0);
   //	uint8_t eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1);
   
   timer1();
   timer2();
   InitCurrent();
   InitSPI_Slave();
   /*
    Bit 0: 1 wenn wdt ausgelšst wurde
    
    */
   sei();
   uint8_t i=0;
   
   while (1)
   {
      //Blinkanzeige
      loopcount0++;
      if (loopcount0==0x8F00)
      {
         loopcount0=0;
        // LOOPLEDPORT ^=(1<<LOOPLED);
         //delay_ms(10);
         loopcount1++;
         if ((loopcount1 & 0x0A)==0)
         {
            LOOPLEDPORT ^=(1<<LOOPLED);
            uint8_t t=sekunde & 0x0FF;
            //lcd_gotoxy(16,0);
            //lcd_putint(t);
            
            sekunde++;
            /*
             lcd_gotoxy(0,3);
             lcd_puts("I0:\0");
             t=impulscount & 0xFF;
             lcd_putint12((t));
             */
            //loopcount1=0;
         }
         
      }
      
      
      //**	Beginn Current-Routinen	***********************
      
      if (currentstatus & (1<<NEWBIT)) // SPI fertig, neue Strommessung anfangen
      {
         
         // schon eine Serie angefangen?
         if (!(currentstatus & (1<<COUNTBIT)))
         {
            messungcounter=0;
            currentstatus |= (1<<COUNTBIT); // neue Messung starten
         }
         
         //     inbuffer[0]=0;
         //     inbuffer[1]=0;
         //     inbuffer[2]=0;
         // test ohne current
         //if (TEST)
         {
            //           inbuffer[0]=0;
            //           inbuffer[1]=0;
            //           inbuffer[2]=0;
            
            
            //          outbuffer[0] = testwert;
            //outbuffer[1] = testwert;
            //outbuffer[2] = testwert;
            out_startdaten = 0xB1;
            //testwert++;
            
            //currentstatus &= ~(1<<NEWBIT);
            
            if ((currentstatus & (1<<IMPULSBIT)) && (currentstatus & (1<<COUNTBIT))) // neuer Impuls angekommen, Zaehlung lauft noch, noch nicht genuegend werte
            {
               //OSZILO;
               inbuffer[0]=0;
               inbuffer[1]=0;
               inbuffer[2]=0;
               in_startdaten=0;
               out_startdaten = 0xB1;
               //OSZILO;
               //currentstatus++; // ein Wert mehr
               messungcounter ++;
               //lcd_gotoxy(0,3);
               //lcd_putint(currentstatus & 0x0F);
               impulszeitsumme += (impulszeit/ANZAHLWERTE);      // float, Wert aufsummieren
               
               if (messungcounter >= ANZAHLWERTE)      // genuegend Werte
               {
                  cli(); // Uebertragung nicht stoeren durch INT0
                  paketcounter++;
                  
                  //outbuffer[0] = testwert;
                  //outbuffer[1] = testwert;
                  //outbuffer[2] = testwert;
                  out_startdaten = 0xB1;
                  
                  //impulsmittelwert = rand();
                  
                  // impulsmittelwert uebernehmen
                  //impulsmittelwert = 12*impulszeitsumme/8;
                  impulsmittelwert = impulszeitsumme;
                  
                  // Leistung berechnen
                  
                  if (impulsmittelwert)
                  {
                     if (F_CPU==8000000)
                     {
                        // impulsmittelwert*= 3;
                        impulsmittelwert/= 4;
                     }
                   //  Impuls vom alten Zaehler entspricht 360 mWh
                  //   leistung =(uint32_t) 360.0/impulsmittelwert*10000.0;// 480us
                     
                     // 1000 imp == 1 kWh
                     // 1 imp = 1 wh = 1000 mWh
                     leistung =(uint32_t) 1000.0/impulsmittelwert*10000.0;// 480us
                     
                     // webleistung = (uint32_t)360.0/impulsmittelwert*1000000.0;
                     //webleistung = (uint32_t)360.0/impulsmittelwert*10000.0;
                     webleistung = (uint32_t)1000.0/impulsmittelwert*10000.0;
                     
                     lcd_gotoxy(0,1);
                     lcd_putc('L');
                     lcd_putc(':');
                     // lcd_putint16(leistung);
                     //lcd_putc(' ');
                     //lcd_putint12(webleistung);
                     //lcd_putc('W');
                     
                     //void lcd_put_frac(char* string, uint8_t start, uint8_t komma, uint8_t frac)
                     dtostrf(webleistung,10,0,stromstring); // 800us
                     
                     // lcd_putc('*');
                     
                     char*  defstromstring = (char*)trimwhitespace(stromstring);
                     uint8_t l=strlen(defstromstring);
                     lcd_gotoxy(2,1);
                     lcd_puts("    ");
                     lcd_gotoxy(6-l,1);
                     //lcd_putint16(leistung);
                     lcd_puts(trimwhitespace(defstromstring));
                     lcd_putc('W');
                     
                  }
                  wattstunden = impulscount/10; // 310us
                  
                  
                  // timer1 setzen
                  
                  //OCR1A = (uint16_t)(impulsmittelwert+0.5); // float to uint16
                  OCR1A = 4*webleistung;
                  // summe resetten
                  impulszeitsumme = 0;
                  
                  messungcounter=0; //
                  
                  // Daten fuer SPI setzen
                  outbuffer[0] = ((uint32_t)impulsmittelwert & 0xFF); // L
                  outbuffer[1] = ((uint32_t)impulsmittelwert>>8) & 0xFF; // H
                  outbuffer[2] = ((uint32_t)impulsmittelwert>>16) & 0xFF; // HH
                  
                  // Messung vollstaendig, currentstatus zuruecksetzen
                  currentstatus &= ~(1<<IMPULSBIT);
                  currentstatus &= ~(1<<COUNTBIT);
                  currentstatus &= ~(1<<NEWBIT);
                  
                  
                  
                  
                  
                  char mittelwertstring[10];
                  dtostrf(impulsmittelwert,8,0,mittelwertstring);
                  lcd_gotoxy(8,0);
                  lcd_puts("     ");
                  lcd_gotoxy(6,0);
                  lcd_putc('m');
                  lcd_putc(':');
                  lcd_puts(trimwhitespace(mittelwertstring));
                  lcd_putc(' ');
                  lcd_putint12(impulsmittelwert);
                  lcd_gotoxy(8,1);
                  if (TEST == 0)
                  {
                  lcd_putc('E');
                  lcd_putc(':');
                  lcd_putint(wattstunden/1000);
                  lcd_putc('.');
                  lcd_putint3(wattstunden);
                  lcd_putc('W');
                  lcd_putc('h');
                  }
                  
                  sei();
                  
                  
                  
               } // if genuegend Werte
               
               //         impulszeit=0;
               //              currentstatus &= ~(1<<IMPULSBIT);
               //OSZIHI;
            }// if IMPULSBIT
            
            continue;
         }
         
         // end test ohne current
         /*
          else  if (currentstatus & (1<<IMPULSBIT)) // neuer Impuls angekommen, Zaehlung lauft
          {
          
          }
          //
          */
         testwert++;
      } // if (currentstatus & (1<<NEWBIT))
      
      //**    End Current-Routinen*************************
      
#pragma mark SPI
      
      /* *** SPI begin **************************************************************/
      
      //lcd_gotoxy(19,0);
      //lcd_putc('-');
      
      // ***********************
      if (SPI_CONTROL_PORTPIN & (1<< SPI_CONTROL_CS_HC)) // CS ist HI, SPI ist Passiv,
      {
         // ***********************
         /*
          Eine Uebertragung hat stattgefunden.
          Die out-Daten sind auf dem HomeCentral-Slave.
          Die in-Daten vom HomeCentral-Slave sind geladen.
          */
         
         // ***********************
         //        SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO); // MISO ist HI in Pausen
         
#pragma mark PASSIVE
         
         if (spistatus &(1<<ACTIVE_BIT)) // ACTIVE_BIT noch HI. Slave ist erst neu passiv geworden. Aufraeumen, Daten uebernehmen
         {
            
            // wenn noch nicht zurueckgeetzt: Strommessung ausschalten
            currentstatus &= ~(1<<IMPULSBIT);
            currentstatus &= ~(1<<COUNTBIT);
            currentstatus &= ~(1<<NEWBIT);
            impulszeitsumme = 0;
            impulszeit = 0;
            messungcounter=0;
            // outbuffer belassen
            
            /*
             lcd_gotoxy(15,3);
             lcd_puts("   ");
             lcd_gotoxy(5,0);
             lcd_puts("   ");
             */
            
            SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO); // MISO ist HI in Pausen
            
            // timer1 wieder ein
            timer1_set(1);
            
            wdt_reset();
            SPI_Call_count0++;
            // Eingang von Interrupt-Routine, Daten von HomeCentral-Slave
            lcd_gotoxy(19,0);
            // lcd_putc(' ');
            
               
            
            testwert++;
            if (TEST)
            {
               lcd_gotoxy(0,2);
               lcd_puts("oM \0");
               //lcd_putc('*');
               lcd_puthex(out_startdaten);
               lcd_putc(' ');
               lcd_putint(outbuffer[0]);
               lcd_putc(' ');
               lcd_putint(outbuffer[1]);
               lcd_putc(' ');
               lcd_putint(outbuffer[2]);
               
               //            lcd_gotoxy(15,0);
               //            lcd_putc('c');
               //            lcd_putint(SPI_Call_count0);
               
               lcd_gotoxy(0,3);
               lcd_puts("iM \0");
               //lcd_putc('i');
               lcd_puthex(in_startdaten);
               
               lcd_putc(' ');
               lcd_putint(inbuffer[0]);
               lcd_putc(' ');
               lcd_putint(inbuffer[1]);
               lcd_putc(' ');
               lcd_putint(inbuffer[2]);
               
               
               
            }
            
            if (in_startdaten == 0)
            {
               spi_errcount++;
            }
            lcd_gotoxy(10,1);
            lcd_putc('e');
            lcd_putint(spi_errcount);
            lcd_putc('c');
            lcd_putint(SPI_Call_count0);
            
            //lcd_gotoxy(16,1);
            //lcd_putint(SendErrCounter);
            
            /*
             lcd_puthex(in_startdaten);
             lcd_putc(' ');
             lcd_puthex(in_hbdaten);
             lcd_puthex(in_lbdaten);
             lcd_putc(' ');
             */
            uint8_t j=0;
            // for (j=0;j<3;j++)
            {
               //lcd_putc(' ');
               //lcd_puthex(outbuffer[j]);
               // lcd_putint(outbuffer[j]);
            }
            OutCounter++;
            
            // Uebertragung pruefen
            
            //lcd_gotoxy(6,0);
            //lcd_puts("bc:\0");
            //lcd_puthex(ByteCounter);
            
            //lcd_gotoxy(0,0);
            //lcd_puts("      \0");
            
            lcd_gotoxy(19,0);
            lcd_putc(' ');
            if (ByteCounter == SPI_BUFSIZE-1) // Uebertragung war vollstaendig
            {
 //              lcd_gotoxy(15,3);
 //              lcd_puts("   ");

               if (out_startdaten + in_enddaten==0xFF)
               {
                  
                //  lcd_gotoxy(15,3);
                //  lcd_puts("    ");
                  lcd_gotoxy(19,3);
                  lcd_putc('+');
                  spistatus |= (1<<SUCCESS_BIT); // Bit fuer vollstaendige und korrekte  Uebertragung setzen
                  
                  //lcd_gotoxy(19,0);
                  //lcd_putc(' ');
                  //lcd_clr_line(3);
                  //lcd_gotoxy(0,1);
                  //lcd_puthex(loopCounterSPI++);
                  //lcd_puts("OK \0");
                  
                  //lcd_puthex(out_startdaten + in_enddaten);
                  //					if (out_startdaten==0xB1)
                  {
                     SendOKCounter++;
                  }
                  spistatus |= (1<<SPI_SHIFT_IN_OK_BIT);
               }
               else
               {
                  spistatus &= ~(1<<SUCCESS_BIT); // Uebertragung fehlerhaft, Bit loeschen
                //  lcd_gotoxy(15,3);
                //  lcd_puts("    ");
                 lcd_gotoxy(19,3);
                  lcd_putc('-');
                  //lcd_clr_line(1);
                  lcd_gotoxy(0,3);
                  lcd_puts("ER1");
                  
                  lcd_putc(' ');
                  lcd_putint(out_startdaten);
                  lcd_putc(' ');
                  lcd_putint(in_enddaten);
                  lcd_putc(' ');
                  lcd_putint(out_startdaten + in_enddaten);
                  
                  spistatus &= ~(1<<SPI_SHIFT_IN_OK_BIT);
                  {
                     SendErrCounter++;
                  }
                  //errCounter++;
               }
               // check
               //         spistatus |= (1<<SPI_SHIFT_IN_OK_BIT);
               //         spistatus |= (1<<SUCCESS_BIT);
            }
            else
            {
               spistatus &= ~(1<<SUCCESS_BIT); //  Uebertragung unvollstaendig, Bit loeschen
               //lcd_clr_line(0);
               lcd_gotoxy(15,3);
               lcd_puts("ER2\0");
               /*
                lcd_putc(' ');
                lcd_puthex(out_startdaten);
                lcd_puthex(in_enddaten);
                lcd_putc(' ');
                lcd_puthex(out_startdaten + in_enddaten);
                */
               //delay_ms(100);
               //errCounter++;
               IncompleteCounter++;
               spistatus &= ~(1<<SPI_SHIFT_IN_OK_BIT);
            }
            
            //lcd_gotoxy(11, 1);							// Events zahelen
            //lcd_puthex(OutCounter);
            /*
             lcd_puthex(SendOKCounter);
             lcd_puthex(SendErrCounter);
             lcd_puthex(IncompleteCounter);
             */
            /*
             lcd_gotoxy(0,0);
             lcd_putc('i');
             lcd_puthex(in_startdaten);
             lcd_puthex(complement);
             lcd_putc(' ');
             lcd_putc('a');
             lcd_puthex(out_startdaten);
             lcd_puthex(in_enddaten);
             lcd_putc(' ');
             lcd_putc('l');
             lcd_puthex(in_lbdaten);
             lcd_putc(' ');
             lcd_putc('h');
             lcd_puthex(in_hbdaten);
             out_hbdaten++;
             out_lbdaten--;
             
             lcd_putc(out_startdaten);
             */
            /*
             lcd_gotoxy(0,0);
             lcd_puthex(inbuffer[9]);
             lcd_puthex(inbuffer[10]);
             lcd_puthex(inbuffer[11]);
             lcd_puthex(inbuffer[12]);
             lcd_puthex(inbuffer[13]);
             */
            //lcd_gotoxy(13,0);								// SPI - Fehler zaehlen
            //lcd_puts("ERR    \0");
            //lcd_gotoxy(17,0);
            //lcd_puthex(errCounter);
            
            // Bits im Zusammenhang mit der Uebertragung zuruecksetzen. Wurden in ISR gesetzt
            
            //           lcd_gotoxy(5,0);
            //           lcd_putint(spi_bitcontrol);
            //           spi_bitcontrol=0;
            
            spistatus &= ~(1<<ACTIVE_BIT);		// Bit 0 loeschen
            spistatus &= ~(1<<STARTDATEN_BIT);	// Bit 1 loeschen
            spistatus &= ~(1<<ENDDATEN_BIT);		// Bit 2 loeschen
            spistatus &= ~(1<<SUCCESS_BIT);		// Bit 3 loeschen
            spistatus &= ~(1<<LB_BIT);				// Bit 4 loeschen
            spistatus &= ~(1<<HB_BIT);				// Bit 5 loeschen
            
            
            
            // aufraeumen
            /*
             out_startdaten=0x00;
             out_hbdaten=0;
             out_lbdaten=0;
             for (i=0;i<SPI_BUFSIZE;i++)
             {
             outbuffer[i]=0;
             }
             */
            /*
             lcd_gotoxy(0,0);				// Fehler zaehlen
             lcd_puts("IC   \0");
             lcd_gotoxy(2,0);
             lcd_puthex(IncompleteCounter);
             lcd_gotoxy(5,0);
             lcd_puts("TW   \0");
             lcd_gotoxy(7,0);
             lcd_puthex(TWI_errCounter);
             
             lcd_gotoxy(5,1);
             lcd_puts("SE   \0");
             lcd_gotoxy(7,1);
             lcd_puthex(SendErrCounter);
             */
            
            // Strom neu messen
            
            currentstatus |= (1<<NEWBIT); // SPI fertig, neue Strommessung initiieren
            
            
            // end Strom messen
            
         } // if Active-Bit  SPI ist neu passiv, Active-bit resetten
         
         
#pragma mark HomeCentral-Tasks
         
      } //  Passiv
      
      
      // letzte Daten vom HomeCentral-Slave sind in inbuffer und in in_startdaten, in_lbdaten, in_hbdaten
      
      else						// CS ist LO, Master will senden (IS_CS_HC_ACTIVE)
      {
         if (!(spistatus & (1<<ACTIVE_BIT))) // CS ist neu aktiv (LO) geworden, Active-Bit 0 ist noch nicht gesetzt
         {
            
            if (messungcounter && (messungcounter < ANZAHLWERTE)) // aktuelle Messung begonnen, aber noch nicht fertig, aufrŠumen
            {
               messungcounter=0;
               out_startdaten = 0xB1;
               
               //impulsmittelwert = rand();
               
               // impulsmittelwert uebernehmen
               impulsmittelwert = impulszeitsumme;
               
               
               // summe resetten
               impulszeitsumme = 0;
               
               messungcounter=0; //
               
               // Daten fuer SPI setzen
               outbuffer[0] = 0x00;
               outbuffer[1] = 0x00;
               outbuffer[2] = 0x00;
               
               // Messung vollstaendig, currentstatus zuruecksetzen
               currentstatus &= ~(1<<IMPULSBIT);
               currentstatus &= ~(1<<COUNTBIT);
               currentstatus &= ~(1<<NEWBIT);
               
            }
            
            /*
             // in Master:
             // Aufnahme der Daten vom HomeCentral-Slave vorbereiten
             
             uint8_t j=0;
             in_startdaten=0;
             in_enddaten=0;
             in_lbdaten=0;
             in_hbdaten=0;
             for (j=0;j<SPI_BUFSIZE;j++)
             {
             inbuffer[j]=0;
             }
             */
            
            // hier:
            // Daten sind in current gesetzt
            
            // Ausnahme:
            in_startdaten=0;
            in_enddaten=0;
            
            //          timer1_set(0);
            
            spistatus |=(1<<ACTIVE_BIT); // Bit 0 setzen: neue Datenserie
            spistatus |=(1<<STARTDATEN_BIT); // Bit 1 setzen: erster Wert ergibt StartDaten
            
            bitpos=0;
            ByteCounter=0;
            //timer0(); // Ueberwachung der Zeit zwischen zwei Bytes. ISR setzt bitpos und ByteCounter zurueck, loescht Bit 0 in spistatus
            
            // Anzeige, das  rxdata vorhanden ist
            lcd_gotoxy(19,0);
            lcd_putc('$');
            lcd_gotoxy(19,3);
            lcd_putc(' ');

            //lcd_clr_line(0);
            
            
            
            
            // SPI-Buffer vorwaertsschalten
            /*
             uint8_t wert0=spibuffer[15];
             for(i=15;i>0;i--)
             {
             spibuffer[i]=spibuffer[i-1];
             }
             spibuffer[0]=wert0;
             */
            
            /*
             // SPI-Buffer rueckwaertsschalten
             
             uint8_t wert0=spibuffer[0];
             for(i=0;i<15;i++)
             {
             spibuffer[i]=spibuffer[i+1];
             }
             spibuffer[15]=wert0;
             */
            
         }//		if (!(spistatus & (1<<ACTIVE_BIT)))
      }//											(IS_CS_HC_ACTIVE)
      
      /* *** SPI end **************************************************************/
      
      
      /*
       
       if (!(PINB & (1<<PB0))) // Taste 0
       {
       //lcd_gotoxy(12,1);
       //lcd_puts("P0 Down\0");
       
       if (! (TastenStatus & (1<<PB0))) //Taste 0 war nich nicht gedrueckt
       {
       //RingD2(5);
       TastenStatus |= (1<<PB0);
       Tastencount=0;
       //lcd_gotoxy(0,1);
       //lcd_puts("P0 \0");
       //lcd_putint(TastenStatus);
       //delay_ms(800);
       }
       else
       {
       
       
       Tastencount ++;
       //lcd_gotoxy(7,1);
       //lcd_puts("TC \0");
       //lcd_putint(Tastencount);
       
       if (Tastencount >= Tastenprellen)
       {
       Tastencount=0;
       TastenStatus &= ~(1<<PB0);
       }
       }//else
       
       }	// Taste 0
       */
      /*
       if (!(PINB & (1<<PB1))) // Taste 1
       {
       //lcd_gotoxy(12,1);
       //lcd_puts("P1 Down\0");
       
       if (! (TastenStatus & (1<<PB1))) //Taste 1 war nicht nicht gedrueckt
       {
       TastenStatus |= (1<<PB1);
       Tastencount=0;
       //lcd_gotoxy(3,1);
       //lcd_puts("P1 \0");
       //lcd_putint(Servoimpulsdauer);
       //delay_ms(800);
       
       }
       else
       {
       //lcd_gotoxy(3,1);
       //lcd_puts("       \0");
       
       Tastencount ++;
       if (Tastencount >= Tastenprellen)
       {
       Tastencount=0;
       TastenStatus &= ~(1<<PB1);
       }
       }//	else
       
       } // Taste 1
       */
      /* ******************** */
      //		initADC(TASTATURPIN);
      //		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
      
      Tastenwert=0;
      
      //lcd_gotoxy(3,1);
      //lcd_putint(Tastenwert);
      
      if (Tastenwert>23)
      {
         /*
          0:
          1:
          2:
          3:
          4:
          5:
          6:
          7:
          8:
          9:
          */
         
         TastaturCount++;
         if (TastaturCount>=50)
         {
            
            //lcd_clr_line(1);
            //				lcd_gotoxy(8,1);
            //				lcd_puts("T:\0");
            //				lcd_putint(Tastenwert);
            
            uint8_t Taste=Tastenwahl(Tastenwert);
            
            lcd_gotoxy(18,1);
            lcd_putint2(Taste);
            //delay_ms(600);
            // lcd_clr_line(1);
            
            
            TastaturCount=0;
            Tastenwert=0x00;
            uint8_t i=0;
            //uint8_t pos=0;
            
            switch (Taste)
            {
               case 0://
               {
                  
               }break;
                  
               case 1://
               {
               }break;
                  
               case 2://
               {
                  
               }break;
                  
               case 3://
               {
                  
               }break;
                  
               case 4://
               {
               }break;
                  
               case 5://
               {
               }break;
                  
               case 6://
               {
               }break;
                  
               case 7://
               {
               }break;
                  
               case 8://
               {
                  
               }break;
                  
               case 9://
               {
                  
               }
            }//switch Tastatur
         }//if TastaturCount
         
      }//	if Tastenwert
      
      //	LOOPLEDPORT &= ~(1<<LOOPLED);
   }//while
   
   
   // return 0;
}
