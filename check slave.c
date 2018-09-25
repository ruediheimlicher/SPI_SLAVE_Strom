
   {
      //Blinkanzeige
      loopcount0++;
      if (loopcount0==0x0800)
      {
         loopcount0=0;
         LOOPLEDPORT ^=(1<<LOOPLED);
         //delay_ms(10);
         loopcount1++;
         if ((loopcount1 & 0x0F)==0)
         {
            uint8_t t=sekunde & 0x0FF;
            // lcd_gotoxy(16,0);
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
      // test
      
      // end test
      
      if (currentstatus & (1<<NEWBIT))
      {
         //     inbuffer[0]=0;
         //     inbuffer[1]=0;
         //     inbuffer[2]=0;
         // test ohne current
         inbuffer[0]=0;
         inbuffer[1]=0;
         inbuffer[2]=0;
         
         
         outbuffer[0] = testwert;
         outbuffer[1] = testwert;
         outbuffer[2] = testwert;
         out_startdaten = 0xB1;
         testwert++;
         currentstatus &= ~(1<<NEWBIT);
         continue;
         
         
         // end test ohne current
         
         if (currentstatus & (1<<IMPULSBIT)) // neuer Impuls angekommen, Zaehlung lauft
         {
            inbuffer[0]=0;
            inbuffer[1]=0;
            inbuffer[2]=0;
            in_startdaten=0;
            //OSZILO;
            //   PORTD |=(1<<ECHOPIN);
            currentstatus++; // ein Wert mehr gemessen
            messungcounter ++;
            impulszeitsumme += (impulszeit/ANZAHLWERTE);      // float, Wert aufsummieren
            
            
            // interger addieren
            integerimpulszeit += impulszeit; // float, float
            
            if (filtercount == 0) // neues Paket
            {
               filtermittelwert = impulszeit;
            }
            else
            {
               if (filtercount < filterfaktor)
               {
                  filtermittelwert = ((filtercount-1)* filtermittelwert + impulszeit)/filtercount;
                  
                  //lcd_gotoxy(19,1);
                  //lcd_putc('a');
               }
               else
               {
                  filtermittelwert = ((filterfaktor-1)* filtermittelwert + impulszeit)/filterfaktor;
                  //lcd_gotoxy(19,1);
                  //lcd_putc('f');
               }
               
            }
            
            
            //            char filterstromstring[8];
            //            filtercount++;
            /*
             lcd_gotoxy(0,0);
             lcd_putint16(impulszeit/100);
             lcd_gotoxy(10,0);
             lcd_putint16(filtermittelwert/100);
             lcd_gotoxy(10,1);
             lcd_putint(filtercount);
             lcd_putc('*');
             
             dtostrf(filtermittelwert,5,1,filterstromstring);
             //lcd_puts(filterstromstring);
             */
            
            //         if (filtercount & (filterfaktor == 0)) // Wert anzeigen
            {
               //lcd_gotoxy(10,0);
               //lcd_putint16(filtermittelwert);
               //lcd_putc('*');
               //dtostrf(filtermittelwert,5,1,filterstromstring);
               //lcd_puts(filterstromstring);
            }
            
            
            if ((currentstatus & 0x0F) == ANZAHLWERTE)      // genuegend Werte
            {
               OSZILO;
               
               //               lcd_gotoxy(19,0);
               //               lcd_putc(' ');
               
               
               //lcd_putc(' ');
               //lcd_gotoxy(6,1);
               //lcd_putc(' ');
               
               //lcd_gotoxy(16,1);
               //lcd_puts("  \0");
               //lcd_gotoxy(0,1);
               //lcd_puts("    \0");
               
               //lcd_gotoxy(0,1);
               //lcd_putint(messungcounter);
               
               
               
               //lcd_gotoxy(0,0);
               //lcd_puts("  \0");
               /*
                if ((paketcounter & 1)==0)
                {
                lcd_gotoxy(8,1);
                lcd_putc(':');
                
                }
                else
                {
                lcd_gotoxy(8,1);
                lcd_putc(' ');
                }
                */
               paketcounter++;
               
               lcd_gotoxy(0,0);
               lcd_putint(paketcounter);
               cli();
               
               currentstatus &= 0xF0; // Bit 0-3 reset
               
               // Wert fuer SPI-Uebertragung
               
               impulsmittelwert = impulszeitsumme; // float = float
               
               // timer1 setzen
               OCR1A = (uint16_t)(impulsmittelwert+0.5); // float to uint16
               
               // summe resetten
               impulszeitsumme = 0;
               
               //               impulsmittelwertl = ((uint32_t)impulsmittelwert & 0xFF);
               //              impulsmittelwerth = ((uint32_t)impulsmittelwert>>8) & 0xFF;
               //             impulsmittelwerthh = ((uint32_t)impulsmittelwert>>16) & 0xFF;
               
               outbuffer[0] = ((uint32_t)impulsmittelwert & 0xFF);
               outbuffer[1] = ((uint32_t)impulsmittelwert>>8) & 0xFF;
               outbuffer[2] = ((uint32_t)impulsmittelwert>>16) & 0xFF;
               //out_startdaten = 0x13;
               
               sei();
               //                lcd_gotoxy(0,1);
               //               lcd_putc('I');
               //lcd_puts("INT0 \0");
               
               //lcd_puthex(hb);
               //lcd_puthex(lb);
               //lcd_putc(':');
               
               //char impstring[12];
               //dtostrf(impulsmittelwert,8,2,impstring);
               
               //            lcd_gotoxy(0,3);
               //           lcd_putint16(((uint16_t)impulsmittelwert) );
               //lcd_putc('*');
               
               
               // lcd_gotoxy(5,0);
               // lcd_putint(sendintervallzeit);
               // lcd_putc('$');
               
               /*
                Impulsdauer: impulsmittelwert * TIMERIMPULSDAUER (10us)
                Umrechnung auf ms: /1000
                Energie pro Zählerimpuls: 360 Ws
                Leistung: (Energie pro Zählerimpuls)/Impulsabstand
                Umrechnung auf Sekunden: *1000
                Faktor: *100000
                */
               
               //     leistung = 0xFFFF/impulsmittelwert;
               //              cli();
               
               // Leistung berechnen
               /*
                if (impulsmittelwert)
                {
                leistung = 360.0/impulsmittelwert*100000.0;// 480us
                
                // webleistung = (uint32_t)360.0/impulsmittelwert*1000000.0;
                webleistung = (uint32_t)360.0/impulsmittelwert*100000.0;
                
                
                //                 lcd_gotoxy(0,1);
                //                 lcd_putint16(webleistung);
                //                 lcd_putc('*');
                }
                wattstunden = impulscount/10; // 310us
                */
               
               //               sei();
               
               //     Stromzaehler
               //OSZILO;
               /*
                // ganze Anzeige 55 ms
                lcd_gotoxy(9,1);
                lcd_putint(wattstunden/1000);
                lcd_putc('.');
                lcd_putint3(wattstunden);
                lcd_putc('W');
                lcd_putc('h');
                */
               //OSZIHI;
               
               
               // dtostrf(leistung,5,0,stromstring); // fuehrt zu 'strom=++123' in URL fuer strom.pl. Funktionierte trotzdem
               
               //         dtostrf(leistung,5,1,stromstring); // 800us
               
               
               //lcd_gotoxy(0,0);
               //lcd_putc('L');
               //lcd_putc(':');
               
               
               //            if (!(paketcounter == 1))
               {
                  
                  //lcd_puts("     \0");
                  
                  //lcd_gotoxy(2,0);
                  //lcd_puts(stromstring);
                  //lcd_putc(' ');
                  //lcd_putc('W');
               }
               //lcd_putc('*');
               //lcd_putc(' ');
               //lcd_putint16(leistung);
               //lcd_putc(' ');
               
               /*
                if (abs(leistung-lastleistung) > 10)
                {
                lastcounter++;
                
                if (lastcounter>3)
                {
                char diff[10];
                dtostrf(leistung-lastleistung,7,2,diff);
                lcd_gotoxy(10,1);
                lcd_putc('D');
                lcd_putc(':');
                lcd_puts(diff);
                lastleistung = leistung;
                }
                }
                else
                {
                lastcounter=0;
                }
                */
               
               // if (paketcounter  >= ANZAHLPAKETE)
               if (webstatus & (1<<DATALOOP))
               {
                  
                  webstatus &= ~(1<<DATALOOP);
                  
                  //uint16_t zufall = rand() % 0x0F + 1;;
                  
                  //lcd_putc(' ');
                  //lcd_putint12(zufall);
                  //leistung += zufall;
                  
                  
                  
                  //dtostrf(leistung,5,1,stromstring); // 800us
                  
                  dtostrf(webleistung,10,0,stromstring); // 800us
                  
                  
                  paketcounter=0;
                  
                  /*
                   uint16_t tempmitte = 0;
                   for (i=0;i<4;i++)
                   {
                   tempmitte+= stromimpulsmittelwertarray[i];
                   }
                   tempmitte/= 4;
                   */
                  /*
                   lcd_gotoxy(14,0);
                   lcd_putc('m');
                   lcd_putint12(tempmitte);
                   */
                  //         filtercount =0;
                  
                  //if (TEST)
                  {
                     //lcd_gotoxy(0,0);
                     //lcd_putint(messungcounter);
                     //lcd_putc(' ');
                     //OSZILO;
                     
                     /*
                      lcd_gotoxy(9,2);
                      lcd_putint(wattstunden/1000);
                      lcd_putc('.');
                      lcd_putint3(wattstunden);
                      //lcd_putc('W');
                      //lcd_putc('h');
                      */
                     //OSZIHI;
                  }
                  
                  
                  // senden aktivieren
                  webstatus |= (1<<DATASEND);
                  webstatus |= (1<<DATAOK);
                  // Messung anhalten
                  webstatus |= (1<<CURRENTSTOP);
                  // Warten aktivieren
                  webstatus |= (1<<CURRENTWAIT);
                  
                  paketcounter=0;
                  //sendWebCount++;
                  //           lcd_gotoxy(6,1);
                  //           lcd_putc('>');
               } // if DATALOOP
               
               //anzeigewert = 0xFF/0x8000*leistung; // 0x8000/0x255 = 0x81
               //anzeigewert = leistung/0x81;
               
               //               cli();
               anzeigewert = leistung /0x18; // /24
               sei();
               
               // if (TEST)
               {
                  //   lcd_gotoxy(9,0);
                  //   lcd_putint(anzeigewert);
               }
               
               //               webstatus |= (1<<CURRENTSEND);
               currentstatus &= ~(1<<NEWBIT);
               OSZIHI;
               
            } // genuegend Werte
            else
            {
               //lcd_gotoxy(8,1);
               //lcd_puts("    \0");
               
            }
            
            //PORTD &= ~(1<<ECHOPIN);
            impulszeit=0;
            currentstatus &= ~(1<<IMPULSBIT);
            //            OSZIHI;
         }
         
      } //  if (currentstatus & (1<<NEWBIT))
      
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
            lcd_gotoxy(15,3);
            lcd_puts("   ");
            lcd_gotoxy(5,0);
            lcd_puts("   ");
            
            
            SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO); // MISO ist HI in Pausen
            
            // timer1 wieder ein
            timer1_set(1);
            
            wdt_reset();
            SPI_Call_count0++;
            // Eingang von Interrupt-Routine, Daten von HomeCentral-Slave
            lcd_gotoxy(19,0);
            // lcd_putc(' ');
            
            // in lcd verschoben
            //lcd_clr_line(2);
            lcd_gotoxy(0,1);
            
            // outbuffer[0]=0;
            // outbuffer[1]=0;
            // outbuffer[2]=0;
            
            // Ausgang anzeigen
            // outbuffer[0] = testwert;
            // outbuffer[1] = testwert;
            // outbuffer[2] = testwert;
            // testwert++;
            
            
            lcd_puts("oM \0");
            lcd_putint(outbuffer[0]);
            lcd_putc('*');
            lcd_putint(outbuffer[1]);
            lcd_putc('*');
            lcd_putint(outbuffer[2]);
            lcd_putc('*');
            lcd_putint(out_startdaten);
            
            lcd_gotoxy(15,0);
            lcd_putc('c');
            lcd_putint(SPI_Call_count0);
            
            lcd_gotoxy(0,2);
            lcd_puts("iM \0");
            lcd_putint(inbuffer[0]);
            lcd_putc('*');
            lcd_putint(inbuffer[1]);
            lcd_putc('*');
            lcd_putint(inbuffer[2]);
            lcd_putc('s');
            lcd_putint(in_startdaten);
            
            if (in_startdaten == 0)
            {
               spi_errcount++;
            }
            lcd_gotoxy(10,0);
            lcd_putint(spi_errcount);
            
            lcd_gotoxy(16,3);
            lcd_putint(SendErrCounter);
            
            /*
             lcd_puthex(in_startdaten);
             lcd_putc(' ');
             lcd_puthex(in_hbdaten);
             lcd_puthex(in_lbdaten);
             lcd_putc(' ');
             */
            uint8_t j=0;
            for (j=0;j<3;j++)
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
            lcd_gotoxy(19,0);
            if (ByteCounter == SPI_BUFSIZE-1) // Uebertragung war vollstaendig
            {
               
               if (out_startdaten + in_enddaten==0xFF)
               {
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
            
            lcd_gotoxy(5,0);
            lcd_putint(spi_bitcontrol);
            spi_bitcontrol=0;
            
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
            
            currentstatus |= (1<<NEWBIT);
            
            
            // end Strom messen
            
         } // if Active-Bit  SPI ist neu passiv, Active-bit resetten
         
         
#pragma mark HomeCentral-Tasks
         
      } //  Passiv
      
      
      // letzte Daten vom HomeCentral-Slave sind in inbuffer und in in_startdaten, in_lbdaten, in_hbdaten
      
      else						// CS ist LO, Master will senden (IS_CS_HC_ACTIVE)
      {
         if (!(spistatus & (1<<ACTIVE_BIT))) // CS ist neu aktiv (LO) geworden, Active-Bit 0 ist noch nicht gesetzt
         {
            
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
            
            timer1_set(0);
            
            spistatus |=(1<<ACTIVE_BIT); // Bit 0 setzen: neue Datenserie
            spistatus |=(1<<STARTDATEN_BIT); // Bit 1 setzen: erster Wert ergibt StartDaten
            
            bitpos=0;
            ByteCounter=0;
            //timer0(); // Ueberwachung der Zeit zwischen zwei Bytes. ISR setzt bitpos und ByteCounter zurueck, loescht Bit 0 in spistatus
            
            // Anzeige, das  rxdata vorhanden ist
            lcd_gotoxy(19,0);
            lcd_putc('$');
            //lcd_clr_line(0);
         }//		if (!(spistatus & (1<<ACTIVE_BIT)))
      }//											(IS_CS_HC_ACTIVE)
      
      /* *** SPI end **************************************************************/
      
 