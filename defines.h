//version.c
#define VERSION "80.1"

#define TEST   0
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

#define LOOPLEDPORT      PORTB
#define LOOPLEDDDR      DDRB

// Define fuer Slave:
#define LOOPLED         0
#define  INTERRUPT   1
