# 1 "main.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "main.c" 2
# 44 "main.c"
# 1 "./mcc_generated_files/mcc.h" 1
# 49 "./mcc_generated_files/mcc.h"
# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\xc.h" 1 3
# 18 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\xc.h" 3
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);



# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\xc8debug.h" 1 3



# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdlib.h" 1 3



# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\musl_xc8.h" 1 3
# 4 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdlib.h" 2 3






# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\features.h" 1 3
# 10 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdlib.h" 2 3
# 21 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdlib.h" 3
# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 18 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int wchar_t;
# 122 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned size_t;
# 168 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __int24 int24_t;
# 204 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __uint24 uint24_t;
# 21 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdlib.h" 2 3


int atoi (const char *);
long atol (const char *);
long long atoll (const char *);
double atof (const char *);

float strtof (const char *restrict, char **restrict);
double strtod (const char *restrict, char **restrict);
long double strtold (const char *restrict, char **restrict);



long strtol (const char *restrict, char **restrict, int);
unsigned long strtoul (const char *restrict, char **restrict, int);
long long strtoll (const char *restrict, char **restrict, int);
unsigned long long strtoull (const char *restrict, char **restrict, int);

int rand (void);
void srand (unsigned);

          void abort (void);
int atexit (void (*) (void));
          void exit (int);
          void _Exit (int);

void *bsearch (const void *, const void *, size_t, size_t, int (*)(const void *, const void *));

__attribute__((nonreentrant)) void qsort (void *, size_t, size_t, int (*)(const void *, const void *));

int abs (int);
long labs (long);
long long llabs (long long);

typedef struct { int quot, rem; } div_t;
typedef struct { long quot, rem; } ldiv_t;
typedef struct { long long quot, rem; } lldiv_t;

div_t div (int, int);
ldiv_t ldiv (long, long);
lldiv_t lldiv (long long, long long);

typedef struct { unsigned int quot, rem; } udiv_t;
typedef struct { unsigned long quot, rem; } uldiv_t;
udiv_t udiv (unsigned int, unsigned int);
uldiv_t uldiv (unsigned long, unsigned long);





size_t __ctype_get_mb_cur_max(void);
# 4 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\xc8debug.h" 2 3








#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);
# 23 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\xc.h" 2 3




# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 1 3




# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\htc.h" 1 3



# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\xc.h" 1 3
# 4 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\htc.h" 2 3
# 5 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 2 3








# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic_chip_select.h" 1 3
# 3334 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic_chip_select.h" 3
# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 1 3
# 44 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\__at.h" 1 3
# 44 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 2 3
# 108 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char INDF0 __attribute__((address(0x000)));

__asm("INDF0 equ 00h");


typedef union {
    struct {
        unsigned INDF0 :8;
    };
} INDF0bits_t;
extern volatile INDF0bits_t INDF0bits __attribute__((address(0x000)));
# 128 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char INDF1 __attribute__((address(0x001)));

__asm("INDF1 equ 01h");


typedef union {
    struct {
        unsigned INDF1 :8;
    };
} INDF1bits_t;
extern volatile INDF1bits_t INDF1bits __attribute__((address(0x001)));
# 148 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PCL __attribute__((address(0x002)));

__asm("PCL equ 02h");


typedef union {
    struct {
        unsigned PCL :8;
    };
} PCLbits_t;
extern volatile PCLbits_t PCLbits __attribute__((address(0x002)));
# 168 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char STATUS __attribute__((address(0x003)));

__asm("STATUS equ 03h");


typedef union {
    struct {
        unsigned C :1;
        unsigned DC :1;
        unsigned Z :1;
        unsigned nPD :1;
        unsigned nTO :1;
    };
    struct {
        unsigned CARRY :1;
        unsigned :1;
        unsigned ZERO :1;
    };
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __attribute__((address(0x003)));
# 227 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short FSR0 __attribute__((address(0x004)));



extern volatile unsigned char FSR0L __attribute__((address(0x004)));

__asm("FSR0L equ 04h");


typedef union {
    struct {
        unsigned FSR0L :8;
    };
} FSR0Lbits_t;
extern volatile FSR0Lbits_t FSR0Lbits __attribute__((address(0x004)));
# 251 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char FSR0H __attribute__((address(0x005)));

__asm("FSR0H equ 05h");


typedef union {
    struct {
        unsigned FSR0H :8;
    };
} FSR0Hbits_t;
extern volatile FSR0Hbits_t FSR0Hbits __attribute__((address(0x005)));
# 271 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short FSR1 __attribute__((address(0x006)));



extern volatile unsigned char FSR1L __attribute__((address(0x006)));

__asm("FSR1L equ 06h");


typedef union {
    struct {
        unsigned FSR1L :8;
    };
} FSR1Lbits_t;
extern volatile FSR1Lbits_t FSR1Lbits __attribute__((address(0x006)));
# 295 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char FSR1H __attribute__((address(0x007)));

__asm("FSR1H equ 07h");


typedef union {
    struct {
        unsigned FSR1H :8;
    };
} FSR1Hbits_t;
extern volatile FSR1Hbits_t FSR1Hbits __attribute__((address(0x007)));
# 315 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char BSR __attribute__((address(0x008)));

__asm("BSR equ 08h");


typedef union {
    struct {
        unsigned BSR :6;
    };
    struct {
        unsigned BSR0 :1;
        unsigned BSR1 :1;
        unsigned BSR2 :1;
        unsigned BSR3 :1;
        unsigned BSR4 :1;
        unsigned BSR5 :1;
    };
} BSRbits_t;
extern volatile BSRbits_t BSRbits __attribute__((address(0x008)));
# 373 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WREG __attribute__((address(0x009)));

__asm("WREG equ 09h");


typedef union {
    struct {
        unsigned WREG0 :8;
    };
} WREGbits_t;
extern volatile WREGbits_t WREGbits __attribute__((address(0x009)));
# 393 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PCLATH __attribute__((address(0x00A)));

__asm("PCLATH equ 0Ah");


typedef union {
    struct {
        unsigned PCLATH :7;
    };
} PCLATHbits_t;
extern volatile PCLATHbits_t PCLATHbits __attribute__((address(0x00A)));
# 413 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char INTCON __attribute__((address(0x00B)));

__asm("INTCON equ 0Bh");


typedef union {
    struct {
        unsigned INTEDG :1;
        unsigned :5;
        unsigned PEIE :1;
        unsigned GIE :1;
    };
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __attribute__((address(0x00B)));
# 446 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PORTA __attribute__((address(0x00C)));

__asm("PORTA equ 0Ch");


typedef union {
    struct {
        unsigned RA0 :1;
        unsigned RA1 :1;
        unsigned RA2 :1;
        unsigned RA3 :1;
        unsigned RA4 :1;
        unsigned RA5 :1;
        unsigned RA6 :1;
        unsigned RA7 :1;
    };
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __attribute__((address(0x00C)));
# 508 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PORTB __attribute__((address(0x00D)));

__asm("PORTB equ 0Dh");


typedef union {
    struct {
        unsigned RB0 :1;
        unsigned RB1 :1;
        unsigned RB2 :1;
        unsigned RB3 :1;
        unsigned RB4 :1;
        unsigned RB5 :1;
        unsigned RB6 :1;
        unsigned RB7 :1;
    };
} PORTBbits_t;
extern volatile PORTBbits_t PORTBbits __attribute__((address(0x00D)));
# 570 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PORTC __attribute__((address(0x00E)));

__asm("PORTC equ 0Eh");


typedef union {
    struct {
        unsigned RC0 :1;
        unsigned RC1 :1;
        unsigned RC2 :1;
        unsigned RC3 :1;
        unsigned RC4 :1;
        unsigned :1;
        unsigned RC6 :1;
        unsigned RC7 :1;
    };
} PORTCbits_t;
extern volatile PORTCbits_t PORTCbits __attribute__((address(0x00E)));
# 627 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PORTE __attribute__((address(0x010)));

__asm("PORTE equ 010h");


typedef union {
    struct {
        unsigned :3;
        unsigned RE3 :1;
    };
} PORTEbits_t;
extern volatile PORTEbits_t PORTEbits __attribute__((address(0x010)));
# 648 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TRISA __attribute__((address(0x012)));

__asm("TRISA equ 012h");


typedef union {
    struct {
        unsigned TRISA0 :1;
        unsigned TRISA1 :1;
        unsigned TRISA2 :1;
        unsigned TRISA3 :1;
        unsigned TRISA4 :1;
        unsigned :1;
        unsigned TRISA6 :1;
        unsigned TRISA7 :1;
    };
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __attribute__((address(0x012)));
# 705 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TRISB __attribute__((address(0x013)));

__asm("TRISB equ 013h");


typedef union {
    struct {
        unsigned TRISB0 :1;
        unsigned TRISB1 :1;
        unsigned TRISB2 :1;
        unsigned TRISB3 :1;
        unsigned TRISB4 :1;
        unsigned TRISB5 :1;
        unsigned TRISB6 :1;
        unsigned TRISB7 :1;
    };
} TRISBbits_t;
extern volatile TRISBbits_t TRISBbits __attribute__((address(0x013)));
# 767 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TRISC __attribute__((address(0x014)));

__asm("TRISC equ 014h");


typedef union {
    struct {
        unsigned TRISC0 :1;
        unsigned TRISC1 :1;
        unsigned TRISC2 :1;
        unsigned TRISC3 :1;
        unsigned TRISC4 :1;
        unsigned :1;
        unsigned TRISC6 :1;
        unsigned TRISC7 :1;
    };
} TRISCbits_t;
extern volatile TRISCbits_t TRISCbits __attribute__((address(0x014)));
# 824 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LATA __attribute__((address(0x018)));

__asm("LATA equ 018h");


typedef union {
    struct {
        unsigned LATA0 :1;
        unsigned LATA1 :1;
        unsigned LATA2 :1;
        unsigned LATA3 :1;
        unsigned LATA4 :1;
        unsigned :1;
        unsigned LATA6 :1;
        unsigned LATA7 :1;
    };
} LATAbits_t;
extern volatile LATAbits_t LATAbits __attribute__((address(0x018)));
# 881 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LATB __attribute__((address(0x019)));

__asm("LATB equ 019h");


typedef union {
    struct {
        unsigned LATB0 :1;
        unsigned LATB1 :1;
        unsigned LATB2 :1;
        unsigned LATB3 :1;
        unsigned LATB4 :1;
        unsigned LATB5 :1;
        unsigned LATB6 :1;
        unsigned LATB7 :1;
    };
} LATBbits_t;
extern volatile LATBbits_t LATBbits __attribute__((address(0x019)));
# 943 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LATC __attribute__((address(0x01A)));

__asm("LATC equ 01Ah");


typedef union {
    struct {
        unsigned LATC0 :1;
        unsigned LATC1 :1;
        unsigned LATC2 :1;
        unsigned LATC3 :1;
        unsigned LATC4 :1;
        unsigned :1;
        unsigned LATC6 :1;
        unsigned LATC7 :1;
    };
} LATCbits_t;
extern volatile LATCbits_t LATCbits __attribute__((address(0x01A)));
# 1000 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADCPCON0 __attribute__((address(0x01F)));

__asm("ADCPCON0 equ 01Fh");


typedef union {
    struct {
        unsigned ADCPRDY :1;
        unsigned :6;
        unsigned ADCPON :1;
    };
} ADCPCON0bits_t;
extern volatile ADCPCON0bits_t ADCPCON0bits __attribute__((address(0x01F)));
# 1027 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short ADLTH __attribute__((address(0x08C)));

__asm("ADLTH equ 08Ch");




extern volatile unsigned char ADLTHL __attribute__((address(0x08C)));

__asm("ADLTHL equ 08Ch");


typedef union {
    struct {
        unsigned LTH :8;
    };
    struct {
        unsigned ADLTH0 :1;
        unsigned ADLTH1 :1;
        unsigned ADLTH2 :1;
        unsigned ADLTH3 :1;
        unsigned ADLTH4 :1;
        unsigned ADLTH5 :1;
        unsigned ADLTH6 :1;
        unsigned ADLTH7 :1;
    };
    struct {
        unsigned ADLTH :8;
    };
    struct {
        unsigned LTH0 :1;
        unsigned LTH1 :1;
        unsigned LTH2 :1;
        unsigned LTH3 :1;
        unsigned LTH4 :1;
        unsigned LTH5 :1;
        unsigned LTH6 :1;
        unsigned LTH7 :1;
    };
} ADLTHLbits_t;
extern volatile ADLTHLbits_t ADLTHLbits __attribute__((address(0x08C)));
# 1162 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADLTHH __attribute__((address(0x08D)));

__asm("ADLTHH equ 08Dh");


typedef union {
    struct {
        unsigned LTH :8;
    };
    struct {
        unsigned ADLTH8 :1;
        unsigned ADLTH9 :1;
        unsigned ADLTH10 :1;
        unsigned ADLTH11 :1;
        unsigned ADLTH12 :1;
        unsigned ADLTH13 :1;
        unsigned ADLTH14 :1;
        unsigned ADLTH15 :1;
    };
    struct {
        unsigned ADLTH :8;
    };
    struct {
        unsigned LTH8 :1;
        unsigned LTH9 :1;
        unsigned LTH10 :1;
        unsigned LTH11 :1;
        unsigned LTH12 :1;
        unsigned LTH13 :1;
        unsigned LTH14 :1;
        unsigned LTH15 :1;
    };
} ADLTHHbits_t;
extern volatile ADLTHHbits_t ADLTHHbits __attribute__((address(0x08D)));
# 1290 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short ADUTH __attribute__((address(0x08E)));

__asm("ADUTH equ 08Eh");




extern volatile unsigned char ADUTHL __attribute__((address(0x08E)));

__asm("ADUTHL equ 08Eh");


typedef union {
    struct {
        unsigned UTH :8;
    };
    struct {
        unsigned ADUTH0 :1;
        unsigned ADUTH1 :1;
        unsigned ADUTH2 :1;
        unsigned ADUTH3 :1;
        unsigned ADUTH4 :1;
        unsigned ADUTH5 :1;
        unsigned ADUTH6 :1;
        unsigned ADUTH7 :1;
    };
    struct {
        unsigned ADUTH :8;
    };
    struct {
        unsigned UTH0 :1;
        unsigned UTH1 :1;
        unsigned UTH2 :1;
        unsigned UTH3 :1;
        unsigned UTH4 :1;
        unsigned UTH5 :1;
        unsigned UTH6 :1;
        unsigned UTH7 :1;
    };
} ADUTHLbits_t;
extern volatile ADUTHLbits_t ADUTHLbits __attribute__((address(0x08E)));
# 1425 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADUTHH __attribute__((address(0x08F)));

__asm("ADUTHH equ 08Fh");


typedef union {
    struct {
        unsigned UTH :8;
    };
    struct {
        unsigned ADUTH8 :1;
        unsigned ADUTH9 :1;
        unsigned ADUTH10 :1;
        unsigned ADUTH11 :1;
        unsigned ADUTH12 :1;
        unsigned ADUTH13 :1;
        unsigned ADUTH14 :1;
        unsigned ADUTH15 :1;
    };
    struct {
        unsigned ADUTH :8;
    };
    struct {
        unsigned UTH8 :1;
        unsigned UTH9 :1;
        unsigned UTH10 :1;
        unsigned UTH11 :1;
        unsigned UTH12 :1;
        unsigned UTH13 :1;
        unsigned UTH14 :1;
        unsigned UTH15 :1;
    };
} ADUTHHbits_t;
extern volatile ADUTHHbits_t ADUTHHbits __attribute__((address(0x08F)));
# 1553 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short ADERR __attribute__((address(0x090)));

__asm("ADERR equ 090h");




extern volatile unsigned char ADERRL __attribute__((address(0x090)));

__asm("ADERRL equ 090h");


typedef union {
    struct {
        unsigned ERR :8;
    };
    struct {
        unsigned ADERR0 :1;
        unsigned ADERR1 :1;
        unsigned ADERR2 :1;
        unsigned ADERR3 :1;
        unsigned ADERR4 :1;
        unsigned ADERR5 :1;
        unsigned ADERR6 :1;
        unsigned ADERR7 :1;
    };
    struct {
        unsigned ADERR :8;
    };
    struct {
        unsigned ERR0 :1;
        unsigned ERR1 :1;
        unsigned ERR2 :1;
        unsigned ERR3 :1;
        unsigned ERR4 :1;
        unsigned ERR5 :1;
        unsigned ERR6 :1;
        unsigned ERR7 :1;
    };
} ADERRLbits_t;
extern volatile ADERRLbits_t ADERRLbits __attribute__((address(0x090)));
# 1688 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADERRH __attribute__((address(0x091)));

__asm("ADERRH equ 091h");


typedef union {
    struct {
        unsigned ERR :8;
    };
    struct {
        unsigned ADERR8 :1;
        unsigned ADERR9 :1;
        unsigned ADERR10 :1;
        unsigned ADERR11 :1;
        unsigned ADERR12 :1;
        unsigned ADERR13 :1;
        unsigned ADERR14 :1;
        unsigned ADERR15 :1;
    };
    struct {
        unsigned ADERR :8;
    };
    struct {
        unsigned ERR8 :1;
        unsigned ERR9 :1;
        unsigned ERR10 :1;
        unsigned ERR11 :1;
        unsigned ERR12 :1;
        unsigned ERR13 :1;
        unsigned ERR14 :1;
        unsigned ERR15 :1;
    };
} ADERRHbits_t;
extern volatile ADERRHbits_t ADERRHbits __attribute__((address(0x091)));
# 1816 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short ADSTPT __attribute__((address(0x092)));

__asm("ADSTPT equ 092h");




extern volatile unsigned char ADSTPTL __attribute__((address(0x092)));

__asm("ADSTPTL equ 092h");


typedef union {
    struct {
        unsigned STPT :8;
    };
    struct {
        unsigned ADSTPT0 :1;
        unsigned ADSTPT1 :1;
        unsigned ADSTPT2 :1;
        unsigned ADSTPT3 :1;
        unsigned ADSTPT4 :1;
        unsigned ADSTPT5 :1;
        unsigned ADSTPT6 :1;
        unsigned ADSTPT7 :1;
    };
    struct {
        unsigned ADSTPT :8;
    };
    struct {
        unsigned STPT0 :1;
        unsigned STPT1 :1;
        unsigned STPT2 :1;
        unsigned STPT3 :1;
        unsigned STPT4 :1;
        unsigned STPT5 :1;
        unsigned STPT6 :1;
        unsigned STPT7 :1;
    };
} ADSTPTLbits_t;
extern volatile ADSTPTLbits_t ADSTPTLbits __attribute__((address(0x092)));
# 1951 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADSTPTH __attribute__((address(0x093)));

__asm("ADSTPTH equ 093h");


typedef union {
    struct {
        unsigned STPT :8;
    };
    struct {
        unsigned ADSTPT8 :1;
        unsigned ADSTPT9 :1;
        unsigned ADSTPT10 :1;
        unsigned ADSTPT11 :1;
        unsigned ADSTPT12 :1;
        unsigned ADSTPT13 :1;
        unsigned ADSTPT14 :1;
        unsigned ADSTPT15 :1;
    };
    struct {
        unsigned ADSTPT :8;
    };
    struct {
        unsigned STPT8 :1;
        unsigned STPT9 :1;
        unsigned STPT10 :1;
        unsigned STPT11 :1;
        unsigned STPT12 :1;
        unsigned STPT13 :1;
        unsigned STPT15 :1;
        unsigned STPT16 :1;
    };
} ADSTPTHbits_t;
extern volatile ADSTPTHbits_t ADSTPTHbits __attribute__((address(0x093)));
# 2079 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short ADFLTR __attribute__((address(0x094)));

__asm("ADFLTR equ 094h");




extern volatile unsigned char ADFLTRL __attribute__((address(0x094)));

__asm("ADFLTRL equ 094h");


typedef union {
    struct {
        unsigned FLTR :8;
    };
    struct {
        unsigned ADFLTR0 :1;
        unsigned ADFLTR1 :1;
        unsigned ADFLTR2 :1;
        unsigned ADFLTR3 :1;
        unsigned ADFLTR4 :1;
        unsigned ADFLTR5 :1;
        unsigned ADFLTR6 :1;
        unsigned ADFLTR7 :1;
    };
    struct {
        unsigned ADFLTR :8;
    };
    struct {
        unsigned FLTR0 :1;
        unsigned FLTR1 :1;
        unsigned FLTR2 :1;
        unsigned FLTR3 :1;
        unsigned FLTR4 :1;
        unsigned FLTR5 :1;
        unsigned FLTR6 :1;
        unsigned FLTR7 :1;
    };
} ADFLTRLbits_t;
extern volatile ADFLTRLbits_t ADFLTRLbits __attribute__((address(0x094)));
# 2214 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADFLTRH __attribute__((address(0x095)));

__asm("ADFLTRH equ 095h");


typedef union {
    struct {
        unsigned FLTR :8;
    };
    struct {
        unsigned ADFLTR8 :1;
        unsigned ADFLTR9 :1;
        unsigned ADFLTR10 :1;
        unsigned ADFLTR11 :1;
        unsigned ADFLTR12 :1;
        unsigned ADFLTR13 :1;
        unsigned ADFLTR14 :1;
        unsigned ADFLTR15 :1;
    };
    struct {
        unsigned ADFLTR :8;
    };
    struct {
        unsigned FLTR8 :1;
        unsigned FLTR9 :1;
        unsigned FLTR10 :1;
        unsigned FLTR11 :1;
        unsigned FLTR12 :1;
        unsigned FLTR13 :1;
        unsigned FLTR14 :1;
        unsigned FLTR15 :1;
    };
} ADFLTRHbits_t;
extern volatile ADFLTRHbits_t ADFLTRHbits __attribute__((address(0x095)));
# 2343 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile __uint24 ADACC __attribute__((address(0x096)));


__asm("ADACC equ 096h");




extern volatile unsigned char ADACCL __attribute__((address(0x096)));

__asm("ADACCL equ 096h");


typedef union {
    struct {
        unsigned ACC :8;
    };
    struct {
        unsigned ADACC0 :1;
        unsigned ADACC1 :1;
        unsigned ADACC2 :1;
        unsigned ADACC3 :1;
        unsigned ADACC4 :1;
        unsigned ADACC5 :1;
        unsigned ADACC6 :1;
        unsigned ADACC7 :1;
    };
    struct {
        unsigned ADACC :8;
    };
    struct {
        unsigned ACC0 :1;
        unsigned ACC1 :1;
        unsigned ACC2 :1;
        unsigned ACC3 :1;
        unsigned ACC4 :1;
        unsigned ACC5 :1;
        unsigned ACC6 :1;
        unsigned ACC7 :1;
    };
} ADACCLbits_t;
extern volatile ADACCLbits_t ADACCLbits __attribute__((address(0x096)));
# 2479 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADACCH __attribute__((address(0x097)));

__asm("ADACCH equ 097h");


typedef union {
    struct {
        unsigned ACC :8;
    };
    struct {
        unsigned ADACC8 :1;
        unsigned ADACC9 :1;
        unsigned ADACC10 :1;
        unsigned ADACC11 :1;
        unsigned ADACC12 :1;
        unsigned ADACC13 :1;
        unsigned ADACC14 :1;
        unsigned ADACC15 :1;
    };
    struct {
        unsigned ADACC :8;
    };
    struct {
        unsigned ACC8 :1;
        unsigned ACC9 :1;
        unsigned ACC10 :1;
        unsigned ACC11 :1;
        unsigned ACC12 :1;
        unsigned ACC13 :1;
        unsigned ACC14 :1;
        unsigned ACC15 :1;
    };
} ADACCHbits_t;
extern volatile ADACCHbits_t ADACCHbits __attribute__((address(0x097)));
# 2607 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADACCU __attribute__((address(0x098)));

__asm("ADACCU equ 098h");


typedef union {
    struct {
        unsigned ACC :8;
    };
    struct {
        unsigned ADACC16 :1;
        unsigned ADACC17 :1;
        unsigned ADACC18 :1;
        unsigned ADACC19 :1;
        unsigned ADACC20 :1;
        unsigned ADACC21 :1;
        unsigned ADACC22 :1;
        unsigned ADACC23 :1;
    };
    struct {
        unsigned ADACC :8;
    };
    struct {
        unsigned ACC16 :1;
        unsigned ACC17 :1;
        unsigned ACC18 :1;
        unsigned ACC19 :1;
        unsigned ACC20 :1;
        unsigned ACC21 :1;
        unsigned ACC22 :1;
        unsigned ACC23 :1;
    };
} ADACCUbits_t;
extern volatile ADACCUbits_t ADACCUbits __attribute__((address(0x098)));
# 2735 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADCNT __attribute__((address(0x099)));

__asm("ADCNT equ 099h");


typedef union {
    struct {
        unsigned CNT :8;
    };
    struct {
        unsigned ADCNT0 :1;
        unsigned ADCNT1 :1;
        unsigned ADCNT2 :1;
        unsigned ADCNT3 :1;
        unsigned ADCNT4 :1;
        unsigned ADCNT5 :1;
        unsigned ADCNT6 :1;
        unsigned ADCNT7 :1;
    };
    struct {
        unsigned ADCNT :8;
    };
    struct {
        unsigned CNT0 :1;
        unsigned CNT1 :1;
        unsigned CNT2 :1;
        unsigned CNT3 :1;
        unsigned CNT4 :1;
        unsigned CNT5 :1;
        unsigned CNT6 :1;
        unsigned CNT7 :1;
    };
} ADCNTbits_t;
extern volatile ADCNTbits_t ADCNTbits __attribute__((address(0x099)));
# 2863 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADRPT __attribute__((address(0x09A)));

__asm("ADRPT equ 09Ah");


typedef union {
    struct {
        unsigned RPT :8;
    };
    struct {
        unsigned ADRPT0 :1;
        unsigned ADRPT1 :1;
        unsigned ADRPT2 :1;
        unsigned ADRPT3 :1;
        unsigned ADRPT4 :1;
        unsigned ADRPT5 :1;
        unsigned ADRPT6 :1;
        unsigned ADRPT7 :1;
    };
    struct {
        unsigned ADRPT :8;
    };
    struct {
        unsigned RPT0 :1;
        unsigned RPT1 :1;
        unsigned RPT2 :1;
        unsigned RPT3 :1;
        unsigned RPT4 :1;
        unsigned RPT5 :1;
        unsigned RPT6 :1;
        unsigned RPT7 :1;
    };
} ADRPTbits_t;
extern volatile ADRPTbits_t ADRPTbits __attribute__((address(0x09A)));
# 2991 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short ADPREV __attribute__((address(0x09B)));

__asm("ADPREV equ 09Bh");




extern volatile unsigned char ADPREVL __attribute__((address(0x09B)));

__asm("ADPREVL equ 09Bh");


typedef union {
    struct {
        unsigned PREV :8;
    };
    struct {
        unsigned ADPREV0 :1;
        unsigned ADPREV1 :1;
        unsigned ADPREV2 :1;
        unsigned ADPREV3 :1;
        unsigned ADPREV4 :1;
        unsigned ADPREV5 :1;
        unsigned ADPREV6 :1;
        unsigned ADPREV7 :1;
    };
    struct {
        unsigned ADPREV :8;
    };
    struct {
        unsigned PREV0 :1;
        unsigned PREV1 :1;
        unsigned PREV2 :1;
        unsigned PREV3 :1;
        unsigned PREV4 :1;
        unsigned PREV5 :1;
        unsigned PREV6 :1;
        unsigned PREV7 :1;
    };
} ADPREVLbits_t;
extern volatile ADPREVLbits_t ADPREVLbits __attribute__((address(0x09B)));
# 3126 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADPREVH __attribute__((address(0x09C)));

__asm("ADPREVH equ 09Ch");


typedef union {
    struct {
        unsigned PREV :8;
    };
    struct {
        unsigned ADPREV8 :1;
        unsigned ADPREV9 :1;
        unsigned ADPREV10 :1;
        unsigned ADPREV11 :1;
        unsigned ADPREV12 :1;
        unsigned ADPREV13 :1;
        unsigned ADPREV14 :1;
        unsigned ADPREV15 :1;
    };
    struct {
        unsigned ADPREV :8;
    };
    struct {
        unsigned PREV8 :1;
        unsigned PREV9 :1;
        unsigned PREV10 :1;
        unsigned PREV11 :1;
        unsigned PREV12 :1;
        unsigned PREV13 :1;
        unsigned PREV14 :1;
        unsigned PREV15 :1;
    };
} ADPREVHbits_t;
extern volatile ADPREVHbits_t ADPREVHbits __attribute__((address(0x09C)));
# 3254 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short ADRES __attribute__((address(0x09D)));

__asm("ADRES equ 09Dh");




extern volatile unsigned char ADRESL __attribute__((address(0x09D)));

__asm("ADRESL equ 09Dh");


typedef union {
    struct {
        unsigned RES :8;
    };
    struct {
        unsigned ADRES0 :1;
        unsigned ADRES1 :1;
        unsigned ADRES2 :1;
        unsigned ADRES3 :1;
        unsigned ADRES4 :1;
        unsigned ADRES5 :1;
        unsigned ADRES6 :1;
        unsigned ADRES7 :1;
    };
    struct {
        unsigned ADRES :8;
    };
    struct {
        unsigned RES0 :1;
        unsigned RES1 :1;
        unsigned RES2 :1;
        unsigned RES3 :1;
        unsigned RES4 :1;
        unsigned RES5 :1;
        unsigned RES6 :1;
        unsigned RES7 :1;
    };
} ADRESLbits_t;
extern volatile ADRESLbits_t ADRESLbits __attribute__((address(0x09D)));
# 3389 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADRESH __attribute__((address(0x09E)));

__asm("ADRESH equ 09Eh");


typedef union {
    struct {
        unsigned ADRES8 :1;
        unsigned ADRES9 :1;
        unsigned ADRES10 :1;
        unsigned ADRES11 :1;
        unsigned ADRES12 :1;
        unsigned ADRES13 :1;
        unsigned ADRES14 :1;
        unsigned ADRES15 :1;
    };
    struct {
        unsigned ADRES :8;
    };
    struct {
        unsigned RES8 :1;
        unsigned RES9 :1;
        unsigned RES10 :1;
        unsigned RES11 :1;
        unsigned RES12 :1;
        unsigned RES13 :1;
        unsigned RES14 :1;
        unsigned RES15 :1;
    };
} ADRESHbits_t;
extern volatile ADRESHbits_t ADRESHbits __attribute__((address(0x09E)));
# 3509 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADPCH __attribute__((address(0x09F)));

__asm("ADPCH equ 09Fh");


typedef union {
    struct {
        unsigned ADPCH :6;
    };
    struct {
        unsigned ADPCH0 :1;
        unsigned ADPCH1 :1;
        unsigned ADPCH2 :1;
        unsigned ADPCH3 :1;
        unsigned ADPCH4 :1;
        unsigned ADPCH5 :1;
    };
} ADPCHbits_t;
extern volatile ADPCHbits_t ADPCHbits __attribute__((address(0x09F)));
# 3567 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short ADACQ __attribute__((address(0x10C)));

__asm("ADACQ equ 010Ch");




extern volatile unsigned char ADACQL __attribute__((address(0x10C)));

__asm("ADACQL equ 010Ch");


typedef union {
    struct {
        unsigned ACQ :8;
    };
    struct {
        unsigned ADACQ0 :1;
        unsigned ADACQ1 :1;
        unsigned ADACQ2 :1;
        unsigned ADACQ3 :1;
        unsigned ADACQ4 :1;
        unsigned ADACQ5 :1;
        unsigned ADACQ6 :1;
        unsigned ADACQ7 :1;
    };
    struct {
        unsigned ADACQ :8;
    };
    struct {
        unsigned ACQ0 :1;
        unsigned ACQ1 :1;
        unsigned ACQ2 :1;
        unsigned ACQ3 :1;
        unsigned ACQ4 :1;
        unsigned ACQ5 :1;
        unsigned ACQ6 :1;
        unsigned ACQ7 :1;
    };
} ADACQLbits_t;
extern volatile ADACQLbits_t ADACQLbits __attribute__((address(0x10C)));
# 3702 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADACQH __attribute__((address(0x10D)));

__asm("ADACQH equ 010Dh");


typedef union {
    struct {
        unsigned ACQ :5;
    };
    struct {
        unsigned ADACQ8 :1;
        unsigned ADACQ9 :1;
        unsigned ADACQ10 :1;
        unsigned ADACQ11 :1;
        unsigned ADACQ12 :1;
    };
    struct {
        unsigned ADACQ :5;
    };
    struct {
        unsigned ACQ8 :1;
        unsigned ACQ9 :1;
        unsigned ACQ10 :1;
        unsigned ACQ11 :1;
        unsigned ACQ12 :1;
    };
} ADACQHbits_t;
extern volatile ADACQHbits_t ADACQHbits __attribute__((address(0x10D)));
# 3794 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADCAP __attribute__((address(0x10E)));

__asm("ADCAP equ 010Eh");


typedef union {
    struct {
        unsigned ADCAP :5;
    };
    struct {
        unsigned ADCAP0 :1;
        unsigned ADCAP1 :1;
        unsigned ADCAP2 :1;
        unsigned ADCAP3 :1;
        unsigned ADCAP4 :1;
    };
} ADCAPbits_t;
extern volatile ADCAPbits_t ADCAPbits __attribute__((address(0x10E)));
# 3846 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short ADPRE __attribute__((address(0x10F)));

__asm("ADPRE equ 010Fh");




extern volatile unsigned char ADPREL __attribute__((address(0x10F)));

__asm("ADPREL equ 010Fh");


typedef union {
    struct {
        unsigned PRE :8;
    };
    struct {
        unsigned PRE0 :1;
        unsigned PRE1 :1;
        unsigned PRE2 :1;
        unsigned PRE3 :1;
        unsigned PRE4 :1;
        unsigned PRE5 :1;
        unsigned PRE6 :1;
        unsigned PRE7 :1;
    };
    struct {
        unsigned ADPRE :8;
    };
    struct {
        unsigned ADPRE0 :1;
        unsigned ADPRE1 :1;
        unsigned ADPRE2 :1;
        unsigned ADPRE3 :1;
        unsigned ADPRE4 :1;
        unsigned ADPRE5 :1;
        unsigned ADPRE6 :1;
        unsigned ADPRE7 :1;
    };
} ADPRELbits_t;
extern volatile ADPRELbits_t ADPRELbits __attribute__((address(0x10F)));
# 3981 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADPREH __attribute__((address(0x110)));

__asm("ADPREH equ 0110h");


typedef union {
    struct {
        unsigned PRE :5;
    };
    struct {
        unsigned PRE8 :1;
        unsigned PRE9 :1;
        unsigned PRE10 :1;
        unsigned PRE11 :1;
        unsigned PRE12 :1;
    };
    struct {
        unsigned ADPRE :5;
    };
    struct {
        unsigned ADPRE8 :1;
        unsigned ADPRE9 :1;
        unsigned ADPRE10 :1;
        unsigned ADPRE11 :1;
        unsigned ADPRE12 :1;
    };
} ADPREHbits_t;
extern volatile ADPREHbits_t ADPREHbits __attribute__((address(0x110)));
# 4073 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADCON0 __attribute__((address(0x111)));

__asm("ADCON0 equ 0111h");


typedef union {
    struct {
        unsigned GO :1;
        unsigned :1;
        unsigned FM :1;
        unsigned :1;
        unsigned CS :1;
        unsigned :1;
        unsigned CONT :1;
        unsigned ON :1;
    };
    struct {
        unsigned ADGO :1;
        unsigned :1;
        unsigned ADFM :1;
        unsigned :1;
        unsigned ADCS :1;
        unsigned :1;
        unsigned ADCONT :1;
        unsigned ADON :1;
    };
    struct {
        unsigned DONE :1;
    };
    struct {
        unsigned GO_nDONE :1;
    };
} ADCON0bits_t;
extern volatile ADCON0bits_t ADCON0bits __attribute__((address(0x111)));
# 4171 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADCON1 __attribute__((address(0x112)));

__asm("ADCON1 equ 0112h");


typedef union {
    struct {
        unsigned DSEN :1;
        unsigned :4;
        unsigned GPOL :1;
        unsigned IPEN :1;
        unsigned PPOL :1;
    };
    struct {
        unsigned ADDSEN :1;
        unsigned :4;
        unsigned ADGPOL :1;
        unsigned ADIPEN :1;
        unsigned ADPPOL :1;
    };
} ADCON1bits_t;
extern volatile ADCON1bits_t ADCON1bits __attribute__((address(0x112)));
# 4237 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADCON2 __attribute__((address(0x113)));

__asm("ADCON2 equ 0113h");


typedef union {
    struct {
        unsigned MODE :3;
        unsigned ACLR :1;
        unsigned CRS :3;
        unsigned PSIS :1;
    };
    struct {
        unsigned ADMD0 :1;
        unsigned ADMD1 :1;
        unsigned ADMD2 :1;
        unsigned ADACLR :1;
        unsigned ADCRS0 :1;
        unsigned ADCRS1 :1;
        unsigned ADCRS2 :1;
        unsigned ADPSIS :1;
    };
    struct {
        unsigned ADMD :3;
        unsigned :1;
        unsigned ADCRS :3;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned :1;
        unsigned CRS0 :1;
        unsigned CRS1 :1;
        unsigned CRS2 :1;
    };
    struct {
        unsigned MD :3;
    };
    struct {
        unsigned MD0 :1;
        unsigned MD1 :1;
        unsigned MD2 :1;
    };
} ADCON2bits_t;
extern volatile ADCON2bits_t ADCON2bits __attribute__((address(0x113)));
# 4407 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADCON3 __attribute__((address(0x114)));

__asm("ADCON3 equ 0114h");


typedef union {
    struct {
        unsigned TMD :3;
        unsigned SOI :1;
        unsigned CALC :3;
    };
    struct {
        unsigned ADTMD0 :1;
        unsigned ADTMD1 :1;
        unsigned ADTMD2 :1;
        unsigned ADSOI :1;
        unsigned ADCALC0 :1;
        unsigned ADCALC1 :1;
        unsigned ADCALC2 :1;
    };
    struct {
        unsigned ADTMD :3;
        unsigned :1;
        unsigned ADCALC :3;
    };
    struct {
        unsigned TMD0 :1;
        unsigned TMD1 :1;
        unsigned TMD2 :1;
        unsigned :1;
        unsigned CALC0 :1;
        unsigned CALC1 :1;
        unsigned CALC2 :1;
    };
} ADCON3bits_t;
extern volatile ADCON3bits_t ADCON3bits __attribute__((address(0x114)));
# 4537 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADSTAT __attribute__((address(0x115)));

__asm("ADSTAT equ 0115h");


typedef union {
    struct {
        unsigned STAT :3;
        unsigned :1;
        unsigned MATH :1;
        unsigned LTHR :1;
        unsigned UTHR :1;
        unsigned OV :1;
    };
    struct {
        unsigned ADSTAT0 :1;
        unsigned ADSTAT1 :1;
        unsigned ADSTAT2 :1;
        unsigned :1;
        unsigned ADMATH :1;
        unsigned ADLTHR :1;
        unsigned ADUTHR :1;
        unsigned ADAOV :1;
    };
    struct {
        unsigned ADSTAT :3;
    };
    struct {
        unsigned STAT0 :1;
        unsigned STAT1 :1;
        unsigned STAT2 :1;
    };
} ADSTATbits_t;
extern volatile ADSTATbits_t ADSTATbits __attribute__((address(0x115)));
# 4655 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADREF __attribute__((address(0x116)));

__asm("ADREF equ 0116h");


typedef union {
    struct {
        unsigned PREF :2;
    };
    struct {
        unsigned ADPREF :2;
    };
    struct {
        unsigned PREF0 :1;
        unsigned PREF1 :1;
    };
    struct {
        unsigned ADPREF0 :1;
        unsigned ADPREF1 :1;
    };
} ADREFbits_t;
extern volatile ADREFbits_t ADREFbits __attribute__((address(0x116)));
# 4711 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADACT __attribute__((address(0x117)));

__asm("ADACT equ 0117h");


typedef union {
    struct {
        unsigned ACT :5;
    };
    struct {
        unsigned ADACT0 :1;
        unsigned ADACT1 :1;
        unsigned ADACT2 :1;
        unsigned ADACT3 :1;
        unsigned ADACT4 :1;
    };
    struct {
        unsigned ADACT :5;
    };
    struct {
        unsigned ACT0 :1;
        unsigned ACT1 :1;
        unsigned ACT2 :1;
        unsigned ACT3 :1;
        unsigned ACT4 :1;
    };
} ADACTbits_t;
extern volatile ADACTbits_t ADACTbits __attribute__((address(0x117)));
# 4803 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADCLK __attribute__((address(0x118)));

__asm("ADCLK equ 0118h");


typedef union {
    struct {
        unsigned CS :6;
    };
    struct {
        unsigned ADCS0 :1;
        unsigned ADCS1 :1;
        unsigned ADCS2 :1;
        unsigned ADCS3 :1;
        unsigned ADCS4 :1;
        unsigned ADCS5 :1;
    };
    struct {
        unsigned ADCS :6;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
        unsigned CS4 :1;
        unsigned CS5 :1;
    };
} ADCLKbits_t;
extern volatile ADCLKbits_t ADCLKbits __attribute__((address(0x118)));
# 4907 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC1REG __attribute__((address(0x119)));

__asm("RC1REG equ 0119h");


extern volatile unsigned char RCREG __attribute__((address(0x119)));

__asm("RCREG equ 0119h");

extern volatile unsigned char RCREG1 __attribute__((address(0x119)));

__asm("RCREG1 equ 0119h");


typedef union {
    struct {
        unsigned RC1REG :8;
    };
} RC1REGbits_t;
extern volatile RC1REGbits_t RC1REGbits __attribute__((address(0x119)));







typedef union {
    struct {
        unsigned RC1REG :8;
    };
} RCREGbits_t;
extern volatile RCREGbits_t RCREGbits __attribute__((address(0x119)));






typedef union {
    struct {
        unsigned RC1REG :8;
    };
} RCREG1bits_t;
extern volatile RCREG1bits_t RCREG1bits __attribute__((address(0x119)));
# 4961 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TX1REG __attribute__((address(0x11A)));

__asm("TX1REG equ 011Ah");


extern volatile unsigned char TXREG1 __attribute__((address(0x11A)));

__asm("TXREG1 equ 011Ah");

extern volatile unsigned char TXREG __attribute__((address(0x11A)));

__asm("TXREG equ 011Ah");


typedef union {
    struct {
        unsigned TX1REG :8;
    };
} TX1REGbits_t;
extern volatile TX1REGbits_t TX1REGbits __attribute__((address(0x11A)));







typedef union {
    struct {
        unsigned TX1REG :8;
    };
} TXREG1bits_t;
extern volatile TXREG1bits_t TXREG1bits __attribute__((address(0x11A)));






typedef union {
    struct {
        unsigned TX1REG :8;
    };
} TXREGbits_t;
extern volatile TXREGbits_t TXREGbits __attribute__((address(0x11A)));
# 5015 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short SP1BRG __attribute__((address(0x11B)));

__asm("SP1BRG equ 011Bh");




extern volatile unsigned char SP1BRGL __attribute__((address(0x11B)));

__asm("SP1BRGL equ 011Bh");


extern volatile unsigned char SPBRG __attribute__((address(0x11B)));

__asm("SPBRG equ 011Bh");

extern volatile unsigned char SPBRG1 __attribute__((address(0x11B)));

__asm("SPBRG1 equ 011Bh");

extern volatile unsigned char SPBRGL __attribute__((address(0x11B)));

__asm("SPBRGL equ 011Bh");


typedef union {
    struct {
        unsigned SP1BRGL :8;
    };
} SP1BRGLbits_t;
extern volatile SP1BRGLbits_t SP1BRGLbits __attribute__((address(0x11B)));







typedef union {
    struct {
        unsigned SP1BRGL :8;
    };
} SPBRGbits_t;
extern volatile SPBRGbits_t SPBRGbits __attribute__((address(0x11B)));






typedef union {
    struct {
        unsigned SP1BRGL :8;
    };
} SPBRG1bits_t;
extern volatile SPBRG1bits_t SPBRG1bits __attribute__((address(0x11B)));






typedef union {
    struct {
        unsigned SP1BRGL :8;
    };
} SPBRGLbits_t;
extern volatile SPBRGLbits_t SPBRGLbits __attribute__((address(0x11B)));
# 5092 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SP1BRGH __attribute__((address(0x11C)));

__asm("SP1BRGH equ 011Ch");


extern volatile unsigned char SPBRGH __attribute__((address(0x11C)));

__asm("SPBRGH equ 011Ch");

extern volatile unsigned char SPBRGH1 __attribute__((address(0x11C)));

__asm("SPBRGH1 equ 011Ch");


typedef union {
    struct {
        unsigned SP1BRGH :8;
    };
} SP1BRGHbits_t;
extern volatile SP1BRGHbits_t SP1BRGHbits __attribute__((address(0x11C)));







typedef union {
    struct {
        unsigned SP1BRGH :8;
    };
} SPBRGHbits_t;
extern volatile SPBRGHbits_t SPBRGHbits __attribute__((address(0x11C)));






typedef union {
    struct {
        unsigned SP1BRGH :8;
    };
} SPBRGH1bits_t;
extern volatile SPBRGH1bits_t SPBRGH1bits __attribute__((address(0x11C)));
# 5146 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC1STA __attribute__((address(0x11D)));

__asm("RC1STA equ 011Dh");


extern volatile unsigned char RCSTA1 __attribute__((address(0x11D)));

__asm("RCSTA1 equ 011Dh");

extern volatile unsigned char RCSTA __attribute__((address(0x11D)));

__asm("RCSTA equ 011Dh");


typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
} RC1STAbits_t;
extern volatile RC1STAbits_t RC1STAbits __attribute__((address(0x11D)));
# 5215 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
} RCSTA1bits_t;
extern volatile RCSTA1bits_t RCSTA1bits __attribute__((address(0x11D)));
# 5269 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
} RCSTAbits_t;
extern volatile RCSTAbits_t RCSTAbits __attribute__((address(0x11D)));
# 5326 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TX1STA __attribute__((address(0x11E)));

__asm("TX1STA equ 011Eh");


extern volatile unsigned char TXSTA1 __attribute__((address(0x11E)));

__asm("TXSTA1 equ 011Eh");

extern volatile unsigned char TXSTA __attribute__((address(0x11E)));

__asm("TXSTA equ 011Eh");


typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
} TX1STAbits_t;
extern volatile TX1STAbits_t TX1STAbits __attribute__((address(0x11E)));
# 5395 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
} TXSTA1bits_t;
extern volatile TXSTA1bits_t TXSTA1bits __attribute__((address(0x11E)));
# 5449 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
} TXSTAbits_t;
extern volatile TXSTAbits_t TXSTAbits __attribute__((address(0x11E)));
# 5506 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char BAUD1CON __attribute__((address(0x11F)));

__asm("BAUD1CON equ 011Fh");


extern volatile unsigned char BAUDCON1 __attribute__((address(0x11F)));

__asm("BAUDCON1 equ 011Fh");

extern volatile unsigned char BAUDCTL1 __attribute__((address(0x11F)));

__asm("BAUDCTL1 equ 011Fh");

extern volatile unsigned char BAUDCON __attribute__((address(0x11F)));

__asm("BAUDCON equ 011Fh");

extern volatile unsigned char BAUDCTL __attribute__((address(0x11F)));

__asm("BAUDCTL equ 011Fh");


typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUD1CONbits_t;
extern volatile BAUD1CONbits_t BAUD1CONbits __attribute__((address(0x11F)));
# 5573 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUDCON1bits_t;
extern volatile BAUDCON1bits_t BAUDCON1bits __attribute__((address(0x11F)));
# 5617 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUDCTL1bits_t;
extern volatile BAUDCTL1bits_t BAUDCTL1bits __attribute__((address(0x11F)));
# 5661 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUDCONbits_t;
extern volatile BAUDCONbits_t BAUDCONbits __attribute__((address(0x11F)));
# 5705 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUDCTLbits_t;
extern volatile BAUDCTLbits_t BAUDCTLbits __attribute__((address(0x11F)));
# 5752 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SSP1BUF __attribute__((address(0x18C)));

__asm("SSP1BUF equ 018Ch");


typedef union {
    struct {
        unsigned SSPBUF :8;
    };
} SSP1BUFbits_t;
extern volatile SSP1BUFbits_t SSP1BUFbits __attribute__((address(0x18C)));
# 5772 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SSP1ADD __attribute__((address(0x18D)));

__asm("SSP1ADD equ 018Dh");


typedef union {
    struct {
        unsigned SSPADD :8;
    };
    struct {
        unsigned MSK0 :1;
        unsigned MSK1 :1;
        unsigned MSK2 :1;
        unsigned MSK3 :1;
        unsigned MSK4 :1;
        unsigned MSK5 :1;
        unsigned MSK6 :1;
        unsigned MSK7 :1;
    };
    struct {
        unsigned MSK01 :1;
        unsigned MSK11 :1;
        unsigned MSK21 :1;
        unsigned MSK31 :1;
        unsigned MSK41 :1;
        unsigned MSK51 :1;
        unsigned MSK61 :1;
        unsigned MSK71 :1;
    };
} SSP1ADDbits_t;
extern volatile SSP1ADDbits_t SSP1ADDbits __attribute__((address(0x18D)));
# 5892 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SSP1MSK __attribute__((address(0x18E)));

__asm("SSP1MSK equ 018Eh");


typedef union {
    struct {
        unsigned SSPMSK :8;
    };
    struct {
        unsigned MSK0 :1;
        unsigned MSK1 :1;
        unsigned MSK2 :1;
        unsigned MSK3 :1;
        unsigned MSK4 :1;
        unsigned MSK5 :1;
        unsigned MSK6 :1;
        unsigned MSK7 :1;
    };
} SSP1MSKbits_t;
extern volatile SSP1MSKbits_t SSP1MSKbits __attribute__((address(0x18E)));
# 5962 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SSP1STAT __attribute__((address(0x18F)));

__asm("SSP1STAT equ 018Fh");


typedef union {
    struct {
        unsigned BF :1;
        unsigned UA :1;
        unsigned R_nW :1;
        unsigned S :1;
        unsigned P :1;
        unsigned D_nA :1;
        unsigned CKE :1;
        unsigned SMP :1;
    };
    struct {
        unsigned :2;
        unsigned R_W :1;
        unsigned :2;
        unsigned D_A :1;
    };
    struct {
        unsigned :2;
        unsigned nW :1;
        unsigned :2;
        unsigned nA :1;
    };
    struct {
        unsigned :2;
        unsigned nWRITE :1;
        unsigned :2;
        unsigned nADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned READ_WRITE :1;
        unsigned :2;
        unsigned DATA_ADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned I2C_READ :1;
        unsigned I2C_START :1;
        unsigned I2C_STOP :1;
        unsigned I2C_DAT :1;
    };
    struct {
        unsigned BF1 :1;
        unsigned UA1 :1;
        unsigned R :1;
        unsigned START :1;
        unsigned STOP :1;
        unsigned D :1;
        unsigned CKE1 :1;
        unsigned SMP1 :1;
    };
    struct {
        unsigned :2;
        unsigned RW :1;
        unsigned START1 :1;
        unsigned STOP1 :1;
        unsigned DA :1;
    };
    struct {
        unsigned :2;
        unsigned RW1 :1;
        unsigned I2C_START1 :1;
        unsigned I2C_STOP2 :1;
        unsigned DA1 :1;
    };
    struct {
        unsigned :2;
        unsigned I2C_READ1 :1;
        unsigned S2 :1;
        unsigned P2 :1;
        unsigned DATA_ADDRESS1 :1;
    };
    struct {
        unsigned :2;
        unsigned READ_WRITE1 :1;
        unsigned :2;
        unsigned D_A1 :1;
    };
    struct {
        unsigned :2;
        unsigned R_W1 :1;
        unsigned :2;
        unsigned D_nA1 :1;
    };
    struct {
        unsigned :2;
        unsigned R_nW1 :1;
        unsigned :2;
        unsigned I2C_DAT1 :1;
    };
    struct {
        unsigned :2;
        unsigned nW2 :1;
        unsigned :2;
        unsigned nA2 :1;
    };
    struct {
        unsigned :2;
        unsigned nWRITE1 :1;
        unsigned :2;
        unsigned nADDRESS1 :1;
    };
} SSP1STATbits_t;
extern volatile SSP1STATbits_t SSP1STATbits __attribute__((address(0x18F)));
# 6326 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SSP1CON1 __attribute__((address(0x190)));

__asm("SSP1CON1 equ 0190h");


typedef union {
    struct {
        unsigned SSPM :4;
        unsigned CKP :1;
        unsigned SSPEN :1;
        unsigned SSPOV :1;
        unsigned WCOL :1;
    };
    struct {
        unsigned SSPM0 :1;
        unsigned SSPM1 :1;
        unsigned SSPM2 :1;
        unsigned SSPM3 :1;
    };
    struct {
        unsigned SSPM01 :1;
        unsigned SSPM11 :1;
        unsigned SSPM21 :1;
        unsigned SSPM31 :1;
        unsigned CKP1 :1;
        unsigned SSPEN1 :1;
        unsigned SSPOV1 :1;
        unsigned WCOL1 :1;
    };
} SSP1CON1bits_t;
extern volatile SSP1CON1bits_t SSP1CON1bits __attribute__((address(0x190)));
# 6446 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SSP1CON2 __attribute__((address(0x191)));

__asm("SSP1CON2 equ 0191h");


typedef union {
    struct {
        unsigned SEN :1;
        unsigned RSEN :1;
        unsigned PEN :1;
        unsigned RCEN :1;
        unsigned ACKEN :1;
        unsigned ACKDT :1;
        unsigned ACKSTAT :1;
        unsigned GCEN :1;
    };
    struct {
        unsigned :1;
        unsigned ADMSK :5;
    };
    struct {
        unsigned :1;
        unsigned ADMSK1 :1;
        unsigned ADMSK2 :1;
        unsigned ADMSK3 :1;
        unsigned ADMSK4 :1;
        unsigned ADMSK5 :1;
    };
    struct {
        unsigned SEN1 :1;
        unsigned ADMSK11 :1;
        unsigned ADMSK21 :1;
        unsigned ADMSK31 :1;
        unsigned ACKEN1 :1;
        unsigned ACKDT1 :1;
        unsigned ACKSTAT1 :1;
        unsigned GCEN1 :1;
    };
    struct {
        unsigned :1;
        unsigned RSEN1 :1;
        unsigned PEN1 :1;
        unsigned RCEN1 :1;
        unsigned ADMSK41 :1;
        unsigned ADMSK51 :1;
    };
} SSP1CON2bits_t;
extern volatile SSP1CON2bits_t SSP1CON2bits __attribute__((address(0x191)));
# 6633 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SSP1CON3 __attribute__((address(0x192)));

__asm("SSP1CON3 equ 0192h");


typedef union {
    struct {
        unsigned DHEN :1;
        unsigned AHEN :1;
        unsigned SBCDE :1;
        unsigned SDAHT :1;
        unsigned BOEN :1;
        unsigned SCIE :1;
        unsigned PCIE :1;
        unsigned ACKTIM :1;
    };
} SSP1CON3bits_t;
extern volatile SSP1CON3bits_t SSP1CON3bits __attribute__((address(0x192)));
# 6695 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short TMR1 __attribute__((address(0x20C)));

__asm("TMR1 equ 020Ch");




extern volatile unsigned char TMR1L __attribute__((address(0x20C)));

__asm("TMR1L equ 020Ch");


typedef union {
    struct {
        unsigned TMR1L0 :1;
        unsigned TMR1L1 :1;
        unsigned TMR1L2 :1;
        unsigned TMR1L3 :1;
        unsigned TMR1L4 :1;
        unsigned TMR1L5 :1;
        unsigned TMR1L6 :1;
        unsigned TMR1L7 :1;
    };
    struct {
        unsigned TMR1L :8;
    };
    struct {
        unsigned TMR10 :1;
        unsigned TMR11 :1;
        unsigned TMR12 :1;
        unsigned TMR13 :1;
        unsigned TMR14 :1;
        unsigned TMR15 :1;
        unsigned TMR16 :1;
        unsigned TMR17 :1;
    };
    struct {
        unsigned CAL01 :1;
        unsigned CAL11 :1;
        unsigned CAL21 :1;
        unsigned CAL31 :1;
        unsigned CAL41 :1;
        unsigned CAL51 :1;
        unsigned CAL61 :1;
        unsigned CAL71 :1;
    };
} TMR1Lbits_t;
extern volatile TMR1Lbits_t TMR1Lbits __attribute__((address(0x20C)));
# 6872 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TMR1H __attribute__((address(0x20D)));

__asm("TMR1H equ 020Dh");


typedef union {
    struct {
        unsigned TMR1H0 :1;
        unsigned TMR1H1 :1;
        unsigned TMR1H2 :1;
        unsigned TMR1H3 :1;
        unsigned TMR1H4 :1;
        unsigned TMR1H5 :1;
        unsigned TMR1H6 :1;
        unsigned TMR1H7 :1;
    };
    struct {
        unsigned TMR1H :8;
    };
    struct {
        unsigned TMR18 :1;
        unsigned TMR19 :1;
        unsigned TMR110 :1;
        unsigned TMR111 :1;
        unsigned TMR112 :1;
        unsigned TMR113 :1;
        unsigned TMR114 :1;
        unsigned TMR115 :1;
    };
} TMR1Hbits_t;
extern volatile TMR1Hbits_t TMR1Hbits __attribute__((address(0x20D)));
# 6992 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T1CON __attribute__((address(0x20E)));

__asm("T1CON equ 020Eh");


typedef union {
    struct {
        unsigned ON :1;
        unsigned RD16 :1;
        unsigned nSYNC :1;
        unsigned :1;
        unsigned CKPS :2;
    };
    struct {
        unsigned TMR1ON :1;
        unsigned T1RD16 :1;
        unsigned nT1SYNC :1;
        unsigned :1;
        unsigned T1CKPS0 :1;
        unsigned T1CKPS1 :1;
    };
    struct {
        unsigned :4;
        unsigned CKPS0 :1;
        unsigned CKPS1 :1;
    };
    struct {
        unsigned :1;
        unsigned RD161 :1;
    };
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __attribute__((address(0x20E)));
# 7088 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T1GCON __attribute__((address(0x20F)));

__asm("T1GCON equ 020Fh");


extern volatile unsigned char PR1 __attribute__((address(0x20F)));

__asm("PR1 equ 020Fh");


typedef union {
    struct {
        unsigned :2;
        unsigned GVAL :1;
        unsigned GGO_nDONE :1;
        unsigned GSPM :1;
        unsigned GTM :1;
        unsigned GPOL :1;
        unsigned GE :1;
    };
    struct {
        unsigned :2;
        unsigned T1GVAL :1;
        unsigned T1GGO_nDONE :1;
        unsigned T1GSPM :1;
        unsigned T1GTM :1;
        unsigned T1GPOL :1;
        unsigned T1GE :1;
    };
    struct {
        unsigned :3;
        unsigned T1GGO :1;
    };
} T1GCONbits_t;
extern volatile T1GCONbits_t T1GCONbits __attribute__((address(0x20F)));
# 7190 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned :2;
        unsigned GVAL :1;
        unsigned GGO_nDONE :1;
        unsigned GSPM :1;
        unsigned GTM :1;
        unsigned GPOL :1;
        unsigned GE :1;
    };
    struct {
        unsigned :2;
        unsigned T1GVAL :1;
        unsigned T1GGO_nDONE :1;
        unsigned T1GSPM :1;
        unsigned T1GTM :1;
        unsigned T1GPOL :1;
        unsigned T1GE :1;
    };
    struct {
        unsigned :3;
        unsigned T1GGO :1;
    };
} PR1bits_t;
extern volatile PR1bits_t PR1bits __attribute__((address(0x20F)));
# 7284 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T1GATE __attribute__((address(0x210)));

__asm("T1GATE equ 0210h");


extern volatile unsigned char TMR1GATE __attribute__((address(0x210)));

__asm("TMR1GATE equ 0210h");


typedef union {
    struct {
        unsigned GSS :5;
    };
    struct {
        unsigned GSS0 :1;
        unsigned GSS1 :1;
        unsigned GSS2 :1;
        unsigned GSS3 :1;
        unsigned GSS4 :1;
    };
    struct {
        unsigned T1GSS0 :1;
        unsigned T1GSS1 :1;
        unsigned T1GSS2 :1;
        unsigned T1GSS3 :1;
        unsigned T1GSS4 :1;
    };
} T1GATEbits_t;
extern volatile T1GATEbits_t T1GATEbits __attribute__((address(0x210)));
# 7371 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned GSS :5;
    };
    struct {
        unsigned GSS0 :1;
        unsigned GSS1 :1;
        unsigned GSS2 :1;
        unsigned GSS3 :1;
        unsigned GSS4 :1;
    };
    struct {
        unsigned T1GSS0 :1;
        unsigned T1GSS1 :1;
        unsigned T1GSS2 :1;
        unsigned T1GSS3 :1;
        unsigned T1GSS4 :1;
    };
} TMR1GATEbits_t;
extern volatile TMR1GATEbits_t TMR1GATEbits __attribute__((address(0x210)));
# 7450 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T1CLK __attribute__((address(0x211)));

__asm("T1CLK equ 0211h");


extern volatile unsigned char TMR1CLK __attribute__((address(0x211)));

__asm("TMR1CLK equ 0211h");


typedef union {
    struct {
        unsigned CS :4;
    };
    struct {
        unsigned T1CS0 :1;
        unsigned T1CS1 :1;
        unsigned T1CS2 :1;
        unsigned T1CS3 :1;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
} T1CLKbits_t;
extern volatile T1CLKbits_t T1CLKbits __attribute__((address(0x211)));
# 7525 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned CS :4;
    };
    struct {
        unsigned T1CS0 :1;
        unsigned T1CS1 :1;
        unsigned T1CS2 :1;
        unsigned T1CS3 :1;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
} TMR1CLKbits_t;
extern volatile TMR1CLKbits_t TMR1CLKbits __attribute__((address(0x211)));
# 7592 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CCPTMRS0 __attribute__((address(0x21E)));

__asm("CCPTMRS0 equ 021Eh");


typedef union {
    struct {
        unsigned C1TSEL :2;
        unsigned C2TSEL :2;
        unsigned C3TSEL :2;
        unsigned C4TSEL :2;
    };
    struct {
        unsigned C1TSEL0 :1;
        unsigned C1TSEL1 :1;
        unsigned C2TSEL0 :1;
        unsigned C2TSEL1 :1;
        unsigned C3TSEL0 :1;
        unsigned C3TSEL1 :1;
        unsigned C4TSEL0 :1;
        unsigned C4TSEL1 :1;
    };
} CCPTMRS0bits_t;
extern volatile CCPTMRS0bits_t CCPTMRS0bits __attribute__((address(0x21E)));
# 7680 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T2TMR __attribute__((address(0x28C)));

__asm("T2TMR equ 028Ch");


extern volatile unsigned char TMR2 __attribute__((address(0x28C)));

__asm("TMR2 equ 028Ch");


typedef union {
    struct {
        unsigned TMR2 :8;
    };
} T2TMRbits_t;
extern volatile T2TMRbits_t T2TMRbits __attribute__((address(0x28C)));







typedef union {
    struct {
        unsigned TMR2 :8;
    };
} TMR2bits_t;
extern volatile TMR2bits_t TMR2bits __attribute__((address(0x28C)));
# 7718 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T2PR __attribute__((address(0x28D)));

__asm("T2PR equ 028Dh");


extern volatile unsigned char PR2 __attribute__((address(0x28D)));

__asm("PR2 equ 028Dh");


typedef union {
    struct {
        unsigned PR2 :8;
    };
} T2PRbits_t;
extern volatile T2PRbits_t T2PRbits __attribute__((address(0x28D)));







typedef union {
    struct {
        unsigned PR2 :8;
    };
} PR2bits_t;
extern volatile PR2bits_t PR2bits __attribute__((address(0x28D)));
# 7756 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T2CON __attribute__((address(0x28E)));

__asm("T2CON equ 028Eh");


typedef union {
    struct {
        unsigned OUTPS :4;
        unsigned CKPS :3;
        unsigned ON :1;
    };
    struct {
        unsigned T2OUTPS :4;
        unsigned T2CKPS :3;
        unsigned T2ON :1;
    };
    struct {
        unsigned T2OUTPS0 :1;
        unsigned T2OUTPS1 :1;
        unsigned T2OUTPS2 :1;
        unsigned T2OUTPS3 :1;
        unsigned T2CKPS0 :1;
        unsigned T2CKPS1 :1;
        unsigned T2CKPS2 :1;
    };
    struct {
        unsigned OUTPS0 :1;
        unsigned OUTPS1 :1;
        unsigned OUTPS2 :1;
        unsigned OUTPS3 :1;
        unsigned CKPS0 :1;
        unsigned CKPS1 :1;
        unsigned CKPS2 :1;
        unsigned TMR2ON :1;
    };
} T2CONbits_t;
extern volatile T2CONbits_t T2CONbits __attribute__((address(0x28E)));
# 7902 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T2HLT __attribute__((address(0x28F)));

__asm("T2HLT equ 028Fh");


typedef union {
    struct {
        unsigned MODE :5;
        unsigned CKSYNC :1;
        unsigned CKPOL :1;
        unsigned PSYNC :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned MODE3 :1;
        unsigned MODE4 :1;
    };
    struct {
        unsigned T2MODE :5;
        unsigned T2CKSYNC :1;
        unsigned T2CKPOL :1;
        unsigned T2PSYNC :1;
    };
    struct {
        unsigned T2MODE0 :1;
        unsigned T2MODE1 :1;
        unsigned T2MODE2 :1;
        unsigned T2MODE3 :1;
        unsigned T2MODE4 :1;
    };
} T2HLTbits_t;
extern volatile T2HLTbits_t T2HLTbits __attribute__((address(0x28F)));
# 8030 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T2CLKCON __attribute__((address(0x290)));

__asm("T2CLKCON equ 0290h");


extern volatile unsigned char T2CLK __attribute__((address(0x290)));

__asm("T2CLK equ 0290h");


typedef union {
    struct {
        unsigned CS :4;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
    struct {
        unsigned T2CS :4;
    };
    struct {
        unsigned T2CS0 :1;
        unsigned T2CS1 :1;
        unsigned T2CS2 :1;
        unsigned T2CS3 :1;
    };
} T2CLKCONbits_t;
extern volatile T2CLKCONbits_t T2CLKCONbits __attribute__((address(0x290)));
# 8113 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned CS :4;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
    struct {
        unsigned T2CS :4;
    };
    struct {
        unsigned T2CS0 :1;
        unsigned T2CS1 :1;
        unsigned T2CS2 :1;
        unsigned T2CS3 :1;
    };
} T2CLKbits_t;
extern volatile T2CLKbits_t T2CLKbits __attribute__((address(0x290)));
# 8188 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T2RST __attribute__((address(0x291)));

__asm("T2RST equ 0291h");


typedef union {
    struct {
        unsigned RSEL :5;
    };
    struct {
        unsigned RSEL0 :1;
        unsigned RSEL1 :1;
        unsigned RSEL2 :1;
        unsigned RSEL3 :1;
        unsigned RSEL4 :1;
    };
    struct {
        unsigned T2RSEL :5;
    };
    struct {
        unsigned T2RSEL0 :1;
        unsigned T2RSEL1 :1;
        unsigned T2RSEL2 :1;
        unsigned T2RSEL3 :1;
        unsigned T2RSEL4 :1;
    };
} T2RSTbits_t;
extern volatile T2RSTbits_t T2RSTbits __attribute__((address(0x291)));
# 8280 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T4TMR __attribute__((address(0x292)));

__asm("T4TMR equ 0292h");


extern volatile unsigned char TMR4 __attribute__((address(0x292)));

__asm("TMR4 equ 0292h");


typedef union {
    struct {
        unsigned TMR4 :8;
    };
} T4TMRbits_t;
extern volatile T4TMRbits_t T4TMRbits __attribute__((address(0x292)));







typedef union {
    struct {
        unsigned TMR4 :8;
    };
} TMR4bits_t;
extern volatile TMR4bits_t TMR4bits __attribute__((address(0x292)));
# 8318 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T4PR __attribute__((address(0x293)));

__asm("T4PR equ 0293h");


extern volatile unsigned char PR4 __attribute__((address(0x293)));

__asm("PR4 equ 0293h");


typedef union {
    struct {
        unsigned PR4 :8;
    };
} T4PRbits_t;
extern volatile T4PRbits_t T4PRbits __attribute__((address(0x293)));







typedef union {
    struct {
        unsigned PR4 :8;
    };
} PR4bits_t;
extern volatile PR4bits_t PR4bits __attribute__((address(0x293)));
# 8356 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T4CON __attribute__((address(0x294)));

__asm("T4CON equ 0294h");


typedef union {
    struct {
        unsigned OUTPS :4;
        unsigned CKPS :3;
        unsigned ON :1;
    };
    struct {
        unsigned T4OUTPS :4;
        unsigned T4CKPS :3;
        unsigned T4ON :1;
    };
    struct {
        unsigned T4OUTPS0 :1;
        unsigned T4OUTPS1 :1;
        unsigned T4OUTPS2 :1;
        unsigned T4OUTPS3 :1;
        unsigned T4CKPS0 :1;
        unsigned T4CKPS1 :1;
        unsigned T4CKPS2 :1;
    };
    struct {
        unsigned OUTPS0 :1;
        unsigned OUTPS1 :1;
        unsigned OUTPS2 :1;
        unsigned OUTPS3 :1;
        unsigned CKPS0 :1;
        unsigned CKPS1 :1;
        unsigned CKPS2 :1;
        unsigned TMR4ON :1;
    };
} T4CONbits_t;
extern volatile T4CONbits_t T4CONbits __attribute__((address(0x294)));
# 8502 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T4HLT __attribute__((address(0x295)));

__asm("T4HLT equ 0295h");


typedef union {
    struct {
        unsigned MODE :5;
        unsigned CKSYNC :1;
        unsigned CKPOL :1;
        unsigned PSYNC :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned MODE3 :1;
        unsigned MODE4 :1;
    };
    struct {
        unsigned T4MODE :5;
        unsigned T4CKSYNC :1;
        unsigned T4CKPOL :1;
        unsigned T4PSYNC :1;
    };
    struct {
        unsigned T4MODE0 :1;
        unsigned T4MODE1 :1;
        unsigned T4MODE2 :1;
        unsigned T4MODE3 :1;
        unsigned T4MODE4 :1;
    };
} T4HLTbits_t;
extern volatile T4HLTbits_t T4HLTbits __attribute__((address(0x295)));
# 8630 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T4CLKCON __attribute__((address(0x296)));

__asm("T4CLKCON equ 0296h");


extern volatile unsigned char T4CLK __attribute__((address(0x296)));

__asm("T4CLK equ 0296h");


typedef union {
    struct {
        unsigned CS :4;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
    struct {
        unsigned T4CS :4;
    };
    struct {
        unsigned T4CS0 :1;
        unsigned T4CS1 :1;
        unsigned T4CS2 :1;
        unsigned T4CS3 :1;
    };
} T4CLKCONbits_t;
extern volatile T4CLKCONbits_t T4CLKCONbits __attribute__((address(0x296)));
# 8713 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned CS :4;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
    struct {
        unsigned T4CS :4;
    };
    struct {
        unsigned T4CS0 :1;
        unsigned T4CS1 :1;
        unsigned T4CS2 :1;
        unsigned T4CS3 :1;
    };
} T4CLKbits_t;
extern volatile T4CLKbits_t T4CLKbits __attribute__((address(0x296)));
# 8788 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T4RST __attribute__((address(0x297)));

__asm("T4RST equ 0297h");


typedef union {
    struct {
        unsigned RSEL :5;
    };
    struct {
        unsigned RSEL0 :1;
        unsigned RSEL1 :1;
        unsigned RSEL2 :1;
        unsigned RSEL3 :1;
        unsigned RSEL4 :1;
    };
    struct {
        unsigned T4RSEL :5;
    };
    struct {
        unsigned T4RSEL0 :1;
        unsigned T4RSEL1 :1;
        unsigned T4RSEL2 :1;
        unsigned T4RSEL3 :1;
        unsigned T4RSEL4 :1;
    };
} T4RSTbits_t;
extern volatile T4RSTbits_t T4RSTbits __attribute__((address(0x297)));
# 8880 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short CCPR1 __attribute__((address(0x30C)));

__asm("CCPR1 equ 030Ch");




extern volatile unsigned char CCPR1L __attribute__((address(0x30C)));

__asm("CCPR1L equ 030Ch");


typedef union {
    struct {
        unsigned RL :8;
    };
} CCPR1Lbits_t;
extern volatile CCPR1Lbits_t CCPR1Lbits __attribute__((address(0x30C)));
# 8907 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CCPR1H __attribute__((address(0x30D)));

__asm("CCPR1H equ 030Dh");


typedef union {
    struct {
        unsigned RH :8;
    };
} CCPR1Hbits_t;
extern volatile CCPR1Hbits_t CCPR1Hbits __attribute__((address(0x30D)));
# 8927 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CCP1CON __attribute__((address(0x30E)));

__asm("CCP1CON equ 030Eh");


typedef union {
    struct {
        unsigned MODE :4;
        unsigned FMT :1;
        unsigned OUT :1;
        unsigned OE :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned MODE3 :1;
    };
    struct {
        unsigned CCP1MODE :4;
        unsigned CCP1FMT :1;
        unsigned CCP1OUT :1;
        unsigned CCP1OE :1;
        unsigned CCP1EN :1;
    };
    struct {
        unsigned CCP1MODE0 :1;
        unsigned CCP1MODE1 :1;
        unsigned CCP1MODE2 :1;
        unsigned CCP1MODE3 :1;
    };
    struct {
        unsigned :6;
        unsigned P1M0 :1;
        unsigned P1M1 :1;
    };
} CCP1CONbits_t;
extern volatile CCP1CONbits_t CCP1CONbits __attribute__((address(0x30E)));
# 9070 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CCP1CAP __attribute__((address(0x30F)));

__asm("CCP1CAP equ 030Fh");


typedef union {
    struct {
        unsigned CTS :8;
    };
    struct {
        unsigned CTS0 :1;
        unsigned CTS1 :1;
        unsigned CTS2 :1;
    };
    struct {
        unsigned CCP1CTS :8;
    };
    struct {
        unsigned CCP1CTS0 :1;
        unsigned CCP1CTS1 :1;
        unsigned CCP1CTS2 :1;
    };
} CCP1CAPbits_t;
extern volatile CCP1CAPbits_t CCP1CAPbits __attribute__((address(0x30F)));
# 9138 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short CCPR2 __attribute__((address(0x310)));

__asm("CCPR2 equ 0310h");




extern volatile unsigned char CCPR2L __attribute__((address(0x310)));

__asm("CCPR2L equ 0310h");


typedef union {
    struct {
        unsigned RL :8;
    };
} CCPR2Lbits_t;
extern volatile CCPR2Lbits_t CCPR2Lbits __attribute__((address(0x310)));
# 9165 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CCPR2H __attribute__((address(0x311)));

__asm("CCPR2H equ 0311h");


typedef union {
    struct {
        unsigned RH :8;
    };
} CCPR2Hbits_t;
extern volatile CCPR2Hbits_t CCPR2Hbits __attribute__((address(0x311)));
# 9185 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CCP2CON __attribute__((address(0x312)));

__asm("CCP2CON equ 0312h");


typedef union {
    struct {
        unsigned MODE :4;
        unsigned FMT :1;
        unsigned OUT :1;
        unsigned OE :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned MODE3 :1;
    };
    struct {
        unsigned CCP2MODE :4;
        unsigned CCP2FMT :1;
        unsigned CCP2OUT :1;
        unsigned CCP2OE :1;
        unsigned CCP2EN :1;
    };
    struct {
        unsigned CCP2MODE0 :1;
        unsigned CCP2MODE1 :1;
        unsigned CCP2MODE2 :1;
        unsigned CCP2MODE3 :1;
    };
    struct {
        unsigned :6;
        unsigned P2M0 :1;
        unsigned P2M1 :1;
    };
} CCP2CONbits_t;
extern volatile CCP2CONbits_t CCP2CONbits __attribute__((address(0x312)));
# 9328 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CCP2CAP __attribute__((address(0x313)));

__asm("CCP2CAP equ 0313h");


typedef union {
    struct {
        unsigned CTS :8;
    };
    struct {
        unsigned CTS0 :1;
        unsigned CTS1 :1;
        unsigned CTS2 :1;
    };
    struct {
        unsigned CCP2CTS :8;
    };
    struct {
        unsigned CCP2CTS0 :1;
        unsigned CCP2CTS1 :1;
        unsigned CCP2CTS2 :1;
    };
} CCP2CAPbits_t;
extern volatile CCP2CAPbits_t CCP2CAPbits __attribute__((address(0x313)));
# 9396 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short PWM3DC __attribute__((address(0x314)));

__asm("PWM3DC equ 0314h");




extern volatile unsigned char PWM3DCL __attribute__((address(0x314)));

__asm("PWM3DCL equ 0314h");


typedef union {
    struct {
        unsigned :6;
        unsigned DC :2;
    };
    struct {
        unsigned :6;
        unsigned DC0 :1;
        unsigned DC1 :1;
    };
    struct {
        unsigned :6;
        unsigned PWM3DC0 :1;
        unsigned PWM3DC1 :1;
    };
    struct {
        unsigned :6;
        unsigned PWMPW0 :1;
        unsigned PWMPW1 :1;
    };
} PWM3DCLbits_t;
extern volatile PWM3DCLbits_t PWM3DCLbits __attribute__((address(0x314)));
# 9469 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PWM3DCH __attribute__((address(0x315)));

__asm("PWM3DCH equ 0315h");


typedef union {
    struct {
        unsigned DC :8;
    };
    struct {
        unsigned DC2 :1;
        unsigned DC3 :1;
        unsigned DC4 :1;
        unsigned DC5 :1;
        unsigned DC6 :1;
        unsigned DC7 :1;
        unsigned DC8 :1;
        unsigned DC9 :1;
    };
    struct {
        unsigned PWM3DC2 :1;
        unsigned PWM3DC3 :1;
        unsigned PWM3DC4 :1;
        unsigned PWM3DC5 :1;
        unsigned PWM3DC6 :1;
        unsigned PWM3DC7 :1;
        unsigned PWM3DC8 :1;
        unsigned PWM3DC9 :1;
    };
    struct {
        unsigned PWMPW2 :1;
        unsigned PWMPW3 :1;
        unsigned PWMPW4 :1;
        unsigned PWMPW5 :1;
        unsigned PWMPW6 :1;
        unsigned PWMPW7 :1;
        unsigned PWMPW8 :1;
        unsigned PWMPW9 :1;
    };
} PWM3DCHbits_t;
extern volatile PWM3DCHbits_t PWM3DCHbits __attribute__((address(0x315)));
# 9639 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PWM3CON __attribute__((address(0x316)));

__asm("PWM3CON equ 0316h");


typedef union {
    struct {
        unsigned :4;
        unsigned POL :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned :4;
        unsigned PWM3POL :1;
        unsigned PWM3OUT :1;
        unsigned :1;
        unsigned PWM3EN :1;
    };
} PWM3CONbits_t;
extern volatile PWM3CONbits_t PWM3CONbits __attribute__((address(0x316)));
# 9695 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short PWM4DC __attribute__((address(0x318)));

__asm("PWM4DC equ 0318h");




extern volatile unsigned char PWM4DCL __attribute__((address(0x318)));

__asm("PWM4DCL equ 0318h");


typedef union {
    struct {
        unsigned :6;
        unsigned DC :2;
    };
    struct {
        unsigned :6;
        unsigned DC0 :1;
        unsigned DC1 :1;
    };
    struct {
        unsigned :6;
        unsigned PWM4DC0 :1;
        unsigned PWM4DC1 :1;
    };
    struct {
        unsigned :6;
        unsigned PWMPW0 :1;
        unsigned PWMPW1 :1;
    };
} PWM4DCLbits_t;
extern volatile PWM4DCLbits_t PWM4DCLbits __attribute__((address(0x318)));
# 9768 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PWM4DCH __attribute__((address(0x319)));

__asm("PWM4DCH equ 0319h");


typedef union {
    struct {
        unsigned DC :8;
    };
    struct {
        unsigned DC2 :1;
        unsigned DC3 :1;
        unsigned DC4 :1;
        unsigned DC5 :1;
        unsigned DC6 :1;
        unsigned DC7 :1;
        unsigned DC8 :1;
        unsigned DC9 :1;
    };
    struct {
        unsigned PWM4DC2 :1;
        unsigned PWM4DC3 :1;
        unsigned PWM4DC4 :1;
        unsigned PWM4DC5 :1;
        unsigned PWM4DC6 :1;
        unsigned PWM4DC7 :1;
        unsigned PWM4DC8 :1;
        unsigned PWM4DC9 :1;
    };
    struct {
        unsigned PWMPW2 :1;
        unsigned PWMPW3 :1;
        unsigned PWMPW4 :1;
        unsigned PWMPW5 :1;
        unsigned PWMPW6 :1;
        unsigned PWMPW7 :1;
        unsigned PWMPW8 :1;
        unsigned PWMPW9 :1;
    };
} PWM4DCHbits_t;
extern volatile PWM4DCHbits_t PWM4DCHbits __attribute__((address(0x319)));
# 9938 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PWM4CON __attribute__((address(0x31A)));

__asm("PWM4CON equ 031Ah");


typedef union {
    struct {
        unsigned :4;
        unsigned POL :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned :4;
        unsigned PWM4POL :1;
        unsigned PWM4OUT :1;
        unsigned :1;
        unsigned PWM4EN :1;
    };
} PWM4CONbits_t;
extern volatile PWM4CONbits_t PWM4CONbits __attribute__((address(0x31A)));
# 9994 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char HIDRVB __attribute__((address(0x40D)));

__asm("HIDRVB equ 040Dh");


typedef union {
    struct {
        unsigned :1;
        unsigned HIDB1 :1;
    };
} HIDRVBbits_t;
extern volatile HIDRVBbits_t HIDRVBbits __attribute__((address(0x40D)));
# 10016 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile __uint24 SMT1TMR __attribute__((address(0x48C)));


__asm("SMT1TMR equ 048Ch");




extern volatile unsigned char SMT1TMRL __attribute__((address(0x48C)));

__asm("SMT1TMRL equ 048Ch");


typedef union {
    struct {
        unsigned SMT1TMR :8;
    };
    struct {
        unsigned SMT1TMR0 :1;
        unsigned SMT1TMR1 :1;
        unsigned SMT1TMR2 :1;
        unsigned SMT1TMR3 :1;
        unsigned SMT1TMR4 :1;
        unsigned SMT1TMR5 :1;
        unsigned SMT1TMR6 :1;
        unsigned SMT1TMR7 :1;
    };
} SMT1TMRLbits_t;
extern volatile SMT1TMRLbits_t SMT1TMRLbits __attribute__((address(0x48C)));
# 10094 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1TMRH __attribute__((address(0x48D)));

__asm("SMT1TMRH equ 048Dh");


typedef union {
    struct {
        unsigned SMT1TMR :8;
    };
    struct {
        unsigned SMT1TMR8 :1;
        unsigned SMT1TMR9 :1;
        unsigned SMT1TMR10 :1;
        unsigned SMT1TMR11 :1;
        unsigned SMT1TMR12 :1;
        unsigned SMT1TMR13 :1;
        unsigned SMT1TMR14 :1;
        unsigned SMT1TMR15 :1;
    };
} SMT1TMRHbits_t;
extern volatile SMT1TMRHbits_t SMT1TMRHbits __attribute__((address(0x48D)));
# 10164 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1TMRU __attribute__((address(0x48E)));

__asm("SMT1TMRU equ 048Eh");


typedef union {
    struct {
        unsigned SMT1TMR :8;
    };
    struct {
        unsigned SMT1TMR16 :1;
        unsigned SMT1TMR17 :1;
        unsigned SMT1TMR18 :1;
        unsigned SMT1TMR19 :1;
        unsigned SMT1TMR20 :1;
        unsigned SMT1TMR21 :1;
        unsigned SMT1TMR22 :1;
        unsigned SMT1TMR23 :1;
    };
} SMT1TMRUbits_t;
extern volatile SMT1TMRUbits_t SMT1TMRUbits __attribute__((address(0x48E)));
# 10235 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile __uint24 SMT1CPR __attribute__((address(0x48F)));


__asm("SMT1CPR equ 048Fh");




extern volatile unsigned char SMT1CPRL __attribute__((address(0x48F)));

__asm("SMT1CPRL equ 048Fh");


typedef union {
    struct {
        unsigned CPR :8;
    };
    struct {
        unsigned CPR0 :1;
        unsigned CPR1 :1;
        unsigned CPR2 :1;
        unsigned CPR3 :1;
        unsigned CPR4 :1;
        unsigned CPR5 :1;
        unsigned CPR6 :1;
        unsigned CPR7 :1;
    };
    struct {
        unsigned SMT1CPR :8;
    };
} SMT1CPRLbits_t;
extern volatile SMT1CPRLbits_t SMT1CPRLbits __attribute__((address(0x48F)));
# 10321 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1CPRH __attribute__((address(0x490)));

__asm("SMT1CPRH equ 0490h");


typedef union {
    struct {
        unsigned CPR :8;
    };
    struct {
        unsigned CPR8 :1;
        unsigned CPR9 :1;
        unsigned CPR10 :1;
        unsigned CPR11 :1;
        unsigned CPR12 :1;
        unsigned CPR13 :1;
        unsigned CPR14 :1;
        unsigned CPR15 :1;
    };
    struct {
        unsigned SMT1CPR8 :1;
        unsigned SMT1CPR9 :1;
        unsigned SMT1CPR10 :1;
        unsigned SMT1CPR11 :1;
        unsigned SMT1CPR12 :1;
        unsigned SMT1CPR13 :1;
        unsigned SMT1CPR14 :1;
        unsigned SMT1CPR15 :1;
    };
    struct {
        unsigned SMT1CPR :8;
    };
} SMT1CPRHbits_t;
extern volatile SMT1CPRHbits_t SMT1CPRHbits __attribute__((address(0x490)));
# 10449 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1CPRU __attribute__((address(0x491)));

__asm("SMT1CPRU equ 0491h");


typedef union {
    struct {
        unsigned CPR :8;
    };
    struct {
        unsigned CPR16 :1;
        unsigned CPR17 :1;
        unsigned CPR18 :1;
        unsigned CPR19 :1;
        unsigned CPR20 :1;
        unsigned CPR21 :1;
        unsigned CPR22 :1;
        unsigned CPR23 :1;
    };
    struct {
        unsigned SMT1CPR16 :1;
        unsigned SMT1CPR17 :1;
        unsigned SMT1CPR18 :1;
        unsigned SMT1CPR19 :1;
        unsigned SMT1CPR20 :1;
        unsigned SMT1CPR21 :1;
        unsigned SMT1CPR22 :1;
        unsigned SMT1CPR23 :1;
    };
    struct {
        unsigned SMT1CPR :8;
    };
} SMT1CPRUbits_t;
extern volatile SMT1CPRUbits_t SMT1CPRUbits __attribute__((address(0x491)));
# 10578 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile __uint24 SMT1CPW __attribute__((address(0x492)));


__asm("SMT1CPW equ 0492h");




extern volatile unsigned char SMT1CPWL __attribute__((address(0x492)));

__asm("SMT1CPWL equ 0492h");


typedef union {
    struct {
        unsigned CPW :8;
    };
    struct {
        unsigned CPW0 :1;
        unsigned CPW1 :1;
        unsigned CPW2 :1;
        unsigned CPW3 :1;
        unsigned CPW4 :1;
        unsigned CPW5 :1;
        unsigned CPW6 :1;
        unsigned CPW7 :1;
    };
    struct {
        unsigned SMT1CPW :8;
    };
    struct {
        unsigned SMT1CPW0 :1;
        unsigned SMT1CPW1 :1;
        unsigned SMT1CPW2 :1;
        unsigned SMT1CPW3 :1;
        unsigned SMT1CPW4 :1;
        unsigned SMT1CPW5 :1;
        unsigned SMT1CPW6 :1;
        unsigned SMT1CPW7 :1;
    };
} SMT1CPWLbits_t;
extern volatile SMT1CPWLbits_t SMT1CPWLbits __attribute__((address(0x492)));
# 10714 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1CPWH __attribute__((address(0x493)));

__asm("SMT1CPWH equ 0493h");


typedef union {
    struct {
        unsigned CPW :8;
    };
    struct {
        unsigned CPW8 :1;
        unsigned CPW9 :1;
        unsigned CPW10 :1;
        unsigned CPW11 :1;
        unsigned CPW12 :1;
        unsigned CPW13 :1;
        unsigned CPW14 :1;
        unsigned CPW15 :1;
    };
    struct {
        unsigned SMT1CPW :8;
    };
    struct {
        unsigned SMT1CPW8 :1;
        unsigned SMT1CPW9 :1;
        unsigned SMT1CPW10 :1;
        unsigned SMT1CPW11 :1;
        unsigned SMT1CPW12 :1;
        unsigned SMT1CPW13 :1;
        unsigned SMT1CPW14 :1;
        unsigned SMT1CPW15 :1;
    };
} SMT1CPWHbits_t;
extern volatile SMT1CPWHbits_t SMT1CPWHbits __attribute__((address(0x493)));
# 10842 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1CPWU __attribute__((address(0x494)));

__asm("SMT1CPWU equ 0494h");


typedef union {
    struct {
        unsigned CPW :8;
    };
    struct {
        unsigned CPW16 :1;
        unsigned CPW17 :1;
        unsigned CPW18 :1;
        unsigned CPW19 :1;
        unsigned CPW20 :1;
        unsigned CPW21 :1;
        unsigned CPW22 :1;
        unsigned CPW23 :1;
    };
    struct {
        unsigned SMT1CPW :8;
    };
    struct {
        unsigned SMT1CPW16 :1;
        unsigned SMT1CPW17 :1;
        unsigned SMT1CPW18 :1;
        unsigned SMT1CPW19 :1;
        unsigned SMT1CPW20 :1;
        unsigned SMT1CPW21 :1;
        unsigned SMT1CPW22 :1;
        unsigned SMT1CPW23 :1;
    };
} SMT1CPWUbits_t;
extern volatile SMT1CPWUbits_t SMT1CPWUbits __attribute__((address(0x494)));
# 10971 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile __uint24 SMT1PR __attribute__((address(0x495)));


__asm("SMT1PR equ 0495h");




extern volatile unsigned char SMT1PRL __attribute__((address(0x495)));

__asm("SMT1PRL equ 0495h");


typedef union {
    struct {
        unsigned SMT1PR :8;
    };
    struct {
        unsigned SMT1PR0 :1;
        unsigned SMT1PR1 :1;
        unsigned SMT1PR2 :1;
        unsigned SMT1PR3 :1;
        unsigned SMT1PR4 :1;
        unsigned SMT1PR5 :1;
        unsigned SMT1PR6 :1;
        unsigned SMT1PR7 :1;
    };
} SMT1PRLbits_t;
extern volatile SMT1PRLbits_t SMT1PRLbits __attribute__((address(0x495)));
# 11049 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1PRH __attribute__((address(0x496)));

__asm("SMT1PRH equ 0496h");


typedef union {
    struct {
        unsigned SMT1PR :8;
    };
    struct {
        unsigned SMT1PR8 :1;
        unsigned SMT1PR9 :1;
        unsigned SMT1PR10 :1;
        unsigned SMT1PR11 :1;
        unsigned SMT1PR12 :1;
        unsigned SMT1PR13 :1;
        unsigned SMT1PR14 :1;
        unsigned SMT1PR15 :1;
    };
} SMT1PRHbits_t;
extern volatile SMT1PRHbits_t SMT1PRHbits __attribute__((address(0x496)));
# 11119 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1PRU __attribute__((address(0x497)));

__asm("SMT1PRU equ 0497h");


typedef union {
    struct {
        unsigned SMT1PR :8;
    };
    struct {
        unsigned SMT1PR16 :1;
        unsigned SMT1PR17 :1;
        unsigned SMT1PR18 :1;
        unsigned SMT1PR19 :1;
        unsigned SMT1PR20 :1;
        unsigned SMT1PR21 :1;
        unsigned SMT1PR22 :1;
        unsigned SMT1PR23 :1;
    };
} SMT1PRUbits_t;
extern volatile SMT1PRUbits_t SMT1PRUbits __attribute__((address(0x497)));
# 11189 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1CON0 __attribute__((address(0x498)));

__asm("SMT1CON0 equ 0498h");


typedef union {
    struct {
        unsigned SMT1PS :2;
        unsigned CPOL :1;
        unsigned SPOL :1;
        unsigned WPOL :1;
        unsigned STP :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned SMT1PS0 :1;
        unsigned SMT1PS1 :1;
        unsigned :5;
        unsigned SMT1EN :1;
    };
} SMT1CON0bits_t;
extern volatile SMT1CON0bits_t SMT1CON0bits __attribute__((address(0x498)));
# 11261 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1CON1 __attribute__((address(0x499)));

__asm("SMT1CON1 equ 0499h");


typedef union {
    struct {
        unsigned MODE :4;
        unsigned :2;
        unsigned REPEAT :1;
        unsigned SMT1GO :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned MODE3 :1;
    };
} SMT1CON1bits_t;
extern volatile SMT1CON1bits_t SMT1CON1bits __attribute__((address(0x499)));
# 11320 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1STAT __attribute__((address(0x49A)));

__asm("SMT1STAT equ 049Ah");


typedef union {
    struct {
        unsigned AS :1;
        unsigned WS :1;
        unsigned TS :1;
        unsigned :2;
        unsigned RST :1;
        unsigned CPWUP :1;
        unsigned CPRUP :1;
    };
    struct {
        unsigned SMT1AS :1;
        unsigned SMT1WS :1;
        unsigned SMT1TS :1;
        unsigned :2;
        unsigned SMT1RESET :1;
        unsigned SMT1CPWUP :1;
        unsigned SMT1CPRUP :1;
    };
} SMT1STATbits_t;
extern volatile SMT1STATbits_t SMT1STATbits __attribute__((address(0x49A)));
# 11410 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1CLK __attribute__((address(0x49B)));

__asm("SMT1CLK equ 049Bh");


typedef union {
    struct {
        unsigned CSEL :3;
    };
    struct {
        unsigned CSEL0 :1;
        unsigned CSEL1 :1;
        unsigned CSEL2 :1;
    };
    struct {
        unsigned SMT1CSEL :3;
    };
    struct {
        unsigned SMT1CSEL0 :1;
        unsigned SMT1CSEL1 :1;
        unsigned SMT1CSEL2 :1;
    };
} SMT1CLKbits_t;
extern volatile SMT1CLKbits_t SMT1CLKbits __attribute__((address(0x49B)));
# 11478 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1SIG __attribute__((address(0x49C)));

__asm("SMT1SIG equ 049Ch");


typedef union {
    struct {
        unsigned SSEL :5;
    };
    struct {
        unsigned SSEL0 :1;
        unsigned SSEL1 :1;
        unsigned SSEL2 :1;
        unsigned SSEL3 :1;
        unsigned SSEL4 :1;
    };
    struct {
        unsigned SMT1SSEL :5;
    };
    struct {
        unsigned SMT1SSEL0 :1;
        unsigned SMT1SSEL1 :1;
        unsigned SMT1SSEL2 :1;
        unsigned SMT1SSEL3 :1;
        unsigned SMT1SSEL4 :1;
    };
} SMT1SIGbits_t;
extern volatile SMT1SIGbits_t SMT1SIGbits __attribute__((address(0x49C)));
# 11570 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1WIN __attribute__((address(0x49D)));

__asm("SMT1WIN equ 049Dh");


typedef union {
    struct {
        unsigned WSEL :5;
    };
    struct {
        unsigned WSEL0 :1;
        unsigned WSEL1 :1;
        unsigned WSEL2 :1;
        unsigned WSEL3 :1;
        unsigned WSEL4 :1;
    };
    struct {
        unsigned SMT1WSEL :5;
    };
    struct {
        unsigned SMT1WSEL0 :1;
        unsigned SMT1WSEL1 :1;
        unsigned SMT1WSEL2 :1;
        unsigned SMT1WSEL3 :1;
        unsigned SMT1WSEL4 :1;
    };
} SMT1WINbits_t;
extern volatile SMT1WINbits_t SMT1WINbits __attribute__((address(0x49D)));
# 11662 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TMR0L __attribute__((address(0x59C)));

__asm("TMR0L equ 059Ch");


extern volatile unsigned char TMR0 __attribute__((address(0x59C)));

__asm("TMR0 equ 059Ch");


typedef union {
    struct {
        unsigned TMR0L :8;
    };
    struct {
        unsigned TMR0L0 :1;
        unsigned TMR0L1 :1;
        unsigned TMR0L2 :1;
        unsigned TMR0L3 :1;
        unsigned TMR0L4 :1;
        unsigned TMR0L5 :1;
        unsigned TMR0L6 :1;
        unsigned TMR0L7 :1;
    };
} TMR0Lbits_t;
extern volatile TMR0Lbits_t TMR0Lbits __attribute__((address(0x59C)));
# 11735 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned TMR0L :8;
    };
    struct {
        unsigned TMR0L0 :1;
        unsigned TMR0L1 :1;
        unsigned TMR0L2 :1;
        unsigned TMR0L3 :1;
        unsigned TMR0L4 :1;
        unsigned TMR0L5 :1;
        unsigned TMR0L6 :1;
        unsigned TMR0L7 :1;
    };
} TMR0bits_t;
extern volatile TMR0bits_t TMR0bits __attribute__((address(0x59C)));
# 11800 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TMR0H __attribute__((address(0x59D)));

__asm("TMR0H equ 059Dh");


extern volatile unsigned char PR0 __attribute__((address(0x59D)));

__asm("PR0 equ 059Dh");


typedef union {
    struct {
        unsigned TMR0H :8;
    };
    struct {
        unsigned T0PR :8;
    };
    struct {
        unsigned TMR0H0 :1;
        unsigned TMR0H1 :1;
        unsigned TMR0H2 :1;
        unsigned TMR0H3 :1;
        unsigned TMR0H4 :1;
        unsigned TMR0H5 :1;
        unsigned TMR0H6 :1;
        unsigned TMR0H7 :1;
    };
    struct {
        unsigned T0PR0 :1;
        unsigned T0PR1 :1;
        unsigned T0PR2 :1;
        unsigned T0PR3 :1;
        unsigned T0PR4 :1;
        unsigned T0PR5 :1;
        unsigned T0PR6 :1;
        unsigned T0PR7 :1;
    };
} TMR0Hbits_t;
extern volatile TMR0Hbits_t TMR0Hbits __attribute__((address(0x59D)));
# 11931 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned TMR0H :8;
    };
    struct {
        unsigned T0PR :8;
    };
    struct {
        unsigned TMR0H0 :1;
        unsigned TMR0H1 :1;
        unsigned TMR0H2 :1;
        unsigned TMR0H3 :1;
        unsigned TMR0H4 :1;
        unsigned TMR0H5 :1;
        unsigned TMR0H6 :1;
        unsigned TMR0H7 :1;
    };
    struct {
        unsigned T0PR0 :1;
        unsigned T0PR1 :1;
        unsigned T0PR2 :1;
        unsigned T0PR3 :1;
        unsigned T0PR4 :1;
        unsigned T0PR5 :1;
        unsigned T0PR6 :1;
        unsigned T0PR7 :1;
    };
} PR0bits_t;
extern volatile PR0bits_t PR0bits __attribute__((address(0x59D)));
# 12054 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T0CON0 __attribute__((address(0x59E)));

__asm("T0CON0 equ 059Eh");


typedef union {
    struct {
        unsigned T0OUTPS :4;
        unsigned T016BIT :1;
        unsigned T0OUT :1;
        unsigned T0OE :1;
        unsigned T0EN :1;
    };
    struct {
        unsigned T0OUTPS0 :1;
        unsigned T0OUTPS1 :1;
        unsigned T0OUTPS2 :1;
        unsigned T0OUTPS3 :1;
    };
} T0CON0bits_t;
extern volatile T0CON0bits_t T0CON0bits __attribute__((address(0x59E)));
# 12124 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T0CON1 __attribute__((address(0x59F)));

__asm("T0CON1 equ 059Fh");


typedef union {
    struct {
        unsigned T0CKPS :4;
        unsigned T0ASYNC :1;
        unsigned T0CS :3;
    };
    struct {
        unsigned T0CKPS0 :1;
        unsigned T0CKPS1 :1;
        unsigned T0CKPS2 :1;
        unsigned T0CKPS3 :1;
        unsigned :1;
        unsigned T0CS0 :1;
        unsigned T0CS1 :1;
        unsigned T0CS2 :1;
    };
    struct {
        unsigned T0PS0 :1;
        unsigned T0PS1 :1;
        unsigned T0PS2 :1;
        unsigned T0PS3 :1;
    };
    struct {
        unsigned T0PS :4;
    };
} T0CON1bits_t;
extern volatile T0CON1bits_t T0CON1bits __attribute__((address(0x59F)));
# 12235 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CWG1CLKCON __attribute__((address(0x60C)));

__asm("CWG1CLKCON equ 060Ch");


typedef union {
    struct {
        unsigned CS :1;
    };
    struct {
        unsigned CWG1CS :1;
    };
} CWG1CLKCONbits_t;
extern volatile CWG1CLKCONbits_t CWG1CLKCONbits __attribute__((address(0x60C)));
# 12263 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CWG1ISM __attribute__((address(0x60D)));

__asm("CWG1ISM equ 060Dh");


typedef union {
    struct {
        unsigned IS :4;
    };
    struct {
        unsigned CWG1ISM0 :1;
        unsigned CWG1ISM1 :1;
        unsigned CWG1ISM2 :1;
        unsigned CWG1ISM3 :1;
    };
} CWG1ISMbits_t;
extern volatile CWG1ISMbits_t CWG1ISMbits __attribute__((address(0x60D)));
# 12309 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CWG1DBR __attribute__((address(0x60E)));

__asm("CWG1DBR equ 060Eh");


typedef union {
    struct {
        unsigned DBR :6;
    };
    struct {
        unsigned DBR0 :1;
        unsigned DBR1 :1;
        unsigned DBR2 :1;
        unsigned DBR3 :1;
        unsigned DBR4 :1;
        unsigned DBR5 :1;
    };
    struct {
        unsigned CWG1DBR :6;
    };
    struct {
        unsigned CWG1DBR0 :1;
        unsigned CWG1DBR1 :1;
        unsigned CWG1DBR2 :1;
        unsigned CWG1DBR3 :1;
        unsigned CWG1DBR4 :1;
        unsigned CWG1DBR5 :1;
    };
} CWG1DBRbits_t;
extern volatile CWG1DBRbits_t CWG1DBRbits __attribute__((address(0x60E)));
# 12413 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CWG1DBF __attribute__((address(0x60F)));

__asm("CWG1DBF equ 060Fh");


typedef union {
    struct {
        unsigned DBF :6;
    };
    struct {
        unsigned DBF0 :1;
        unsigned DBF1 :1;
        unsigned DBF2 :1;
        unsigned DBF3 :1;
        unsigned DBF4 :1;
        unsigned DBF5 :1;
    };
    struct {
        unsigned CWG1DBF :6;
    };
    struct {
        unsigned CWG1DBF0 :1;
        unsigned CWG1DBF1 :1;
        unsigned CWG1DBF2 :1;
        unsigned CWG1DBF3 :1;
        unsigned CWG1DBF4 :1;
        unsigned CWG1DBF5 :1;
    };
} CWG1DBFbits_t;
extern volatile CWG1DBFbits_t CWG1DBFbits __attribute__((address(0x60F)));
# 12517 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CWG1CON0 __attribute__((address(0x610)));

__asm("CWG1CON0 equ 0610h");


typedef union {
    struct {
        unsigned MODE :3;
        unsigned :3;
        unsigned LD :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned :4;
        unsigned G1EN :1;
    };
    struct {
        unsigned CWG1MODE :3;
        unsigned :3;
        unsigned CWG1LD :1;
        unsigned CWG1EN :1;
    };
    struct {
        unsigned CWG1MODE0 :1;
        unsigned CWG1MODE1 :1;
        unsigned CWG1MODE2 :1;
    };
} CWG1CON0bits_t;
extern volatile CWG1CON0bits_t CWG1CON0bits __attribute__((address(0x610)));
# 12618 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CWG1CON1 __attribute__((address(0x611)));

__asm("CWG1CON1 equ 0611h");


typedef union {
    struct {
        unsigned POLA :1;
        unsigned POLB :1;
        unsigned POLC :1;
        unsigned POLD :1;
        unsigned :1;
        unsigned IN :1;
    };
    struct {
        unsigned CWG1POLA :1;
        unsigned CWG1POLB :1;
        unsigned CWG1POLC :1;
        unsigned CWG1POLD :1;
        unsigned :1;
        unsigned CWG1IN :1;
    };
} CWG1CON1bits_t;
extern volatile CWG1CON1bits_t CWG1CON1bits __attribute__((address(0x611)));
# 12696 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CWG1AS0 __attribute__((address(0x612)));

__asm("CWG1AS0 equ 0612h");


typedef union {
    struct {
        unsigned :2;
        unsigned LSAC :2;
        unsigned LSBD :2;
        unsigned REN :1;
        unsigned SHUTDOWN :1;
    };
    struct {
        unsigned :2;
        unsigned LSAC0 :1;
        unsigned LSAC1 :1;
        unsigned LSBD0 :1;
        unsigned LSBD1 :1;
    };
    struct {
        unsigned :2;
        unsigned CWG1LSAC :2;
        unsigned CWG1LSBD :2;
        unsigned CWG1REN :1;
        unsigned CWG1SHUTDOWN :1;
    };
    struct {
        unsigned :2;
        unsigned CWG1LSAC0 :1;
        unsigned CWG1LSAC1 :1;
        unsigned CWG1LSBD0 :1;
        unsigned CWG1LSBD1 :1;
    };
} CWG1AS0bits_t;
extern volatile CWG1AS0bits_t CWG1AS0bits __attribute__((address(0x612)));
# 12816 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CWG1AS1 __attribute__((address(0x613)));

__asm("CWG1AS1 equ 0613h");


typedef union {
    struct {
        unsigned AS0E :1;
        unsigned AS1E :1;
        unsigned AS2E :1;
        unsigned AS3E :1;
        unsigned AS4E :1;
        unsigned AS5E :1;
        unsigned AS6E :1;
    };
} CWG1AS1bits_t;
extern volatile CWG1AS1bits_t CWG1AS1bits __attribute__((address(0x613)));
# 12872 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CWG1STR __attribute__((address(0x614)));

__asm("CWG1STR equ 0614h");


typedef union {
    struct {
        unsigned STRA :1;
        unsigned STRB :1;
        unsigned STRC :1;
        unsigned STRD :1;
        unsigned OVRA :1;
        unsigned OVRB :1;
        unsigned OVRC :1;
        unsigned OVRD :1;
    };
    struct {
        unsigned CWG1STRA :1;
        unsigned CWG1STRB :1;
        unsigned CWG1STRC :1;
        unsigned CWG1STRD :1;
        unsigned CWG1OVRA :1;
        unsigned CWG1OVRB :1;
        unsigned CWG1OVRC :1;
        unsigned CWG1OVRD :1;
    };
} CWG1STRbits_t;
extern volatile CWG1STRbits_t CWG1STRbits __attribute__((address(0x614)));
# 12984 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIR0 __attribute__((address(0x70C)));

__asm("PIR0 equ 070Ch");


typedef union {
    struct {
        unsigned INTF :1;
        unsigned :3;
        unsigned IOCIF :1;
        unsigned TMR0IF :1;
    };
} PIR0bits_t;
extern volatile PIR0bits_t PIR0bits __attribute__((address(0x70C)));
# 13017 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIR1 __attribute__((address(0x70D)));

__asm("PIR1 equ 070Dh");


typedef union {
    struct {
        unsigned ADIF :1;
        unsigned ADTIF :1;
        unsigned :4;
        unsigned CSWIF :1;
        unsigned OSFIF :1;
    };
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __attribute__((address(0x70D)));
# 13056 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIR2 __attribute__((address(0x70E)));

__asm("PIR2 equ 070Eh");


typedef union {
    struct {
        unsigned C1IF :1;
        unsigned C2IF :1;
        unsigned :4;
        unsigned ZCDIF :1;
    };
} PIR2bits_t;
extern volatile PIR2bits_t PIR2bits __attribute__((address(0x70E)));
# 13089 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIR3 __attribute__((address(0x70F)));

__asm("PIR3 equ 070Fh");


typedef union {
    struct {
        unsigned SSP1IF :1;
        unsigned BCL1IF :1;
        unsigned :2;
        unsigned TX1IF :1;
        unsigned RC1IF :1;
        unsigned TX2IF :1;
        unsigned RC2IF :1;
    };
} PIR3bits_t;
extern volatile PIR3bits_t PIR3bits __attribute__((address(0x70F)));
# 13140 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIR4 __attribute__((address(0x710)));

__asm("PIR4 equ 0710h");


typedef union {
    struct {
        unsigned TMR1IF :1;
        unsigned TMR2IF :1;
        unsigned :1;
        unsigned TMR4IF :1;
    };
} PIR4bits_t;
extern volatile PIR4bits_t PIR4bits __attribute__((address(0x710)));
# 13173 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIR5 __attribute__((address(0x711)));

__asm("PIR5 equ 0711h");


typedef union {
    struct {
        unsigned TMR1GIF :1;
        unsigned :3;
        unsigned CLC1IF :1;
        unsigned CLC2IF :1;
        unsigned CLC3IF :1;
        unsigned CLC4IF :1;
    };
} PIR5bits_t;
extern volatile PIR5bits_t PIR5bits __attribute__((address(0x711)));
# 13218 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIR6 __attribute__((address(0x712)));

__asm("PIR6 equ 0712h");


typedef union {
    struct {
        unsigned CCP1IF :1;
        unsigned CCP2IF :1;
        unsigned :5;
        unsigned CRIF :1;
    };
} PIR6bits_t;
extern volatile PIR6bits_t PIR6bits __attribute__((address(0x712)));
# 13251 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIR7 __attribute__((address(0x713)));

__asm("PIR7 equ 0713h");


typedef union {
    struct {
        unsigned CWG1IF :1;
        unsigned :4;
        unsigned NVMIF :1;
    };
} PIR7bits_t;
extern volatile PIR7bits_t PIR7bits __attribute__((address(0x713)));
# 13278 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIR8 __attribute__((address(0x714)));

__asm("PIR8 equ 0714h");


typedef union {
    struct {
        unsigned SMT1IF :1;
        unsigned SMT1PRAIF :1;
        unsigned SMT1PWAIF :1;
        unsigned :3;
        unsigned RTCCIF :1;
        unsigned LCDIF :1;
    };
} PIR8bits_t;
extern volatile PIR8bits_t PIR8bits __attribute__((address(0x714)));
# 13323 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIE0 __attribute__((address(0x716)));

__asm("PIE0 equ 0716h");


typedef union {
    struct {
        unsigned INTE :1;
        unsigned :3;
        unsigned IOCIE :1;
        unsigned TMR0IE :1;
    };
} PIE0bits_t;
extern volatile PIE0bits_t PIE0bits __attribute__((address(0x716)));
# 13356 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIE1 __attribute__((address(0x717)));

__asm("PIE1 equ 0717h");


typedef union {
    struct {
        unsigned ADIE :1;
        unsigned ADTIE :1;
        unsigned :4;
        unsigned CSWIE :1;
        unsigned OSFIE :1;
    };
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __attribute__((address(0x717)));
# 13395 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIE2 __attribute__((address(0x718)));

__asm("PIE2 equ 0718h");


typedef union {
    struct {
        unsigned C1IE :1;
        unsigned C2IE :1;
        unsigned :4;
        unsigned ZCDIE :1;
    };
} PIE2bits_t;
extern volatile PIE2bits_t PIE2bits __attribute__((address(0x718)));
# 13428 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIE3 __attribute__((address(0x719)));

__asm("PIE3 equ 0719h");


typedef union {
    struct {
        unsigned SSP1IE :1;
        unsigned BCL1IE :1;
        unsigned :2;
        unsigned TX1IE :1;
        unsigned RC1IE :1;
        unsigned TX2IE :1;
        unsigned RC2IE :1;
    };
} PIE3bits_t;
extern volatile PIE3bits_t PIE3bits __attribute__((address(0x719)));
# 13479 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIE4 __attribute__((address(0x71A)));

__asm("PIE4 equ 071Ah");


typedef union {
    struct {
        unsigned TMR1IE :1;
        unsigned TMR2IE :1;
        unsigned :1;
        unsigned TMR4IE :1;
    };
} PIE4bits_t;
extern volatile PIE4bits_t PIE4bits __attribute__((address(0x71A)));
# 13512 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIE5 __attribute__((address(0x71B)));

__asm("PIE5 equ 071Bh");


typedef union {
    struct {
        unsigned TMR1GIE :1;
        unsigned :3;
        unsigned CLC1IE :1;
        unsigned CLC2IE :1;
        unsigned CLC3IE :1;
        unsigned CLC4IE :1;
    };
} PIE5bits_t;
extern volatile PIE5bits_t PIE5bits __attribute__((address(0x71B)));
# 13557 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIE6 __attribute__((address(0x71C)));

__asm("PIE6 equ 071Ch");


typedef union {
    struct {
        unsigned CCP1IE :1;
        unsigned CCP2IE :1;
        unsigned :5;
        unsigned CRIE :1;
    };
} PIE6bits_t;
extern volatile PIE6bits_t PIE6bits __attribute__((address(0x71C)));
# 13590 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIE7 __attribute__((address(0x71D)));

__asm("PIE7 equ 071Dh");


typedef union {
    struct {
        unsigned CWG1IE :1;
        unsigned :4;
        unsigned NVMIE :1;
    };
} PIE7bits_t;
extern volatile PIE7bits_t PIE7bits __attribute__((address(0x71D)));
# 13617 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PIE8 __attribute__((address(0x71E)));

__asm("PIE8 equ 071Eh");


typedef union {
    struct {
        unsigned SMT1IE :1;
        unsigned SMT1PRAIE :1;
        unsigned SMT1PWAIE :1;
        unsigned :3;
        unsigned RTCCIE :1;
        unsigned LCDIE :1;
    };
} PIE8bits_t;
extern volatile PIE8bits_t PIE8bits __attribute__((address(0x71E)));
# 13662 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PMD0 __attribute__((address(0x796)));

__asm("PMD0 equ 0796h");


typedef union {
    struct {
        unsigned IOCMD :1;
        unsigned :1;
        unsigned NVMMD :1;
        unsigned :2;
        unsigned ACTMD :1;
        unsigned FVRMD :1;
        unsigned SYSCMD :1;
    };
} PMD0bits_t;
extern volatile PMD0bits_t PMD0bits __attribute__((address(0x796)));
# 13708 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PMD1 __attribute__((address(0x797)));

__asm("PMD1 equ 0797h");


typedef union {
    struct {
        unsigned TMR0MD :1;
        unsigned TMR1MD :1;
        unsigned TMR2MD :1;
        unsigned :1;
        unsigned TMR4MD :1;
    };
} PMD1bits_t;
extern volatile PMD1bits_t PMD1bits __attribute__((address(0x797)));
# 13747 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PMD2 __attribute__((address(0x798)));

__asm("PMD2 equ 0798h");


typedef union {
    struct {
        unsigned ZCDMD :1;
        unsigned CMP1MD :1;
        unsigned CMP2MD :1;
        unsigned :2;
        unsigned ADCMD :1;
        unsigned DACMD :1;
        unsigned RTCCMD :1;
    };
} PMD2bits_t;
extern volatile PMD2bits_t PMD2bits __attribute__((address(0x798)));
# 13798 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PMD3 __attribute__((address(0x799)));

__asm("PMD3 equ 0799h");


typedef union {
    struct {
        unsigned CCP1MD :1;
        unsigned CCP2MD :1;
        unsigned CCP3MD :1;
        unsigned CCP4MD :1;
    };
} PMD3bits_t;
extern volatile PMD3bits_t PMD3bits __attribute__((address(0x799)));
# 13836 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PMD4 __attribute__((address(0x79A)));

__asm("PMD4 equ 079Ah");


typedef union {
    struct {
        unsigned CWG1MD :1;
        unsigned :3;
        unsigned MSSP1MD :1;
        unsigned :1;
        unsigned UART1MD :1;
        unsigned UART2MD :1;
    };
} PMD4bits_t;
extern volatile PMD4bits_t PMD4bits __attribute__((address(0x79A)));
# 13876 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PMD5 __attribute__((address(0x79B)));

__asm("PMD5 equ 079Bh");


typedef union {
    struct {
        unsigned :1;
        unsigned CLC1MD :1;
        unsigned CLC2MD :1;
        unsigned CLC3MD :1;
        unsigned CLC4MD :1;
        unsigned LCDMD :1;
        unsigned SMT1MD :1;
    };
} PMD5bits_t;
extern volatile PMD5bits_t PMD5bits __attribute__((address(0x79B)));
# 13927 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WDTCON0 __attribute__((address(0x80C)));

__asm("WDTCON0 equ 080Ch");


typedef union {
    struct {
        unsigned SEN :1;
        unsigned WDTPS :5;
    };
    struct {
        unsigned SWDTEN :1;
    };
    struct {
        unsigned WDTSEN :1;
    };
    struct {
        unsigned :1;
        unsigned WDTPS0 :1;
        unsigned WDTPS1 :1;
        unsigned WDTPS2 :1;
        unsigned WDTPS3 :1;
        unsigned WDTPS4 :1;
    };
} WDTCON0bits_t;
extern volatile WDTCON0bits_t WDTCON0bits __attribute__((address(0x80C)));
# 14002 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WDTCON1 __attribute__((address(0x80D)));

__asm("WDTCON1 equ 080Dh");


typedef union {
    struct {
        unsigned WINDOW :3;
        unsigned :1;
        unsigned WDTCS :3;
    };
    struct {
        unsigned WINDOW0 :1;
        unsigned WINDOW1 :1;
        unsigned WINDOW2 :1;
    };
    struct {
        unsigned WDTWINDOW :3;
    };
    struct {
        unsigned WDTWINDOW0 :1;
        unsigned WDTWINDOW1 :1;
        unsigned WDTWINDOW2 :1;
        unsigned :1;
        unsigned WDTCS0 :1;
        unsigned WDTCS1 :1;
        unsigned WDTCS2 :1;
    };
} WDTCON1bits_t;
extern volatile WDTCON1bits_t WDTCON1bits __attribute__((address(0x80D)));
# 14096 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WDTPSL __attribute__((address(0x80E)));

__asm("WDTPSL equ 080Eh");


typedef union {
    struct {
        unsigned PSCNT :8;
    };
    struct {
        unsigned PSCNT0 :1;
        unsigned PSCNT1 :1;
        unsigned PSCNT2 :1;
        unsigned PSCNT3 :1;
        unsigned PSCNT4 :1;
        unsigned PSCNT5 :1;
        unsigned PSCNT6 :1;
        unsigned PSCNT7 :1;
    };
    struct {
        unsigned WDTPSCNT :8;
    };
    struct {
        unsigned WDTPSCNT0 :1;
        unsigned WDTPSCNT1 :1;
        unsigned WDTPSCNT2 :1;
        unsigned WDTPSCNT3 :1;
        unsigned WDTPSCNT4 :1;
        unsigned WDTPSCNT5 :1;
        unsigned WDTPSCNT6 :1;
        unsigned WDTPSCNT7 :1;
    };
} WDTPSLbits_t;
extern volatile WDTPSLbits_t WDTPSLbits __attribute__((address(0x80E)));
# 14224 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WDTPSH __attribute__((address(0x80F)));

__asm("WDTPSH equ 080Fh");


typedef union {
    struct {
        unsigned PSCNT :8;
    };
    struct {
        unsigned PSCNT8 :1;
        unsigned PSCNT9 :1;
        unsigned PSCNT10 :1;
        unsigned PSCNT11 :1;
        unsigned PSCNT12 :1;
        unsigned PSCNT13 :1;
        unsigned PSCNT14 :1;
        unsigned PSCNT15 :1;
    };
    struct {
        unsigned WDTPSCNT :8;
    };
    struct {
        unsigned WDTPSCNT8 :1;
        unsigned WDTPSCNT9 :1;
        unsigned WDTPSCNT10 :1;
        unsigned WDTPSCNT11 :1;
        unsigned WDTPSCNT12 :1;
        unsigned WDTPSCNT13 :1;
        unsigned WDTPSCNT14 :1;
        unsigned WDTPSCNT15 :1;
    };
} WDTPSHbits_t;
extern volatile WDTPSHbits_t WDTPSHbits __attribute__((address(0x80F)));
# 14352 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WDTTMR __attribute__((address(0x810)));

__asm("WDTTMR equ 0810h");


typedef union {
    struct {
        unsigned PSCNT16 :1;
        unsigned PSCNT17 :1;
        unsigned STATE :1;
        unsigned WDTTMR :5;
    };
    struct {
        unsigned WDTPSCNT16 :1;
        unsigned WDTPSCNT17 :1;
        unsigned WDTSTATE :1;
        unsigned WDTTMR0 :1;
        unsigned WDTTMR1 :1;
        unsigned WDTTMR2 :1;
        unsigned WDTTMR3 :1;
    };
} WDTTMRbits_t;
extern volatile WDTTMRbits_t WDTTMRbits __attribute__((address(0x810)));
# 14434 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char BORCON __attribute__((address(0x811)));

__asm("BORCON equ 0811h");


typedef union {
    struct {
        unsigned BORRDY :1;
        unsigned :6;
        unsigned SBOREN :1;
    };
} BORCONbits_t;
extern volatile BORCONbits_t BORCONbits __attribute__((address(0x811)));
# 14461 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PCON0 __attribute__((address(0x813)));

__asm("PCON0 equ 0813h");


typedef union {
    struct {
        unsigned nBOR :1;
        unsigned nPOR :1;
        unsigned nRI :1;
        unsigned nRMCLR :1;
        unsigned nRWDT :1;
        unsigned nWDTWV :1;
        unsigned STKUNF :1;
        unsigned STKOVF :1;
    };
} PCON0bits_t;
extern volatile PCON0bits_t PCON0bits __attribute__((address(0x813)));
# 14523 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PCON1 __attribute__((address(0x814)));

__asm("PCON1 equ 0814h");


typedef union {
    struct {
        unsigned VBATBOR :1;
        unsigned MEMV :1;
    };
} PCON1bits_t;
extern volatile PCON1bits_t PCON1bits __attribute__((address(0x814)));
# 14549 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char NVMADRL __attribute__((address(0x81A)));

__asm("NVMADRL equ 081Ah");


typedef union {
    struct {
        unsigned NVMADR0 :1;
        unsigned NVMADR1 :1;
        unsigned NVMADR2 :1;
        unsigned NVMADR3 :1;
        unsigned NVMADR4 :1;
        unsigned NVMADR5 :1;
        unsigned NVMADR6 :1;
        unsigned NVMADR7 :1;
    };
} NVMADRLbits_t;
extern volatile NVMADRLbits_t NVMADRLbits __attribute__((address(0x81A)));
# 14611 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char NVMADRH __attribute__((address(0x81B)));

__asm("NVMADRH equ 081Bh");


typedef union {
    struct {
        unsigned NVMADR8 :1;
        unsigned NVMADR9 :1;
        unsigned NVMADR10 :1;
        unsigned NVMADR11 :1;
        unsigned NVMADR12 :1;
        unsigned NVMADR13 :1;
        unsigned NVMADR14 :1;
    };
} NVMADRHbits_t;
extern volatile NVMADRHbits_t NVMADRHbits __attribute__((address(0x81B)));
# 14667 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char NVMDATL __attribute__((address(0x81C)));

__asm("NVMDATL equ 081Ch");


typedef union {
    struct {
        unsigned NVMDAT0 :1;
        unsigned NVMDAT1 :1;
        unsigned NVMDAT2 :1;
        unsigned NVMDAT3 :1;
        unsigned NVMDAT4 :1;
        unsigned NVMDAT5 :1;
        unsigned NVMDAT6 :1;
        unsigned NVMDAT7 :1;
    };
} NVMDATLbits_t;
extern volatile NVMDATLbits_t NVMDATLbits __attribute__((address(0x81C)));
# 14729 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char NVMDATH __attribute__((address(0x81D)));

__asm("NVMDATH equ 081Dh");


typedef union {
    struct {
        unsigned NVMDAT8 :1;
        unsigned NVMDAT9 :1;
        unsigned NVMDAT10 :1;
        unsigned NVMDAT11 :1;
        unsigned NVMDAT12 :1;
        unsigned NVMDAT13 :1;
    };
} NVMDATHbits_t;
extern volatile NVMDATHbits_t NVMDATHbits __attribute__((address(0x81D)));
# 14779 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char NVMCON1 __attribute__((address(0x81E)));

__asm("NVMCON1 equ 081Eh");


typedef union {
    struct {
        unsigned RD :1;
        unsigned WR :1;
        unsigned WREN :1;
        unsigned WRERR :1;
        unsigned FREE :1;
        unsigned LWLO :1;
        unsigned NVMREGS :1;
    };
} NVMCON1bits_t;
extern volatile NVMCON1bits_t NVMCON1bits __attribute__((address(0x81E)));
# 14835 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char NVMCON2 __attribute__((address(0x81F)));

__asm("NVMCON2 equ 081Fh");




extern volatile unsigned char CPUDOZE __attribute__((address(0x88C)));

__asm("CPUDOZE equ 088Ch");


typedef union {
    struct {
        unsigned DOZE0 :1;
        unsigned DOZE1 :1;
        unsigned DOZE2 :1;
        unsigned :1;
        unsigned DOE :1;
        unsigned ROI :1;
        unsigned DOZEN :1;
        unsigned IDLEN :1;
    };
    struct {
        unsigned DOZE :3;
    };
} CPUDOZEbits_t;
extern volatile CPUDOZEbits_t CPUDOZEbits __attribute__((address(0x88C)));
# 14907 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char OSCCON1 __attribute__((address(0x88D)));

__asm("OSCCON1 equ 088Dh");


typedef union {
    struct {
        unsigned NDIV :4;
        unsigned NOSC :3;
    };
    struct {
        unsigned NDIV0 :1;
        unsigned NDIV1 :1;
        unsigned NDIV2 :1;
        unsigned NDIV3 :1;
        unsigned NOSC0 :1;
        unsigned NOSC1 :1;
        unsigned NOSC2 :1;
    };
} OSCCON1bits_t;
extern volatile OSCCON1bits_t OSCCON1bits __attribute__((address(0x88D)));
# 14977 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char OSCCON2 __attribute__((address(0x88E)));

__asm("OSCCON2 equ 088Eh");


typedef union {
    struct {
        unsigned CDIV :4;
        unsigned COSC :3;
    };
    struct {
        unsigned CDIV0 :1;
        unsigned CDIV1 :1;
        unsigned CDIV2 :1;
        unsigned CDIV3 :1;
        unsigned COSC0 :1;
        unsigned COSC1 :1;
        unsigned COSC2 :1;
    };
} OSCCON2bits_t;
extern volatile OSCCON2bits_t OSCCON2bits __attribute__((address(0x88E)));
# 15047 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char OSCCON3 __attribute__((address(0x88F)));

__asm("OSCCON3 equ 088Fh");


typedef union {
    struct {
        unsigned :3;
        unsigned NOSCR :1;
        unsigned ORDY :1;
        unsigned :1;
        unsigned SOSCPWR :1;
        unsigned CSWHOLD :1;
    };
} OSCCON3bits_t;
extern volatile OSCCON3bits_t OSCCON3bits __attribute__((address(0x88F)));
# 15087 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char OSCSTAT __attribute__((address(0x890)));

__asm("OSCSTAT equ 0890h");


typedef union {
    struct {
        unsigned PLLR :1;
        unsigned :1;
        unsigned ADOR :1;
        unsigned SOR :1;
        unsigned LFOR :1;
        unsigned MFOR :1;
        unsigned HFOR :1;
        unsigned EXTOR :1;
    };
} OSCSTATbits_t;
extern volatile OSCSTATbits_t OSCSTATbits __attribute__((address(0x890)));
# 15144 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char OSCEN __attribute__((address(0x891)));

__asm("OSCEN equ 0891h");


typedef union {
    struct {
        unsigned :2;
        unsigned ADOEN :1;
        unsigned SOSCEN :1;
        unsigned LFOEN :1;
        unsigned MFOEN :1;
        unsigned HFOEN :1;
        unsigned EXTOEN :1;
    };
} OSCENbits_t;
extern volatile OSCENbits_t OSCENbits __attribute__((address(0x891)));
# 15195 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char OSCTUNE __attribute__((address(0x892)));

__asm("OSCTUNE equ 0892h");


typedef union {
    struct {
        unsigned TUN :6;
    };
    struct {
        unsigned TUN0 :1;
        unsigned TUN1 :1;
        unsigned TUN2 :1;
        unsigned TUN3 :1;
        unsigned TUN4 :1;
        unsigned TUN5 :1;
    };
} OSCTUNEbits_t;
extern volatile OSCTUNEbits_t OSCTUNEbits __attribute__((address(0x892)));
# 15253 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char OSCFRQ __attribute__((address(0x893)));

__asm("OSCFRQ equ 0893h");


typedef union {
    struct {
        unsigned HFFRQ :3;
    };
    struct {
        unsigned HFFRQ0 :1;
        unsigned HFFRQ1 :1;
        unsigned HFFRQ2 :1;
    };
} OSCFRQbits_t;
extern volatile OSCFRQbits_t OSCFRQbits __attribute__((address(0x893)));
# 15293 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ACTCON __attribute__((address(0x894)));

__asm("ACTCON equ 0894h");


typedef union {
    struct {
        unsigned :1;
        unsigned ACTORS :1;
        unsigned :1;
        unsigned ACTLOCK :1;
        unsigned :2;
        unsigned ACTUD :1;
        unsigned ACTEN :1;
    };
} ACTCONbits_t;
extern volatile ACTCONbits_t ACTCONbits __attribute__((address(0x894)));
# 15334 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char FVRCON __attribute__((address(0x90C)));

__asm("FVRCON equ 090Ch");


typedef union {
    struct {
        unsigned ADFVR :2;
        unsigned CDAFVR :2;
        unsigned TSRNG :1;
        unsigned TSEN :1;
        unsigned FVRRDY :1;
        unsigned FVREN :1;
    };
    struct {
        unsigned ADFVR0 :1;
        unsigned ADFVR1 :1;
        unsigned CDAFVR0 :1;
        unsigned CDAFVR1 :1;
    };
} FVRCONbits_t;
extern volatile FVRCONbits_t FVRCONbits __attribute__((address(0x90C)));
# 15410 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char DAC1CON0 __attribute__((address(0x90E)));

__asm("DAC1CON0 equ 090Eh");


typedef union {
    struct {
        unsigned NSS :1;
        unsigned :1;
        unsigned PSS :2;
        unsigned OE2 :1;
        unsigned OE1 :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned DAC1NSS :1;
        unsigned :1;
        unsigned DAC1PSS0 :1;
        unsigned DAC1PSS1 :1;
        unsigned DAC1OE2 :1;
        unsigned DAC1OE1 :1;
        unsigned :1;
        unsigned DAC1EN :1;
    };
    struct {
        unsigned :2;
        unsigned PSS0 :1;
        unsigned PSS1 :1;
    };
} DAC1CON0bits_t;
extern volatile DAC1CON0bits_t DAC1CON0bits __attribute__((address(0x90E)));
# 15511 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char DAC1CON1 __attribute__((address(0x90F)));

__asm("DAC1CON1 equ 090Fh");


typedef union {
    struct {
        unsigned DAC1R :5;
    };
    struct {
        unsigned DAC1R0 :1;
        unsigned DAC1R1 :1;
        unsigned DAC1R2 :1;
        unsigned DAC1R3 :1;
        unsigned DAC1R4 :1;
    };
} DAC1CON1bits_t;
extern volatile DAC1CON1bits_t DAC1CON1bits __attribute__((address(0x90F)));
# 15563 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ZCDCON __attribute__((address(0x91F)));

__asm("ZCDCON equ 091Fh");


typedef union {
    struct {
        unsigned ZCDINTN :1;
        unsigned ZCDINTP :1;
        unsigned :2;
        unsigned ZCDPOL :1;
        unsigned ZCDOUT :1;
        unsigned :1;
        unsigned ZCDSEN :1;
    };
} ZCDCONbits_t;
extern volatile ZCDCONbits_t ZCDCONbits __attribute__((address(0x91F)));
# 15609 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CMOUT __attribute__((address(0x98F)));

__asm("CMOUT equ 098Fh");


extern volatile unsigned char CMSTAT __attribute__((address(0x98F)));

__asm("CMSTAT equ 098Fh");


typedef union {
    struct {
        unsigned MC1OUT :1;
        unsigned MC2OUT :1;
    };
    struct {
        unsigned C1OUT :1;
        unsigned C2OUT :1;
    };
} CMOUTbits_t;
extern volatile CMOUTbits_t CMOUTbits __attribute__((address(0x98F)));
# 15652 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned MC1OUT :1;
        unsigned MC2OUT :1;
    };
    struct {
        unsigned C1OUT :1;
        unsigned C2OUT :1;
    };
} CMSTATbits_t;
extern volatile CMSTATbits_t CMSTATbits __attribute__((address(0x98F)));
# 15687 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CM1CON0 __attribute__((address(0x990)));

__asm("CM1CON0 equ 0990h");


typedef union {
    struct {
        unsigned SYNC :1;
        unsigned HYS :1;
        unsigned :2;
        unsigned POL :1;
        unsigned :1;
        unsigned OUT :1;
        unsigned ON :1;
    };
    struct {
        unsigned C1SYNC :1;
        unsigned C1HYS :1;
        unsigned :2;
        unsigned C1POL :1;
        unsigned :1;
        unsigned C1OUT :1;
        unsigned C1ON :1;
    };
} CM1CON0bits_t;
extern volatile CM1CON0bits_t CM1CON0bits __attribute__((address(0x990)));
# 15767 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CM1CON1 __attribute__((address(0x991)));

__asm("CM1CON1 equ 0991h");


typedef union {
    struct {
        unsigned INTN :1;
        unsigned INTP :1;
    };
    struct {
        unsigned C1INTN :1;
        unsigned C1INTP :1;
    };
} CM1CON1bits_t;
extern volatile CM1CON1bits_t CM1CON1bits __attribute__((address(0x991)));
# 15807 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CM1NSEL __attribute__((address(0x992)));

__asm("CM1NSEL equ 0992h");


typedef union {
    struct {
        unsigned NCH :3;
    };
    struct {
        unsigned NCH0 :1;
        unsigned NCH1 :1;
        unsigned NCH2 :1;
    };
    struct {
        unsigned C1NCH :3;
    };
    struct {
        unsigned C1NCH0 :1;
        unsigned C1NCH1 :1;
        unsigned C1NCH2 :1;
    };
} CM1NSELbits_t;
extern volatile CM1NSELbits_t CM1NSELbits __attribute__((address(0x992)));
# 15875 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CM1PSEL __attribute__((address(0x993)));

__asm("CM1PSEL equ 0993h");


typedef union {
    struct {
        unsigned PCH :4;
    };
    struct {
        unsigned PCH0 :1;
        unsigned PCH1 :1;
        unsigned PCH2 :1;
    };
    struct {
        unsigned C1PCH :4;
    };
    struct {
        unsigned C1PCH0 :1;
        unsigned C1PCH1 :1;
        unsigned C1PCH2 :1;
    };
} CM1PSELbits_t;
extern volatile CM1PSELbits_t CM1PSELbits __attribute__((address(0x993)));
# 15943 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CM2CON0 __attribute__((address(0x994)));

__asm("CM2CON0 equ 0994h");


typedef union {
    struct {
        unsigned SYNC :1;
        unsigned HYS :1;
        unsigned :2;
        unsigned POL :1;
        unsigned :1;
        unsigned OUT :1;
        unsigned ON :1;
    };
    struct {
        unsigned C2SYNC :1;
        unsigned C2HYS :1;
        unsigned :2;
        unsigned C2POL :1;
        unsigned :1;
        unsigned C2OUT :1;
        unsigned C2ON :1;
    };
} CM2CON0bits_t;
extern volatile CM2CON0bits_t CM2CON0bits __attribute__((address(0x994)));
# 16023 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CM2CON1 __attribute__((address(0x995)));

__asm("CM2CON1 equ 0995h");


typedef union {
    struct {
        unsigned INTN :1;
        unsigned INTP :1;
    };
    struct {
        unsigned C2INTN :1;
        unsigned C2INTP :1;
    };
} CM2CON1bits_t;
extern volatile CM2CON1bits_t CM2CON1bits __attribute__((address(0x995)));
# 16063 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CM2NSEL __attribute__((address(0x996)));

__asm("CM2NSEL equ 0996h");


typedef union {
    struct {
        unsigned NCH :3;
    };
    struct {
        unsigned NCH0 :1;
        unsigned NCH1 :1;
        unsigned NCH2 :1;
    };
    struct {
        unsigned C2NCH :3;
    };
    struct {
        unsigned C2NCH0 :1;
        unsigned C2NCH1 :1;
        unsigned C2NCH2 :1;
    };
} CM2NSELbits_t;
extern volatile CM2NSELbits_t CM2NSELbits __attribute__((address(0x996)));
# 16131 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CM2PSEL __attribute__((address(0x997)));

__asm("CM2PSEL equ 0997h");


typedef union {
    struct {
        unsigned PCH :4;
    };
    struct {
        unsigned PCH0 :1;
        unsigned PCH1 :1;
        unsigned PCH2 :1;
    };
    struct {
        unsigned C2PCH :4;
    };
    struct {
        unsigned C2PCH0 :1;
        unsigned C2PCH1 :1;
        unsigned C2PCH2 :1;
    };
} CM2PSELbits_t;
extern volatile CM2PSELbits_t CM2PSELbits __attribute__((address(0x997)));
# 16199 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC2REG __attribute__((address(0xA19)));

__asm("RC2REG equ 0A19h");


extern volatile unsigned char RCREG2 __attribute__((address(0xA19)));

__asm("RCREG2 equ 0A19h");


typedef union {
    struct {
        unsigned RC2REG :8;
    };
} RC2REGbits_t;
extern volatile RC2REGbits_t RC2REGbits __attribute__((address(0xA19)));







typedef union {
    struct {
        unsigned RC2REG :8;
    };
} RCREG2bits_t;
extern volatile RCREG2bits_t RCREG2bits __attribute__((address(0xA19)));
# 16237 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TX2REG __attribute__((address(0xA1A)));

__asm("TX2REG equ 0A1Ah");


extern volatile unsigned char TXREG2 __attribute__((address(0xA1A)));

__asm("TXREG2 equ 0A1Ah");


typedef union {
    struct {
        unsigned TX2REG :8;
    };
} TX2REGbits_t;
extern volatile TX2REGbits_t TX2REGbits __attribute__((address(0xA1A)));







typedef union {
    struct {
        unsigned TX2REG :8;
    };
} TXREG2bits_t;
extern volatile TXREG2bits_t TXREG2bits __attribute__((address(0xA1A)));
# 16275 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned short SP2BRG __attribute__((address(0xA1B)));

__asm("SP2BRG equ 0A1Bh");




extern volatile unsigned char SP2BRGL __attribute__((address(0xA1B)));

__asm("SP2BRGL equ 0A1Bh");


extern volatile unsigned char SPBRG2 __attribute__((address(0xA1B)));

__asm("SPBRG2 equ 0A1Bh");


typedef union {
    struct {
        unsigned SP2BRGL :8;
    };
} SP2BRGLbits_t;
extern volatile SP2BRGLbits_t SP2BRGLbits __attribute__((address(0xA1B)));







typedef union {
    struct {
        unsigned SP2BRGL :8;
    };
} SPBRG2bits_t;
extern volatile SPBRG2bits_t SPBRG2bits __attribute__((address(0xA1B)));
# 16320 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SP2BRGH __attribute__((address(0xA1C)));

__asm("SP2BRGH equ 0A1Ch");


extern volatile unsigned char SPBRGH2 __attribute__((address(0xA1C)));

__asm("SPBRGH2 equ 0A1Ch");


typedef union {
    struct {
        unsigned SP2BRGH :8;
    };
} SP2BRGHbits_t;
extern volatile SP2BRGHbits_t SP2BRGHbits __attribute__((address(0xA1C)));







typedef union {
    struct {
        unsigned SP2BRGH :8;
    };
} SPBRGH2bits_t;
extern volatile SPBRGH2bits_t SPBRGH2bits __attribute__((address(0xA1C)));
# 16358 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC2STA __attribute__((address(0xA1D)));

__asm("RC2STA equ 0A1Dh");


extern volatile unsigned char RCSTA2 __attribute__((address(0xA1D)));

__asm("RCSTA2 equ 0A1Dh");


typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
} RC2STAbits_t;
extern volatile RC2STAbits_t RC2STAbits __attribute__((address(0xA1D)));
# 16423 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
} RCSTA2bits_t;
extern volatile RCSTA2bits_t RCSTA2bits __attribute__((address(0xA1D)));
# 16480 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TX2STA __attribute__((address(0xA1E)));

__asm("TX2STA equ 0A1Eh");


extern volatile unsigned char TXSTA2 __attribute__((address(0xA1E)));

__asm("TXSTA2 equ 0A1Eh");


typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
} TX2STAbits_t;
extern volatile TX2STAbits_t TX2STAbits __attribute__((address(0xA1E)));
# 16545 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
} TXSTA2bits_t;
extern volatile TXSTA2bits_t TXSTA2bits __attribute__((address(0xA1E)));
# 16602 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char BAUD2CON __attribute__((address(0xA1F)));

__asm("BAUD2CON equ 0A1Fh");


extern volatile unsigned char BAUDCON2 __attribute__((address(0xA1F)));

__asm("BAUDCON2 equ 0A1Fh");

extern volatile unsigned char BAUDCTL2 __attribute__((address(0xA1F)));

__asm("BAUDCTL2 equ 0A1Fh");


typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUD2CONbits_t;
extern volatile BAUD2CONbits_t BAUD2CONbits __attribute__((address(0xA1F)));
# 16661 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUDCON2bits_t;
extern volatile BAUDCON2bits_t BAUDCON2bits __attribute__((address(0xA1F)));
# 16705 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUDCTL2bits_t;
extern volatile BAUDCTL2bits_t BAUDCTL2bits __attribute__((address(0xA1F)));
# 16752 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RTCCON __attribute__((address(0xC0C)));

__asm("RTCCON equ 0C0Ch");


typedef union {
    struct {
        unsigned RTCCLKSEL :2;
        unsigned :1;
        unsigned HALFSEC :1;
        unsigned RTCSYNC :1;
        unsigned RTCWREN :1;
        unsigned :1;
        unsigned RTCEN :1;
    };
    struct {
        unsigned RTCCLKSEL0 :1;
        unsigned RTCCLKSEL1 :1;
    };
} RTCCONbits_t;
extern volatile RTCCONbits_t RTCCONbits __attribute__((address(0xC0C)));
# 16812 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RTCCAL __attribute__((address(0xC0D)));

__asm("RTCCAL equ 0C0Dh");


typedef union {
    struct {
        unsigned CAL :8;
    };
    struct {
        unsigned CAL0 :1;
        unsigned CAL1 :1;
        unsigned CAL2 :1;
        unsigned CAL3 :1;
        unsigned CAL4 :1;
        unsigned CAL5 :1;
        unsigned CAL6 :1;
        unsigned CAL7 :1;
    };
} RTCCALbits_t;
extern volatile RTCCALbits_t RTCCALbits __attribute__((address(0xC0D)));
# 16882 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ALRMCON __attribute__((address(0xC0E)));

__asm("ALRMCON equ 0C0Eh");


typedef union {
    struct {
        unsigned :2;
        unsigned AMASK :4;
        unsigned CHIME :1;
        unsigned ALRMEN :1;
    };
    struct {
        unsigned :2;
        unsigned AMASK0 :1;
        unsigned AMASK1 :1;
        unsigned AMASK2 :1;
        unsigned AMASK3 :1;
    };
} ALRMCONbits_t;
extern volatile ALRMCONbits_t ALRMCONbits __attribute__((address(0xC0E)));
# 16942 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ALRMRPT __attribute__((address(0xC0F)));

__asm("ALRMRPT equ 0C0Fh");


typedef union {
    struct {
        unsigned ARPT :8;
    };
    struct {
        unsigned ARPT0 :1;
        unsigned ARPT1 :1;
        unsigned ARPT2 :1;
        unsigned ARPT3 :1;
        unsigned ARPT4 :1;
        unsigned ARPT5 :1;
        unsigned ARPT6 :1;
        unsigned ARPT7 :1;
    };
} ALRMRPTbits_t;
extern volatile ALRMRPTbits_t ALRMRPTbits __attribute__((address(0xC0F)));
# 17012 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char YEAR __attribute__((address(0xC10)));

__asm("YEAR equ 0C10h");


typedef union {
    struct {
        unsigned YEARL :4;
        unsigned YEARH :4;
    };
    struct {
        unsigned YEARL0 :1;
        unsigned YEARL1 :1;
        unsigned YEARL2 :1;
        unsigned YEARL3 :1;
        unsigned YEARH0 :1;
        unsigned YEARH1 :1;
        unsigned YEARH2 :1;
        unsigned YEARH3 :1;
    };
} YEARbits_t;
extern volatile YEARbits_t YEARbits __attribute__((address(0xC10)));
# 17088 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char MONTH __attribute__((address(0xC11)));

__asm("MONTH equ 0C11h");


typedef union {
    struct {
        unsigned MONTHL :4;
    };
    struct {
        unsigned MONTHL0 :1;
        unsigned MONTHL1 :1;
        unsigned MONTHL2 :1;
        unsigned MONTHL3 :1;
        unsigned MONTHH :1;
    };
} MONTHbits_t;
extern volatile MONTHbits_t MONTHbits __attribute__((address(0xC11)));
# 17140 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WEEKDAY __attribute__((address(0xC12)));

__asm("WEEKDAY equ 0C12h");


typedef union {
    struct {
        unsigned WDAY :3;
    };
    struct {
        unsigned WDAY0 :1;
        unsigned WDAY1 :1;
        unsigned WDAY2 :1;
    };
} WEEKDAYbits_t;
extern volatile WEEKDAYbits_t WEEKDAYbits __attribute__((address(0xC12)));
# 17180 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char DAY __attribute__((address(0xC13)));

__asm("DAY equ 0C13h");


typedef union {
    struct {
        unsigned DAYL :4;
        unsigned DAYH :2;
    };
    struct {
        unsigned DAYL0 :1;
        unsigned DAYL1 :1;
        unsigned DAYL2 :1;
        unsigned DAYL3 :1;
        unsigned DAYH0 :1;
        unsigned DAYH1 :1;
    };
} DAYbits_t;
extern volatile DAYbits_t DAYbits __attribute__((address(0xC13)));
# 17244 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char HOURS __attribute__((address(0xC14)));

__asm("HOURS equ 0C14h");


typedef union {
    struct {
        unsigned HRL :4;
        unsigned HRH :2;
    };
    struct {
        unsigned HRL0 :1;
        unsigned HRL1 :1;
        unsigned HRL2 :1;
        unsigned HRL3 :1;
        unsigned HRH0 :1;
        unsigned HRH1 :1;
    };
} HOURSbits_t;
extern volatile HOURSbits_t HOURSbits __attribute__((address(0xC14)));
# 17308 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char MINUTES __attribute__((address(0xC15)));

__asm("MINUTES equ 0C15h");


typedef union {
    struct {
        unsigned MINL :4;
        unsigned MINH :3;
    };
    struct {
        unsigned MINL0 :1;
        unsigned MINL1 :1;
        unsigned MINL2 :1;
        unsigned MINL3 :1;
        unsigned MINH0 :1;
        unsigned MINH1 :1;
        unsigned MINH2 :1;
    };
} MINUTESbits_t;
extern volatile MINUTESbits_t MINUTESbits __attribute__((address(0xC15)));
# 17378 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SECONDS __attribute__((address(0xC16)));

__asm("SECONDS equ 0C16h");


typedef union {
    struct {
        unsigned SECL :4;
        unsigned SECH :3;
    };
    struct {
        unsigned SECL0 :1;
        unsigned SECL1 :1;
        unsigned SECL2 :1;
        unsigned SECL3 :1;
        unsigned SECH0 :1;
        unsigned SECH1 :1;
        unsigned SECH2 :1;
    };
} SECONDSbits_t;
extern volatile SECONDSbits_t SECONDSbits __attribute__((address(0xC16)));
# 17448 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ALRMMTH __attribute__((address(0xC17)));

__asm("ALRMMTH equ 0C17h");


typedef union {
    struct {
        unsigned MTHALRML :4;
        unsigned MTHALRMH0 :1;
    };
    struct {
        unsigned MTHALRML0 :1;
        unsigned MTHALRML1 :1;
        unsigned MTHALRML2 :1;
        unsigned MTHALRML3 :1;
    };
} ALRMMTHbits_t;
extern volatile ALRMMTHbits_t ALRMMTHbits __attribute__((address(0xC17)));
# 17500 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ALRMWD __attribute__((address(0xC18)));

__asm("ALRMWD equ 0C18h");


typedef union {
    struct {
        unsigned WDALRM :3;
    };
    struct {
        unsigned WDALRM0 :1;
        unsigned WDALRM1 :1;
        unsigned WDALRM2 :1;
    };
} ALRMWDbits_t;
extern volatile ALRMWDbits_t ALRMWDbits __attribute__((address(0xC18)));
# 17540 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ALRMDAY __attribute__((address(0xC19)));

__asm("ALRMDAY equ 0C19h");


typedef union {
    struct {
        unsigned DAYALRML :4;
        unsigned DAYALRMH :2;
    };
    struct {
        unsigned DAYALRML0 :1;
        unsigned DAYALRML1 :1;
        unsigned DAYALRML2 :1;
        unsigned DAYALRML3 :1;
        unsigned DAYALRMH0 :1;
        unsigned DAYALRMH1 :1;
    };
} ALRMDAYbits_t;
extern volatile ALRMDAYbits_t ALRMDAYbits __attribute__((address(0xC19)));
# 17604 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ALRMHR __attribute__((address(0xC1A)));

__asm("ALRMHR equ 0C1Ah");


typedef union {
    struct {
        unsigned HRALRML :4;
        unsigned HRALRMH :2;
    };
    struct {
        unsigned HRALRML0 :1;
        unsigned HRALRML1 :1;
        unsigned HRALRML2 :1;
        unsigned HRALRML3 :1;
        unsigned HRALRMH0 :1;
        unsigned HRALRMH1 :1;
    };
} ALRMHRbits_t;
extern volatile ALRMHRbits_t ALRMHRbits __attribute__((address(0xC1A)));
# 17668 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ALRMMIN __attribute__((address(0xC1B)));

__asm("ALRMMIN equ 0C1Bh");


typedef union {
    struct {
        unsigned MINALRML :4;
        unsigned MINALRMH :3;
    };
    struct {
        unsigned MINALRML0 :1;
        unsigned MINALRML1 :1;
        unsigned MINALRML2 :1;
        unsigned MINALRML3 :1;
        unsigned MINALRMH0 :1;
        unsigned MINALRMH1 :1;
        unsigned MINALRMH2 :1;
    };
} ALRMMINbits_t;
extern volatile ALRMMINbits_t ALRMMINbits __attribute__((address(0xC1B)));
# 17738 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ALRMSEC __attribute__((address(0xC1C)));

__asm("ALRMSEC equ 0C1Ch");


typedef union {
    struct {
        unsigned SECALRML :4;
        unsigned SECALRMH :3;
    };
    struct {
        unsigned SECALRML0 :1;
        unsigned SECALRML1 :1;
        unsigned SECALRML2 :1;
        unsigned SECALRML3 :1;
        unsigned SECALRMH0 :1;
        unsigned SECALRMH1 :1;
        unsigned SECALRMH2 :1;
    };
} ALRMSECbits_t;
extern volatile ALRMSECbits_t ALRMSECbits __attribute__((address(0xC1C)));
# 17808 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char VB0GPR __attribute__((address(0xE8C)));

__asm("VB0GPR equ 0E8Ch");


typedef union {
    struct {
        unsigned VB0GPR :8;
    };
    struct {
        unsigned VB0GPR0 :1;
        unsigned VB0GPR1 :1;
        unsigned VB0GPR2 :1;
        unsigned VB0GPR3 :1;
        unsigned VB0GPR4 :1;
        unsigned VB0GPR5 :1;
        unsigned VB0GPR6 :1;
        unsigned VB0GPR7 :1;
    };
} VB0GPRbits_t;
extern volatile VB0GPRbits_t VB0GPRbits __attribute__((address(0xE8C)));
# 17878 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char VB1GPR __attribute__((address(0xE8D)));

__asm("VB1GPR equ 0E8Dh");


typedef union {
    struct {
        unsigned VB1GPR :8;
    };
    struct {
        unsigned VB1GPR0 :1;
        unsigned VB1GPR1 :1;
        unsigned VB1GPR2 :1;
        unsigned VB1GPR3 :1;
        unsigned VB1GPR4 :1;
        unsigned VB1GPR5 :1;
        unsigned VB1GPR6 :1;
        unsigned VB1GPR7 :1;
    };
} VB1GPRbits_t;
extern volatile VB1GPRbits_t VB1GPRbits __attribute__((address(0xE8D)));
# 17948 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char VB2GPR __attribute__((address(0xE8E)));

__asm("VB2GPR equ 0E8Eh");


typedef union {
    struct {
        unsigned VB2GPR :8;
    };
    struct {
        unsigned VB2GPR0 :1;
        unsigned VB2GPR1 :1;
        unsigned VB2GPR2 :1;
        unsigned VB2GPR3 :1;
        unsigned VB2GPR4 :1;
        unsigned VB2GPR5 :1;
        unsigned VB2GPR6 :1;
        unsigned VB2GPR7 :1;
    };
} VB2GPRbits_t;
extern volatile VB2GPRbits_t VB2GPRbits __attribute__((address(0xE8E)));
# 18018 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char VB3GPR __attribute__((address(0xE8F)));

__asm("VB3GPR equ 0E8Fh");


typedef union {
    struct {
        unsigned VB3GPR :8;
    };
    struct {
        unsigned VB3GPR0 :1;
        unsigned VB3GPR1 :1;
        unsigned VB3GPR2 :1;
        unsigned VB3GPR3 :1;
        unsigned VB3GPR4 :1;
        unsigned VB3GPR5 :1;
        unsigned VB3GPR6 :1;
        unsigned VB3GPR7 :1;
    };
} VB3GPRbits_t;
extern volatile VB3GPRbits_t VB3GPRbits __attribute__((address(0xE8F)));
# 18088 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDCON __attribute__((address(0x1D0C)));

__asm("LCDCON equ 01D0Ch");


typedef union {
    struct {
        unsigned LMUX :4;
        unsigned CS :1;
        unsigned WERR :1;
        unsigned SLPEN :1;
        unsigned LCDEN :1;
    };
    struct {
        unsigned LMUX0 :1;
        unsigned LMUX1 :1;
        unsigned LMUX2 :1;
        unsigned LMUX3 :1;
    };
} LCDCONbits_t;
extern volatile LCDCONbits_t LCDCONbits __attribute__((address(0x1D0C)));
# 18158 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDPS __attribute__((address(0x1D0D)));

__asm("LCDPS equ 01D0Dh");


typedef union {
    struct {
        unsigned LP :4;
        unsigned WA :1;
        unsigned LCDA :1;
        unsigned :1;
        unsigned WFT :1;
    };
    struct {
        unsigned LP0 :1;
        unsigned LP1 :1;
        unsigned LP2 :1;
        unsigned LP3 :1;
    };
} LCDPSbits_t;
extern volatile LCDPSbits_t LCDPSbits __attribute__((address(0x1D0D)));
# 18223 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDSE0 __attribute__((address(0x1D0E)));

__asm("LCDSE0 equ 01D0Eh");


typedef union {
    struct {
        unsigned SE00 :1;
        unsigned SE01 :1;
        unsigned SE02 :1;
        unsigned SE03 :1;
        unsigned SE04 :1;
        unsigned SE05 :1;
        unsigned SE06 :1;
        unsigned SE07 :1;
    };
} LCDSE0bits_t;
extern volatile LCDSE0bits_t LCDSE0bits __attribute__((address(0x1D0E)));
# 18285 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDSE1 __attribute__((address(0x1D0F)));

__asm("LCDSE1 equ 01D0Fh");


typedef union {
    struct {
        unsigned SE08 :1;
        unsigned SE09 :1;
        unsigned SE10 :1;
        unsigned SE11 :1;
        unsigned SE12 :1;
        unsigned SE13 :1;
        unsigned SE14 :1;
        unsigned SE15 :1;
    };
} LCDSE1bits_t;
extern volatile LCDSE1bits_t LCDSE1bits __attribute__((address(0x1D0F)));
# 18347 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDSE2 __attribute__((address(0x1D10)));

__asm("LCDSE2 equ 01D10h");


typedef union {
    struct {
        unsigned SE16 :1;
        unsigned SE17 :1;
        unsigned SE18 :1;
        unsigned SE19 :1;
        unsigned SE20 :1;
        unsigned SE21 :1;
        unsigned SE22 :1;
        unsigned SE23 :1;
    };
} LCDSE2bits_t;
extern volatile LCDSE2bits_t LCDSE2bits __attribute__((address(0x1D10)));
# 18409 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDSE3 __attribute__((address(0x1D11)));

__asm("LCDSE3 equ 01D11h");


typedef union {
    struct {
        unsigned SE24 :1;
        unsigned SE25 :1;
        unsigned SE26 :1;
        unsigned SE27 :1;
        unsigned SE28 :1;
        unsigned SE29 :1;
        unsigned SE30 :1;
        unsigned SE31 :1;
    };
} LCDSE3bits_t;
extern volatile LCDSE3bits_t LCDSE3bits __attribute__((address(0x1D11)));
# 18471 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDSE4 __attribute__((address(0x1D12)));

__asm("LCDSE4 equ 01D12h");


typedef union {
    struct {
        unsigned SE32 :1;
        unsigned SE33 :1;
        unsigned SE34 :1;
        unsigned SE35 :1;
        unsigned SE36 :1;
        unsigned SE37 :1;
        unsigned SE38 :1;
        unsigned SE39 :1;
    };
} LCDSE4bits_t;
extern volatile LCDSE4bits_t LCDSE4bits __attribute__((address(0x1D12)));
# 18533 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDSE5 __attribute__((address(0x1D13)));

__asm("LCDSE5 equ 01D13h");


typedef union {
    struct {
        unsigned SE40 :1;
        unsigned SE41 :1;
        unsigned SE42 :1;
        unsigned SE43 :1;
        unsigned SE44 :1;
        unsigned SE45 :1;
        unsigned SE46 :1;
        unsigned SE47 :1;
    };
} LCDSE5bits_t;
extern volatile LCDSE5bits_t LCDSE5bits __attribute__((address(0x1D13)));
# 18595 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDVCON1 __attribute__((address(0x1D14)));

__asm("LCDVCON1 equ 01D14h");


typedef union {
    struct {
        unsigned BIAS :3;
        unsigned :3;
        unsigned EN5V :1;
        unsigned LPEN :1;
    };
    struct {
        unsigned BIAS0 :1;
        unsigned BIAS1 :1;
        unsigned BIAS2 :1;
    };
} LCDVCON1bits_t;
extern volatile LCDVCON1bits_t LCDVCON1bits __attribute__((address(0x1D14)));
# 18648 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDVCON2 __attribute__((address(0x1D15)));

__asm("LCDVCON2 equ 01D15h");


typedef union {
    struct {
        unsigned LCDVSRC :4;
        unsigned :3;
        unsigned CPWDT :1;
    };
    struct {
        unsigned LCDVSRC0 :1;
        unsigned LCDVSRC1 :1;
        unsigned LCDVSRC2 :1;
        unsigned LCDVSRC3 :1;
    };
} LCDVCON2bits_t;
extern volatile LCDVCON2bits_t LCDVCON2bits __attribute__((address(0x1D15)));
# 18701 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDREF __attribute__((address(0x1D16)));

__asm("LCDREF equ 01D16h");


typedef union {
    struct {
        unsigned LCDCST :3;
    };
    struct {
        unsigned LCDCST0 :1;
        unsigned LCDCST1 :1;
        unsigned LCDCST2 :1;
    };
} LCDREFbits_t;
extern volatile LCDREFbits_t LCDREFbits __attribute__((address(0x1D16)));
# 18741 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDRL __attribute__((address(0x1D17)));

__asm("LCDRL equ 01D17h");


typedef union {
    struct {
        unsigned LRLAT :3;
        unsigned LCDIRI :1;
        unsigned LRLBP :2;
        unsigned LRLAP :2;
    };
    struct {
        unsigned LRLAT0 :1;
        unsigned LRLAT1 :1;
        unsigned LRLAT2 :1;
        unsigned :1;
        unsigned LRLBP0 :1;
        unsigned LRLBP1 :1;
        unsigned LRLAP0 :1;
        unsigned LRLAP1 :1;
    };
} LCDRLbits_t;
extern volatile LCDRLbits_t LCDRLbits __attribute__((address(0x1D17)));
# 18824 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA0 __attribute__((address(0x1D18)));

__asm("LCDDATA0 equ 01D18h");


typedef union {
    struct {
        unsigned S00C0 :1;
        unsigned S01C0 :1;
        unsigned S02C0 :1;
        unsigned S03C0 :1;
        unsigned S04C0 :1;
        unsigned :1;
        unsigned S06C0 :1;
        unsigned S07C0 :1;
    };
} LCDDATA0bits_t;
extern volatile LCDDATA0bits_t LCDDATA0bits __attribute__((address(0x1D18)));
# 18881 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA1 __attribute__((address(0x1D19)));

__asm("LCDDATA1 equ 01D19h");


typedef union {
    struct {
        unsigned S08C0 :1;
        unsigned S09C0 :1;
        unsigned S10C0 :1;
        unsigned S11C0 :1;
        unsigned :1;
        unsigned S13C0 :1;
        unsigned S14C0 :1;
        unsigned S15C0 :1;
    };
} LCDDATA1bits_t;
extern volatile LCDDATA1bits_t LCDDATA1bits __attribute__((address(0x1D19)));
# 18938 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA2 __attribute__((address(0x1D1A)));

__asm("LCDDATA2 equ 01D1Ah");


typedef union {
    struct {
        unsigned :2;
        unsigned S18C0 :1;
        unsigned S19C0 :1;
        unsigned S20C0 :1;
        unsigned :1;
        unsigned S22C0 :1;
        unsigned S23C0 :1;
    };
} LCDDATA2bits_t;
extern volatile LCDDATA2bits_t LCDDATA2bits __attribute__((address(0x1D1A)));
# 18984 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA6 __attribute__((address(0x1D1E)));

__asm("LCDDATA6 equ 01D1Eh");


typedef union {
    struct {
        unsigned S00C1 :1;
        unsigned S01C1 :1;
        unsigned S02C1 :1;
        unsigned S03C1 :1;
        unsigned S04C1 :1;
        unsigned :1;
        unsigned S06C1 :1;
        unsigned S07C1 :1;
    };
} LCDDATA6bits_t;
extern volatile LCDDATA6bits_t LCDDATA6bits __attribute__((address(0x1D1E)));
# 19041 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA7 __attribute__((address(0x1D1F)));

__asm("LCDDATA7 equ 01D1Fh");


typedef union {
    struct {
        unsigned S08C1 :1;
        unsigned S09C1 :1;
        unsigned S10C1 :1;
        unsigned S11C1 :1;
        unsigned :1;
        unsigned S13C1 :1;
        unsigned S14C1 :1;
        unsigned S15C1 :1;
    };
} LCDDATA7bits_t;
extern volatile LCDDATA7bits_t LCDDATA7bits __attribute__((address(0x1D1F)));
# 19098 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA8 __attribute__((address(0x1D20)));

__asm("LCDDATA8 equ 01D20h");


typedef union {
    struct {
        unsigned :2;
        unsigned S18C1 :1;
        unsigned S19C1 :1;
        unsigned S20C1 :1;
        unsigned :1;
        unsigned S22C1 :1;
        unsigned S23C1 :1;
    };
} LCDDATA8bits_t;
extern volatile LCDDATA8bits_t LCDDATA8bits __attribute__((address(0x1D20)));
# 19144 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA12 __attribute__((address(0x1D24)));

__asm("LCDDATA12 equ 01D24h");


typedef union {
    struct {
        unsigned S00C2 :1;
        unsigned S01C2 :1;
        unsigned S02C2 :1;
        unsigned S03C2 :1;
        unsigned S04C2 :1;
        unsigned :1;
        unsigned S06C2 :1;
        unsigned S07C2 :1;
    };
} LCDDATA12bits_t;
extern volatile LCDDATA12bits_t LCDDATA12bits __attribute__((address(0x1D24)));
# 19201 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA13 __attribute__((address(0x1D25)));

__asm("LCDDATA13 equ 01D25h");


typedef union {
    struct {
        unsigned S08C2 :1;
        unsigned S09C2 :1;
        unsigned S10C2 :1;
        unsigned S11C2 :1;
        unsigned :1;
        unsigned S13C2 :1;
        unsigned S14C2 :1;
        unsigned S15C2 :1;
    };
} LCDDATA13bits_t;
extern volatile LCDDATA13bits_t LCDDATA13bits __attribute__((address(0x1D25)));
# 19258 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA14 __attribute__((address(0x1D26)));

__asm("LCDDATA14 equ 01D26h");


typedef union {
    struct {
        unsigned :2;
        unsigned S18C2 :1;
        unsigned S19C2 :1;
        unsigned S20C2 :1;
        unsigned :1;
        unsigned S22C2 :1;
        unsigned S23C2 :1;
    };
} LCDDATA14bits_t;
extern volatile LCDDATA14bits_t LCDDATA14bits __attribute__((address(0x1D26)));
# 19304 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA18 __attribute__((address(0x1D2A)));

__asm("LCDDATA18 equ 01D2Ah");


typedef union {
    struct {
        unsigned S00C3 :1;
        unsigned S01C3 :1;
        unsigned S02C3 :1;
        unsigned S03C3 :1;
        unsigned S04C3 :1;
        unsigned :1;
        unsigned S06C3 :1;
        unsigned S07C3 :1;
    };
} LCDDATA18bits_t;
extern volatile LCDDATA18bits_t LCDDATA18bits __attribute__((address(0x1D2A)));
# 19361 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA19 __attribute__((address(0x1D2B)));

__asm("LCDDATA19 equ 01D2Bh");


typedef union {
    struct {
        unsigned S08C3 :1;
        unsigned S09C3 :1;
        unsigned S10C3 :1;
        unsigned S11C3 :1;
        unsigned :1;
        unsigned S13C3 :1;
        unsigned S14C3 :1;
        unsigned S15C3 :1;
    };
} LCDDATA19bits_t;
extern volatile LCDDATA19bits_t LCDDATA19bits __attribute__((address(0x1D2B)));
# 19418 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA20 __attribute__((address(0x1D2C)));

__asm("LCDDATA20 equ 01D2Ch");


typedef union {
    struct {
        unsigned :2;
        unsigned S18C3 :1;
        unsigned S19C3 :1;
        unsigned S20C3 :1;
        unsigned :1;
        unsigned S22C3 :1;
        unsigned S23C3 :1;
    };
} LCDDATA20bits_t;
extern volatile LCDDATA20bits_t LCDDATA20bits __attribute__((address(0x1D2C)));
# 19464 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA24 __attribute__((address(0x1D30)));

__asm("LCDDATA24 equ 01D30h");


typedef union {
    struct {
        unsigned S00C4 :1;
        unsigned S01C4 :1;
        unsigned S02C4 :1;
        unsigned S03C4 :1;
        unsigned S04C4 :1;
        unsigned :1;
        unsigned S06C4 :1;
        unsigned S07C4 :1;
    };
} LCDDATA24bits_t;
extern volatile LCDDATA24bits_t LCDDATA24bits __attribute__((address(0x1D30)));
# 19521 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA25 __attribute__((address(0x1D31)));

__asm("LCDDATA25 equ 01D31h");


typedef union {
    struct {
        unsigned S08C4 :1;
        unsigned S09C4 :1;
        unsigned S10C4 :1;
        unsigned S11C4 :1;
        unsigned :1;
        unsigned S13C4 :1;
        unsigned S14C4 :1;
        unsigned S15C4 :1;
    };
} LCDDATA25bits_t;
extern volatile LCDDATA25bits_t LCDDATA25bits __attribute__((address(0x1D31)));
# 19578 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA26 __attribute__((address(0x1D32)));

__asm("LCDDATA26 equ 01D32h");


typedef union {
    struct {
        unsigned :2;
        unsigned S18C4 :1;
        unsigned S19C4 :1;
        unsigned S20C4 :1;
        unsigned :1;
        unsigned S22C4 :1;
        unsigned S23C4 :1;
    };
} LCDDATA26bits_t;
extern volatile LCDDATA26bits_t LCDDATA26bits __attribute__((address(0x1D32)));
# 19624 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA30 __attribute__((address(0x1D36)));

__asm("LCDDATA30 equ 01D36h");


typedef union {
    struct {
        unsigned S00C5 :1;
        unsigned S01C5 :1;
        unsigned S02C5 :1;
        unsigned S03C5 :1;
        unsigned S04C5 :1;
        unsigned :1;
        unsigned S06C5 :1;
        unsigned S07C5 :1;
    };
} LCDDATA30bits_t;
extern volatile LCDDATA30bits_t LCDDATA30bits __attribute__((address(0x1D36)));
# 19681 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA31 __attribute__((address(0x1D37)));

__asm("LCDDATA31 equ 01D37h");


typedef union {
    struct {
        unsigned S08C5 :1;
        unsigned S09C5 :1;
        unsigned S10C5 :1;
        unsigned S11C5 :1;
        unsigned :1;
        unsigned S13C5 :1;
        unsigned S14C5 :1;
        unsigned S15C5 :1;
    };
} LCDDATA31bits_t;
extern volatile LCDDATA31bits_t LCDDATA31bits __attribute__((address(0x1D37)));
# 19738 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA32 __attribute__((address(0x1D38)));

__asm("LCDDATA32 equ 01D38h");


typedef union {
    struct {
        unsigned :2;
        unsigned S18C5 :1;
        unsigned S19C5 :1;
        unsigned S20C5 :1;
        unsigned :1;
        unsigned S22C5 :1;
        unsigned S23C5 :1;
    };
} LCDDATA32bits_t;
extern volatile LCDDATA32bits_t LCDDATA32bits __attribute__((address(0x1D38)));
# 19784 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA36 __attribute__((address(0x1D3C)));

__asm("LCDDATA36 equ 01D3Ch");


typedef union {
    struct {
        unsigned S00C6 :1;
        unsigned S01C6 :1;
        unsigned S02C6 :1;
        unsigned S03C6 :1;
        unsigned S04C6 :1;
        unsigned :1;
        unsigned S06C6 :1;
        unsigned S07C6 :1;
    };
} LCDDATA36bits_t;
extern volatile LCDDATA36bits_t LCDDATA36bits __attribute__((address(0x1D3C)));
# 19841 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA37 __attribute__((address(0x1D3D)));

__asm("LCDDATA37 equ 01D3Dh");


typedef union {
    struct {
        unsigned S08C6 :1;
        unsigned S09C6 :1;
        unsigned S10C6 :1;
        unsigned S11C6 :1;
        unsigned :1;
        unsigned S13C6 :1;
        unsigned S14C6 :1;
        unsigned S15C6 :1;
    };
} LCDDATA37bits_t;
extern volatile LCDDATA37bits_t LCDDATA37bits __attribute__((address(0x1D3D)));
# 19898 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA38 __attribute__((address(0x1D3E)));

__asm("LCDDATA38 equ 01D3Eh");


typedef union {
    struct {
        unsigned :2;
        unsigned S18C6 :1;
        unsigned S19C6 :1;
        unsigned S20C6 :1;
        unsigned :1;
        unsigned S22C6 :1;
        unsigned S23C6 :1;
    };
} LCDDATA38bits_t;
extern volatile LCDDATA38bits_t LCDDATA38bits __attribute__((address(0x1D3E)));
# 19944 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA42 __attribute__((address(0x1D42)));

__asm("LCDDATA42 equ 01D42h");


typedef union {
    struct {
        unsigned S00C7 :1;
        unsigned S01C7 :1;
        unsigned S02C7 :1;
        unsigned S03C7 :1;
        unsigned S04C7 :1;
        unsigned :1;
        unsigned S06C7 :1;
        unsigned S07C7 :1;
    };
} LCDDATA42bits_t;
extern volatile LCDDATA42bits_t LCDDATA42bits __attribute__((address(0x1D42)));
# 20001 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA43 __attribute__((address(0x1D43)));

__asm("LCDDATA43 equ 01D43h");


typedef union {
    struct {
        unsigned S08C7 :1;
        unsigned S09C7 :1;
        unsigned S10C7 :1;
        unsigned S11C7 :1;
        unsigned :1;
        unsigned S13C7 :1;
        unsigned S14C7 :1;
        unsigned S15C7 :1;
    };
} LCDDATA43bits_t;
extern volatile LCDDATA43bits_t LCDDATA43bits __attribute__((address(0x1D43)));
# 20058 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char LCDDATA44 __attribute__((address(0x1D44)));

__asm("LCDDATA44 equ 01D44h");


typedef union {
    struct {
        unsigned :2;
        unsigned S18C7 :1;
        unsigned S19C7 :1;
        unsigned S20C7 :1;
        unsigned :1;
        unsigned S22C7 :1;
        unsigned S23C7 :1;
    };
} LCDDATA44bits_t;
extern volatile LCDDATA44bits_t LCDDATA44bits __attribute__((address(0x1D44)));
# 20104 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLCDATA __attribute__((address(0x1E0F)));

__asm("CLCDATA equ 01E0Fh");


typedef union {
    struct {
        unsigned MLC1OUT :1;
        unsigned MLC2OUT :1;
        unsigned MLC3OUT :1;
        unsigned MLC4OUT :1;
    };
} CLCDATAbits_t;
extern volatile CLCDATAbits_t CLCDATAbits __attribute__((address(0x1E0F)));
# 20142 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC1CON __attribute__((address(0x1E10)));

__asm("CLC1CON equ 01E10h");


typedef union {
    struct {
        unsigned LC1MODE :3;
        unsigned LC1INTN :1;
        unsigned LC1INTP :1;
        unsigned LC1OUT :1;
        unsigned :1;
        unsigned LC1EN :1;
    };
    struct {
        unsigned LC1MODE0 :1;
        unsigned LC1MODE1 :1;
        unsigned LC1MODE2 :1;
    };
    struct {
        unsigned MODE :3;
        unsigned INTN :1;
        unsigned INTP :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
    };
} CLC1CONbits_t;
extern volatile CLC1CONbits_t CLC1CONbits __attribute__((address(0x1E10)));
# 20260 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC1POL __attribute__((address(0x1E11)));

__asm("CLC1POL equ 01E11h");


typedef union {
    struct {
        unsigned LC1G1POL :1;
        unsigned LC1G2POL :1;
        unsigned LC1G3POL :1;
        unsigned LC1G4POL :1;
        unsigned :3;
        unsigned LC1POL :1;
    };
    struct {
        unsigned G1POL :1;
        unsigned G2POL :1;
        unsigned G3POL :1;
        unsigned G4POL :1;
        unsigned :3;
        unsigned POL :1;
    };
} CLC1POLbits_t;
extern volatile CLC1POLbits_t CLC1POLbits __attribute__((address(0x1E11)));
# 20338 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC1SEL0 __attribute__((address(0x1E12)));

__asm("CLC1SEL0 equ 01E12h");


typedef union {
    struct {
        unsigned LC1D1S0 :1;
        unsigned LC1D1S1 :1;
        unsigned LC1D1S2 :1;
        unsigned LC1D1S3 :1;
        unsigned LC1D1S4 :1;
        unsigned LC1D1S5 :1;
        unsigned LC1D1S6 :1;
        unsigned LC1D1S7 :1;
    };
    struct {
        unsigned LC1D1S :8;
    };
    struct {
        unsigned D1S :8;
    };
    struct {
        unsigned D1S0 :1;
        unsigned D1S1 :1;
        unsigned D1S2 :1;
        unsigned D1S3 :1;
        unsigned D1S4 :1;
        unsigned D1S5 :1;
        unsigned D1S6 :1;
        unsigned D1S7 :1;
    };
} CLC1SEL0bits_t;
extern volatile CLC1SEL0bits_t CLC1SEL0bits __attribute__((address(0x1E12)));
# 20466 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC1SEL1 __attribute__((address(0x1E13)));

__asm("CLC1SEL1 equ 01E13h");


typedef union {
    struct {
        unsigned LC1D2S0 :1;
        unsigned LC1D2S1 :1;
        unsigned LC1D2S2 :1;
        unsigned LC1D2S3 :1;
        unsigned LC1D2S4 :1;
        unsigned LC1D2S5 :1;
        unsigned LC1D2S6 :1;
        unsigned LC1D2S7 :1;
    };
    struct {
        unsigned LC1D2S :8;
    };
    struct {
        unsigned D2S :8;
    };
    struct {
        unsigned D2S0 :1;
        unsigned D2S1 :1;
        unsigned D2S2 :1;
        unsigned D2S3 :1;
        unsigned D2S4 :1;
        unsigned D2S5 :1;
        unsigned D2S6 :1;
        unsigned D2S7 :1;
    };
} CLC1SEL1bits_t;
extern volatile CLC1SEL1bits_t CLC1SEL1bits __attribute__((address(0x1E13)));
# 20594 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC1SEL2 __attribute__((address(0x1E14)));

__asm("CLC1SEL2 equ 01E14h");


typedef union {
    struct {
        unsigned LC1D3S0 :1;
        unsigned LC1D3S1 :1;
        unsigned LC1D3S2 :1;
        unsigned LC1D3S3 :1;
        unsigned LC1D3S4 :1;
        unsigned LC1D3S5 :1;
        unsigned LC1D3S6 :1;
        unsigned LC1D3S7 :1;
    };
    struct {
        unsigned LC1D3S :8;
    };
    struct {
        unsigned D3S :8;
    };
    struct {
        unsigned D3S0 :1;
        unsigned D3S1 :1;
        unsigned D3S2 :1;
        unsigned D3S3 :1;
        unsigned D3S4 :1;
        unsigned D3S5 :1;
        unsigned D3S6 :1;
        unsigned D3S7 :1;
    };
} CLC1SEL2bits_t;
extern volatile CLC1SEL2bits_t CLC1SEL2bits __attribute__((address(0x1E14)));
# 20722 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC1SEL3 __attribute__((address(0x1E15)));

__asm("CLC1SEL3 equ 01E15h");


typedef union {
    struct {
        unsigned LC1D4S0 :1;
        unsigned LC1D4S1 :1;
        unsigned LC1D4S2 :1;
        unsigned LC1D4S3 :1;
        unsigned LC1D4S4 :1;
        unsigned LC1D4S5 :1;
        unsigned LC1D4S6 :1;
        unsigned LC1D4S7 :1;
    };
    struct {
        unsigned LC1D4S :8;
    };
    struct {
        unsigned D4S :8;
    };
    struct {
        unsigned D4S0 :1;
        unsigned D4S1 :1;
        unsigned D4S2 :1;
        unsigned D4S3 :1;
        unsigned D4S4 :1;
        unsigned D4S5 :1;
        unsigned D4S6 :1;
        unsigned D4S7 :1;
    };
} CLC1SEL3bits_t;
extern volatile CLC1SEL3bits_t CLC1SEL3bits __attribute__((address(0x1E15)));
# 20850 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC1GLS0 __attribute__((address(0x1E16)));

__asm("CLC1GLS0 equ 01E16h");


typedef union {
    struct {
        unsigned LC1G1D1N :1;
        unsigned LC1G1D1T :1;
        unsigned LC1G1D2N :1;
        unsigned LC1G1D2T :1;
        unsigned LC1G1D3N :1;
        unsigned LC1G1D3T :1;
        unsigned LC1G1D4N :1;
        unsigned LC1G1D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC1GLS0bits_t;
extern volatile CLC1GLS0bits_t CLC1GLS0bits __attribute__((address(0x1E16)));
# 20962 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC1GLS1 __attribute__((address(0x1E17)));

__asm("CLC1GLS1 equ 01E17h");


typedef union {
    struct {
        unsigned LC1G2D1N :1;
        unsigned LC1G2D1T :1;
        unsigned LC1G2D2N :1;
        unsigned LC1G2D2T :1;
        unsigned LC1G2D3N :1;
        unsigned LC1G2D3T :1;
        unsigned LC1G2D4N :1;
        unsigned LC1G2D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC1GLS1bits_t;
extern volatile CLC1GLS1bits_t CLC1GLS1bits __attribute__((address(0x1E17)));
# 21074 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC1GLS2 __attribute__((address(0x1E18)));

__asm("CLC1GLS2 equ 01E18h");


typedef union {
    struct {
        unsigned LC1G3D1N :1;
        unsigned LC1G3D1T :1;
        unsigned LC1G3D2N :1;
        unsigned LC1G3D2T :1;
        unsigned LC1G3D3N :1;
        unsigned LC1G3D3T :1;
        unsigned LC1G3D4N :1;
        unsigned LC1G3D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC1GLS2bits_t;
extern volatile CLC1GLS2bits_t CLC1GLS2bits __attribute__((address(0x1E18)));
# 21186 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC1GLS3 __attribute__((address(0x1E19)));

__asm("CLC1GLS3 equ 01E19h");


typedef union {
    struct {
        unsigned LC1G4D1N :1;
        unsigned LC1G4D1T :1;
        unsigned LC1G4D2N :1;
        unsigned LC1G4D2T :1;
        unsigned LC1G4D3N :1;
        unsigned LC1G4D3T :1;
        unsigned LC1G4D4N :1;
        unsigned LC1G4D4T :1;
    };
    struct {
        unsigned G4D1N :1;
        unsigned G4D1T :1;
        unsigned G4D2N :1;
        unsigned G4D2T :1;
        unsigned G4D3N :1;
        unsigned G4D3T :1;
        unsigned G4D4N :1;
        unsigned G4D4T :1;
    };
} CLC1GLS3bits_t;
extern volatile CLC1GLS3bits_t CLC1GLS3bits __attribute__((address(0x1E19)));
# 21298 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC2CON __attribute__((address(0x1E1A)));

__asm("CLC2CON equ 01E1Ah");


typedef union {
    struct {
        unsigned LC2MODE :3;
        unsigned LC2INTN :1;
        unsigned LC2INTP :1;
        unsigned LC2OUT :1;
        unsigned :1;
        unsigned LC2EN :1;
    };
    struct {
        unsigned LC2MODE0 :1;
        unsigned LC2MODE1 :1;
        unsigned LC2MODE2 :1;
    };
    struct {
        unsigned MODE :3;
        unsigned INTN :1;
        unsigned INTP :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
    };
} CLC2CONbits_t;
extern volatile CLC2CONbits_t CLC2CONbits __attribute__((address(0x1E1A)));
# 21416 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC2POL __attribute__((address(0x1E1B)));

__asm("CLC2POL equ 01E1Bh");


typedef union {
    struct {
        unsigned LC2G1POL :1;
        unsigned LC2G2POL :1;
        unsigned LC2G3POL :1;
        unsigned LC2G4POL :1;
        unsigned :3;
        unsigned LC2POL :1;
    };
    struct {
        unsigned G1POL :1;
        unsigned G2POL :1;
        unsigned G3POL :1;
        unsigned G4POL :1;
        unsigned :3;
        unsigned POL :1;
    };
} CLC2POLbits_t;
extern volatile CLC2POLbits_t CLC2POLbits __attribute__((address(0x1E1B)));
# 21494 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC2SEL0 __attribute__((address(0x1E1C)));

__asm("CLC2SEL0 equ 01E1Ch");


typedef union {
    struct {
        unsigned LC2D1S0 :1;
        unsigned LC2D1S1 :1;
        unsigned LC2D1S2 :1;
        unsigned LC2D1S3 :1;
        unsigned LC2D1S4 :1;
        unsigned LC2D1S5 :1;
        unsigned LC2D1S6 :1;
        unsigned LC2D1S7 :1;
    };
    struct {
        unsigned LC2D1S :8;
    };
    struct {
        unsigned D1S :8;
    };
    struct {
        unsigned D1S0 :1;
        unsigned D1S1 :1;
        unsigned D1S2 :1;
        unsigned D1S3 :1;
        unsigned D1S4 :1;
        unsigned D1S5 :1;
        unsigned D1S6 :1;
        unsigned D1S7 :1;
    };
} CLC2SEL0bits_t;
extern volatile CLC2SEL0bits_t CLC2SEL0bits __attribute__((address(0x1E1C)));
# 21622 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC2SEL1 __attribute__((address(0x1E1D)));

__asm("CLC2SEL1 equ 01E1Dh");


typedef union {
    struct {
        unsigned LC2D2S0 :1;
        unsigned LC2D2S1 :1;
        unsigned LC2D2S2 :1;
        unsigned LC2D2S3 :1;
        unsigned LC2D2S4 :1;
        unsigned LC2D2S5 :1;
        unsigned LC2D2S6 :1;
        unsigned LC2D2S7 :1;
    };
    struct {
        unsigned LC2D2S :8;
    };
    struct {
        unsigned D2S :8;
    };
    struct {
        unsigned D2S0 :1;
        unsigned D2S1 :1;
        unsigned D2S2 :1;
        unsigned D2S3 :1;
        unsigned D2S4 :1;
        unsigned D2S5 :1;
        unsigned D2S6 :1;
        unsigned D2S7 :1;
    };
} CLC2SEL1bits_t;
extern volatile CLC2SEL1bits_t CLC2SEL1bits __attribute__((address(0x1E1D)));
# 21750 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC2SEL2 __attribute__((address(0x1E1E)));

__asm("CLC2SEL2 equ 01E1Eh");


typedef union {
    struct {
        unsigned LC2D3S0 :1;
        unsigned LC2D3S1 :1;
        unsigned LC2D3S2 :1;
        unsigned LC2D3S3 :1;
        unsigned LC2D3S4 :1;
        unsigned LC2D3S5 :1;
        unsigned LC2D3S6 :1;
        unsigned LC2D3S7 :1;
    };
    struct {
        unsigned LC2D3S :8;
    };
    struct {
        unsigned D3S :8;
    };
    struct {
        unsigned D3S0 :1;
        unsigned D3S1 :1;
        unsigned D3S2 :1;
        unsigned D3S3 :1;
        unsigned D3S4 :1;
        unsigned D3S5 :1;
        unsigned D3S6 :1;
        unsigned D3S7 :1;
    };
} CLC2SEL2bits_t;
extern volatile CLC2SEL2bits_t CLC2SEL2bits __attribute__((address(0x1E1E)));
# 21878 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC2SEL3 __attribute__((address(0x1E1F)));

__asm("CLC2SEL3 equ 01E1Fh");


typedef union {
    struct {
        unsigned LC2D4S0 :1;
        unsigned LC2D4S1 :1;
        unsigned LC2D4S2 :1;
        unsigned LC2D4S3 :1;
        unsigned LC2D4S4 :1;
        unsigned LC2D4S5 :1;
        unsigned LC2D4S6 :1;
        unsigned LC2D4S7 :1;
    };
    struct {
        unsigned LC2D4S :8;
    };
    struct {
        unsigned D4S :8;
    };
    struct {
        unsigned D4S0 :1;
        unsigned D4S1 :1;
        unsigned D4S2 :1;
        unsigned D4S3 :1;
        unsigned D4S4 :1;
        unsigned D4S5 :1;
        unsigned D4S6 :1;
        unsigned D4S7 :1;
    };
} CLC2SEL3bits_t;
extern volatile CLC2SEL3bits_t CLC2SEL3bits __attribute__((address(0x1E1F)));
# 22006 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC2GLS0 __attribute__((address(0x1E20)));

__asm("CLC2GLS0 equ 01E20h");


typedef union {
    struct {
        unsigned LC2G1D1N :1;
        unsigned LC2G1D1T :1;
        unsigned LC2G1D2N :1;
        unsigned LC2G1D2T :1;
        unsigned LC2G1D3N :1;
        unsigned LC2G1D3T :1;
        unsigned LC2G1D4N :1;
        unsigned LC2G1D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC2GLS0bits_t;
extern volatile CLC2GLS0bits_t CLC2GLS0bits __attribute__((address(0x1E20)));
# 22118 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC2GLS1 __attribute__((address(0x1E21)));

__asm("CLC2GLS1 equ 01E21h");


typedef union {
    struct {
        unsigned LC2G2D1N :1;
        unsigned LC2G2D1T :1;
        unsigned LC2G2D2N :1;
        unsigned LC2G2D2T :1;
        unsigned LC2G2D3N :1;
        unsigned LC2G2D3T :1;
        unsigned LC2G2D4N :1;
        unsigned LC2G2D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC2GLS1bits_t;
extern volatile CLC2GLS1bits_t CLC2GLS1bits __attribute__((address(0x1E21)));
# 22230 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC2GLS2 __attribute__((address(0x1E22)));

__asm("CLC2GLS2 equ 01E22h");


typedef union {
    struct {
        unsigned LC2G3D1N :1;
        unsigned LC2G3D1T :1;
        unsigned LC2G3D2N :1;
        unsigned LC2G3D2T :1;
        unsigned LC2G3D3N :1;
        unsigned LC2G3D3T :1;
        unsigned LC2G3D4N :1;
        unsigned LC2G3D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC2GLS2bits_t;
extern volatile CLC2GLS2bits_t CLC2GLS2bits __attribute__((address(0x1E22)));
# 22342 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC2GLS3 __attribute__((address(0x1E23)));

__asm("CLC2GLS3 equ 01E23h");


typedef union {
    struct {
        unsigned LC2G4D1N :1;
        unsigned LC2G4D1T :1;
        unsigned LC2G4D2N :1;
        unsigned LC2G4D2T :1;
        unsigned LC2G4D3N :1;
        unsigned LC2G4D3T :1;
        unsigned LC2G4D4N :1;
        unsigned LC2G4D4T :1;
    };
    struct {
        unsigned G4D1N :1;
        unsigned G4D1T :1;
        unsigned G4D2N :1;
        unsigned G4D2T :1;
        unsigned G4D3N :1;
        unsigned G4D3T :1;
        unsigned G4D4N :1;
        unsigned G4D4T :1;
    };
} CLC2GLS3bits_t;
extern volatile CLC2GLS3bits_t CLC2GLS3bits __attribute__((address(0x1E23)));
# 22454 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC3CON __attribute__((address(0x1E24)));

__asm("CLC3CON equ 01E24h");


typedef union {
    struct {
        unsigned LC3MODE :3;
        unsigned LC3INTN :1;
        unsigned LC3INTP :1;
        unsigned LC3OUT :1;
        unsigned :1;
        unsigned LC3EN :1;
    };
    struct {
        unsigned LC3MODE0 :1;
        unsigned LC3MODE1 :1;
        unsigned LC3MODE2 :1;
    };
    struct {
        unsigned MODE :3;
        unsigned INTN :1;
        unsigned INTP :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
    };
} CLC3CONbits_t;
extern volatile CLC3CONbits_t CLC3CONbits __attribute__((address(0x1E24)));
# 22572 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC3POL __attribute__((address(0x1E25)));

__asm("CLC3POL equ 01E25h");


typedef union {
    struct {
        unsigned LC3G1POL :1;
        unsigned LC3G2POL :1;
        unsigned LC3G3POL :1;
        unsigned LC3G4POL :1;
        unsigned :3;
        unsigned LC3POL :1;
    };
    struct {
        unsigned G1POL :1;
        unsigned G2POL :1;
        unsigned G3POL :1;
        unsigned G4POL :1;
        unsigned :3;
        unsigned POL :1;
    };
} CLC3POLbits_t;
extern volatile CLC3POLbits_t CLC3POLbits __attribute__((address(0x1E25)));
# 22650 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC3SEL0 __attribute__((address(0x1E26)));

__asm("CLC3SEL0 equ 01E26h");


typedef union {
    struct {
        unsigned LC3D1S0 :1;
        unsigned LC3D1S1 :1;
        unsigned LC3D1S2 :1;
        unsigned LC3D1S3 :1;
        unsigned LC3D1S4 :1;
        unsigned LC3D1S5 :1;
        unsigned LC3D1S6 :1;
        unsigned LC3D1S7 :1;
    };
    struct {
        unsigned LC3D1S :8;
    };
    struct {
        unsigned D1S :8;
    };
    struct {
        unsigned D1S0 :1;
        unsigned D1S1 :1;
        unsigned D1S2 :1;
        unsigned D1S3 :1;
        unsigned D1S4 :1;
        unsigned D1S5 :1;
        unsigned D1S6 :1;
        unsigned D1S7 :1;
    };
} CLC3SEL0bits_t;
extern volatile CLC3SEL0bits_t CLC3SEL0bits __attribute__((address(0x1E26)));
# 22778 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC3SEL1 __attribute__((address(0x1E27)));

__asm("CLC3SEL1 equ 01E27h");


typedef union {
    struct {
        unsigned LC3D2S0 :1;
        unsigned LC3D2S1 :1;
        unsigned LC3D2S2 :1;
        unsigned LC3D2S3 :1;
        unsigned LC3D2S4 :1;
        unsigned LC3D2S5 :1;
        unsigned LC3D2S6 :1;
        unsigned LC3D2S7 :1;
    };
    struct {
        unsigned LC3D2S :8;
    };
    struct {
        unsigned D2S :8;
    };
    struct {
        unsigned D2S0 :1;
        unsigned D2S1 :1;
        unsigned D2S2 :1;
        unsigned D2S3 :1;
        unsigned D2S4 :1;
        unsigned D2S5 :1;
        unsigned D2S6 :1;
        unsigned D2S7 :1;
    };
} CLC3SEL1bits_t;
extern volatile CLC3SEL1bits_t CLC3SEL1bits __attribute__((address(0x1E27)));
# 22906 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC3SEL2 __attribute__((address(0x1E28)));

__asm("CLC3SEL2 equ 01E28h");


typedef union {
    struct {
        unsigned LC3D3S0 :1;
        unsigned LC3D3S1 :1;
        unsigned LC3D3S2 :1;
        unsigned LC3D3S3 :1;
        unsigned LC3D3S4 :1;
        unsigned LC3D3S5 :1;
        unsigned LC3D3S6 :1;
        unsigned LC3D3S7 :1;
    };
    struct {
        unsigned LC3D3S :8;
    };
    struct {
        unsigned D3S :8;
    };
    struct {
        unsigned D3S0 :1;
        unsigned D3S1 :1;
        unsigned D3S2 :1;
        unsigned D3S3 :1;
        unsigned D3S4 :1;
        unsigned D3S5 :1;
        unsigned D3S6 :1;
        unsigned D3S7 :1;
    };
} CLC3SEL2bits_t;
extern volatile CLC3SEL2bits_t CLC3SEL2bits __attribute__((address(0x1E28)));
# 23034 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC3SEL3 __attribute__((address(0x1E29)));

__asm("CLC3SEL3 equ 01E29h");


typedef union {
    struct {
        unsigned LC3D4S0 :1;
        unsigned LC3D4S1 :1;
        unsigned LC3D4S2 :1;
        unsigned LC3D4S3 :1;
        unsigned LC3D4S4 :1;
        unsigned LC3D4S5 :1;
        unsigned LC3D4S6 :1;
        unsigned LC3D4S7 :1;
    };
    struct {
        unsigned LC3D4S :8;
    };
    struct {
        unsigned D4S :8;
    };
    struct {
        unsigned D4S0 :1;
        unsigned D4S1 :1;
        unsigned D4S2 :1;
        unsigned D4S3 :1;
        unsigned D4S4 :1;
        unsigned D4S5 :1;
        unsigned D4S6 :1;
        unsigned D4S7 :1;
    };
} CLC3SEL3bits_t;
extern volatile CLC3SEL3bits_t CLC3SEL3bits __attribute__((address(0x1E29)));
# 23162 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC3GLS0 __attribute__((address(0x1E2A)));

__asm("CLC3GLS0 equ 01E2Ah");


typedef union {
    struct {
        unsigned LC3G1D1N :1;
        unsigned LC3G1D1T :1;
        unsigned LC3G1D2N :1;
        unsigned LC3G1D2T :1;
        unsigned LC3G1D3N :1;
        unsigned LC3G1D3T :1;
        unsigned LC3G1D4N :1;
        unsigned LC3G1D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC3GLS0bits_t;
extern volatile CLC3GLS0bits_t CLC3GLS0bits __attribute__((address(0x1E2A)));
# 23274 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC3GLS1 __attribute__((address(0x1E2B)));

__asm("CLC3GLS1 equ 01E2Bh");


typedef union {
    struct {
        unsigned LC3G2D1N :1;
        unsigned LC3G2D1T :1;
        unsigned LC3G2D2N :1;
        unsigned LC3G2D2T :1;
        unsigned LC3G2D3N :1;
        unsigned LC3G2D3T :1;
        unsigned LC3G2D4N :1;
        unsigned LC3G2D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC3GLS1bits_t;
extern volatile CLC3GLS1bits_t CLC3GLS1bits __attribute__((address(0x1E2B)));
# 23386 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC3GLS2 __attribute__((address(0x1E2C)));

__asm("CLC3GLS2 equ 01E2Ch");


typedef union {
    struct {
        unsigned LC3G3D1N :1;
        unsigned LC3G3D1T :1;
        unsigned LC3G3D2N :1;
        unsigned LC3G3D2T :1;
        unsigned LC3G3D3N :1;
        unsigned LC3G3D3T :1;
        unsigned LC3G3D4N :1;
        unsigned LC3G3D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC3GLS2bits_t;
extern volatile CLC3GLS2bits_t CLC3GLS2bits __attribute__((address(0x1E2C)));
# 23498 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC3GLS3 __attribute__((address(0x1E2D)));

__asm("CLC3GLS3 equ 01E2Dh");


typedef union {
    struct {
        unsigned LC3G4D1N :1;
        unsigned LC3G4D1T :1;
        unsigned LC3G4D2N :1;
        unsigned LC3G4D2T :1;
        unsigned LC3G4D3N :1;
        unsigned LC3G4D3T :1;
        unsigned LC3G4D4N :1;
        unsigned LC3G4D4T :1;
    };
    struct {
        unsigned G4D1N :1;
        unsigned G4D1T :1;
        unsigned G4D2N :1;
        unsigned G4D2T :1;
        unsigned G4D3N :1;
        unsigned G4D3T :1;
        unsigned G4D4N :1;
        unsigned G4D4T :1;
    };
} CLC3GLS3bits_t;
extern volatile CLC3GLS3bits_t CLC3GLS3bits __attribute__((address(0x1E2D)));
# 23610 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC4CON __attribute__((address(0x1E2E)));

__asm("CLC4CON equ 01E2Eh");


typedef union {
    struct {
        unsigned LC4MODE :3;
        unsigned LC4INTN :1;
        unsigned LC4INTP :1;
        unsigned LC4OUT :1;
        unsigned :1;
        unsigned LC4EN :1;
    };
    struct {
        unsigned LC4MODE0 :1;
        unsigned LC4MODE1 :1;
        unsigned LC4MODE2 :1;
    };
    struct {
        unsigned MODE :3;
        unsigned INTN :1;
        unsigned INTP :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
    };
} CLC4CONbits_t;
extern volatile CLC4CONbits_t CLC4CONbits __attribute__((address(0x1E2E)));
# 23728 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC4POL __attribute__((address(0x1E2F)));

__asm("CLC4POL equ 01E2Fh");


typedef union {
    struct {
        unsigned LC4G1POL :1;
        unsigned LC4G2POL :1;
        unsigned LC4G3POL :1;
        unsigned LC4G4POL :1;
        unsigned :3;
        unsigned LC4POL :1;
    };
    struct {
        unsigned G1POL :1;
        unsigned G2POL :1;
        unsigned G3POL :1;
        unsigned G4POL :1;
        unsigned :3;
        unsigned POL :1;
    };
} CLC4POLbits_t;
extern volatile CLC4POLbits_t CLC4POLbits __attribute__((address(0x1E2F)));
# 23806 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC4SEL0 __attribute__((address(0x1E30)));

__asm("CLC4SEL0 equ 01E30h");


typedef union {
    struct {
        unsigned LC4D1S0 :1;
        unsigned LC4D1S1 :1;
        unsigned LC4D1S2 :1;
        unsigned LC4D1S3 :1;
        unsigned LC4D1S4 :1;
        unsigned LC4D1S5 :1;
        unsigned LC4D1S6 :1;
        unsigned LC4D1S7 :1;
    };
    struct {
        unsigned LC4D1S :8;
    };
    struct {
        unsigned D1S :8;
    };
    struct {
        unsigned D1S0 :1;
        unsigned D1S1 :1;
        unsigned D1S2 :1;
        unsigned D1S3 :1;
        unsigned D1S4 :1;
        unsigned D1S5 :1;
        unsigned D1S6 :1;
        unsigned D1S7 :1;
    };
} CLC4SEL0bits_t;
extern volatile CLC4SEL0bits_t CLC4SEL0bits __attribute__((address(0x1E30)));
# 23934 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC4SEL1 __attribute__((address(0x1E31)));

__asm("CLC4SEL1 equ 01E31h");


typedef union {
    struct {
        unsigned LC4D2S0 :1;
        unsigned LC4D2S1 :1;
        unsigned LC4D2S2 :1;
        unsigned LC4D2S3 :1;
        unsigned LC4D2S4 :1;
        unsigned LC4D2S5 :1;
        unsigned LC4D2S6 :1;
        unsigned LC4D2S7 :1;
    };
    struct {
        unsigned LC4D2S :8;
    };
    struct {
        unsigned D2S :8;
    };
    struct {
        unsigned D2S0 :1;
        unsigned D2S1 :1;
        unsigned D2S2 :1;
        unsigned D2S3 :1;
        unsigned D2S4 :1;
        unsigned D2S5 :1;
        unsigned D2S6 :1;
        unsigned D2S7 :1;
    };
} CLC4SEL1bits_t;
extern volatile CLC4SEL1bits_t CLC4SEL1bits __attribute__((address(0x1E31)));
# 24062 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC4SEL2 __attribute__((address(0x1E32)));

__asm("CLC4SEL2 equ 01E32h");


typedef union {
    struct {
        unsigned LC4D3S0 :1;
        unsigned LC4D3S1 :1;
        unsigned LC4D3S2 :1;
        unsigned LC4D3S3 :1;
        unsigned LC4D3S4 :1;
        unsigned LC4D3S5 :1;
        unsigned LC4D3S6 :1;
        unsigned LC4D3S7 :1;
    };
    struct {
        unsigned LC4D3S :8;
    };
    struct {
        unsigned D3S :8;
    };
    struct {
        unsigned D3S0 :1;
        unsigned D3S1 :1;
        unsigned D3S2 :1;
        unsigned D3S3 :1;
        unsigned D3S4 :1;
        unsigned D3S5 :1;
        unsigned D3S6 :1;
        unsigned D3S7 :1;
    };
} CLC4SEL2bits_t;
extern volatile CLC4SEL2bits_t CLC4SEL2bits __attribute__((address(0x1E32)));
# 24190 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC4SEL3 __attribute__((address(0x1E33)));

__asm("CLC4SEL3 equ 01E33h");


typedef union {
    struct {
        unsigned LC4D4S0 :1;
        unsigned LC4D4S1 :1;
        unsigned LC4D4S2 :1;
        unsigned LC4D4S3 :1;
        unsigned LC4D4S4 :1;
        unsigned LC4D4S5 :1;
        unsigned LC4D4S6 :1;
        unsigned LC4D4S7 :1;
    };
    struct {
        unsigned LC4D4S :8;
    };
    struct {
        unsigned D4S :8;
    };
    struct {
        unsigned D4S0 :1;
        unsigned D4S1 :1;
        unsigned D4S2 :1;
        unsigned D4S3 :1;
        unsigned D4S4 :1;
        unsigned D4S5 :1;
        unsigned D4S6 :1;
        unsigned D4S7 :1;
    };
} CLC4SEL3bits_t;
extern volatile CLC4SEL3bits_t CLC4SEL3bits __attribute__((address(0x1E33)));
# 24318 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC4GLS0 __attribute__((address(0x1E34)));

__asm("CLC4GLS0 equ 01E34h");


typedef union {
    struct {
        unsigned LC4G1D1N :1;
        unsigned LC4G1D1T :1;
        unsigned LC4G1D2N :1;
        unsigned LC4G1D2T :1;
        unsigned LC4G1D3N :1;
        unsigned LC4G1D3T :1;
        unsigned LC4G1D4N :1;
        unsigned LC4G1D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC4GLS0bits_t;
extern volatile CLC4GLS0bits_t CLC4GLS0bits __attribute__((address(0x1E34)));
# 24430 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC4GLS1 __attribute__((address(0x1E35)));

__asm("CLC4GLS1 equ 01E35h");


typedef union {
    struct {
        unsigned LC4G2D1N :1;
        unsigned LC4G2D1T :1;
        unsigned LC4G2D2N :1;
        unsigned LC4G2D2T :1;
        unsigned LC4G2D3N :1;
        unsigned LC4G2D3T :1;
        unsigned LC4G2D4N :1;
        unsigned LC4G2D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC4GLS1bits_t;
extern volatile CLC4GLS1bits_t CLC4GLS1bits __attribute__((address(0x1E35)));
# 24542 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC4GLS2 __attribute__((address(0x1E36)));

__asm("CLC4GLS2 equ 01E36h");


typedef union {
    struct {
        unsigned LC4G3D1N :1;
        unsigned LC4G3D1T :1;
        unsigned LC4G3D2N :1;
        unsigned LC4G3D2T :1;
        unsigned LC4G3D3N :1;
        unsigned LC4G3D3T :1;
        unsigned LC4G3D4N :1;
        unsigned LC4G3D4T :1;
    };
    struct {
        unsigned D1N :1;
        unsigned D1T :1;
        unsigned D2N :1;
        unsigned D2T :1;
        unsigned D3N :1;
        unsigned D3T :1;
        unsigned D4N :1;
        unsigned D4T :1;
    };
} CLC4GLS2bits_t;
extern volatile CLC4GLS2bits_t CLC4GLS2bits __attribute__((address(0x1E36)));
# 24654 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLC4GLS3 __attribute__((address(0x1E37)));

__asm("CLC4GLS3 equ 01E37h");


typedef union {
    struct {
        unsigned LC4G4D1N :1;
        unsigned LC4G4D1T :1;
        unsigned LC4G4D2N :1;
        unsigned LC4G4D2T :1;
        unsigned LC4G4D3N :1;
        unsigned LC4G4D3T :1;
        unsigned LC4G4D4N :1;
        unsigned LC4G4D4T :1;
    };
    struct {
        unsigned G4D1N :1;
        unsigned G4D1T :1;
        unsigned G4D2N :1;
        unsigned G4D2T :1;
        unsigned G4D3N :1;
        unsigned G4D3T :1;
        unsigned G4D4N :1;
        unsigned G4D4T :1;
    };
} CLC4GLS3bits_t;
extern volatile CLC4GLS3bits_t CLC4GLS3bits __attribute__((address(0x1E37)));
# 24766 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PPSLOCK __attribute__((address(0x1E8F)));

__asm("PPSLOCK equ 01E8Fh");


typedef union {
    struct {
        unsigned PPSLOCKED :1;
    };
} PPSLOCKbits_t;
extern volatile PPSLOCKbits_t PPSLOCKbits __attribute__((address(0x1E8F)));
# 24786 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char INTPPS __attribute__((address(0x1E90)));

__asm("INTPPS equ 01E90h");


typedef union {
    struct {
        unsigned INTPPS :4;
    };
    struct {
        unsigned INTPPS0 :1;
        unsigned INTPPS1 :1;
        unsigned INTPPS2 :1;
        unsigned INTPPS3 :1;
    };
} INTPPSbits_t;
extern volatile INTPPSbits_t INTPPSbits __attribute__((address(0x1E90)));
# 24832 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T0CKIPPS __attribute__((address(0x1E91)));

__asm("T0CKIPPS equ 01E91h");


typedef union {
    struct {
        unsigned T0CKIPPS :4;
    };
    struct {
        unsigned T0CKIPPS0 :1;
        unsigned T0CKIPPS1 :1;
        unsigned T0CKIPPS2 :1;
        unsigned T0CKIPPS3 :1;
    };
} T0CKIPPSbits_t;
extern volatile T0CKIPPSbits_t T0CKIPPSbits __attribute__((address(0x1E91)));
# 24878 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T1CKIPPS __attribute__((address(0x1E92)));

__asm("T1CKIPPS equ 01E92h");


typedef union {
    struct {
        unsigned T1CKIPPS :5;
    };
    struct {
        unsigned T1CKIPPS0 :1;
        unsigned T1CKIPPS1 :1;
        unsigned T1CKIPPS2 :1;
        unsigned T1CKIPPS3 :1;
        unsigned T1CKIPPS4 :1;
    };
} T1CKIPPSbits_t;
extern volatile T1CKIPPSbits_t T1CKIPPSbits __attribute__((address(0x1E92)));
# 24930 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T1GPPS __attribute__((address(0x1E93)));

__asm("T1GPPS equ 01E93h");


typedef union {
    struct {
        unsigned T1GPPS :5;
    };
    struct {
        unsigned T1GPPS0 :1;
        unsigned T1GPPS1 :1;
        unsigned T1GPPS2 :1;
        unsigned T1GPPS3 :1;
        unsigned T1GPPS4 :1;
    };
} T1GPPSbits_t;
extern volatile T1GPPSbits_t T1GPPSbits __attribute__((address(0x1E93)));
# 24982 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T2INPPS __attribute__((address(0x1E9C)));

__asm("T2INPPS equ 01E9Ch");


typedef union {
    struct {
        unsigned T2INPPS :5;
    };
    struct {
        unsigned T2INPPS0 :1;
        unsigned T2INPPS1 :1;
        unsigned T2INPPS2 :1;
        unsigned T2INPPS3 :1;
        unsigned T2INPPS4 :1;
    };
} T2INPPSbits_t;
extern volatile T2INPPSbits_t T2INPPSbits __attribute__((address(0x1E9C)));
# 25034 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char T4INPPS __attribute__((address(0x1E9D)));

__asm("T4INPPS equ 01E9Dh");


typedef union {
    struct {
        unsigned T4INPPS :5;
    };
    struct {
        unsigned T4INPPS0 :1;
        unsigned T4INPPS1 :1;
        unsigned T4INPPS2 :1;
        unsigned T4INPPS3 :1;
        unsigned T4INPPS4 :1;
    };
} T4INPPSbits_t;
extern volatile T4INPPSbits_t T4INPPSbits __attribute__((address(0x1E9D)));
# 25086 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CCP1PPS __attribute__((address(0x1EA1)));

__asm("CCP1PPS equ 01EA1h");


typedef union {
    struct {
        unsigned CCP1PPS :5;
    };
    struct {
        unsigned CCP1PPS0 :1;
        unsigned CCP1PPS1 :1;
        unsigned CCP1PPS2 :1;
        unsigned CCP1PPS3 :1;
        unsigned CCP1PPS4 :1;
    };
} CCP1PPSbits_t;
extern volatile CCP1PPSbits_t CCP1PPSbits __attribute__((address(0x1EA1)));
# 25138 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CCP2PPS __attribute__((address(0x1EA2)));

__asm("CCP2PPS equ 01EA2h");


typedef union {
    struct {
        unsigned CCP2PPS :5;
    };
    struct {
        unsigned CCP2PPS0 :1;
        unsigned CCP2PPS1 :1;
        unsigned CCP2PPS2 :1;
        unsigned CCP2PPS3 :1;
        unsigned CCP2PPS4 :1;
    };
} CCP2PPSbits_t;
extern volatile CCP2PPSbits_t CCP2PPSbits __attribute__((address(0x1EA2)));
# 25190 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1WINPPS __attribute__((address(0x1EA9)));

__asm("SMT1WINPPS equ 01EA9h");


typedef union {
    struct {
        unsigned SMT1WINPPS :5;
    };
    struct {
        unsigned SMU1WINPPS0 :1;
        unsigned SMU1WINPPS1 :1;
        unsigned SMU1WINPPS2 :1;
        unsigned SMU1WINPPS3 :1;
        unsigned SMU1WINPPS4 :1;
    };
} SMT1WINPPSbits_t;
extern volatile SMT1WINPPSbits_t SMT1WINPPSbits __attribute__((address(0x1EA9)));
# 25242 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SMT1SIGPPS __attribute__((address(0x1EAA)));

__asm("SMT1SIGPPS equ 01EAAh");


typedef union {
    struct {
        unsigned SMT1SIGPPS :5;
    };
    struct {
        unsigned SMU1SIGPPS0 :1;
        unsigned SMU1SIGPPS1 :1;
        unsigned SMU1SIGPPS2 :1;
        unsigned SMU1SIGPPS3 :1;
        unsigned SMU1SIGPPS4 :1;
    };
} SMT1SIGPPSbits_t;
extern volatile SMT1SIGPPSbits_t SMT1SIGPPSbits __attribute__((address(0x1EAA)));
# 25294 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CWG1PPS __attribute__((address(0x1EB1)));

__asm("CWG1PPS equ 01EB1h");


typedef union {
    struct {
        unsigned CWG1PPS :5;
    };
    struct {
        unsigned CWG1PPS0 :1;
        unsigned CWG1PPS1 :1;
        unsigned CWG1PPS2 :1;
        unsigned CWG1PPS3 :1;
        unsigned CWG1PPS4 :1;
    };
} CWG1PPSbits_t;
extern volatile CWG1PPSbits_t CWG1PPSbits __attribute__((address(0x1EB1)));
# 25346 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLCIN0PPS __attribute__((address(0x1EBB)));

__asm("CLCIN0PPS equ 01EBBh");


typedef union {
    struct {
        unsigned CLCIN0PPS :5;
    };
    struct {
        unsigned CLCIN0PPS0 :1;
        unsigned CLCIN0PPS1 :1;
        unsigned CLCIN0PPS2 :1;
        unsigned CLCIN0PPS3 :1;
        unsigned CLCIN0PPS4 :1;
    };
} CLCIN0PPSbits_t;
extern volatile CLCIN0PPSbits_t CLCIN0PPSbits __attribute__((address(0x1EBB)));
# 25398 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLCIN1PPS __attribute__((address(0x1EBC)));

__asm("CLCIN1PPS equ 01EBCh");


typedef union {
    struct {
        unsigned CLCIN1PPS :5;
    };
    struct {
        unsigned CLCIN1PPS0 :1;
        unsigned CLCIN1PPS1 :1;
        unsigned CLCIN1PPS2 :1;
        unsigned CLCIN1PPS3 :1;
        unsigned CLCIN1PPS4 :1;
    };
} CLCIN1PPSbits_t;
extern volatile CLCIN1PPSbits_t CLCIN1PPSbits __attribute__((address(0x1EBC)));
# 25450 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLCIN2PPS __attribute__((address(0x1EBD)));

__asm("CLCIN2PPS equ 01EBDh");


typedef union {
    struct {
        unsigned CLCIN2PPS :5;
    };
    struct {
        unsigned CLCIN2PPS0 :1;
        unsigned CLCIN2PPS1 :1;
        unsigned CLCIN2PPS2 :1;
        unsigned CLCIN2PPS3 :1;
        unsigned CLCIN2PPS4 :1;
    };
} CLCIN2PPSbits_t;
extern volatile CLCIN2PPSbits_t CLCIN2PPSbits __attribute__((address(0x1EBD)));
# 25502 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char CLCIN3PPS __attribute__((address(0x1EBE)));

__asm("CLCIN3PPS equ 01EBEh");


typedef union {
    struct {
        unsigned CLCIN3PPS :5;
    };
    struct {
        unsigned CLCIN3PPS0 :1;
        unsigned CLCIN3PPS1 :1;
        unsigned CLCIN3PPS2 :1;
        unsigned CLCIN3PPS3 :1;
        unsigned CLCIN3PPS4 :1;
    };
} CLCIN3PPSbits_t;
extern volatile CLCIN3PPSbits_t CLCIN3PPSbits __attribute__((address(0x1EBE)));
# 25554 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ADCACTPPS __attribute__((address(0x1EC3)));

__asm("ADCACTPPS equ 01EC3h");


typedef union {
    struct {
        unsigned ADCACTPPS :5;
    };
    struct {
        unsigned ADCACTPPS0 :1;
        unsigned ADCACTPPS1 :1;
        unsigned ADCACTPPS2 :1;
        unsigned ADCACTPPS3 :1;
        unsigned ADCACTPPS4 :1;
    };
} ADCACTPPSbits_t;
extern volatile ADCACTPPSbits_t ADCACTPPSbits __attribute__((address(0x1EC3)));
# 25606 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SSP1CLKPPS __attribute__((address(0x1EC5)));

__asm("SSP1CLKPPS equ 01EC5h");


typedef union {
    struct {
        unsigned SSP1CLKPPS :5;
    };
    struct {
        unsigned SSP1CLKPPS0 :1;
        unsigned SSP1CLKPPS1 :1;
        unsigned SSP1CLKPPS2 :1;
        unsigned SSP1CLKPPS3 :1;
        unsigned SSP1CLKPPS4 :1;
    };
} SSP1CLKPPSbits_t;
extern volatile SSP1CLKPPSbits_t SSP1CLKPPSbits __attribute__((address(0x1EC5)));
# 25658 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SSP1DATPPS __attribute__((address(0x1EC6)));

__asm("SSP1DATPPS equ 01EC6h");


typedef union {
    struct {
        unsigned SSP1DATPPS :5;
    };
    struct {
        unsigned SSP1DATPPS0 :1;
        unsigned SSP1DATPPS1 :1;
        unsigned SSP1DATPPS2 :1;
        unsigned SSP1DATPPS3 :1;
        unsigned SSP1DATPPS4 :1;
    };
} SSP1DATPPSbits_t;
extern volatile SSP1DATPPSbits_t SSP1DATPPSbits __attribute__((address(0x1EC6)));
# 25710 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SSP1SSPPS __attribute__((address(0x1EC7)));

__asm("SSP1SSPPS equ 01EC7h");


typedef union {
    struct {
        unsigned SSP1SSPPS :5;
    };
    struct {
        unsigned SSP1SSPPS0 :1;
        unsigned SSP1SSPPS1 :1;
        unsigned SSP1SSPPS2 :1;
        unsigned SSP1SSPPS3 :1;
        unsigned SSP1SSPPS4 :1;
    };
} SSP1SSPPSbits_t;
extern volatile SSP1SSPPSbits_t SSP1SSPPSbits __attribute__((address(0x1EC7)));
# 25762 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RX1PPS __attribute__((address(0x1ECB)));

__asm("RX1PPS equ 01ECBh");


typedef union {
    struct {
        unsigned RX1PPS :5;
    };
    struct {
        unsigned RX1PPS0 :1;
        unsigned RX1PPS1 :1;
        unsigned RX1PPS2 :1;
        unsigned RX1PPS3 :1;
        unsigned RX1PPS4 :1;
    };
} RX1PPSbits_t;
extern volatile RX1PPSbits_t RX1PPSbits __attribute__((address(0x1ECB)));
# 25814 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TX1PPS __attribute__((address(0x1ECC)));

__asm("TX1PPS equ 01ECCh");


typedef union {
    struct {
        unsigned TX1PPS :5;
    };
    struct {
        unsigned TX1PPS0 :1;
        unsigned TX1PPS1 :1;
        unsigned TX1PPS2 :1;
        unsigned TX1PPS3 :1;
        unsigned TX1PPS4 :1;
    };
} TX1PPSbits_t;
extern volatile TX1PPSbits_t TX1PPSbits __attribute__((address(0x1ECC)));
# 25866 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RX2PPS __attribute__((address(0x1ECD)));

__asm("RX2PPS equ 01ECDh");


typedef union {
    struct {
        unsigned RX2PPS :5;
    };
    struct {
        unsigned RX2PPS0 :1;
        unsigned RX2PPS1 :1;
        unsigned RX3PPS2 :1;
        unsigned RX4PPS3 :1;
        unsigned RX5PPS4 :1;
    };
} RX2PPSbits_t;
extern volatile RX2PPSbits_t RX2PPSbits __attribute__((address(0x1ECD)));
# 25918 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TX2PPS __attribute__((address(0x1ECE)));

__asm("TX2PPS equ 01ECEh");


typedef union {
    struct {
        unsigned TX2PPS :5;
    };
    struct {
        unsigned TX2PPS0 :1;
        unsigned TX2PPS1 :1;
        unsigned TX2PPS2 :1;
        unsigned TX2PPS3 :1;
        unsigned TX2PPS4 :1;
    };
} TX2PPSbits_t;
extern volatile TX2PPSbits_t TX2PPSbits __attribute__((address(0x1ECE)));
# 25970 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RA0PPS __attribute__((address(0x1F10)));

__asm("RA0PPS equ 01F10h");


typedef union {
    struct {
        unsigned RA0PPS0 :1;
        unsigned RA0PPS1 :1;
        unsigned RA0PPS2 :1;
        unsigned RA0PPS3 :1;
        unsigned RA0PPS4 :1;
        unsigned RA0PPS5 :1;
    };
} RA0PPSbits_t;
extern volatile RA0PPSbits_t RA0PPSbits __attribute__((address(0x1F10)));
# 26020 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RA1PPS __attribute__((address(0x1F11)));

__asm("RA1PPS equ 01F11h");


typedef union {
    struct {
        unsigned RA1PPS0 :1;
        unsigned RA1PPS1 :1;
        unsigned RA1PPS2 :1;
        unsigned RA1PPS3 :1;
        unsigned RA1PPS4 :1;
        unsigned RA1PPS5 :1;
    };
} RA1PPSbits_t;
extern volatile RA1PPSbits_t RA1PPSbits __attribute__((address(0x1F11)));
# 26070 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RA2PPS __attribute__((address(0x1F12)));

__asm("RA2PPS equ 01F12h");


typedef union {
    struct {
        unsigned RA2PPS0 :1;
        unsigned RA2PPS1 :1;
        unsigned RA2PPS2 :1;
        unsigned RA2PPS3 :1;
        unsigned RA2PPS4 :1;
        unsigned RA2PPS5 :1;
    };
} RA2PPSbits_t;
extern volatile RA2PPSbits_t RA2PPSbits __attribute__((address(0x1F12)));
# 26120 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RA3PPS __attribute__((address(0x1F13)));

__asm("RA3PPS equ 01F13h");


typedef union {
    struct {
        unsigned RA3PPS0 :1;
        unsigned RA3PPS1 :1;
        unsigned RA3PPS2 :1;
        unsigned RA3PPS3 :1;
        unsigned RA3PPS4 :1;
        unsigned RA3PPS5 :1;
    };
} RA3PPSbits_t;
extern volatile RA3PPSbits_t RA3PPSbits __attribute__((address(0x1F13)));
# 26170 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RA4PPS __attribute__((address(0x1F14)));

__asm("RA4PPS equ 01F14h");


typedef union {
    struct {
        unsigned RA4PPS0 :1;
        unsigned RA4PPS1 :1;
        unsigned RA4PPS2 :1;
        unsigned RA4PPS3 :1;
        unsigned RA4PPS4 :1;
        unsigned RA4PPS5 :1;
    };
} RA4PPSbits_t;
extern volatile RA4PPSbits_t RA4PPSbits __attribute__((address(0x1F14)));
# 26220 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RA5PPS __attribute__((address(0x1F15)));

__asm("RA5PPS equ 01F15h");


typedef union {
    struct {
        unsigned RA5PPS0 :1;
        unsigned RA5PPS1 :1;
        unsigned RA5PPS2 :1;
        unsigned RA5PPS3 :1;
        unsigned RA5PPS4 :1;
        unsigned RA5PPS5 :1;
    };
} RA5PPSbits_t;
extern volatile RA5PPSbits_t RA5PPSbits __attribute__((address(0x1F15)));
# 26270 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RA6PPS __attribute__((address(0x1F16)));

__asm("RA6PPS equ 01F16h");


typedef union {
    struct {
        unsigned RA6PPS0 :1;
        unsigned RA6PPS1 :1;
        unsigned RA6PPS2 :1;
        unsigned RA6PPS3 :1;
        unsigned RA6PPS4 :1;
        unsigned RA6PPS5 :1;
    };
} RA6PPSbits_t;
extern volatile RA6PPSbits_t RA6PPSbits __attribute__((address(0x1F16)));
# 26320 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RA7PPS __attribute__((address(0x1F17)));

__asm("RA7PPS equ 01F17h");


typedef union {
    struct {
        unsigned RA7PPS0 :1;
        unsigned RA7PPS1 :1;
        unsigned RA7PPS2 :1;
        unsigned RA7PPS3 :1;
        unsigned RA7PPS4 :1;
        unsigned RA7PPS5 :1;
    };
} RA7PPSbits_t;
extern volatile RA7PPSbits_t RA7PPSbits __attribute__((address(0x1F17)));
# 26370 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RB0PPS __attribute__((address(0x1F18)));

__asm("RB0PPS equ 01F18h");


typedef union {
    struct {
        unsigned RB0PPS0 :1;
        unsigned RB0PPS1 :1;
        unsigned RB0PPS2 :1;
        unsigned RB0PPS3 :1;
        unsigned RB0PPS4 :1;
        unsigned RB0PPS5 :1;
    };
} RB0PPSbits_t;
extern volatile RB0PPSbits_t RB0PPSbits __attribute__((address(0x1F18)));
# 26420 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RB1PPS __attribute__((address(0x1F19)));

__asm("RB1PPS equ 01F19h");


typedef union {
    struct {
        unsigned RB1PPS0 :1;
        unsigned RB1PPS1 :1;
        unsigned RB1PPS2 :1;
        unsigned RB1PPS3 :1;
        unsigned RB1PPS4 :1;
        unsigned RB1PPS5 :1;
    };
} RB1PPSbits_t;
extern volatile RB1PPSbits_t RB1PPSbits __attribute__((address(0x1F19)));
# 26470 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RB2PPS __attribute__((address(0x1F1A)));

__asm("RB2PPS equ 01F1Ah");


typedef union {
    struct {
        unsigned RB2PPS0 :1;
        unsigned RB2PPS1 :1;
        unsigned RB2PPS2 :1;
        unsigned RB2PPS3 :1;
        unsigned RB2PPS4 :1;
        unsigned RB2PPS5 :1;
    };
} RB2PPSbits_t;
extern volatile RB2PPSbits_t RB2PPSbits __attribute__((address(0x1F1A)));
# 26520 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RB3PPS __attribute__((address(0x1F1B)));

__asm("RB3PPS equ 01F1Bh");


typedef union {
    struct {
        unsigned RB3PPS0 :1;
        unsigned RB3PPS1 :1;
        unsigned RB3PPS2 :1;
        unsigned RB3PPS3 :1;
        unsigned RB3PPS4 :1;
        unsigned RB3PPS5 :1;
    };
} RB3PPSbits_t;
extern volatile RB3PPSbits_t RB3PPSbits __attribute__((address(0x1F1B)));
# 26570 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RB4PPS __attribute__((address(0x1F1C)));

__asm("RB4PPS equ 01F1Ch");


typedef union {
    struct {
        unsigned RB4PPS0 :1;
        unsigned RB4PPS1 :1;
        unsigned RB4PPS2 :1;
        unsigned RB4PPS3 :1;
        unsigned RB4PPS4 :1;
        unsigned RB4PPS5 :1;
    };
} RB4PPSbits_t;
extern volatile RB4PPSbits_t RB4PPSbits __attribute__((address(0x1F1C)));
# 26620 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RB5PPS __attribute__((address(0x1F1D)));

__asm("RB5PPS equ 01F1Dh");


typedef union {
    struct {
        unsigned RB5PPS0 :1;
        unsigned RB5PPS1 :1;
        unsigned RB5PPS2 :1;
        unsigned RB5PPS3 :1;
        unsigned RB5PPS4 :1;
        unsigned RB5PPS5 :1;
    };
} RB5PPSbits_t;
extern volatile RB5PPSbits_t RB5PPSbits __attribute__((address(0x1F1D)));
# 26670 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RB6PPS __attribute__((address(0x1F1E)));

__asm("RB6PPS equ 01F1Eh");


typedef union {
    struct {
        unsigned RB6PPS0 :1;
        unsigned RB6PPS1 :1;
        unsigned RB6PPS2 :1;
        unsigned RB6PPS3 :1;
        unsigned RB6PPS4 :1;
        unsigned RB6PPS5 :1;
    };
} RB6PPSbits_t;
extern volatile RB6PPSbits_t RB6PPSbits __attribute__((address(0x1F1E)));
# 26720 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RB7PPS __attribute__((address(0x1F1F)));

__asm("RB7PPS equ 01F1Fh");


typedef union {
    struct {
        unsigned RB7PPS0 :1;
        unsigned RB7PPS1 :1;
        unsigned RB7PPS2 :1;
        unsigned RB7PPS3 :1;
        unsigned RB7PPS4 :1;
        unsigned RB7PPS5 :1;
    };
} RB7PPSbits_t;
extern volatile RB7PPSbits_t RB7PPSbits __attribute__((address(0x1F1F)));
# 26770 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC0PPS __attribute__((address(0x1F20)));

__asm("RC0PPS equ 01F20h");


typedef union {
    struct {
        unsigned RC0PPS0 :1;
        unsigned RC0PPS1 :1;
        unsigned RC0PPS2 :1;
        unsigned RC0PPS3 :1;
        unsigned RC0PPS4 :1;
        unsigned RC0PPS5 :1;
    };
} RC0PPSbits_t;
extern volatile RC0PPSbits_t RC0PPSbits __attribute__((address(0x1F20)));
# 26820 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC1PPS __attribute__((address(0x1F21)));

__asm("RC1PPS equ 01F21h");


typedef union {
    struct {
        unsigned RC1PPS0 :1;
        unsigned RC1PPS1 :1;
        unsigned RC1PPS2 :1;
        unsigned RC1PPS3 :1;
        unsigned RC1PPS4 :1;
        unsigned RC1PPS5 :1;
    };
} RC1PPSbits_t;
extern volatile RC1PPSbits_t RC1PPSbits __attribute__((address(0x1F21)));
# 26870 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC2PPS __attribute__((address(0x1F22)));

__asm("RC2PPS equ 01F22h");


typedef union {
    struct {
        unsigned RC2PPS0 :1;
        unsigned RC2PPS1 :1;
        unsigned RC2PPS2 :1;
        unsigned RC2PPS3 :1;
        unsigned RC2PPS4 :1;
        unsigned RC2PPS5 :1;
    };
} RC2PPSbits_t;
extern volatile RC2PPSbits_t RC2PPSbits __attribute__((address(0x1F22)));
# 26920 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC3PPS __attribute__((address(0x1F23)));

__asm("RC3PPS equ 01F23h");


typedef union {
    struct {
        unsigned RC3PPS0 :1;
        unsigned RC3PPS1 :1;
        unsigned RC3PPS2 :1;
        unsigned RC3PPS3 :1;
        unsigned RC3PPS4 :1;
        unsigned RC3PPS5 :1;
    };
} RC3PPSbits_t;
extern volatile RC3PPSbits_t RC3PPSbits __attribute__((address(0x1F23)));
# 26970 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC4PPS __attribute__((address(0x1F24)));

__asm("RC4PPS equ 01F24h");


typedef union {
    struct {
        unsigned RC4PPS0 :1;
        unsigned RC4PPS1 :1;
        unsigned RC4PPS2 :1;
        unsigned RC4PPS3 :1;
        unsigned RC4PPS4 :1;
        unsigned RC4PPS5 :1;
    };
} RC4PPSbits_t;
extern volatile RC4PPSbits_t RC4PPSbits __attribute__((address(0x1F24)));
# 27020 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC6PPS __attribute__((address(0x1F26)));

__asm("RC6PPS equ 01F26h");


typedef union {
    struct {
        unsigned RC6PPS0 :1;
        unsigned RC6PPS1 :1;
        unsigned RC6PPS2 :1;
        unsigned RC6PPS3 :1;
        unsigned RC6PPS4 :1;
        unsigned RC6PPS5 :1;
    };
} RC6PPSbits_t;
extern volatile RC6PPSbits_t RC6PPSbits __attribute__((address(0x1F26)));
# 27070 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char RC7PPS __attribute__((address(0x1F27)));

__asm("RC7PPS equ 01F27h");


typedef union {
    struct {
        unsigned RC7PPS0 :1;
        unsigned RC7PPS1 :1;
        unsigned RC7PPS2 :1;
        unsigned RC7PPS3 :1;
        unsigned RC7PPS4 :1;
        unsigned RC7PPS5 :1;
    };
} RC7PPSbits_t;
extern volatile RC7PPSbits_t RC7PPSbits __attribute__((address(0x1F27)));
# 27120 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ANSELA __attribute__((address(0x1F38)));

__asm("ANSELA equ 01F38h");


typedef union {
    struct {
        unsigned ANSA0 :1;
        unsigned ANSA1 :1;
        unsigned ANSA2 :1;
        unsigned ANSA3 :1;
        unsigned ANSA4 :1;
        unsigned :1;
        unsigned ANSA6 :1;
        unsigned ANSA7 :1;
    };
} ANSELAbits_t;
extern volatile ANSELAbits_t ANSELAbits __attribute__((address(0x1F38)));
# 27177 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WPUA __attribute__((address(0x1F39)));

__asm("WPUA equ 01F39h");


typedef union {
    struct {
        unsigned WPUA0 :1;
        unsigned WPUA1 :1;
        unsigned WPUA2 :1;
        unsigned WPUA3 :1;
        unsigned WPUA4 :1;
        unsigned WPUA5 :1;
        unsigned WPUA6 :1;
        unsigned WPUA7 :1;
    };
} WPUAbits_t;
extern volatile WPUAbits_t WPUAbits __attribute__((address(0x1F39)));
# 27239 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ODCONA __attribute__((address(0x1F3A)));

__asm("ODCONA equ 01F3Ah");


typedef union {
    struct {
        unsigned ODCA0 :1;
        unsigned ODCA1 :1;
        unsigned ODCA2 :1;
        unsigned ODCA3 :1;
        unsigned ODCA4 :1;
        unsigned :1;
        unsigned ODCA6 :1;
        unsigned ODCA7 :1;
    };
} ODCONAbits_t;
extern volatile ODCONAbits_t ODCONAbits __attribute__((address(0x1F3A)));
# 27296 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SLRCONA __attribute__((address(0x1F3B)));

__asm("SLRCONA equ 01F3Bh");


typedef union {
    struct {
        unsigned SLRA0 :1;
        unsigned SLRA1 :1;
        unsigned SLRA2 :1;
        unsigned SLRA3 :1;
        unsigned SLRA4 :1;
        unsigned :1;
        unsigned SLRA6 :1;
        unsigned SLRA7 :1;
    };
} SLRCONAbits_t;
extern volatile SLRCONAbits_t SLRCONAbits __attribute__((address(0x1F3B)));
# 27353 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char INLVLA __attribute__((address(0x1F3C)));

__asm("INLVLA equ 01F3Ch");


typedef union {
    struct {
        unsigned INLVLA0 :1;
        unsigned INLVLA1 :1;
        unsigned INLVLA2 :1;
        unsigned INLVLA3 :1;
        unsigned INLVLA4 :1;
        unsigned INLVLA5 :1;
        unsigned INLVLA6 :1;
        unsigned INLVLA7 :1;
    };
} INLVLAbits_t;
extern volatile INLVLAbits_t INLVLAbits __attribute__((address(0x1F3C)));
# 27415 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCAP __attribute__((address(0x1F3D)));

__asm("IOCAP equ 01F3Dh");


typedef union {
    struct {
        unsigned IOCAP0 :1;
        unsigned IOCAP1 :1;
        unsigned IOCAP2 :1;
        unsigned IOCAP3 :1;
        unsigned IOCAP4 :1;
        unsigned IOCAP5 :1;
        unsigned IOCAP6 :1;
        unsigned IOCAP7 :1;
    };
} IOCAPbits_t;
extern volatile IOCAPbits_t IOCAPbits __attribute__((address(0x1F3D)));
# 27477 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCAN __attribute__((address(0x1F3E)));

__asm("IOCAN equ 01F3Eh");


typedef union {
    struct {
        unsigned IOCAN0 :1;
        unsigned IOCAN1 :1;
        unsigned IOCAN2 :1;
        unsigned IOCAN3 :1;
        unsigned IOCAN4 :1;
        unsigned IOCAN5 :1;
        unsigned IOCAN6 :1;
        unsigned IOCAN7 :1;
    };
} IOCANbits_t;
extern volatile IOCANbits_t IOCANbits __attribute__((address(0x1F3E)));
# 27539 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCAF __attribute__((address(0x1F3F)));

__asm("IOCAF equ 01F3Fh");


typedef union {
    struct {
        unsigned IOCAF0 :1;
        unsigned IOCAF1 :1;
        unsigned IOCAF2 :1;
        unsigned IOCAF3 :1;
        unsigned IOCAF4 :1;
        unsigned IOCAF5 :1;
        unsigned IOCAF6 :1;
        unsigned IOCAF7 :1;
    };
} IOCAFbits_t;
extern volatile IOCAFbits_t IOCAFbits __attribute__((address(0x1F3F)));
# 27601 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ANSELB __attribute__((address(0x1F43)));

__asm("ANSELB equ 01F43h");


typedef union {
    struct {
        unsigned ANSB0 :1;
        unsigned ANSB1 :1;
        unsigned ANSB2 :1;
        unsigned ANSB3 :1;
        unsigned ANSB4 :1;
        unsigned ANSB5 :1;
        unsigned ANSB6 :1;
        unsigned ANSB7 :1;
    };
} ANSELBbits_t;
extern volatile ANSELBbits_t ANSELBbits __attribute__((address(0x1F43)));
# 27663 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WPUB __attribute__((address(0x1F44)));

__asm("WPUB equ 01F44h");


typedef union {
    struct {
        unsigned WPUB0 :1;
        unsigned WPUB1 :1;
        unsigned WPUB2 :1;
        unsigned WPUB3 :1;
        unsigned WPUB4 :1;
        unsigned WPUB5 :1;
        unsigned WPUB6 :1;
        unsigned WPUB7 :1;
    };
} WPUBbits_t;
extern volatile WPUBbits_t WPUBbits __attribute__((address(0x1F44)));
# 27725 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ODCONB __attribute__((address(0x1F45)));

__asm("ODCONB equ 01F45h");


typedef union {
    struct {
        unsigned ODCB0 :1;
        unsigned ODCB1 :1;
        unsigned ODCB2 :1;
        unsigned ODCB3 :1;
        unsigned ODCB4 :1;
        unsigned ODCB5 :1;
        unsigned ODCB6 :1;
        unsigned ODCB7 :1;
    };
} ODCONBbits_t;
extern volatile ODCONBbits_t ODCONBbits __attribute__((address(0x1F45)));
# 27787 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SLRCONB __attribute__((address(0x1F46)));

__asm("SLRCONB equ 01F46h");


typedef union {
    struct {
        unsigned SLRB0 :1;
        unsigned SLRB1 :1;
        unsigned SLRB2 :1;
        unsigned SLRB3 :1;
        unsigned SLRB4 :1;
        unsigned SLRB5 :1;
        unsigned SLRB6 :1;
        unsigned SLRB7 :1;
    };
} SLRCONBbits_t;
extern volatile SLRCONBbits_t SLRCONBbits __attribute__((address(0x1F46)));
# 27849 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char INLVLB __attribute__((address(0x1F47)));

__asm("INLVLB equ 01F47h");


typedef union {
    struct {
        unsigned INLVLB0 :1;
        unsigned INLVLB1 :1;
        unsigned INLVLB2 :1;
        unsigned INLVLB3 :1;
        unsigned INLVLB4 :1;
        unsigned INLVLB5 :1;
        unsigned INLVLB6 :1;
        unsigned INLVLB7 :1;
    };
} INLVLBbits_t;
extern volatile INLVLBbits_t INLVLBbits __attribute__((address(0x1F47)));
# 27911 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCBP __attribute__((address(0x1F48)));

__asm("IOCBP equ 01F48h");


typedef union {
    struct {
        unsigned IOCBP0 :1;
        unsigned IOCBP1 :1;
        unsigned IOCBP2 :1;
        unsigned IOCBP3 :1;
        unsigned IOCBP4 :1;
        unsigned IOCBP5 :1;
        unsigned IOCBP6 :1;
        unsigned IOCBP7 :1;
    };
} IOCBPbits_t;
extern volatile IOCBPbits_t IOCBPbits __attribute__((address(0x1F48)));
# 27973 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCBN __attribute__((address(0x1F49)));

__asm("IOCBN equ 01F49h");


typedef union {
    struct {
        unsigned IOCBN0 :1;
        unsigned IOCBN1 :1;
        unsigned IOCBN2 :1;
        unsigned IOCBN3 :1;
        unsigned IOCBN4 :1;
        unsigned IOCBN5 :1;
        unsigned IOCBN6 :1;
        unsigned IOCBN7 :1;
    };
} IOCBNbits_t;
extern volatile IOCBNbits_t IOCBNbits __attribute__((address(0x1F49)));
# 28035 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCBF __attribute__((address(0x1F4A)));

__asm("IOCBF equ 01F4Ah");


typedef union {
    struct {
        unsigned IOCBF0 :1;
        unsigned IOCBF1 :1;
        unsigned IOCBF2 :1;
        unsigned IOCBF3 :1;
        unsigned IOCBF4 :1;
        unsigned IOCBF5 :1;
        unsigned IOCBF6 :1;
        unsigned IOCBF7 :1;
    };
} IOCBFbits_t;
extern volatile IOCBFbits_t IOCBFbits __attribute__((address(0x1F4A)));
# 28097 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ANSELC __attribute__((address(0x1F4E)));

__asm("ANSELC equ 01F4Eh");


typedef union {
    struct {
        unsigned ANSC0 :1;
        unsigned ANSC1 :1;
        unsigned ANSC2 :1;
        unsigned ANSC3 :1;
        unsigned ANSC4 :1;
        unsigned :1;
        unsigned ANSC6 :1;
        unsigned ANSC7 :1;
    };
} ANSELCbits_t;
extern volatile ANSELCbits_t ANSELCbits __attribute__((address(0x1F4E)));
# 28154 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WPUC __attribute__((address(0x1F4F)));

__asm("WPUC equ 01F4Fh");


typedef union {
    struct {
        unsigned WPUC0 :1;
        unsigned WPUC1 :1;
        unsigned WPUC2 :1;
        unsigned WPUC3 :1;
        unsigned WPUC4 :1;
        unsigned :1;
        unsigned WPUC6 :1;
        unsigned WPUC7 :1;
    };
} WPUCbits_t;
extern volatile WPUCbits_t WPUCbits __attribute__((address(0x1F4F)));
# 28211 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char ODCONC __attribute__((address(0x1F50)));

__asm("ODCONC equ 01F50h");


typedef union {
    struct {
        unsigned ODCC0 :1;
        unsigned ODCC1 :1;
        unsigned ODCC2 :1;
        unsigned ODCC3 :1;
        unsigned ODCC4 :1;
        unsigned :1;
        unsigned ODCC6 :1;
        unsigned ODCC7 :1;
    };
} ODCONCbits_t;
extern volatile ODCONCbits_t ODCONCbits __attribute__((address(0x1F50)));
# 28268 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char SLRCONC __attribute__((address(0x1F51)));

__asm("SLRCONC equ 01F51h");


typedef union {
    struct {
        unsigned SLRC0 :1;
        unsigned SLRC1 :1;
        unsigned SLRC2 :1;
        unsigned SLRC3 :1;
        unsigned SLRC4 :1;
        unsigned :1;
        unsigned SLRC6 :1;
        unsigned SLRC7 :1;
    };
} SLRCONCbits_t;
extern volatile SLRCONCbits_t SLRCONCbits __attribute__((address(0x1F51)));
# 28325 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char INLVLC __attribute__((address(0x1F52)));

__asm("INLVLC equ 01F52h");


typedef union {
    struct {
        unsigned INLVLC0 :1;
        unsigned INLVLC1 :1;
        unsigned INLVLC2 :1;
        unsigned INLVLC3 :1;
        unsigned INLVLC4 :1;
        unsigned :1;
        unsigned INLVLC6 :1;
        unsigned INLVLC7 :1;
    };
} INLVLCbits_t;
extern volatile INLVLCbits_t INLVLCbits __attribute__((address(0x1F52)));
# 28382 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCCP __attribute__((address(0x1F53)));

__asm("IOCCP equ 01F53h");


typedef union {
    struct {
        unsigned IOCCP0 :1;
        unsigned IOCCP1 :1;
        unsigned IOCCP2 :1;
        unsigned IOCCP3 :1;
        unsigned IOCCP4 :1;
        unsigned :1;
        unsigned IOCCP6 :1;
        unsigned IOCCP7 :1;
    };
} IOCCPbits_t;
extern volatile IOCCPbits_t IOCCPbits __attribute__((address(0x1F53)));
# 28439 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCCN __attribute__((address(0x1F54)));

__asm("IOCCN equ 01F54h");


typedef union {
    struct {
        unsigned IOCCN0 :1;
        unsigned IOCCN1 :1;
        unsigned IOCCN2 :1;
        unsigned IOCCN3 :1;
        unsigned IOCCN4 :1;
        unsigned :1;
        unsigned IOCCN6 :1;
        unsigned IOCCN7 :1;
    };
} IOCCNbits_t;
extern volatile IOCCNbits_t IOCCNbits __attribute__((address(0x1F54)));
# 28496 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCCF __attribute__((address(0x1F55)));

__asm("IOCCF equ 01F55h");


typedef union {
    struct {
        unsigned IOCCF0 :1;
        unsigned IOCCF1 :1;
        unsigned IOCCF2 :1;
        unsigned IOCCF3 :1;
        unsigned IOCCF4 :1;
        unsigned :1;
        unsigned IOCCF6 :1;
        unsigned IOCCF7 :1;
    };
} IOCCFbits_t;
extern volatile IOCCFbits_t IOCCFbits __attribute__((address(0x1F55)));
# 28553 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WPUE __attribute__((address(0x1F65)));

__asm("WPUE equ 01F65h");


typedef union {
    struct {
        unsigned WPUE0 :1;
        unsigned WPUE1 :1;
        unsigned WPUE2 :1;
        unsigned WPUE3 :1;
    };
} WPUEbits_t;
extern volatile WPUEbits_t WPUEbits __attribute__((address(0x1F65)));
# 28591 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char INLVLE __attribute__((address(0x1F68)));

__asm("INLVLE equ 01F68h");


typedef union {
    struct {
        unsigned :3;
        unsigned INLVLE3 :1;
    };
} INLVLEbits_t;
extern volatile INLVLEbits_t INLVLEbits __attribute__((address(0x1F68)));
# 28612 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCEP __attribute__((address(0x1F69)));

__asm("IOCEP equ 01F69h");


typedef union {
    struct {
        unsigned :3;
        unsigned IOCEP3 :1;
    };
} IOCEPbits_t;
extern volatile IOCEPbits_t IOCEPbits __attribute__((address(0x1F69)));
# 28633 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCEN __attribute__((address(0x1F6A)));

__asm("IOCEN equ 01F6Ah");


typedef union {
    struct {
        unsigned :3;
        unsigned IOCEN3 :1;
    };
} IOCENbits_t;
extern volatile IOCENbits_t IOCENbits __attribute__((address(0x1F6A)));
# 28654 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char IOCEF __attribute__((address(0x1F6B)));

__asm("IOCEF equ 01F6Bh");


typedef union {
    struct {
        unsigned :3;
        unsigned IOCEF3 :1;
    };
} IOCEFbits_t;
extern volatile IOCEFbits_t IOCEFbits __attribute__((address(0x1F6B)));
# 28675 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char STATUS_SHAD __attribute__((address(0x1FE4)));

__asm("STATUS_SHAD equ 01FE4h");


typedef union {
    struct {
        unsigned C_SHAD :1;
        unsigned DC_SHAD :1;
        unsigned Z_SHAD :1;
    };
} STATUS_SHADbits_t;
extern volatile STATUS_SHADbits_t STATUS_SHADbits __attribute__((address(0x1FE4)));
# 28707 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char WREG_SHAD __attribute__((address(0x1FE5)));

__asm("WREG_SHAD equ 01FE5h");


typedef union {
    struct {
        unsigned WREG_SHAD :8;
    };
} WREG_SHADbits_t;
extern volatile WREG_SHADbits_t WREG_SHADbits __attribute__((address(0x1FE5)));
# 28727 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char BSR_SHAD __attribute__((address(0x1FE6)));

__asm("BSR_SHAD equ 01FE6h");


typedef union {
    struct {
        unsigned BSR_SHAD :5;
    };
} BSR_SHADbits_t;
extern volatile BSR_SHADbits_t BSR_SHADbits __attribute__((address(0x1FE6)));
# 28747 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char PCLATH_SHAD __attribute__((address(0x1FE7)));

__asm("PCLATH_SHAD equ 01FE7h");


typedef union {
    struct {
        unsigned PCLATH_SHAD :7;
    };
} PCLATH_SHADbits_t;
extern volatile PCLATH_SHADbits_t PCLATH_SHADbits __attribute__((address(0x1FE7)));
# 28767 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char FSR0L_SHAD __attribute__((address(0x1FE8)));

__asm("FSR0L_SHAD equ 01FE8h");


typedef union {
    struct {
        unsigned FSR0L_SHAD :8;
    };
} FSR0L_SHADbits_t;
extern volatile FSR0L_SHADbits_t FSR0L_SHADbits __attribute__((address(0x1FE8)));
# 28787 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char FSR0H_SHAD __attribute__((address(0x1FE9)));

__asm("FSR0H_SHAD equ 01FE9h");


typedef union {
    struct {
        unsigned FSR0H_SHAD :8;
    };
} FSR0H_SHADbits_t;
extern volatile FSR0H_SHADbits_t FSR0H_SHADbits __attribute__((address(0x1FE9)));
# 28807 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char FSR1L_SHAD __attribute__((address(0x1FEA)));

__asm("FSR1L_SHAD equ 01FEAh");


typedef union {
    struct {
        unsigned FSR1L_SHAD :8;
    };
} FSR1L_SHADbits_t;
extern volatile FSR1L_SHADbits_t FSR1L_SHADbits __attribute__((address(0x1FEA)));
# 28827 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char FSR1H_SHAD __attribute__((address(0x1FEB)));

__asm("FSR1H_SHAD equ 01FEBh");


typedef union {
    struct {
        unsigned FSR1H_SHAD :8;
    };
} FSR1H_SHADbits_t;
extern volatile FSR1H_SHADbits_t FSR1H_SHADbits __attribute__((address(0x1FEB)));
# 28847 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char STKPTR __attribute__((address(0x1FED)));

__asm("STKPTR equ 01FEDh");


typedef union {
    struct {
        unsigned STKPTR :5;
    };
} STKPTRbits_t;
extern volatile STKPTRbits_t STKPTRbits __attribute__((address(0x1FED)));
# 28867 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TOSL __attribute__((address(0x1FEE)));

__asm("TOSL equ 01FEEh");


typedef union {
    struct {
        unsigned TOSL :8;
    };
} TOSLbits_t;
extern volatile TOSLbits_t TOSLbits __attribute__((address(0x1FEE)));
# 28887 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile unsigned char TOSH __attribute__((address(0x1FEF)));

__asm("TOSH equ 01FEFh");


typedef union {
    struct {
        unsigned TOSH :7;
    };
} TOSHbits_t;
extern volatile TOSHbits_t TOSHbits __attribute__((address(0x1FEF)));
# 28913 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic16lf19156.h" 3
extern volatile __bit ACC0 __attribute__((address(0x4B0)));


extern volatile __bit ACC1 __attribute__((address(0x4B1)));


extern volatile __bit ACC10 __attribute__((address(0x4BA)));


extern volatile __bit ACC11 __attribute__((address(0x4BB)));


extern volatile __bit ACC12 __attribute__((address(0x4BC)));


extern volatile __bit ACC13 __attribute__((address(0x4BD)));


extern volatile __bit ACC14 __attribute__((address(0x4BE)));


extern volatile __bit ACC15 __attribute__((address(0x4BF)));


extern volatile __bit ACC16 __attribute__((address(0x4C0)));


extern volatile __bit ACC17 __attribute__((address(0x4C1)));


extern volatile __bit ACC18 __attribute__((address(0x4C2)));


extern volatile __bit ACC19 __attribute__((address(0x4C3)));


extern volatile __bit ACC2 __attribute__((address(0x4B2)));


extern volatile __bit ACC20 __attribute__((address(0x4C4)));


extern volatile __bit ACC21 __attribute__((address(0x4C5)));


extern volatile __bit ACC22 __attribute__((address(0x4C6)));


extern volatile __bit ACC23 __attribute__((address(0x4C7)));


extern volatile __bit ACC3 __attribute__((address(0x4B3)));


extern volatile __bit ACC4 __attribute__((address(0x4B4)));


extern volatile __bit ACC5 __attribute__((address(0x4B5)));


extern volatile __bit ACC6 __attribute__((address(0x4B6)));


extern volatile __bit ACC7 __attribute__((address(0x4B7)));


extern volatile __bit ACC8 __attribute__((address(0x4B8)));


extern volatile __bit ACC9 __attribute__((address(0x4B9)));


extern volatile __bit ACKDT __attribute__((address(0xC8D)));


extern volatile __bit ACKDT1 __attribute__((address(0xC8D)));


extern volatile __bit ACKEN __attribute__((address(0xC8C)));


extern volatile __bit ACKEN1 __attribute__((address(0xC8C)));


extern volatile __bit ACKSTAT __attribute__((address(0xC8E)));


extern volatile __bit ACKSTAT1 __attribute__((address(0xC8E)));


extern volatile __bit ACKTIM __attribute__((address(0xC97)));


extern volatile __bit ACLR __attribute__((address(0x89B)));


extern volatile __bit ACQ0 __attribute__((address(0x860)));


extern volatile __bit ACQ1 __attribute__((address(0x861)));


extern volatile __bit ACQ10 __attribute__((address(0x86A)));


extern volatile __bit ACQ11 __attribute__((address(0x86B)));


extern volatile __bit ACQ12 __attribute__((address(0x86C)));


extern volatile __bit ACQ2 __attribute__((address(0x862)));


extern volatile __bit ACQ3 __attribute__((address(0x863)));


extern volatile __bit ACQ4 __attribute__((address(0x864)));


extern volatile __bit ACQ5 __attribute__((address(0x865)));


extern volatile __bit ACQ6 __attribute__((address(0x866)));


extern volatile __bit ACQ7 __attribute__((address(0x867)));


extern volatile __bit ACQ8 __attribute__((address(0x868)));


extern volatile __bit ACQ9 __attribute__((address(0x869)));


extern volatile __bit ACT0 __attribute__((address(0x8B8)));


extern volatile __bit ACT1 __attribute__((address(0x8B9)));


extern volatile __bit ACT2 __attribute__((address(0x8BA)));


extern volatile __bit ACT3 __attribute__((address(0x8BB)));


extern volatile __bit ACT4 __attribute__((address(0x8BC)));


extern volatile __bit ACTEN __attribute__((address(0x44A7)));


extern volatile __bit ACTLOCK __attribute__((address(0x44A3)));


extern volatile __bit ACTMD __attribute__((address(0x3CB5)));


extern volatile __bit ACTORS __attribute__((address(0x44A1)));


extern volatile __bit ACTUD __attribute__((address(0x44A6)));


extern volatile __bit ADACC0 __attribute__((address(0x4B0)));


extern volatile __bit ADACC1 __attribute__((address(0x4B1)));


extern volatile __bit ADACC10 __attribute__((address(0x4BA)));


extern volatile __bit ADACC11 __attribute__((address(0x4BB)));


extern volatile __bit ADACC12 __attribute__((address(0x4BC)));


extern volatile __bit ADACC13 __attribute__((address(0x4BD)));


extern volatile __bit ADACC14 __attribute__((address(0x4BE)));


extern volatile __bit ADACC15 __attribute__((address(0x4BF)));


extern volatile __bit ADACC16 __attribute__((address(0x4C0)));


extern volatile __bit ADACC17 __attribute__((address(0x4C1)));


extern volatile __bit ADACC18 __attribute__((address(0x4C2)));


extern volatile __bit ADACC19 __attribute__((address(0x4C3)));


extern volatile __bit ADACC2 __attribute__((address(0x4B2)));


extern volatile __bit ADACC20 __attribute__((address(0x4C4)));


extern volatile __bit ADACC21 __attribute__((address(0x4C5)));


extern volatile __bit ADACC22 __attribute__((address(0x4C6)));


extern volatile __bit ADACC23 __attribute__((address(0x4C7)));


extern volatile __bit ADACC3 __attribute__((address(0x4B3)));


extern volatile __bit ADACC4 __attribute__((address(0x4B4)));


extern volatile __bit ADACC5 __attribute__((address(0x4B5)));


extern volatile __bit ADACC6 __attribute__((address(0x4B6)));


extern volatile __bit ADACC7 __attribute__((address(0x4B7)));


extern volatile __bit ADACC8 __attribute__((address(0x4B8)));


extern volatile __bit ADACC9 __attribute__((address(0x4B9)));


extern volatile __bit ADACLR __attribute__((address(0x89B)));


extern volatile __bit ADACQ0 __attribute__((address(0x860)));


extern volatile __bit ADACQ1 __attribute__((address(0x861)));


extern volatile __bit ADACQ10 __attribute__((address(0x86A)));


extern volatile __bit ADACQ11 __attribute__((address(0x86B)));


extern volatile __bit ADACQ12 __attribute__((address(0x86C)));


extern volatile __bit ADACQ2 __attribute__((address(0x862)));


extern volatile __bit ADACQ3 __attribute__((address(0x863)));


extern volatile __bit ADACQ4 __attribute__((address(0x864)));


extern volatile __bit ADACQ5 __attribute__((address(0x865)));


extern volatile __bit ADACQ6 __attribute__((address(0x866)));


extern volatile __bit ADACQ7 __attribute__((address(0x867)));


extern volatile __bit ADACQ8 __attribute__((address(0x868)));


extern volatile __bit ADACQ9 __attribute__((address(0x869)));


extern volatile __bit ADACT0 __attribute__((address(0x8B8)));


extern volatile __bit ADACT1 __attribute__((address(0x8B9)));


extern volatile __bit ADACT2 __attribute__((address(0x8BA)));


extern volatile __bit ADACT3 __attribute__((address(0x8BB)));


extern volatile __bit ADACT4 __attribute__((address(0x8BC)));


extern volatile __bit ADAOV __attribute__((address(0x8AF)));


extern volatile __bit ADCACTPPS0 __attribute__((address(0xF618)));


extern volatile __bit ADCACTPPS1 __attribute__((address(0xF619)));


extern volatile __bit ADCACTPPS2 __attribute__((address(0xF61A)));


extern volatile __bit ADCACTPPS3 __attribute__((address(0xF61B)));


extern volatile __bit ADCACTPPS4 __attribute__((address(0xF61C)));


extern volatile __bit ADCALC0 __attribute__((address(0x8A4)));


extern volatile __bit ADCALC1 __attribute__((address(0x8A5)));


extern volatile __bit ADCALC2 __attribute__((address(0x8A6)));


extern volatile __bit ADCAP0 __attribute__((address(0x870)));


extern volatile __bit ADCAP1 __attribute__((address(0x871)));


extern volatile __bit ADCAP2 __attribute__((address(0x872)));


extern volatile __bit ADCAP3 __attribute__((address(0x873)));


extern volatile __bit ADCAP4 __attribute__((address(0x874)));


extern volatile __bit ADCMD __attribute__((address(0x3CC5)));


extern volatile __bit ADCNT0 __attribute__((address(0x4C8)));


extern volatile __bit ADCNT1 __attribute__((address(0x4C9)));


extern volatile __bit ADCNT2 __attribute__((address(0x4CA)));


extern volatile __bit ADCNT3 __attribute__((address(0x4CB)));


extern volatile __bit ADCNT4 __attribute__((address(0x4CC)));


extern volatile __bit ADCNT5 __attribute__((address(0x4CD)));


extern volatile __bit ADCNT6 __attribute__((address(0x4CE)));


extern volatile __bit ADCNT7 __attribute__((address(0x4CF)));


extern volatile __bit ADCONT __attribute__((address(0x88E)));


extern volatile __bit ADCPON __attribute__((address(0xFF)));


extern volatile __bit ADCPRDY __attribute__((address(0xF8)));


extern volatile __bit ADCRS0 __attribute__((address(0x89C)));


extern volatile __bit ADCRS1 __attribute__((address(0x89D)));


extern volatile __bit ADCRS2 __attribute__((address(0x89E)));


extern volatile __bit ADCS __attribute__((address(0x88C)));


extern volatile __bit ADCS0 __attribute__((address(0x8C0)));


extern volatile __bit ADCS1 __attribute__((address(0x8C1)));


extern volatile __bit ADCS2 __attribute__((address(0x8C2)));


extern volatile __bit ADCS3 __attribute__((address(0x8C3)));


extern volatile __bit ADCS4 __attribute__((address(0x8C4)));


extern volatile __bit ADCS5 __attribute__((address(0x8C5)));


extern volatile __bit ADDSEN __attribute__((address(0x890)));


extern volatile __bit ADERR0 __attribute__((address(0x480)));


extern volatile __bit ADERR1 __attribute__((address(0x481)));


extern volatile __bit ADERR10 __attribute__((address(0x48A)));


extern volatile __bit ADERR11 __attribute__((address(0x48B)));


extern volatile __bit ADERR12 __attribute__((address(0x48C)));


extern volatile __bit ADERR13 __attribute__((address(0x48D)));


extern volatile __bit ADERR14 __attribute__((address(0x48E)));


extern volatile __bit ADERR15 __attribute__((address(0x48F)));


extern volatile __bit ADERR2 __attribute__((address(0x482)));


extern volatile __bit ADERR3 __attribute__((address(0x483)));


extern volatile __bit ADERR4 __attribute__((address(0x484)));


extern volatile __bit ADERR5 __attribute__((address(0x485)));


extern volatile __bit ADERR6 __attribute__((address(0x486)));


extern volatile __bit ADERR7 __attribute__((address(0x487)));


extern volatile __bit ADERR8 __attribute__((address(0x488)));


extern volatile __bit ADERR9 __attribute__((address(0x489)));


extern volatile __bit ADFLTR0 __attribute__((address(0x4A0)));


extern volatile __bit ADFLTR1 __attribute__((address(0x4A1)));


extern volatile __bit ADFLTR10 __attribute__((address(0x4AA)));


extern volatile __bit ADFLTR11 __attribute__((address(0x4AB)));


extern volatile __bit ADFLTR12 __attribute__((address(0x4AC)));


extern volatile __bit ADFLTR13 __attribute__((address(0x4AD)));


extern volatile __bit ADFLTR14 __attribute__((address(0x4AE)));


extern volatile __bit ADFLTR15 __attribute__((address(0x4AF)));


extern volatile __bit ADFLTR2 __attribute__((address(0x4A2)));


extern volatile __bit ADFLTR3 __attribute__((address(0x4A3)));


extern volatile __bit ADFLTR4 __attribute__((address(0x4A4)));


extern volatile __bit ADFLTR5 __attribute__((address(0x4A5)));


extern volatile __bit ADFLTR6 __attribute__((address(0x4A6)));


extern volatile __bit ADFLTR7 __attribute__((address(0x4A7)));


extern volatile __bit ADFLTR8 __attribute__((address(0x4A8)));


extern volatile __bit ADFLTR9 __attribute__((address(0x4A9)));


extern volatile __bit ADFM __attribute__((address(0x88A)));


extern volatile __bit ADFVR0 __attribute__((address(0x4860)));


extern volatile __bit ADFVR1 __attribute__((address(0x4861)));


extern volatile __bit ADGO __attribute__((address(0x888)));


extern volatile __bit ADGPOL __attribute__((address(0x895)));


extern volatile __bit ADIE __attribute__((address(0x38B8)));


extern volatile __bit ADIF __attribute__((address(0x3868)));


extern volatile __bit ADIPEN __attribute__((address(0x896)));


extern volatile __bit ADLTH0 __attribute__((address(0x460)));


extern volatile __bit ADLTH1 __attribute__((address(0x461)));


extern volatile __bit ADLTH10 __attribute__((address(0x46A)));


extern volatile __bit ADLTH11 __attribute__((address(0x46B)));


extern volatile __bit ADLTH12 __attribute__((address(0x46C)));


extern volatile __bit ADLTH13 __attribute__((address(0x46D)));


extern volatile __bit ADLTH14 __attribute__((address(0x46E)));


extern volatile __bit ADLTH15 __attribute__((address(0x46F)));


extern volatile __bit ADLTH2 __attribute__((address(0x462)));


extern volatile __bit ADLTH3 __attribute__((address(0x463)));


extern volatile __bit ADLTH4 __attribute__((address(0x464)));


extern volatile __bit ADLTH5 __attribute__((address(0x465)));


extern volatile __bit ADLTH6 __attribute__((address(0x466)));


extern volatile __bit ADLTH7 __attribute__((address(0x467)));


extern volatile __bit ADLTH8 __attribute__((address(0x468)));


extern volatile __bit ADLTH9 __attribute__((address(0x469)));


extern volatile __bit ADLTHR __attribute__((address(0x8AD)));


extern volatile __bit ADMATH __attribute__((address(0x8AC)));


extern volatile __bit ADMD0 __attribute__((address(0x898)));


extern volatile __bit ADMD1 __attribute__((address(0x899)));


extern volatile __bit ADMD2 __attribute__((address(0x89A)));


extern volatile __bit ADMSK1 __attribute__((address(0xC89)));


extern volatile __bit ADMSK11 __attribute__((address(0xC89)));


extern volatile __bit ADMSK2 __attribute__((address(0xC8A)));


extern volatile __bit ADMSK21 __attribute__((address(0xC8A)));


extern volatile __bit ADMSK3 __attribute__((address(0xC8B)));


extern volatile __bit ADMSK31 __attribute__((address(0xC8B)));


extern volatile __bit ADMSK4 __attribute__((address(0xC8C)));


extern volatile __bit ADMSK41 __attribute__((address(0xC8C)));


extern volatile __bit ADMSK5 __attribute__((address(0xC8D)));


extern volatile __bit ADMSK51 __attribute__((address(0xC8D)));


extern volatile __bit ADOEN __attribute__((address(0x448A)));


extern volatile __bit ADON __attribute__((address(0x88F)));


extern volatile __bit ADOR __attribute__((address(0x4482)));


extern volatile __bit ADPCH0 __attribute__((address(0x4F8)));


extern volatile __bit ADPCH1 __attribute__((address(0x4F9)));


extern volatile __bit ADPCH2 __attribute__((address(0x4FA)));


extern volatile __bit ADPCH3 __attribute__((address(0x4FB)));


extern volatile __bit ADPCH4 __attribute__((address(0x4FC)));


extern volatile __bit ADPCH5 __attribute__((address(0x4FD)));


extern volatile __bit ADPPOL __attribute__((address(0x897)));


extern volatile __bit ADPRE0 __attribute__((address(0x878)));


extern volatile __bit ADPRE1 __attribute__((address(0x879)));


extern volatile __bit ADPRE10 __attribute__((address(0x882)));


extern volatile __bit ADPRE11 __attribute__((address(0x883)));


extern volatile __bit ADPRE12 __attribute__((address(0x884)));


extern volatile __bit ADPRE2 __attribute__((address(0x87A)));


extern volatile __bit ADPRE3 __attribute__((address(0x87B)));


extern volatile __bit ADPRE4 __attribute__((address(0x87C)));


extern volatile __bit ADPRE5 __attribute__((address(0x87D)));


extern volatile __bit ADPRE6 __attribute__((address(0x87E)));


extern volatile __bit ADPRE7 __attribute__((address(0x87F)));


extern volatile __bit ADPRE8 __attribute__((address(0x880)));


extern volatile __bit ADPRE9 __attribute__((address(0x881)));


extern volatile __bit ADPREF0 __attribute__((address(0x8B0)));


extern volatile __bit ADPREF1 __attribute__((address(0x8B1)));


extern volatile __bit ADPREV0 __attribute__((address(0x4D8)));


extern volatile __bit ADPREV1 __attribute__((address(0x4D9)));


extern volatile __bit ADPREV10 __attribute__((address(0x4E2)));


extern volatile __bit ADPREV11 __attribute__((address(0x4E3)));


extern volatile __bit ADPREV12 __attribute__((address(0x4E4)));


extern volatile __bit ADPREV13 __attribute__((address(0x4E5)));


extern volatile __bit ADPREV14 __attribute__((address(0x4E6)));


extern volatile __bit ADPREV15 __attribute__((address(0x4E7)));


extern volatile __bit ADPREV2 __attribute__((address(0x4DA)));


extern volatile __bit ADPREV3 __attribute__((address(0x4DB)));


extern volatile __bit ADPREV4 __attribute__((address(0x4DC)));


extern volatile __bit ADPREV5 __attribute__((address(0x4DD)));


extern volatile __bit ADPREV6 __attribute__((address(0x4DE)));


extern volatile __bit ADPREV7 __attribute__((address(0x4DF)));


extern volatile __bit ADPREV8 __attribute__((address(0x4E0)));


extern volatile __bit ADPREV9 __attribute__((address(0x4E1)));


extern volatile __bit ADPSIS __attribute__((address(0x89F)));


extern volatile __bit ADRES0 __attribute__((address(0x4E8)));


extern volatile __bit ADRES1 __attribute__((address(0x4E9)));


extern volatile __bit ADRES10 __attribute__((address(0x4F2)));


extern volatile __bit ADRES11 __attribute__((address(0x4F3)));


extern volatile __bit ADRES12 __attribute__((address(0x4F4)));


extern volatile __bit ADRES13 __attribute__((address(0x4F5)));


extern volatile __bit ADRES14 __attribute__((address(0x4F6)));


extern volatile __bit ADRES15 __attribute__((address(0x4F7)));


extern volatile __bit ADRES2 __attribute__((address(0x4EA)));


extern volatile __bit ADRES3 __attribute__((address(0x4EB)));


extern volatile __bit ADRES4 __attribute__((address(0x4EC)));


extern volatile __bit ADRES5 __attribute__((address(0x4ED)));


extern volatile __bit ADRES6 __attribute__((address(0x4EE)));


extern volatile __bit ADRES7 __attribute__((address(0x4EF)));


extern volatile __bit ADRES8 __attribute__((address(0x4F0)));


extern volatile __bit ADRES9 __attribute__((address(0x4F1)));


extern volatile __bit ADRPT0 __attribute__((address(0x4D0)));


extern volatile __bit ADRPT1 __attribute__((address(0x4D1)));


extern volatile __bit ADRPT2 __attribute__((address(0x4D2)));


extern volatile __bit ADRPT3 __attribute__((address(0x4D3)));


extern volatile __bit ADRPT4 __attribute__((address(0x4D4)));


extern volatile __bit ADRPT5 __attribute__((address(0x4D5)));


extern volatile __bit ADRPT6 __attribute__((address(0x4D6)));


extern volatile __bit ADRPT7 __attribute__((address(0x4D7)));


extern volatile __bit ADSOI __attribute__((address(0x8A3)));


extern volatile __bit ADSTAT0 __attribute__((address(0x8A8)));


extern volatile __bit ADSTAT1 __attribute__((address(0x8A9)));


extern volatile __bit ADSTAT2 __attribute__((address(0x8AA)));


extern volatile __bit ADSTPT0 __attribute__((address(0x490)));


extern volatile __bit ADSTPT1 __attribute__((address(0x491)));


extern volatile __bit ADSTPT10 __attribute__((address(0x49A)));


extern volatile __bit ADSTPT11 __attribute__((address(0x49B)));


extern volatile __bit ADSTPT12 __attribute__((address(0x49C)));


extern volatile __bit ADSTPT13 __attribute__((address(0x49D)));


extern volatile __bit ADSTPT14 __attribute__((address(0x49E)));


extern volatile __bit ADSTPT15 __attribute__((address(0x49F)));


extern volatile __bit ADSTPT2 __attribute__((address(0x492)));


extern volatile __bit ADSTPT3 __attribute__((address(0x493)));


extern volatile __bit ADSTPT4 __attribute__((address(0x494)));


extern volatile __bit ADSTPT5 __attribute__((address(0x495)));


extern volatile __bit ADSTPT6 __attribute__((address(0x496)));


extern volatile __bit ADSTPT7 __attribute__((address(0x497)));


extern volatile __bit ADSTPT8 __attribute__((address(0x498)));


extern volatile __bit ADSTPT9 __attribute__((address(0x499)));


extern volatile __bit ADTIE __attribute__((address(0x38B9)));


extern volatile __bit ADTIF __attribute__((address(0x3869)));


extern volatile __bit ADTMD0 __attribute__((address(0x8A0)));


extern volatile __bit ADTMD1 __attribute__((address(0x8A1)));


extern volatile __bit ADTMD2 __attribute__((address(0x8A2)));


extern volatile __bit ADUTH0 __attribute__((address(0x470)));


extern volatile __bit ADUTH1 __attribute__((address(0x471)));


extern volatile __bit ADUTH10 __attribute__((address(0x47A)));


extern volatile __bit ADUTH11 __attribute__((address(0x47B)));


extern volatile __bit ADUTH12 __attribute__((address(0x47C)));


extern volatile __bit ADUTH13 __attribute__((address(0x47D)));


extern volatile __bit ADUTH14 __attribute__((address(0x47E)));


extern volatile __bit ADUTH15 __attribute__((address(0x47F)));


extern volatile __bit ADUTH2 __attribute__((address(0x472)));


extern volatile __bit ADUTH3 __attribute__((address(0x473)));


extern volatile __bit ADUTH4 __attribute__((address(0x474)));


extern volatile __bit ADUTH5 __attribute__((address(0x475)));


extern volatile __bit ADUTH6 __attribute__((address(0x476)));


extern volatile __bit ADUTH7 __attribute__((address(0x477)));


extern volatile __bit ADUTH8 __attribute__((address(0x478)));


extern volatile __bit ADUTH9 __attribute__((address(0x479)));


extern volatile __bit ADUTHR __attribute__((address(0x8AE)));


extern volatile __bit AHEN __attribute__((address(0xC91)));


extern volatile __bit ALRMEN __attribute__((address(0x6077)));


extern volatile __bit AMASK0 __attribute__((address(0x6072)));


extern volatile __bit AMASK1 __attribute__((address(0x6073)));


extern volatile __bit AMASK2 __attribute__((address(0x6074)));


extern volatile __bit AMASK3 __attribute__((address(0x6075)));


extern volatile __bit ANSA0 __attribute__((address(0xF9C0)));


extern volatile __bit ANSA1 __attribute__((address(0xF9C1)));


extern volatile __bit ANSA2 __attribute__((address(0xF9C2)));


extern volatile __bit ANSA3 __attribute__((address(0xF9C3)));


extern volatile __bit ANSA4 __attribute__((address(0xF9C4)));


extern volatile __bit ANSA6 __attribute__((address(0xF9C6)));


extern volatile __bit ANSA7 __attribute__((address(0xF9C7)));


extern volatile __bit ANSB0 __attribute__((address(0xFA18)));


extern volatile __bit ANSB1 __attribute__((address(0xFA19)));


extern volatile __bit ANSB2 __attribute__((address(0xFA1A)));


extern volatile __bit ANSB3 __attribute__((address(0xFA1B)));


extern volatile __bit ANSB4 __attribute__((address(0xFA1C)));


extern volatile __bit ANSB5 __attribute__((address(0xFA1D)));


extern volatile __bit ANSB6 __attribute__((address(0xFA1E)));


extern volatile __bit ANSB7 __attribute__((address(0xFA1F)));


extern volatile __bit ANSC0 __attribute__((address(0xFA70)));


extern volatile __bit ANSC1 __attribute__((address(0xFA71)));


extern volatile __bit ANSC2 __attribute__((address(0xFA72)));


extern volatile __bit ANSC3 __attribute__((address(0xFA73)));


extern volatile __bit ANSC4 __attribute__((address(0xFA74)));


extern volatile __bit ANSC6 __attribute__((address(0xFA76)));


extern volatile __bit ANSC7 __attribute__((address(0xFA77)));


extern volatile __bit ARPT0 __attribute__((address(0x6078)));


extern volatile __bit ARPT1 __attribute__((address(0x6079)));


extern volatile __bit ARPT2 __attribute__((address(0x607A)));


extern volatile __bit ARPT3 __attribute__((address(0x607B)));


extern volatile __bit ARPT4 __attribute__((address(0x607C)));


extern volatile __bit ARPT5 __attribute__((address(0x607D)));


extern volatile __bit ARPT6 __attribute__((address(0x607E)));


extern volatile __bit ARPT7 __attribute__((address(0x607F)));


extern volatile __bit AS __attribute__((address(0x24D0)));


extern volatile __bit AS0E __attribute__((address(0x3098)));


extern volatile __bit AS1E __attribute__((address(0x3099)));


extern volatile __bit AS2E __attribute__((address(0x309A)));


extern volatile __bit AS3E __attribute__((address(0x309B)));


extern volatile __bit AS4E __attribute__((address(0x309C)));


extern volatile __bit AS5E __attribute__((address(0x309D)));


extern volatile __bit AS6E __attribute__((address(0x309E)));


extern volatile __bit BCL1IE __attribute__((address(0x38C9)));


extern volatile __bit BCL1IF __attribute__((address(0x3879)));


extern volatile __bit BF __attribute__((address(0xC78)));


extern volatile __bit BF1 __attribute__((address(0xC78)));


extern volatile __bit BIAS0 __attribute__((address(0xE8A0)));


extern volatile __bit BIAS1 __attribute__((address(0xE8A1)));


extern volatile __bit BIAS2 __attribute__((address(0xE8A2)));


extern volatile __bit BOEN __attribute__((address(0xC94)));


extern volatile __bit BORRDY __attribute__((address(0x4088)));


extern volatile __bit BSR0 __attribute__((address(0x40)));


extern volatile __bit BSR1 __attribute__((address(0x41)));


extern volatile __bit BSR2 __attribute__((address(0x42)));


extern volatile __bit BSR3 __attribute__((address(0x43)));


extern volatile __bit BSR4 __attribute__((address(0x44)));


extern volatile __bit BSR5 __attribute__((address(0x45)));


extern volatile __bit C1HYS __attribute__((address(0x4C81)));


extern volatile __bit C1IE __attribute__((address(0x38C0)));


extern volatile __bit C1IF __attribute__((address(0x3870)));


extern volatile __bit C1INTN __attribute__((address(0x4C88)));


extern volatile __bit C1INTP __attribute__((address(0x4C89)));


extern volatile __bit C1NCH0 __attribute__((address(0x4C90)));


extern volatile __bit C1NCH1 __attribute__((address(0x4C91)));


extern volatile __bit C1NCH2 __attribute__((address(0x4C92)));


extern volatile __bit C1ON __attribute__((address(0x4C87)));


extern volatile __bit C1PCH0 __attribute__((address(0x4C98)));


extern volatile __bit C1PCH1 __attribute__((address(0x4C99)));


extern volatile __bit C1PCH2 __attribute__((address(0x4C9A)));


extern volatile __bit C1POL __attribute__((address(0x4C84)));


extern volatile __bit C1SYNC __attribute__((address(0x4C80)));


extern volatile __bit C1TSEL0 __attribute__((address(0x10F0)));


extern volatile __bit C1TSEL1 __attribute__((address(0x10F1)));


extern volatile __bit C2HYS __attribute__((address(0x4CA1)));


extern volatile __bit C2IE __attribute__((address(0x38C1)));


extern volatile __bit C2IF __attribute__((address(0x3871)));


extern volatile __bit C2INTN __attribute__((address(0x4CA8)));


extern volatile __bit C2INTP __attribute__((address(0x4CA9)));


extern volatile __bit C2NCH0 __attribute__((address(0x4CB0)));


extern volatile __bit C2NCH1 __attribute__((address(0x4CB1)));


extern volatile __bit C2NCH2 __attribute__((address(0x4CB2)));


extern volatile __bit C2ON __attribute__((address(0x4CA7)));


extern volatile __bit C2PCH0 __attribute__((address(0x4CB8)));


extern volatile __bit C2PCH1 __attribute__((address(0x4CB9)));


extern volatile __bit C2PCH2 __attribute__((address(0x4CBA)));


extern volatile __bit C2POL __attribute__((address(0x4CA4)));


extern volatile __bit C2SYNC __attribute__((address(0x4CA0)));


extern volatile __bit C2TSEL0 __attribute__((address(0x10F2)));


extern volatile __bit C2TSEL1 __attribute__((address(0x10F3)));


extern volatile __bit C3TSEL0 __attribute__((address(0x10F4)));


extern volatile __bit C3TSEL1 __attribute__((address(0x10F5)));


extern volatile __bit C4TSEL0 __attribute__((address(0x10F6)));


extern volatile __bit C4TSEL1 __attribute__((address(0x10F7)));


extern volatile __bit CAL0 __attribute__((address(0x6068)));


extern volatile __bit CAL01 __attribute__((address(0x1060)));


extern volatile __bit CAL1 __attribute__((address(0x6069)));


extern volatile __bit CAL11 __attribute__((address(0x1061)));


extern volatile __bit CAL2 __attribute__((address(0x606A)));


extern volatile __bit CAL21 __attribute__((address(0x1062)));


extern volatile __bit CAL3 __attribute__((address(0x606B)));


extern volatile __bit CAL31 __attribute__((address(0x1063)));


extern volatile __bit CAL4 __attribute__((address(0x606C)));


extern volatile __bit CAL41 __attribute__((address(0x1064)));


extern volatile __bit CAL5 __attribute__((address(0x606D)));


extern volatile __bit CAL51 __attribute__((address(0x1065)));


extern volatile __bit CAL6 __attribute__((address(0x606E)));


extern volatile __bit CAL61 __attribute__((address(0x1066)));


extern volatile __bit CAL7 __attribute__((address(0x606F)));


extern volatile __bit CAL71 __attribute__((address(0x1067)));


extern volatile __bit CALC0 __attribute__((address(0x8A4)));


extern volatile __bit CALC1 __attribute__((address(0x8A5)));


extern volatile __bit CALC2 __attribute__((address(0x8A6)));


extern volatile __bit CARRY __attribute__((address(0x18)));


extern volatile __bit CCP1CTS0 __attribute__((address(0x1878)));


extern volatile __bit CCP1CTS1 __attribute__((address(0x1879)));


extern volatile __bit CCP1CTS2 __attribute__((address(0x187A)));


extern volatile __bit CCP1EN __attribute__((address(0x1877)));


extern volatile __bit CCP1FMT __attribute__((address(0x1874)));


extern volatile __bit CCP1IE __attribute__((address(0x38E0)));


extern volatile __bit CCP1IF __attribute__((address(0x3890)));


extern volatile __bit CCP1MD __attribute__((address(0x3CC8)));


extern volatile __bit CCP1MODE0 __attribute__((address(0x1870)));


extern volatile __bit CCP1MODE1 __attribute__((address(0x1871)));


extern volatile __bit CCP1MODE2 __attribute__((address(0x1872)));


extern volatile __bit CCP1MODE3 __attribute__((address(0x1873)));


extern volatile __bit CCP1OE __attribute__((address(0x1876)));


extern volatile __bit CCP1OUT __attribute__((address(0x1875)));


extern volatile __bit CCP1PPS0 __attribute__((address(0xF508)));


extern volatile __bit CCP1PPS1 __attribute__((address(0xF509)));


extern volatile __bit CCP1PPS2 __attribute__((address(0xF50A)));


extern volatile __bit CCP1PPS3 __attribute__((address(0xF50B)));


extern volatile __bit CCP1PPS4 __attribute__((address(0xF50C)));


extern volatile __bit CCP2CTS0 __attribute__((address(0x1898)));


extern volatile __bit CCP2CTS1 __attribute__((address(0x1899)));


extern volatile __bit CCP2CTS2 __attribute__((address(0x189A)));


extern volatile __bit CCP2EN __attribute__((address(0x1897)));


extern volatile __bit CCP2FMT __attribute__((address(0x1894)));


extern volatile __bit CCP2IE __attribute__((address(0x38E1)));


extern volatile __bit CCP2IF __attribute__((address(0x3891)));


extern volatile __bit CCP2MD __attribute__((address(0x3CC9)));


extern volatile __bit CCP2MODE0 __attribute__((address(0x1890)));


extern volatile __bit CCP2MODE1 __attribute__((address(0x1891)));


extern volatile __bit CCP2MODE2 __attribute__((address(0x1892)));


extern volatile __bit CCP2MODE3 __attribute__((address(0x1893)));


extern volatile __bit CCP2OE __attribute__((address(0x1896)));


extern volatile __bit CCP2OUT __attribute__((address(0x1895)));


extern volatile __bit CCP2PPS0 __attribute__((address(0xF510)));


extern volatile __bit CCP2PPS1 __attribute__((address(0xF511)));


extern volatile __bit CCP2PPS2 __attribute__((address(0xF512)));


extern volatile __bit CCP2PPS3 __attribute__((address(0xF513)));


extern volatile __bit CCP2PPS4 __attribute__((address(0xF514)));


extern volatile __bit CCP3MD __attribute__((address(0x3CCA)));


extern volatile __bit CCP4MD __attribute__((address(0x3CCB)));


extern volatile __bit CDAFVR0 __attribute__((address(0x4862)));


extern volatile __bit CDAFVR1 __attribute__((address(0x4863)));


extern volatile __bit CDIV0 __attribute__((address(0x4470)));


extern volatile __bit CDIV1 __attribute__((address(0x4471)));


extern volatile __bit CDIV2 __attribute__((address(0x4472)));


extern volatile __bit CDIV3 __attribute__((address(0x4473)));


extern volatile __bit CHIME __attribute__((address(0x6076)));


extern volatile __bit CKE __attribute__((address(0xC7E)));


extern volatile __bit CKE1 __attribute__((address(0xC7E)));


extern volatile __bit CKP __attribute__((address(0xC84)));


extern volatile __bit CKP1 __attribute__((address(0xC84)));


extern volatile __bit CLC1IE __attribute__((address(0x38DC)));


extern volatile __bit CLC1IF __attribute__((address(0x388C)));


extern volatile __bit CLC1MD __attribute__((address(0x3CD9)));


extern volatile __bit CLC2IE __attribute__((address(0x38DD)));


extern volatile __bit CLC2IF __attribute__((address(0x388D)));


extern volatile __bit CLC2MD __attribute__((address(0x3CDA)));


extern volatile __bit CLC3IE __attribute__((address(0x38DE)));


extern volatile __bit CLC3IF __attribute__((address(0x388E)));


extern volatile __bit CLC3MD __attribute__((address(0x3CDB)));


extern volatile __bit CLC4IE __attribute__((address(0x38DF)));


extern volatile __bit CLC4IF __attribute__((address(0x388F)));


extern volatile __bit CLC4MD __attribute__((address(0x3CDC)));


extern volatile __bit CLCIN0PPS0 __attribute__((address(0xF5D8)));


extern volatile __bit CLCIN0PPS1 __attribute__((address(0xF5D9)));


extern volatile __bit CLCIN0PPS2 __attribute__((address(0xF5DA)));


extern volatile __bit CLCIN0PPS3 __attribute__((address(0xF5DB)));


extern volatile __bit CLCIN0PPS4 __attribute__((address(0xF5DC)));


extern volatile __bit CLCIN1PPS0 __attribute__((address(0xF5E0)));


extern volatile __bit CLCIN1PPS1 __attribute__((address(0xF5E1)));


extern volatile __bit CLCIN1PPS2 __attribute__((address(0xF5E2)));


extern volatile __bit CLCIN1PPS3 __attribute__((address(0xF5E3)));


extern volatile __bit CLCIN1PPS4 __attribute__((address(0xF5E4)));


extern volatile __bit CLCIN2PPS0 __attribute__((address(0xF5E8)));


extern volatile __bit CLCIN2PPS1 __attribute__((address(0xF5E9)));


extern volatile __bit CLCIN2PPS2 __attribute__((address(0xF5EA)));


extern volatile __bit CLCIN2PPS3 __attribute__((address(0xF5EB)));


extern volatile __bit CLCIN2PPS4 __attribute__((address(0xF5EC)));


extern volatile __bit CLCIN3PPS0 __attribute__((address(0xF5F0)));


extern volatile __bit CLCIN3PPS1 __attribute__((address(0xF5F1)));


extern volatile __bit CLCIN3PPS2 __attribute__((address(0xF5F2)));


extern volatile __bit CLCIN3PPS3 __attribute__((address(0xF5F3)));


extern volatile __bit CLCIN3PPS4 __attribute__((address(0xF5F4)));


extern volatile __bit CMP1MD __attribute__((address(0x3CC1)));


extern volatile __bit CMP2MD __attribute__((address(0x3CC2)));


extern volatile __bit CNT0 __attribute__((address(0x4C8)));


extern volatile __bit CNT1 __attribute__((address(0x4C9)));


extern volatile __bit CNT2 __attribute__((address(0x4CA)));


extern volatile __bit CNT3 __attribute__((address(0x4CB)));


extern volatile __bit CNT4 __attribute__((address(0x4CC)));


extern volatile __bit CNT5 __attribute__((address(0x4CD)));


extern volatile __bit CNT6 __attribute__((address(0x4CE)));


extern volatile __bit CNT7 __attribute__((address(0x4CF)));


extern volatile __bit CONT __attribute__((address(0x88E)));


extern volatile __bit COSC0 __attribute__((address(0x4474)));


extern volatile __bit COSC1 __attribute__((address(0x4475)));


extern volatile __bit COSC2 __attribute__((address(0x4476)));


extern volatile __bit CPOL __attribute__((address(0x24C2)));


extern volatile __bit CPR0 __attribute__((address(0x2478)));


extern volatile __bit CPR1 __attribute__((address(0x2479)));


extern volatile __bit CPR10 __attribute__((address(0x2482)));


extern volatile __bit CPR11 __attribute__((address(0x2483)));


extern volatile __bit CPR12 __attribute__((address(0x2484)));


extern volatile __bit CPR13 __attribute__((address(0x2485)));


extern volatile __bit CPR14 __attribute__((address(0x2486)));


extern volatile __bit CPR15 __attribute__((address(0x2487)));


extern volatile __bit CPR16 __attribute__((address(0x2488)));


extern volatile __bit CPR17 __attribute__((address(0x2489)));


extern volatile __bit CPR18 __attribute__((address(0x248A)));


extern volatile __bit CPR19 __attribute__((address(0x248B)));


extern volatile __bit CPR2 __attribute__((address(0x247A)));


extern volatile __bit CPR20 __attribute__((address(0x248C)));


extern volatile __bit CPR21 __attribute__((address(0x248D)));


extern volatile __bit CPR22 __attribute__((address(0x248E)));


extern volatile __bit CPR23 __attribute__((address(0x248F)));


extern volatile __bit CPR3 __attribute__((address(0x247B)));


extern volatile __bit CPR4 __attribute__((address(0x247C)));


extern volatile __bit CPR5 __attribute__((address(0x247D)));


extern volatile __bit CPR6 __attribute__((address(0x247E)));


extern volatile __bit CPR7 __attribute__((address(0x247F)));


extern volatile __bit CPR8 __attribute__((address(0x2480)));


extern volatile __bit CPR9 __attribute__((address(0x2481)));


extern volatile __bit CPRUP __attribute__((address(0x24D7)));


extern volatile __bit CPW0 __attribute__((address(0x2490)));


extern volatile __bit CPW1 __attribute__((address(0x2491)));


extern volatile __bit CPW10 __attribute__((address(0x249A)));


extern volatile __bit CPW11 __attribute__((address(0x249B)));


extern volatile __bit CPW12 __attribute__((address(0x249C)));


extern volatile __bit CPW13 __attribute__((address(0x249D)));


extern volatile __bit CPW14 __attribute__((address(0x249E)));


extern volatile __bit CPW15 __attribute__((address(0x249F)));


extern volatile __bit CPW16 __attribute__((address(0x24A0)));


extern volatile __bit CPW17 __attribute__((address(0x24A1)));


extern volatile __bit CPW18 __attribute__((address(0x24A2)));


extern volatile __bit CPW19 __attribute__((address(0x24A3)));


extern volatile __bit CPW2 __attribute__((address(0x2492)));


extern volatile __bit CPW20 __attribute__((address(0x24A4)));


extern volatile __bit CPW21 __attribute__((address(0x24A5)));


extern volatile __bit CPW22 __attribute__((address(0x24A6)));


extern volatile __bit CPW23 __attribute__((address(0x24A7)));


extern volatile __bit CPW3 __attribute__((address(0x2493)));


extern volatile __bit CPW4 __attribute__((address(0x2494)));


extern volatile __bit CPW5 __attribute__((address(0x2495)));


extern volatile __bit CPW6 __attribute__((address(0x2496)));


extern volatile __bit CPW7 __attribute__((address(0x2497)));


extern volatile __bit CPW8 __attribute__((address(0x2498)));


extern volatile __bit CPW9 __attribute__((address(0x2499)));


extern volatile __bit CPWDT __attribute__((address(0xE8AF)));


extern volatile __bit CPWUP __attribute__((address(0x24D6)));


extern volatile __bit CRIE __attribute__((address(0x38E7)));


extern volatile __bit CRIF __attribute__((address(0x3897)));


extern volatile __bit CRS0 __attribute__((address(0x89C)));


extern volatile __bit CRS1 __attribute__((address(0x89D)));


extern volatile __bit CRS2 __attribute__((address(0x89E)));


extern volatile __bit CS4 __attribute__((address(0x8C4)));


extern volatile __bit CS5 __attribute__((address(0x8C5)));


extern volatile __bit CSEL0 __attribute__((address(0x24D8)));


extern volatile __bit CSEL1 __attribute__((address(0x24D9)));


extern volatile __bit CSEL2 __attribute__((address(0x24DA)));


extern volatile __bit CSWHOLD __attribute__((address(0x447F)));


extern volatile __bit CSWIE __attribute__((address(0x38BE)));


extern volatile __bit CSWIF __attribute__((address(0x386E)));


extern volatile __bit CWG1CS __attribute__((address(0x3060)));


extern volatile __bit CWG1DBF0 __attribute__((address(0x3078)));


extern volatile __bit CWG1DBF1 __attribute__((address(0x3079)));


extern volatile __bit CWG1DBF2 __attribute__((address(0x307A)));


extern volatile __bit CWG1DBF3 __attribute__((address(0x307B)));


extern volatile __bit CWG1DBF4 __attribute__((address(0x307C)));


extern volatile __bit CWG1DBF5 __attribute__((address(0x307D)));


extern volatile __bit CWG1DBR0 __attribute__((address(0x3070)));


extern volatile __bit CWG1DBR1 __attribute__((address(0x3071)));


extern volatile __bit CWG1DBR2 __attribute__((address(0x3072)));


extern volatile __bit CWG1DBR3 __attribute__((address(0x3073)));


extern volatile __bit CWG1DBR4 __attribute__((address(0x3074)));


extern volatile __bit CWG1DBR5 __attribute__((address(0x3075)));


extern volatile __bit CWG1EN __attribute__((address(0x3087)));


extern volatile __bit CWG1IE __attribute__((address(0x38E8)));


extern volatile __bit CWG1IF __attribute__((address(0x3898)));


extern volatile __bit CWG1IN __attribute__((address(0x308D)));


extern volatile __bit CWG1ISM0 __attribute__((address(0x3068)));


extern volatile __bit CWG1ISM1 __attribute__((address(0x3069)));


extern volatile __bit CWG1ISM2 __attribute__((address(0x306A)));


extern volatile __bit CWG1ISM3 __attribute__((address(0x306B)));


extern volatile __bit CWG1LD __attribute__((address(0x3086)));


extern volatile __bit CWG1LSAC0 __attribute__((address(0x3092)));


extern volatile __bit CWG1LSAC1 __attribute__((address(0x3093)));


extern volatile __bit CWG1LSBD0 __attribute__((address(0x3094)));


extern volatile __bit CWG1LSBD1 __attribute__((address(0x3095)));


extern volatile __bit CWG1MD __attribute__((address(0x3CD0)));


extern volatile __bit CWG1MODE0 __attribute__((address(0x3080)));


extern volatile __bit CWG1MODE1 __attribute__((address(0x3081)));


extern volatile __bit CWG1MODE2 __attribute__((address(0x3082)));


extern volatile __bit CWG1OVRA __attribute__((address(0x30A4)));


extern volatile __bit CWG1OVRB __attribute__((address(0x30A5)));


extern volatile __bit CWG1OVRC __attribute__((address(0x30A6)));


extern volatile __bit CWG1OVRD __attribute__((address(0x30A7)));


extern volatile __bit CWG1POLA __attribute__((address(0x3088)));


extern volatile __bit CWG1POLB __attribute__((address(0x3089)));


extern volatile __bit CWG1POLC __attribute__((address(0x308A)));


extern volatile __bit CWG1POLD __attribute__((address(0x308B)));


extern volatile __bit CWG1PPS0 __attribute__((address(0xF588)));


extern volatile __bit CWG1PPS1 __attribute__((address(0xF589)));


extern volatile __bit CWG1PPS2 __attribute__((address(0xF58A)));


extern volatile __bit CWG1PPS3 __attribute__((address(0xF58B)));


extern volatile __bit CWG1PPS4 __attribute__((address(0xF58C)));


extern volatile __bit CWG1REN __attribute__((address(0x3096)));


extern volatile __bit CWG1SHUTDOWN __attribute__((address(0x3097)));


extern volatile __bit CWG1STRA __attribute__((address(0x30A0)));


extern volatile __bit CWG1STRB __attribute__((address(0x30A1)));


extern volatile __bit CWG1STRC __attribute__((address(0x30A2)));


extern volatile __bit CWG1STRD __attribute__((address(0x30A3)));


extern volatile __bit C_SHAD __attribute__((address(0xFF20)));


extern volatile __bit DA __attribute__((address(0xC7D)));


extern volatile __bit DA1 __attribute__((address(0xC7D)));


extern volatile __bit DAC1EN __attribute__((address(0x4877)));


extern volatile __bit DAC1NSS __attribute__((address(0x4870)));


extern volatile __bit DAC1OE1 __attribute__((address(0x4875)));


extern volatile __bit DAC1OE2 __attribute__((address(0x4874)));


extern volatile __bit DAC1PSS0 __attribute__((address(0x4872)));


extern volatile __bit DAC1PSS1 __attribute__((address(0x4873)));


extern volatile __bit DAC1R0 __attribute__((address(0x4878)));


extern volatile __bit DAC1R1 __attribute__((address(0x4879)));


extern volatile __bit DAC1R2 __attribute__((address(0x487A)));


extern volatile __bit DAC1R3 __attribute__((address(0x487B)));


extern volatile __bit DAC1R4 __attribute__((address(0x487C)));


extern volatile __bit DACMD __attribute__((address(0x3CC6)));


extern volatile __bit DATA_ADDRESS __attribute__((address(0xC7D)));


extern volatile __bit DATA_ADDRESS1 __attribute__((address(0xC7D)));


extern volatile __bit DAYALRMH0 __attribute__((address(0x60CC)));


extern volatile __bit DAYALRMH1 __attribute__((address(0x60CD)));


extern volatile __bit DAYALRML0 __attribute__((address(0x60C8)));


extern volatile __bit DAYALRML1 __attribute__((address(0x60C9)));


extern volatile __bit DAYALRML2 __attribute__((address(0x60CA)));


extern volatile __bit DAYALRML3 __attribute__((address(0x60CB)));


extern volatile __bit DAYH0 __attribute__((address(0x609C)));


extern volatile __bit DAYH1 __attribute__((address(0x609D)));


extern volatile __bit DAYL0 __attribute__((address(0x6098)));


extern volatile __bit DAYL1 __attribute__((address(0x6099)));


extern volatile __bit DAYL2 __attribute__((address(0x609A)));


extern volatile __bit DAYL3 __attribute__((address(0x609B)));


extern volatile __bit DBF0 __attribute__((address(0x3078)));


extern volatile __bit DBF1 __attribute__((address(0x3079)));


extern volatile __bit DBF2 __attribute__((address(0x307A)));


extern volatile __bit DBF3 __attribute__((address(0x307B)));


extern volatile __bit DBF4 __attribute__((address(0x307C)));


extern volatile __bit DBF5 __attribute__((address(0x307D)));


extern volatile __bit DBR0 __attribute__((address(0x3070)));


extern volatile __bit DBR1 __attribute__((address(0x3071)));


extern volatile __bit DBR2 __attribute__((address(0x3072)));


extern volatile __bit DBR3 __attribute__((address(0x3073)));


extern volatile __bit DBR4 __attribute__((address(0x3074)));


extern volatile __bit DBR5 __attribute__((address(0x3075)));


extern volatile __bit DC __attribute__((address(0x19)));


extern volatile __bit DC_SHAD __attribute__((address(0xFF21)));


extern volatile __bit DHEN __attribute__((address(0xC90)));


extern volatile __bit DOE __attribute__((address(0x4464)));


extern volatile __bit DONE __attribute__((address(0x888)));


extern volatile __bit DOZE0 __attribute__((address(0x4460)));


extern volatile __bit DOZE1 __attribute__((address(0x4461)));


extern volatile __bit DOZE2 __attribute__((address(0x4462)));


extern volatile __bit DOZEN __attribute__((address(0x4466)));


extern volatile __bit DSEN __attribute__((address(0x890)));


extern volatile __bit D_A __attribute__((address(0xC7D)));


extern volatile __bit D_A1 __attribute__((address(0xC7D)));


extern volatile __bit D_nA __attribute__((address(0xC7D)));


extern volatile __bit D_nA1 __attribute__((address(0xC7D)));


extern volatile __bit EN5V __attribute__((address(0xE8A6)));


extern volatile __bit ERR0 __attribute__((address(0x480)));


extern volatile __bit ERR1 __attribute__((address(0x481)));


extern volatile __bit ERR10 __attribute__((address(0x48A)));


extern volatile __bit ERR11 __attribute__((address(0x48B)));


extern volatile __bit ERR12 __attribute__((address(0x48C)));


extern volatile __bit ERR13 __attribute__((address(0x48D)));


extern volatile __bit ERR14 __attribute__((address(0x48E)));


extern volatile __bit ERR15 __attribute__((address(0x48F)));


extern volatile __bit ERR2 __attribute__((address(0x482)));


extern volatile __bit ERR3 __attribute__((address(0x483)));


extern volatile __bit ERR4 __attribute__((address(0x484)));


extern volatile __bit ERR5 __attribute__((address(0x485)));


extern volatile __bit ERR6 __attribute__((address(0x486)));


extern volatile __bit ERR7 __attribute__((address(0x487)));


extern volatile __bit ERR8 __attribute__((address(0x488)));


extern volatile __bit ERR9 __attribute__((address(0x489)));


extern volatile __bit EXTOEN __attribute__((address(0x448F)));


extern volatile __bit EXTOR __attribute__((address(0x4487)));


extern volatile __bit FLTR0 __attribute__((address(0x4A0)));


extern volatile __bit FLTR1 __attribute__((address(0x4A1)));


extern volatile __bit FLTR10 __attribute__((address(0x4AA)));


extern volatile __bit FLTR11 __attribute__((address(0x4AB)));


extern volatile __bit FLTR12 __attribute__((address(0x4AC)));


extern volatile __bit FLTR13 __attribute__((address(0x4AD)));


extern volatile __bit FLTR14 __attribute__((address(0x4AE)));


extern volatile __bit FLTR15 __attribute__((address(0x4AF)));


extern volatile __bit FLTR2 __attribute__((address(0x4A2)));


extern volatile __bit FLTR3 __attribute__((address(0x4A3)));


extern volatile __bit FLTR4 __attribute__((address(0x4A4)));


extern volatile __bit FLTR5 __attribute__((address(0x4A5)));


extern volatile __bit FLTR6 __attribute__((address(0x4A6)));


extern volatile __bit FLTR7 __attribute__((address(0x4A7)));


extern volatile __bit FLTR8 __attribute__((address(0x4A8)));


extern volatile __bit FLTR9 __attribute__((address(0x4A9)));


extern volatile __bit FM __attribute__((address(0x88A)));


extern volatile __bit FREE __attribute__((address(0x40F4)));


extern volatile __bit FVREN __attribute__((address(0x4867)));


extern volatile __bit FVRMD __attribute__((address(0x3CB6)));


extern volatile __bit FVRRDY __attribute__((address(0x4866)));


extern volatile __bit G1EN __attribute__((address(0x3087)));


extern volatile __bit GCEN __attribute__((address(0xC8F)));


extern volatile __bit GCEN1 __attribute__((address(0xC8F)));


extern volatile __bit GE __attribute__((address(0x107F)));


extern volatile __bit GGO_nDONE __attribute__((address(0x107B)));


extern volatile __bit GIE __attribute__((address(0x5F)));


extern volatile __bit GO __attribute__((address(0x888)));


extern volatile __bit GO_nDONE __attribute__((address(0x888)));


extern volatile __bit GSPM __attribute__((address(0x107C)));


extern volatile __bit GSS0 __attribute__((address(0x1080)));


extern volatile __bit GSS1 __attribute__((address(0x1081)));


extern volatile __bit GSS2 __attribute__((address(0x1082)));


extern volatile __bit GSS3 __attribute__((address(0x1083)));


extern volatile __bit GSS4 __attribute__((address(0x1084)));


extern volatile __bit GTM __attribute__((address(0x107D)));


extern volatile __bit GVAL __attribute__((address(0x107A)));


extern volatile __bit HALFSEC __attribute__((address(0x6063)));


extern volatile __bit HFFRQ0 __attribute__((address(0x4498)));


extern volatile __bit HFFRQ1 __attribute__((address(0x4499)));


extern volatile __bit HFFRQ2 __attribute__((address(0x449A)));


extern volatile __bit HFOEN __attribute__((address(0x448E)));


extern volatile __bit HFOR __attribute__((address(0x4486)));


extern volatile __bit HIDB1 __attribute__((address(0x2069)));


extern volatile __bit HRALRMH0 __attribute__((address(0x60D4)));


extern volatile __bit HRALRMH1 __attribute__((address(0x60D5)));


extern volatile __bit HRALRML0 __attribute__((address(0x60D0)));


extern volatile __bit HRALRML1 __attribute__((address(0x60D1)));


extern volatile __bit HRALRML2 __attribute__((address(0x60D2)));


extern volatile __bit HRALRML3 __attribute__((address(0x60D3)));


extern volatile __bit HRH0 __attribute__((address(0x60A4)));


extern volatile __bit HRH1 __attribute__((address(0x60A5)));


extern volatile __bit HRL0 __attribute__((address(0x60A0)));


extern volatile __bit HRL1 __attribute__((address(0x60A1)));


extern volatile __bit HRL2 __attribute__((address(0x60A2)));


extern volatile __bit HRL3 __attribute__((address(0x60A3)));


extern volatile __bit I2C_DAT __attribute__((address(0xC7D)));


extern volatile __bit I2C_DAT1 __attribute__((address(0xC7D)));


extern volatile __bit I2C_READ __attribute__((address(0xC7A)));


extern volatile __bit I2C_READ1 __attribute__((address(0xC7A)));


extern volatile __bit I2C_START __attribute__((address(0xC7B)));


extern volatile __bit I2C_START1 __attribute__((address(0xC7B)));


extern volatile __bit I2C_STOP __attribute__((address(0xC7C)));


extern volatile __bit I2C_STOP2 __attribute__((address(0xC7C)));


extern volatile __bit IDLEN __attribute__((address(0x4467)));


extern volatile __bit IN __attribute__((address(0x308D)));


extern volatile __bit INLVLA0 __attribute__((address(0xF9E0)));


extern volatile __bit INLVLA1 __attribute__((address(0xF9E1)));


extern volatile __bit INLVLA2 __attribute__((address(0xF9E2)));


extern volatile __bit INLVLA3 __attribute__((address(0xF9E3)));


extern volatile __bit INLVLA4 __attribute__((address(0xF9E4)));


extern volatile __bit INLVLA5 __attribute__((address(0xF9E5)));


extern volatile __bit INLVLA6 __attribute__((address(0xF9E6)));


extern volatile __bit INLVLA7 __attribute__((address(0xF9E7)));


extern volatile __bit INLVLB0 __attribute__((address(0xFA38)));


extern volatile __bit INLVLB1 __attribute__((address(0xFA39)));


extern volatile __bit INLVLB2 __attribute__((address(0xFA3A)));


extern volatile __bit INLVLB3 __attribute__((address(0xFA3B)));


extern volatile __bit INLVLB4 __attribute__((address(0xFA3C)));


extern volatile __bit INLVLB5 __attribute__((address(0xFA3D)));


extern volatile __bit INLVLB6 __attribute__((address(0xFA3E)));


extern volatile __bit INLVLB7 __attribute__((address(0xFA3F)));


extern volatile __bit INLVLC0 __attribute__((address(0xFA90)));


extern volatile __bit INLVLC1 __attribute__((address(0xFA91)));


extern volatile __bit INLVLC2 __attribute__((address(0xFA92)));


extern volatile __bit INLVLC3 __attribute__((address(0xFA93)));


extern volatile __bit INLVLC4 __attribute__((address(0xFA94)));


extern volatile __bit INLVLC6 __attribute__((address(0xFA96)));


extern volatile __bit INLVLC7 __attribute__((address(0xFA97)));


extern volatile __bit INLVLE3 __attribute__((address(0xFB43)));


extern volatile __bit INTE __attribute__((address(0x38B0)));


extern volatile __bit INTEDG __attribute__((address(0x58)));


extern volatile __bit INTF __attribute__((address(0x3860)));


extern volatile __bit INTPPS0 __attribute__((address(0xF480)));


extern volatile __bit INTPPS1 __attribute__((address(0xF481)));


extern volatile __bit INTPPS2 __attribute__((address(0xF482)));


extern volatile __bit INTPPS3 __attribute__((address(0xF483)));


extern volatile __bit IOCAF0 __attribute__((address(0xF9F8)));


extern volatile __bit IOCAF1 __attribute__((address(0xF9F9)));


extern volatile __bit IOCAF2 __attribute__((address(0xF9FA)));


extern volatile __bit IOCAF3 __attribute__((address(0xF9FB)));


extern volatile __bit IOCAF4 __attribute__((address(0xF9FC)));


extern volatile __bit IOCAF5 __attribute__((address(0xF9FD)));


extern volatile __bit IOCAF6 __attribute__((address(0xF9FE)));


extern volatile __bit IOCAF7 __attribute__((address(0xF9FF)));


extern volatile __bit IOCAN0 __attribute__((address(0xF9F0)));


extern volatile __bit IOCAN1 __attribute__((address(0xF9F1)));


extern volatile __bit IOCAN2 __attribute__((address(0xF9F2)));


extern volatile __bit IOCAN3 __attribute__((address(0xF9F3)));


extern volatile __bit IOCAN4 __attribute__((address(0xF9F4)));


extern volatile __bit IOCAN5 __attribute__((address(0xF9F5)));


extern volatile __bit IOCAN6 __attribute__((address(0xF9F6)));


extern volatile __bit IOCAN7 __attribute__((address(0xF9F7)));


extern volatile __bit IOCAP0 __attribute__((address(0xF9E8)));


extern volatile __bit IOCAP1 __attribute__((address(0xF9E9)));


extern volatile __bit IOCAP2 __attribute__((address(0xF9EA)));


extern volatile __bit IOCAP3 __attribute__((address(0xF9EB)));


extern volatile __bit IOCAP4 __attribute__((address(0xF9EC)));


extern volatile __bit IOCAP5 __attribute__((address(0xF9ED)));


extern volatile __bit IOCAP6 __attribute__((address(0xF9EE)));


extern volatile __bit IOCAP7 __attribute__((address(0xF9EF)));


extern volatile __bit IOCBF0 __attribute__((address(0xFA50)));


extern volatile __bit IOCBF1 __attribute__((address(0xFA51)));


extern volatile __bit IOCBF2 __attribute__((address(0xFA52)));


extern volatile __bit IOCBF3 __attribute__((address(0xFA53)));


extern volatile __bit IOCBF4 __attribute__((address(0xFA54)));


extern volatile __bit IOCBF5 __attribute__((address(0xFA55)));


extern volatile __bit IOCBF6 __attribute__((address(0xFA56)));


extern volatile __bit IOCBF7 __attribute__((address(0xFA57)));


extern volatile __bit IOCBN0 __attribute__((address(0xFA48)));


extern volatile __bit IOCBN1 __attribute__((address(0xFA49)));


extern volatile __bit IOCBN2 __attribute__((address(0xFA4A)));


extern volatile __bit IOCBN3 __attribute__((address(0xFA4B)));


extern volatile __bit IOCBN4 __attribute__((address(0xFA4C)));


extern volatile __bit IOCBN5 __attribute__((address(0xFA4D)));


extern volatile __bit IOCBN6 __attribute__((address(0xFA4E)));


extern volatile __bit IOCBN7 __attribute__((address(0xFA4F)));


extern volatile __bit IOCBP0 __attribute__((address(0xFA40)));


extern volatile __bit IOCBP1 __attribute__((address(0xFA41)));


extern volatile __bit IOCBP2 __attribute__((address(0xFA42)));


extern volatile __bit IOCBP3 __attribute__((address(0xFA43)));


extern volatile __bit IOCBP4 __attribute__((address(0xFA44)));


extern volatile __bit IOCBP5 __attribute__((address(0xFA45)));


extern volatile __bit IOCBP6 __attribute__((address(0xFA46)));


extern volatile __bit IOCBP7 __attribute__((address(0xFA47)));


extern volatile __bit IOCCF0 __attribute__((address(0xFAA8)));


extern volatile __bit IOCCF1 __attribute__((address(0xFAA9)));


extern volatile __bit IOCCF2 __attribute__((address(0xFAAA)));


extern volatile __bit IOCCF3 __attribute__((address(0xFAAB)));


extern volatile __bit IOCCF4 __attribute__((address(0xFAAC)));


extern volatile __bit IOCCF6 __attribute__((address(0xFAAE)));


extern volatile __bit IOCCF7 __attribute__((address(0xFAAF)));


extern volatile __bit IOCCN0 __attribute__((address(0xFAA0)));


extern volatile __bit IOCCN1 __attribute__((address(0xFAA1)));


extern volatile __bit IOCCN2 __attribute__((address(0xFAA2)));


extern volatile __bit IOCCN3 __attribute__((address(0xFAA3)));


extern volatile __bit IOCCN4 __attribute__((address(0xFAA4)));


extern volatile __bit IOCCN6 __attribute__((address(0xFAA6)));


extern volatile __bit IOCCN7 __attribute__((address(0xFAA7)));


extern volatile __bit IOCCP0 __attribute__((address(0xFA98)));


extern volatile __bit IOCCP1 __attribute__((address(0xFA99)));


extern volatile __bit IOCCP2 __attribute__((address(0xFA9A)));


extern volatile __bit IOCCP3 __attribute__((address(0xFA9B)));


extern volatile __bit IOCCP4 __attribute__((address(0xFA9C)));


extern volatile __bit IOCCP6 __attribute__((address(0xFA9E)));


extern volatile __bit IOCCP7 __attribute__((address(0xFA9F)));


extern volatile __bit IOCEF3 __attribute__((address(0xFB5B)));


extern volatile __bit IOCEN3 __attribute__((address(0xFB53)));


extern volatile __bit IOCEP3 __attribute__((address(0xFB4B)));


extern volatile __bit IOCIE __attribute__((address(0x38B4)));


extern volatile __bit IOCIF __attribute__((address(0x3864)));


extern volatile __bit IOCMD __attribute__((address(0x3CB0)));


extern volatile __bit IPEN __attribute__((address(0x896)));


extern volatile __bit LATA0 __attribute__((address(0xC0)));


extern volatile __bit LATA1 __attribute__((address(0xC1)));


extern volatile __bit LATA2 __attribute__((address(0xC2)));


extern volatile __bit LATA3 __attribute__((address(0xC3)));


extern volatile __bit LATA4 __attribute__((address(0xC4)));


extern volatile __bit LATA6 __attribute__((address(0xC6)));


extern volatile __bit LATA7 __attribute__((address(0xC7)));


extern volatile __bit LATB0 __attribute__((address(0xC8)));


extern volatile __bit LATB1 __attribute__((address(0xC9)));


extern volatile __bit LATB2 __attribute__((address(0xCA)));


extern volatile __bit LATB3 __attribute__((address(0xCB)));


extern volatile __bit LATB4 __attribute__((address(0xCC)));


extern volatile __bit LATB5 __attribute__((address(0xCD)));


extern volatile __bit LATB6 __attribute__((address(0xCE)));


extern volatile __bit LATB7 __attribute__((address(0xCF)));


extern volatile __bit LATC0 __attribute__((address(0xD0)));


extern volatile __bit LATC1 __attribute__((address(0xD1)));


extern volatile __bit LATC2 __attribute__((address(0xD2)));


extern volatile __bit LATC3 __attribute__((address(0xD3)));


extern volatile __bit LATC4 __attribute__((address(0xD4)));


extern volatile __bit LATC6 __attribute__((address(0xD6)));


extern volatile __bit LATC7 __attribute__((address(0xD7)));


extern volatile __bit LC1D1S0 __attribute__((address(0xF090)));


extern volatile __bit LC1D1S1 __attribute__((address(0xF091)));


extern volatile __bit LC1D1S2 __attribute__((address(0xF092)));


extern volatile __bit LC1D1S3 __attribute__((address(0xF093)));


extern volatile __bit LC1D1S4 __attribute__((address(0xF094)));


extern volatile __bit LC1D1S5 __attribute__((address(0xF095)));


extern volatile __bit LC1D1S6 __attribute__((address(0xF096)));


extern volatile __bit LC1D1S7 __attribute__((address(0xF097)));


extern volatile __bit LC1D2S0 __attribute__((address(0xF098)));


extern volatile __bit LC1D2S1 __attribute__((address(0xF099)));


extern volatile __bit LC1D2S2 __attribute__((address(0xF09A)));


extern volatile __bit LC1D2S3 __attribute__((address(0xF09B)));


extern volatile __bit LC1D2S4 __attribute__((address(0xF09C)));


extern volatile __bit LC1D2S5 __attribute__((address(0xF09D)));


extern volatile __bit LC1D2S6 __attribute__((address(0xF09E)));


extern volatile __bit LC1D2S7 __attribute__((address(0xF09F)));


extern volatile __bit LC1D3S0 __attribute__((address(0xF0A0)));


extern volatile __bit LC1D3S1 __attribute__((address(0xF0A1)));


extern volatile __bit LC1D3S2 __attribute__((address(0xF0A2)));


extern volatile __bit LC1D3S3 __attribute__((address(0xF0A3)));


extern volatile __bit LC1D3S4 __attribute__((address(0xF0A4)));


extern volatile __bit LC1D3S5 __attribute__((address(0xF0A5)));


extern volatile __bit LC1D3S6 __attribute__((address(0xF0A6)));


extern volatile __bit LC1D3S7 __attribute__((address(0xF0A7)));


extern volatile __bit LC1D4S0 __attribute__((address(0xF0A8)));


extern volatile __bit LC1D4S1 __attribute__((address(0xF0A9)));


extern volatile __bit LC1D4S2 __attribute__((address(0xF0AA)));


extern volatile __bit LC1D4S3 __attribute__((address(0xF0AB)));


extern volatile __bit LC1D4S4 __attribute__((address(0xF0AC)));


extern volatile __bit LC1D4S5 __attribute__((address(0xF0AD)));


extern volatile __bit LC1D4S6 __attribute__((address(0xF0AE)));


extern volatile __bit LC1D4S7 __attribute__((address(0xF0AF)));


extern volatile __bit LC1EN __attribute__((address(0xF087)));


extern volatile __bit LC1G1D1N __attribute__((address(0xF0B0)));


extern volatile __bit LC1G1D1T __attribute__((address(0xF0B1)));


extern volatile __bit LC1G1D2N __attribute__((address(0xF0B2)));


extern volatile __bit LC1G1D2T __attribute__((address(0xF0B3)));


extern volatile __bit LC1G1D3N __attribute__((address(0xF0B4)));


extern volatile __bit LC1G1D3T __attribute__((address(0xF0B5)));


extern volatile __bit LC1G1D4N __attribute__((address(0xF0B6)));


extern volatile __bit LC1G1D4T __attribute__((address(0xF0B7)));


extern volatile __bit LC1G1POL __attribute__((address(0xF088)));


extern volatile __bit LC1G2D1N __attribute__((address(0xF0B8)));


extern volatile __bit LC1G2D1T __attribute__((address(0xF0B9)));


extern volatile __bit LC1G2D2N __attribute__((address(0xF0BA)));


extern volatile __bit LC1G2D2T __attribute__((address(0xF0BB)));


extern volatile __bit LC1G2D3N __attribute__((address(0xF0BC)));


extern volatile __bit LC1G2D3T __attribute__((address(0xF0BD)));


extern volatile __bit LC1G2D4N __attribute__((address(0xF0BE)));


extern volatile __bit LC1G2D4T __attribute__((address(0xF0BF)));


extern volatile __bit LC1G2POL __attribute__((address(0xF089)));


extern volatile __bit LC1G3D1N __attribute__((address(0xF0C0)));


extern volatile __bit LC1G3D1T __attribute__((address(0xF0C1)));


extern volatile __bit LC1G3D2N __attribute__((address(0xF0C2)));


extern volatile __bit LC1G3D2T __attribute__((address(0xF0C3)));


extern volatile __bit LC1G3D3N __attribute__((address(0xF0C4)));


extern volatile __bit LC1G3D3T __attribute__((address(0xF0C5)));


extern volatile __bit LC1G3D4N __attribute__((address(0xF0C6)));


extern volatile __bit LC1G3D4T __attribute__((address(0xF0C7)));


extern volatile __bit LC1G3POL __attribute__((address(0xF08A)));


extern volatile __bit LC1G4D1N __attribute__((address(0xF0C8)));


extern volatile __bit LC1G4D1T __attribute__((address(0xF0C9)));


extern volatile __bit LC1G4D2N __attribute__((address(0xF0CA)));


extern volatile __bit LC1G4D2T __attribute__((address(0xF0CB)));


extern volatile __bit LC1G4D3N __attribute__((address(0xF0CC)));


extern volatile __bit LC1G4D3T __attribute__((address(0xF0CD)));


extern volatile __bit LC1G4D4N __attribute__((address(0xF0CE)));


extern volatile __bit LC1G4D4T __attribute__((address(0xF0CF)));


extern volatile __bit LC1G4POL __attribute__((address(0xF08B)));


extern volatile __bit LC1INTN __attribute__((address(0xF083)));


extern volatile __bit LC1INTP __attribute__((address(0xF084)));


extern volatile __bit LC1MODE0 __attribute__((address(0xF080)));


extern volatile __bit LC1MODE1 __attribute__((address(0xF081)));


extern volatile __bit LC1MODE2 __attribute__((address(0xF082)));


extern volatile __bit LC1OUT __attribute__((address(0xF085)));


extern volatile __bit LC1POL __attribute__((address(0xF08F)));


extern volatile __bit LC2D1S0 __attribute__((address(0xF0E0)));


extern volatile __bit LC2D1S1 __attribute__((address(0xF0E1)));


extern volatile __bit LC2D1S2 __attribute__((address(0xF0E2)));


extern volatile __bit LC2D1S3 __attribute__((address(0xF0E3)));


extern volatile __bit LC2D1S4 __attribute__((address(0xF0E4)));


extern volatile __bit LC2D1S5 __attribute__((address(0xF0E5)));


extern volatile __bit LC2D1S6 __attribute__((address(0xF0E6)));


extern volatile __bit LC2D1S7 __attribute__((address(0xF0E7)));


extern volatile __bit LC2D2S0 __attribute__((address(0xF0E8)));


extern volatile __bit LC2D2S1 __attribute__((address(0xF0E9)));


extern volatile __bit LC2D2S2 __attribute__((address(0xF0EA)));


extern volatile __bit LC2D2S3 __attribute__((address(0xF0EB)));


extern volatile __bit LC2D2S4 __attribute__((address(0xF0EC)));


extern volatile __bit LC2D2S5 __attribute__((address(0xF0ED)));


extern volatile __bit LC2D2S6 __attribute__((address(0xF0EE)));


extern volatile __bit LC2D2S7 __attribute__((address(0xF0EF)));


extern volatile __bit LC2D3S0 __attribute__((address(0xF0F0)));


extern volatile __bit LC2D3S1 __attribute__((address(0xF0F1)));


extern volatile __bit LC2D3S2 __attribute__((address(0xF0F2)));


extern volatile __bit LC2D3S3 __attribute__((address(0xF0F3)));


extern volatile __bit LC2D3S4 __attribute__((address(0xF0F4)));


extern volatile __bit LC2D3S5 __attribute__((address(0xF0F5)));


extern volatile __bit LC2D3S6 __attribute__((address(0xF0F6)));


extern volatile __bit LC2D3S7 __attribute__((address(0xF0F7)));


extern volatile __bit LC2D4S0 __attribute__((address(0xF0F8)));


extern volatile __bit LC2D4S1 __attribute__((address(0xF0F9)));


extern volatile __bit LC2D4S2 __attribute__((address(0xF0FA)));


extern volatile __bit LC2D4S3 __attribute__((address(0xF0FB)));


extern volatile __bit LC2D4S4 __attribute__((address(0xF0FC)));


extern volatile __bit LC2D4S5 __attribute__((address(0xF0FD)));


extern volatile __bit LC2D4S6 __attribute__((address(0xF0FE)));


extern volatile __bit LC2D4S7 __attribute__((address(0xF0FF)));


extern volatile __bit LC2EN __attribute__((address(0xF0D7)));


extern volatile __bit LC2G1D1N __attribute__((address(0xF100)));


extern volatile __bit LC2G1D1T __attribute__((address(0xF101)));


extern volatile __bit LC2G1D2N __attribute__((address(0xF102)));


extern volatile __bit LC2G1D2T __attribute__((address(0xF103)));


extern volatile __bit LC2G1D3N __attribute__((address(0xF104)));


extern volatile __bit LC2G1D3T __attribute__((address(0xF105)));


extern volatile __bit LC2G1D4N __attribute__((address(0xF106)));


extern volatile __bit LC2G1D4T __attribute__((address(0xF107)));


extern volatile __bit LC2G1POL __attribute__((address(0xF0D8)));


extern volatile __bit LC2G2D1N __attribute__((address(0xF108)));


extern volatile __bit LC2G2D1T __attribute__((address(0xF109)));


extern volatile __bit LC2G2D2N __attribute__((address(0xF10A)));


extern volatile __bit LC2G2D2T __attribute__((address(0xF10B)));


extern volatile __bit LC2G2D3N __attribute__((address(0xF10C)));


extern volatile __bit LC2G2D3T __attribute__((address(0xF10D)));


extern volatile __bit LC2G2D4N __attribute__((address(0xF10E)));


extern volatile __bit LC2G2D4T __attribute__((address(0xF10F)));


extern volatile __bit LC2G2POL __attribute__((address(0xF0D9)));


extern volatile __bit LC2G3D1N __attribute__((address(0xF110)));


extern volatile __bit LC2G3D1T __attribute__((address(0xF111)));


extern volatile __bit LC2G3D2N __attribute__((address(0xF112)));


extern volatile __bit LC2G3D2T __attribute__((address(0xF113)));


extern volatile __bit LC2G3D3N __attribute__((address(0xF114)));


extern volatile __bit LC2G3D3T __attribute__((address(0xF115)));


extern volatile __bit LC2G3D4N __attribute__((address(0xF116)));


extern volatile __bit LC2G3D4T __attribute__((address(0xF117)));


extern volatile __bit LC2G3POL __attribute__((address(0xF0DA)));


extern volatile __bit LC2G4D1N __attribute__((address(0xF118)));


extern volatile __bit LC2G4D1T __attribute__((address(0xF119)));


extern volatile __bit LC2G4D2N __attribute__((address(0xF11A)));


extern volatile __bit LC2G4D2T __attribute__((address(0xF11B)));


extern volatile __bit LC2G4D3N __attribute__((address(0xF11C)));


extern volatile __bit LC2G4D3T __attribute__((address(0xF11D)));


extern volatile __bit LC2G4D4N __attribute__((address(0xF11E)));


extern volatile __bit LC2G4D4T __attribute__((address(0xF11F)));


extern volatile __bit LC2G4POL __attribute__((address(0xF0DB)));


extern volatile __bit LC2INTN __attribute__((address(0xF0D3)));


extern volatile __bit LC2INTP __attribute__((address(0xF0D4)));


extern volatile __bit LC2MODE0 __attribute__((address(0xF0D0)));


extern volatile __bit LC2MODE1 __attribute__((address(0xF0D1)));


extern volatile __bit LC2MODE2 __attribute__((address(0xF0D2)));


extern volatile __bit LC2OUT __attribute__((address(0xF0D5)));


extern volatile __bit LC2POL __attribute__((address(0xF0DF)));


extern volatile __bit LC3D1S0 __attribute__((address(0xF130)));


extern volatile __bit LC3D1S1 __attribute__((address(0xF131)));


extern volatile __bit LC3D1S2 __attribute__((address(0xF132)));


extern volatile __bit LC3D1S3 __attribute__((address(0xF133)));


extern volatile __bit LC3D1S4 __attribute__((address(0xF134)));


extern volatile __bit LC3D1S5 __attribute__((address(0xF135)));


extern volatile __bit LC3D1S6 __attribute__((address(0xF136)));


extern volatile __bit LC3D1S7 __attribute__((address(0xF137)));


extern volatile __bit LC3D2S0 __attribute__((address(0xF138)));


extern volatile __bit LC3D2S1 __attribute__((address(0xF139)));


extern volatile __bit LC3D2S2 __attribute__((address(0xF13A)));


extern volatile __bit LC3D2S3 __attribute__((address(0xF13B)));


extern volatile __bit LC3D2S4 __attribute__((address(0xF13C)));


extern volatile __bit LC3D2S5 __attribute__((address(0xF13D)));


extern volatile __bit LC3D2S6 __attribute__((address(0xF13E)));


extern volatile __bit LC3D2S7 __attribute__((address(0xF13F)));


extern volatile __bit LC3D3S0 __attribute__((address(0xF140)));


extern volatile __bit LC3D3S1 __attribute__((address(0xF141)));


extern volatile __bit LC3D3S2 __attribute__((address(0xF142)));


extern volatile __bit LC3D3S3 __attribute__((address(0xF143)));


extern volatile __bit LC3D3S4 __attribute__((address(0xF144)));


extern volatile __bit LC3D3S5 __attribute__((address(0xF145)));


extern volatile __bit LC3D3S6 __attribute__((address(0xF146)));


extern volatile __bit LC3D3S7 __attribute__((address(0xF147)));


extern volatile __bit LC3D4S0 __attribute__((address(0xF148)));


extern volatile __bit LC3D4S1 __attribute__((address(0xF149)));


extern volatile __bit LC3D4S2 __attribute__((address(0xF14A)));


extern volatile __bit LC3D4S3 __attribute__((address(0xF14B)));


extern volatile __bit LC3D4S4 __attribute__((address(0xF14C)));


extern volatile __bit LC3D4S5 __attribute__((address(0xF14D)));


extern volatile __bit LC3D4S6 __attribute__((address(0xF14E)));


extern volatile __bit LC3D4S7 __attribute__((address(0xF14F)));


extern volatile __bit LC3EN __attribute__((address(0xF127)));


extern volatile __bit LC3G1D1N __attribute__((address(0xF150)));


extern volatile __bit LC3G1D1T __attribute__((address(0xF151)));


extern volatile __bit LC3G1D2N __attribute__((address(0xF152)));


extern volatile __bit LC3G1D2T __attribute__((address(0xF153)));


extern volatile __bit LC3G1D3N __attribute__((address(0xF154)));


extern volatile __bit LC3G1D3T __attribute__((address(0xF155)));


extern volatile __bit LC3G1D4N __attribute__((address(0xF156)));


extern volatile __bit LC3G1D4T __attribute__((address(0xF157)));


extern volatile __bit LC3G1POL __attribute__((address(0xF128)));


extern volatile __bit LC3G2D1N __attribute__((address(0xF158)));


extern volatile __bit LC3G2D1T __attribute__((address(0xF159)));


extern volatile __bit LC3G2D2N __attribute__((address(0xF15A)));


extern volatile __bit LC3G2D2T __attribute__((address(0xF15B)));


extern volatile __bit LC3G2D3N __attribute__((address(0xF15C)));


extern volatile __bit LC3G2D3T __attribute__((address(0xF15D)));


extern volatile __bit LC3G2D4N __attribute__((address(0xF15E)));


extern volatile __bit LC3G2D4T __attribute__((address(0xF15F)));


extern volatile __bit LC3G2POL __attribute__((address(0xF129)));


extern volatile __bit LC3G3D1N __attribute__((address(0xF160)));


extern volatile __bit LC3G3D1T __attribute__((address(0xF161)));


extern volatile __bit LC3G3D2N __attribute__((address(0xF162)));


extern volatile __bit LC3G3D2T __attribute__((address(0xF163)));


extern volatile __bit LC3G3D3N __attribute__((address(0xF164)));


extern volatile __bit LC3G3D3T __attribute__((address(0xF165)));


extern volatile __bit LC3G3D4N __attribute__((address(0xF166)));


extern volatile __bit LC3G3D4T __attribute__((address(0xF167)));


extern volatile __bit LC3G3POL __attribute__((address(0xF12A)));


extern volatile __bit LC3G4D1N __attribute__((address(0xF168)));


extern volatile __bit LC3G4D1T __attribute__((address(0xF169)));


extern volatile __bit LC3G4D2N __attribute__((address(0xF16A)));


extern volatile __bit LC3G4D2T __attribute__((address(0xF16B)));


extern volatile __bit LC3G4D3N __attribute__((address(0xF16C)));


extern volatile __bit LC3G4D3T __attribute__((address(0xF16D)));


extern volatile __bit LC3G4D4N __attribute__((address(0xF16E)));


extern volatile __bit LC3G4D4T __attribute__((address(0xF16F)));


extern volatile __bit LC3G4POL __attribute__((address(0xF12B)));


extern volatile __bit LC3INTN __attribute__((address(0xF123)));


extern volatile __bit LC3INTP __attribute__((address(0xF124)));


extern volatile __bit LC3MODE0 __attribute__((address(0xF120)));


extern volatile __bit LC3MODE1 __attribute__((address(0xF121)));


extern volatile __bit LC3MODE2 __attribute__((address(0xF122)));


extern volatile __bit LC3OUT __attribute__((address(0xF125)));


extern volatile __bit LC3POL __attribute__((address(0xF12F)));


extern volatile __bit LC4D1S0 __attribute__((address(0xF180)));


extern volatile __bit LC4D1S1 __attribute__((address(0xF181)));


extern volatile __bit LC4D1S2 __attribute__((address(0xF182)));


extern volatile __bit LC4D1S3 __attribute__((address(0xF183)));


extern volatile __bit LC4D1S4 __attribute__((address(0xF184)));


extern volatile __bit LC4D1S5 __attribute__((address(0xF185)));


extern volatile __bit LC4D1S6 __attribute__((address(0xF186)));


extern volatile __bit LC4D1S7 __attribute__((address(0xF187)));


extern volatile __bit LC4D2S0 __attribute__((address(0xF188)));


extern volatile __bit LC4D2S1 __attribute__((address(0xF189)));


extern volatile __bit LC4D2S2 __attribute__((address(0xF18A)));


extern volatile __bit LC4D2S3 __attribute__((address(0xF18B)));


extern volatile __bit LC4D2S4 __attribute__((address(0xF18C)));


extern volatile __bit LC4D2S5 __attribute__((address(0xF18D)));


extern volatile __bit LC4D2S6 __attribute__((address(0xF18E)));


extern volatile __bit LC4D2S7 __attribute__((address(0xF18F)));


extern volatile __bit LC4D3S0 __attribute__((address(0xF190)));


extern volatile __bit LC4D3S1 __attribute__((address(0xF191)));


extern volatile __bit LC4D3S2 __attribute__((address(0xF192)));


extern volatile __bit LC4D3S3 __attribute__((address(0xF193)));


extern volatile __bit LC4D3S4 __attribute__((address(0xF194)));


extern volatile __bit LC4D3S5 __attribute__((address(0xF195)));


extern volatile __bit LC4D3S6 __attribute__((address(0xF196)));


extern volatile __bit LC4D3S7 __attribute__((address(0xF197)));


extern volatile __bit LC4D4S0 __attribute__((address(0xF198)));


extern volatile __bit LC4D4S1 __attribute__((address(0xF199)));


extern volatile __bit LC4D4S2 __attribute__((address(0xF19A)));


extern volatile __bit LC4D4S3 __attribute__((address(0xF19B)));


extern volatile __bit LC4D4S4 __attribute__((address(0xF19C)));


extern volatile __bit LC4D4S5 __attribute__((address(0xF19D)));


extern volatile __bit LC4D4S6 __attribute__((address(0xF19E)));


extern volatile __bit LC4D4S7 __attribute__((address(0xF19F)));


extern volatile __bit LC4EN __attribute__((address(0xF177)));


extern volatile __bit LC4G1D1N __attribute__((address(0xF1A0)));


extern volatile __bit LC4G1D1T __attribute__((address(0xF1A1)));


extern volatile __bit LC4G1D2N __attribute__((address(0xF1A2)));


extern volatile __bit LC4G1D2T __attribute__((address(0xF1A3)));


extern volatile __bit LC4G1D3N __attribute__((address(0xF1A4)));


extern volatile __bit LC4G1D3T __attribute__((address(0xF1A5)));


extern volatile __bit LC4G1D4N __attribute__((address(0xF1A6)));


extern volatile __bit LC4G1D4T __attribute__((address(0xF1A7)));


extern volatile __bit LC4G1POL __attribute__((address(0xF178)));


extern volatile __bit LC4G2D1N __attribute__((address(0xF1A8)));


extern volatile __bit LC4G2D1T __attribute__((address(0xF1A9)));


extern volatile __bit LC4G2D2N __attribute__((address(0xF1AA)));


extern volatile __bit LC4G2D2T __attribute__((address(0xF1AB)));


extern volatile __bit LC4G2D3N __attribute__((address(0xF1AC)));


extern volatile __bit LC4G2D3T __attribute__((address(0xF1AD)));


extern volatile __bit LC4G2D4N __attribute__((address(0xF1AE)));


extern volatile __bit LC4G2D4T __attribute__((address(0xF1AF)));


extern volatile __bit LC4G2POL __attribute__((address(0xF179)));


extern volatile __bit LC4G3D1N __attribute__((address(0xF1B0)));


extern volatile __bit LC4G3D1T __attribute__((address(0xF1B1)));


extern volatile __bit LC4G3D2N __attribute__((address(0xF1B2)));


extern volatile __bit LC4G3D2T __attribute__((address(0xF1B3)));


extern volatile __bit LC4G3D3N __attribute__((address(0xF1B4)));


extern volatile __bit LC4G3D3T __attribute__((address(0xF1B5)));


extern volatile __bit LC4G3D4N __attribute__((address(0xF1B6)));


extern volatile __bit LC4G3D4T __attribute__((address(0xF1B7)));


extern volatile __bit LC4G3POL __attribute__((address(0xF17A)));


extern volatile __bit LC4G4D1N __attribute__((address(0xF1B8)));


extern volatile __bit LC4G4D1T __attribute__((address(0xF1B9)));


extern volatile __bit LC4G4D2N __attribute__((address(0xF1BA)));


extern volatile __bit LC4G4D2T __attribute__((address(0xF1BB)));


extern volatile __bit LC4G4D3N __attribute__((address(0xF1BC)));


extern volatile __bit LC4G4D3T __attribute__((address(0xF1BD)));


extern volatile __bit LC4G4D4N __attribute__((address(0xF1BE)));


extern volatile __bit LC4G4D4T __attribute__((address(0xF1BF)));


extern volatile __bit LC4G4POL __attribute__((address(0xF17B)));


extern volatile __bit LC4INTN __attribute__((address(0xF173)));


extern volatile __bit LC4INTP __attribute__((address(0xF174)));


extern volatile __bit LC4MODE0 __attribute__((address(0xF170)));


extern volatile __bit LC4MODE1 __attribute__((address(0xF171)));


extern volatile __bit LC4MODE2 __attribute__((address(0xF172)));


extern volatile __bit LC4OUT __attribute__((address(0xF175)));


extern volatile __bit LC4POL __attribute__((address(0xF17F)));


extern volatile __bit LCDA __attribute__((address(0xE86D)));


extern volatile __bit LCDCST0 __attribute__((address(0xE8B0)));


extern volatile __bit LCDCST1 __attribute__((address(0xE8B1)));


extern volatile __bit LCDCST2 __attribute__((address(0xE8B2)));


extern volatile __bit LCDEN __attribute__((address(0xE867)));


extern volatile __bit LCDIE __attribute__((address(0x38F7)));


extern volatile __bit LCDIF __attribute__((address(0x38A7)));


extern volatile __bit LCDIRI __attribute__((address(0xE8BB)));


extern volatile __bit LCDMD __attribute__((address(0x3CDD)));


extern volatile __bit LCDVSRC0 __attribute__((address(0xE8A8)));


extern volatile __bit LCDVSRC1 __attribute__((address(0xE8A9)));


extern volatile __bit LCDVSRC2 __attribute__((address(0xE8AA)));


extern volatile __bit LCDVSRC3 __attribute__((address(0xE8AB)));


extern volatile __bit LD __attribute__((address(0x3086)));


extern volatile __bit LFOEN __attribute__((address(0x448C)));


extern volatile __bit LFOR __attribute__((address(0x4484)));


extern volatile __bit LMUX0 __attribute__((address(0xE860)));


extern volatile __bit LMUX1 __attribute__((address(0xE861)));


extern volatile __bit LMUX2 __attribute__((address(0xE862)));


extern volatile __bit LMUX3 __attribute__((address(0xE863)));


extern volatile __bit LP0 __attribute__((address(0xE868)));


extern volatile __bit LP1 __attribute__((address(0xE869)));


extern volatile __bit LP2 __attribute__((address(0xE86A)));


extern volatile __bit LP3 __attribute__((address(0xE86B)));


extern volatile __bit LPEN __attribute__((address(0xE8A7)));


extern volatile __bit LRLAP0 __attribute__((address(0xE8BE)));


extern volatile __bit LRLAP1 __attribute__((address(0xE8BF)));


extern volatile __bit LRLAT0 __attribute__((address(0xE8B8)));


extern volatile __bit LRLAT1 __attribute__((address(0xE8B9)));


extern volatile __bit LRLAT2 __attribute__((address(0xE8BA)));


extern volatile __bit LRLBP0 __attribute__((address(0xE8BC)));


extern volatile __bit LRLBP1 __attribute__((address(0xE8BD)));


extern volatile __bit LSAC0 __attribute__((address(0x3092)));


extern volatile __bit LSAC1 __attribute__((address(0x3093)));


extern volatile __bit LSBD0 __attribute__((address(0x3094)));


extern volatile __bit LSBD1 __attribute__((address(0x3095)));


extern volatile __bit LTH0 __attribute__((address(0x460)));


extern volatile __bit LTH1 __attribute__((address(0x461)));


extern volatile __bit LTH10 __attribute__((address(0x46A)));


extern volatile __bit LTH11 __attribute__((address(0x46B)));


extern volatile __bit LTH12 __attribute__((address(0x46C)));


extern volatile __bit LTH13 __attribute__((address(0x46D)));


extern volatile __bit LTH14 __attribute__((address(0x46E)));


extern volatile __bit LTH15 __attribute__((address(0x46F)));


extern volatile __bit LTH2 __attribute__((address(0x462)));


extern volatile __bit LTH3 __attribute__((address(0x463)));


extern volatile __bit LTH4 __attribute__((address(0x464)));


extern volatile __bit LTH5 __attribute__((address(0x465)));


extern volatile __bit LTH6 __attribute__((address(0x466)));


extern volatile __bit LTH7 __attribute__((address(0x467)));


extern volatile __bit LTH8 __attribute__((address(0x468)));


extern volatile __bit LTH9 __attribute__((address(0x469)));


extern volatile __bit LTHR __attribute__((address(0x8AD)));


extern volatile __bit LWLO __attribute__((address(0x40F5)));


extern volatile __bit MATH __attribute__((address(0x8AC)));


extern volatile __bit MC1OUT __attribute__((address(0x4C78)));


extern volatile __bit MC2OUT __attribute__((address(0x4C79)));


extern volatile __bit MD0 __attribute__((address(0x898)));


extern volatile __bit MD1 __attribute__((address(0x899)));


extern volatile __bit MD2 __attribute__((address(0x89A)));


extern volatile __bit MEMV __attribute__((address(0x40A1)));


extern volatile __bit MFOEN __attribute__((address(0x448D)));


extern volatile __bit MFOR __attribute__((address(0x4485)));


extern volatile __bit MINALRMH0 __attribute__((address(0x60DC)));


extern volatile __bit MINALRMH1 __attribute__((address(0x60DD)));


extern volatile __bit MINALRMH2 __attribute__((address(0x60DE)));


extern volatile __bit MINALRML0 __attribute__((address(0x60D8)));


extern volatile __bit MINALRML1 __attribute__((address(0x60D9)));


extern volatile __bit MINALRML2 __attribute__((address(0x60DA)));


extern volatile __bit MINALRML3 __attribute__((address(0x60DB)));


extern volatile __bit MINH0 __attribute__((address(0x60AC)));


extern volatile __bit MINH1 __attribute__((address(0x60AD)));


extern volatile __bit MINH2 __attribute__((address(0x60AE)));


extern volatile __bit MINL0 __attribute__((address(0x60A8)));


extern volatile __bit MINL1 __attribute__((address(0x60A9)));


extern volatile __bit MINL2 __attribute__((address(0x60AA)));


extern volatile __bit MINL3 __attribute__((address(0x60AB)));


extern volatile __bit MLC1OUT __attribute__((address(0xF078)));


extern volatile __bit MLC2OUT __attribute__((address(0xF079)));


extern volatile __bit MLC3OUT __attribute__((address(0xF07A)));


extern volatile __bit MLC4OUT __attribute__((address(0xF07B)));


extern volatile __bit MONTHH __attribute__((address(0x608C)));


extern volatile __bit MONTHL0 __attribute__((address(0x6088)));


extern volatile __bit MONTHL1 __attribute__((address(0x6089)));


extern volatile __bit MONTHL2 __attribute__((address(0x608A)));


extern volatile __bit MONTHL3 __attribute__((address(0x608B)));


extern volatile __bit MSK01 __attribute__((address(0xC68)));


extern volatile __bit MSK11 __attribute__((address(0xC69)));


extern volatile __bit MSK21 __attribute__((address(0xC6A)));


extern volatile __bit MSK31 __attribute__((address(0xC6B)));


extern volatile __bit MSK41 __attribute__((address(0xC6C)));


extern volatile __bit MSK51 __attribute__((address(0xC6D)));


extern volatile __bit MSK61 __attribute__((address(0xC6E)));


extern volatile __bit MSK71 __attribute__((address(0xC6F)));


extern volatile __bit MSSP1MD __attribute__((address(0x3CD4)));


extern volatile __bit MTHALRMH0 __attribute__((address(0x60BC)));


extern volatile __bit MTHALRML0 __attribute__((address(0x60B8)));


extern volatile __bit MTHALRML1 __attribute__((address(0x60B9)));


extern volatile __bit MTHALRML2 __attribute__((address(0x60BA)));


extern volatile __bit MTHALRML3 __attribute__((address(0x60BB)));


extern volatile __bit NDIV0 __attribute__((address(0x4468)));


extern volatile __bit NDIV1 __attribute__((address(0x4469)));


extern volatile __bit NDIV2 __attribute__((address(0x446A)));


extern volatile __bit NDIV3 __attribute__((address(0x446B)));


extern volatile __bit NOSC0 __attribute__((address(0x446C)));


extern volatile __bit NOSC1 __attribute__((address(0x446D)));


extern volatile __bit NOSC2 __attribute__((address(0x446E)));


extern volatile __bit NOSCR __attribute__((address(0x447B)));


extern volatile __bit NSS __attribute__((address(0x4870)));


extern volatile __bit NVMADR0 __attribute__((address(0x40D0)));


extern volatile __bit NVMADR1 __attribute__((address(0x40D1)));


extern volatile __bit NVMADR10 __attribute__((address(0x40DA)));


extern volatile __bit NVMADR11 __attribute__((address(0x40DB)));


extern volatile __bit NVMADR12 __attribute__((address(0x40DC)));


extern volatile __bit NVMADR13 __attribute__((address(0x40DD)));


extern volatile __bit NVMADR14 __attribute__((address(0x40DE)));


extern volatile __bit NVMADR2 __attribute__((address(0x40D2)));


extern volatile __bit NVMADR3 __attribute__((address(0x40D3)));


extern volatile __bit NVMADR4 __attribute__((address(0x40D4)));


extern volatile __bit NVMADR5 __attribute__((address(0x40D5)));


extern volatile __bit NVMADR6 __attribute__((address(0x40D6)));


extern volatile __bit NVMADR7 __attribute__((address(0x40D7)));


extern volatile __bit NVMADR8 __attribute__((address(0x40D8)));


extern volatile __bit NVMADR9 __attribute__((address(0x40D9)));


extern volatile __bit NVMDAT0 __attribute__((address(0x40E0)));


extern volatile __bit NVMDAT1 __attribute__((address(0x40E1)));


extern volatile __bit NVMDAT10 __attribute__((address(0x40EA)));


extern volatile __bit NVMDAT11 __attribute__((address(0x40EB)));


extern volatile __bit NVMDAT12 __attribute__((address(0x40EC)));


extern volatile __bit NVMDAT13 __attribute__((address(0x40ED)));


extern volatile __bit NVMDAT2 __attribute__((address(0x40E2)));


extern volatile __bit NVMDAT3 __attribute__((address(0x40E3)));


extern volatile __bit NVMDAT4 __attribute__((address(0x40E4)));


extern volatile __bit NVMDAT5 __attribute__((address(0x40E5)));


extern volatile __bit NVMDAT6 __attribute__((address(0x40E6)));


extern volatile __bit NVMDAT7 __attribute__((address(0x40E7)));


extern volatile __bit NVMDAT8 __attribute__((address(0x40E8)));


extern volatile __bit NVMDAT9 __attribute__((address(0x40E9)));


extern volatile __bit NVMIE __attribute__((address(0x38ED)));


extern volatile __bit NVMIF __attribute__((address(0x389D)));


extern volatile __bit NVMMD __attribute__((address(0x3CB2)));


extern volatile __bit NVMREGS __attribute__((address(0x40F6)));


extern volatile __bit ODCA0 __attribute__((address(0xF9D0)));


extern volatile __bit ODCA1 __attribute__((address(0xF9D1)));


extern volatile __bit ODCA2 __attribute__((address(0xF9D2)));


extern volatile __bit ODCA3 __attribute__((address(0xF9D3)));


extern volatile __bit ODCA4 __attribute__((address(0xF9D4)));


extern volatile __bit ODCA6 __attribute__((address(0xF9D6)));


extern volatile __bit ODCA7 __attribute__((address(0xF9D7)));


extern volatile __bit ODCB0 __attribute__((address(0xFA28)));


extern volatile __bit ODCB1 __attribute__((address(0xFA29)));


extern volatile __bit ODCB2 __attribute__((address(0xFA2A)));


extern volatile __bit ODCB3 __attribute__((address(0xFA2B)));


extern volatile __bit ODCB4 __attribute__((address(0xFA2C)));


extern volatile __bit ODCB5 __attribute__((address(0xFA2D)));


extern volatile __bit ODCB6 __attribute__((address(0xFA2E)));


extern volatile __bit ODCB7 __attribute__((address(0xFA2F)));


extern volatile __bit ODCC0 __attribute__((address(0xFA80)));


extern volatile __bit ODCC1 __attribute__((address(0xFA81)));


extern volatile __bit ODCC2 __attribute__((address(0xFA82)));


extern volatile __bit ODCC3 __attribute__((address(0xFA83)));


extern volatile __bit ODCC4 __attribute__((address(0xFA84)));


extern volatile __bit ODCC6 __attribute__((address(0xFA86)));


extern volatile __bit ODCC7 __attribute__((address(0xFA87)));


extern volatile __bit OE1 __attribute__((address(0x4875)));


extern volatile __bit OE2 __attribute__((address(0x4874)));


extern volatile __bit ORDY __attribute__((address(0x447C)));


extern volatile __bit OSFIE __attribute__((address(0x38BF)));


extern volatile __bit OSFIF __attribute__((address(0x386F)));


extern volatile __bit OV __attribute__((address(0x8AF)));


extern volatile __bit OVRA __attribute__((address(0x30A4)));


extern volatile __bit OVRB __attribute__((address(0x30A5)));


extern volatile __bit OVRC __attribute__((address(0x30A6)));


extern volatile __bit OVRD __attribute__((address(0x30A7)));


extern volatile __bit P1M0 __attribute__((address(0x1876)));


extern volatile __bit P1M1 __attribute__((address(0x1877)));


extern volatile __bit P2 __attribute__((address(0xC7C)));


extern volatile __bit P2M0 __attribute__((address(0x1896)));


extern volatile __bit P2M1 __attribute__((address(0x1897)));


extern volatile __bit PCIE __attribute__((address(0xC96)));


extern volatile __bit PEIE __attribute__((address(0x5E)));


extern volatile __bit PEN __attribute__((address(0xC8A)));


extern volatile __bit PEN1 __attribute__((address(0xC8A)));


extern volatile __bit PLLR __attribute__((address(0x4480)));


extern volatile __bit POLA __attribute__((address(0x3088)));


extern volatile __bit POLB __attribute__((address(0x3089)));


extern volatile __bit POLC __attribute__((address(0x308A)));


extern volatile __bit POLD __attribute__((address(0x308B)));


extern volatile __bit PPOL __attribute__((address(0x897)));


extern volatile __bit PPSLOCKED __attribute__((address(0xF478)));


extern volatile __bit PRE0 __attribute__((address(0x878)));


extern volatile __bit PRE1 __attribute__((address(0x879)));


extern volatile __bit PRE10 __attribute__((address(0x882)));


extern volatile __bit PRE11 __attribute__((address(0x883)));


extern volatile __bit PRE12 __attribute__((address(0x884)));


extern volatile __bit PRE2 __attribute__((address(0x87A)));


extern volatile __bit PRE3 __attribute__((address(0x87B)));


extern volatile __bit PRE4 __attribute__((address(0x87C)));


extern volatile __bit PRE5 __attribute__((address(0x87D)));


extern volatile __bit PRE6 __attribute__((address(0x87E)));


extern volatile __bit PRE7 __attribute__((address(0x87F)));


extern volatile __bit PRE8 __attribute__((address(0x880)));


extern volatile __bit PRE9 __attribute__((address(0x881)));


extern volatile __bit PREF0 __attribute__((address(0x8B0)));


extern volatile __bit PREF1 __attribute__((address(0x8B1)));


extern volatile __bit PREV0 __attribute__((address(0x4D8)));


extern volatile __bit PREV1 __attribute__((address(0x4D9)));


extern volatile __bit PREV10 __attribute__((address(0x4E2)));


extern volatile __bit PREV11 __attribute__((address(0x4E3)));


extern volatile __bit PREV12 __attribute__((address(0x4E4)));


extern volatile __bit PREV13 __attribute__((address(0x4E5)));


extern volatile __bit PREV14 __attribute__((address(0x4E6)));


extern volatile __bit PREV15 __attribute__((address(0x4E7)));


extern volatile __bit PREV2 __attribute__((address(0x4DA)));


extern volatile __bit PREV3 __attribute__((address(0x4DB)));


extern volatile __bit PREV4 __attribute__((address(0x4DC)));


extern volatile __bit PREV5 __attribute__((address(0x4DD)));


extern volatile __bit PREV6 __attribute__((address(0x4DE)));


extern volatile __bit PREV7 __attribute__((address(0x4DF)));


extern volatile __bit PREV8 __attribute__((address(0x4E0)));


extern volatile __bit PREV9 __attribute__((address(0x4E1)));


extern volatile __bit PSCNT0 __attribute__((address(0x4070)));


extern volatile __bit PSCNT1 __attribute__((address(0x4071)));


extern volatile __bit PSCNT10 __attribute__((address(0x407A)));


extern volatile __bit PSCNT11 __attribute__((address(0x407B)));


extern volatile __bit PSCNT12 __attribute__((address(0x407C)));


extern volatile __bit PSCNT13 __attribute__((address(0x407D)));


extern volatile __bit PSCNT14 __attribute__((address(0x407E)));


extern volatile __bit PSCNT15 __attribute__((address(0x407F)));


extern volatile __bit PSCNT16 __attribute__((address(0x4080)));


extern volatile __bit PSCNT17 __attribute__((address(0x4081)));


extern volatile __bit PSCNT2 __attribute__((address(0x4072)));


extern volatile __bit PSCNT3 __attribute__((address(0x4073)));


extern volatile __bit PSCNT4 __attribute__((address(0x4074)));


extern volatile __bit PSCNT5 __attribute__((address(0x4075)));


extern volatile __bit PSCNT6 __attribute__((address(0x4076)));


extern volatile __bit PSCNT7 __attribute__((address(0x4077)));


extern volatile __bit PSCNT8 __attribute__((address(0x4078)));


extern volatile __bit PSCNT9 __attribute__((address(0x4079)));


extern volatile __bit PSIS __attribute__((address(0x89F)));


extern volatile __bit PSS0 __attribute__((address(0x4872)));


extern volatile __bit PSS1 __attribute__((address(0x4873)));


extern volatile __bit PWM3DC0 __attribute__((address(0x18A6)));


extern volatile __bit PWM3DC1 __attribute__((address(0x18A7)));


extern volatile __bit PWM3DC2 __attribute__((address(0x18A8)));


extern volatile __bit PWM3DC3 __attribute__((address(0x18A9)));


extern volatile __bit PWM3DC4 __attribute__((address(0x18AA)));


extern volatile __bit PWM3DC5 __attribute__((address(0x18AB)));


extern volatile __bit PWM3DC6 __attribute__((address(0x18AC)));


extern volatile __bit PWM3DC7 __attribute__((address(0x18AD)));


extern volatile __bit PWM3DC8 __attribute__((address(0x18AE)));


extern volatile __bit PWM3DC9 __attribute__((address(0x18AF)));


extern volatile __bit PWM3EN __attribute__((address(0x18B7)));


extern volatile __bit PWM3OUT __attribute__((address(0x18B5)));


extern volatile __bit PWM3POL __attribute__((address(0x18B4)));


extern volatile __bit PWM4DC0 __attribute__((address(0x18C6)));


extern volatile __bit PWM4DC1 __attribute__((address(0x18C7)));


extern volatile __bit PWM4DC2 __attribute__((address(0x18C8)));


extern volatile __bit PWM4DC3 __attribute__((address(0x18C9)));


extern volatile __bit PWM4DC4 __attribute__((address(0x18CA)));


extern volatile __bit PWM4DC5 __attribute__((address(0x18CB)));


extern volatile __bit PWM4DC6 __attribute__((address(0x18CC)));


extern volatile __bit PWM4DC7 __attribute__((address(0x18CD)));


extern volatile __bit PWM4DC8 __attribute__((address(0x18CE)));


extern volatile __bit PWM4DC9 __attribute__((address(0x18CF)));


extern volatile __bit PWM4EN __attribute__((address(0x18D7)));


extern volatile __bit PWM4OUT __attribute__((address(0x18D5)));


extern volatile __bit PWM4POL __attribute__((address(0x18D4)));


extern volatile __bit RA0 __attribute__((address(0x60)));


extern volatile __bit RA0PPS0 __attribute__((address(0xF880)));


extern volatile __bit RA0PPS1 __attribute__((address(0xF881)));


extern volatile __bit RA0PPS2 __attribute__((address(0xF882)));


extern volatile __bit RA0PPS3 __attribute__((address(0xF883)));


extern volatile __bit RA0PPS4 __attribute__((address(0xF884)));


extern volatile __bit RA0PPS5 __attribute__((address(0xF885)));


extern volatile __bit RA1 __attribute__((address(0x61)));


extern volatile __bit RA1PPS0 __attribute__((address(0xF888)));


extern volatile __bit RA1PPS1 __attribute__((address(0xF889)));


extern volatile __bit RA1PPS2 __attribute__((address(0xF88A)));


extern volatile __bit RA1PPS3 __attribute__((address(0xF88B)));


extern volatile __bit RA1PPS4 __attribute__((address(0xF88C)));


extern volatile __bit RA1PPS5 __attribute__((address(0xF88D)));


extern volatile __bit RA2 __attribute__((address(0x62)));


extern volatile __bit RA2PPS0 __attribute__((address(0xF890)));


extern volatile __bit RA2PPS1 __attribute__((address(0xF891)));


extern volatile __bit RA2PPS2 __attribute__((address(0xF892)));


extern volatile __bit RA2PPS3 __attribute__((address(0xF893)));


extern volatile __bit RA2PPS4 __attribute__((address(0xF894)));


extern volatile __bit RA2PPS5 __attribute__((address(0xF895)));


extern volatile __bit RA3 __attribute__((address(0x63)));


extern volatile __bit RA3PPS0 __attribute__((address(0xF898)));


extern volatile __bit RA3PPS1 __attribute__((address(0xF899)));


extern volatile __bit RA3PPS2 __attribute__((address(0xF89A)));


extern volatile __bit RA3PPS3 __attribute__((address(0xF89B)));


extern volatile __bit RA3PPS4 __attribute__((address(0xF89C)));


extern volatile __bit RA3PPS5 __attribute__((address(0xF89D)));


extern volatile __bit RA4 __attribute__((address(0x64)));


extern volatile __bit RA4PPS0 __attribute__((address(0xF8A0)));


extern volatile __bit RA4PPS1 __attribute__((address(0xF8A1)));


extern volatile __bit RA4PPS2 __attribute__((address(0xF8A2)));


extern volatile __bit RA4PPS3 __attribute__((address(0xF8A3)));


extern volatile __bit RA4PPS4 __attribute__((address(0xF8A4)));


extern volatile __bit RA4PPS5 __attribute__((address(0xF8A5)));


extern volatile __bit RA5 __attribute__((address(0x65)));


extern volatile __bit RA5PPS0 __attribute__((address(0xF8A8)));


extern volatile __bit RA5PPS1 __attribute__((address(0xF8A9)));


extern volatile __bit RA5PPS2 __attribute__((address(0xF8AA)));


extern volatile __bit RA5PPS3 __attribute__((address(0xF8AB)));


extern volatile __bit RA5PPS4 __attribute__((address(0xF8AC)));


extern volatile __bit RA5PPS5 __attribute__((address(0xF8AD)));


extern volatile __bit RA6 __attribute__((address(0x66)));


extern volatile __bit RA6PPS0 __attribute__((address(0xF8B0)));


extern volatile __bit RA6PPS1 __attribute__((address(0xF8B1)));


extern volatile __bit RA6PPS2 __attribute__((address(0xF8B2)));


extern volatile __bit RA6PPS3 __attribute__((address(0xF8B3)));


extern volatile __bit RA6PPS4 __attribute__((address(0xF8B4)));


extern volatile __bit RA6PPS5 __attribute__((address(0xF8B5)));


extern volatile __bit RA7 __attribute__((address(0x67)));


extern volatile __bit RA7PPS0 __attribute__((address(0xF8B8)));


extern volatile __bit RA7PPS1 __attribute__((address(0xF8B9)));


extern volatile __bit RA7PPS2 __attribute__((address(0xF8BA)));


extern volatile __bit RA7PPS3 __attribute__((address(0xF8BB)));


extern volatile __bit RA7PPS4 __attribute__((address(0xF8BC)));


extern volatile __bit RA7PPS5 __attribute__((address(0xF8BD)));


extern volatile __bit RB0 __attribute__((address(0x68)));


extern volatile __bit RB0PPS0 __attribute__((address(0xF8C0)));


extern volatile __bit RB0PPS1 __attribute__((address(0xF8C1)));


extern volatile __bit RB0PPS2 __attribute__((address(0xF8C2)));


extern volatile __bit RB0PPS3 __attribute__((address(0xF8C3)));


extern volatile __bit RB0PPS4 __attribute__((address(0xF8C4)));


extern volatile __bit RB0PPS5 __attribute__((address(0xF8C5)));


extern volatile __bit RB1 __attribute__((address(0x69)));


extern volatile __bit RB1PPS0 __attribute__((address(0xF8C8)));


extern volatile __bit RB1PPS1 __attribute__((address(0xF8C9)));


extern volatile __bit RB1PPS2 __attribute__((address(0xF8CA)));


extern volatile __bit RB1PPS3 __attribute__((address(0xF8CB)));


extern volatile __bit RB1PPS4 __attribute__((address(0xF8CC)));


extern volatile __bit RB1PPS5 __attribute__((address(0xF8CD)));


extern volatile __bit RB2 __attribute__((address(0x6A)));


extern volatile __bit RB2PPS0 __attribute__((address(0xF8D0)));


extern volatile __bit RB2PPS1 __attribute__((address(0xF8D1)));


extern volatile __bit RB2PPS2 __attribute__((address(0xF8D2)));


extern volatile __bit RB2PPS3 __attribute__((address(0xF8D3)));


extern volatile __bit RB2PPS4 __attribute__((address(0xF8D4)));


extern volatile __bit RB2PPS5 __attribute__((address(0xF8D5)));


extern volatile __bit RB3 __attribute__((address(0x6B)));


extern volatile __bit RB3PPS0 __attribute__((address(0xF8D8)));


extern volatile __bit RB3PPS1 __attribute__((address(0xF8D9)));


extern volatile __bit RB3PPS2 __attribute__((address(0xF8DA)));


extern volatile __bit RB3PPS3 __attribute__((address(0xF8DB)));


extern volatile __bit RB3PPS4 __attribute__((address(0xF8DC)));


extern volatile __bit RB3PPS5 __attribute__((address(0xF8DD)));


extern volatile __bit RB4 __attribute__((address(0x6C)));


extern volatile __bit RB4PPS0 __attribute__((address(0xF8E0)));


extern volatile __bit RB4PPS1 __attribute__((address(0xF8E1)));


extern volatile __bit RB4PPS2 __attribute__((address(0xF8E2)));


extern volatile __bit RB4PPS3 __attribute__((address(0xF8E3)));


extern volatile __bit RB4PPS4 __attribute__((address(0xF8E4)));


extern volatile __bit RB4PPS5 __attribute__((address(0xF8E5)));


extern volatile __bit RB5 __attribute__((address(0x6D)));


extern volatile __bit RB5PPS0 __attribute__((address(0xF8E8)));


extern volatile __bit RB5PPS1 __attribute__((address(0xF8E9)));


extern volatile __bit RB5PPS2 __attribute__((address(0xF8EA)));


extern volatile __bit RB5PPS3 __attribute__((address(0xF8EB)));


extern volatile __bit RB5PPS4 __attribute__((address(0xF8EC)));


extern volatile __bit RB5PPS5 __attribute__((address(0xF8ED)));


extern volatile __bit RB6 __attribute__((address(0x6E)));


extern volatile __bit RB6PPS0 __attribute__((address(0xF8F0)));


extern volatile __bit RB6PPS1 __attribute__((address(0xF8F1)));


extern volatile __bit RB6PPS2 __attribute__((address(0xF8F2)));


extern volatile __bit RB6PPS3 __attribute__((address(0xF8F3)));


extern volatile __bit RB6PPS4 __attribute__((address(0xF8F4)));


extern volatile __bit RB6PPS5 __attribute__((address(0xF8F5)));


extern volatile __bit RB7 __attribute__((address(0x6F)));


extern volatile __bit RB7PPS0 __attribute__((address(0xF8F8)));


extern volatile __bit RB7PPS1 __attribute__((address(0xF8F9)));


extern volatile __bit RB7PPS2 __attribute__((address(0xF8FA)));


extern volatile __bit RB7PPS3 __attribute__((address(0xF8FB)));


extern volatile __bit RB7PPS4 __attribute__((address(0xF8FC)));


extern volatile __bit RB7PPS5 __attribute__((address(0xF8FD)));


extern volatile __bit RC0 __attribute__((address(0x70)));


extern volatile __bit RC0PPS0 __attribute__((address(0xF900)));


extern volatile __bit RC0PPS1 __attribute__((address(0xF901)));


extern volatile __bit RC0PPS2 __attribute__((address(0xF902)));


extern volatile __bit RC0PPS3 __attribute__((address(0xF903)));


extern volatile __bit RC0PPS4 __attribute__((address(0xF904)));


extern volatile __bit RC0PPS5 __attribute__((address(0xF905)));


extern volatile __bit RC1 __attribute__((address(0x71)));


extern volatile __bit RC1IE __attribute__((address(0x38CD)));


extern volatile __bit RC1IF __attribute__((address(0x387D)));


extern volatile __bit RC1PPS0 __attribute__((address(0xF908)));


extern volatile __bit RC1PPS1 __attribute__((address(0xF909)));


extern volatile __bit RC1PPS2 __attribute__((address(0xF90A)));


extern volatile __bit RC1PPS3 __attribute__((address(0xF90B)));


extern volatile __bit RC1PPS4 __attribute__((address(0xF90C)));


extern volatile __bit RC1PPS5 __attribute__((address(0xF90D)));


extern volatile __bit RC2 __attribute__((address(0x72)));


extern volatile __bit RC2IE __attribute__((address(0x38CF)));


extern volatile __bit RC2IF __attribute__((address(0x387F)));


extern volatile __bit RC2PPS0 __attribute__((address(0xF910)));


extern volatile __bit RC2PPS1 __attribute__((address(0xF911)));


extern volatile __bit RC2PPS2 __attribute__((address(0xF912)));


extern volatile __bit RC2PPS3 __attribute__((address(0xF913)));


extern volatile __bit RC2PPS4 __attribute__((address(0xF914)));


extern volatile __bit RC2PPS5 __attribute__((address(0xF915)));


extern volatile __bit RC3 __attribute__((address(0x73)));


extern volatile __bit RC3PPS0 __attribute__((address(0xF918)));


extern volatile __bit RC3PPS1 __attribute__((address(0xF919)));


extern volatile __bit RC3PPS2 __attribute__((address(0xF91A)));


extern volatile __bit RC3PPS3 __attribute__((address(0xF91B)));


extern volatile __bit RC3PPS4 __attribute__((address(0xF91C)));


extern volatile __bit RC3PPS5 __attribute__((address(0xF91D)));


extern volatile __bit RC4 __attribute__((address(0x74)));


extern volatile __bit RC4PPS0 __attribute__((address(0xF920)));


extern volatile __bit RC4PPS1 __attribute__((address(0xF921)));


extern volatile __bit RC4PPS2 __attribute__((address(0xF922)));


extern volatile __bit RC4PPS3 __attribute__((address(0xF923)));


extern volatile __bit RC4PPS4 __attribute__((address(0xF924)));


extern volatile __bit RC4PPS5 __attribute__((address(0xF925)));


extern volatile __bit RC6 __attribute__((address(0x76)));


extern volatile __bit RC6PPS0 __attribute__((address(0xF930)));


extern volatile __bit RC6PPS1 __attribute__((address(0xF931)));


extern volatile __bit RC6PPS2 __attribute__((address(0xF932)));


extern volatile __bit RC6PPS3 __attribute__((address(0xF933)));


extern volatile __bit RC6PPS4 __attribute__((address(0xF934)));


extern volatile __bit RC6PPS5 __attribute__((address(0xF935)));


extern volatile __bit RC7 __attribute__((address(0x77)));


extern volatile __bit RC7PPS0 __attribute__((address(0xF938)));


extern volatile __bit RC7PPS1 __attribute__((address(0xF939)));


extern volatile __bit RC7PPS2 __attribute__((address(0xF93A)));


extern volatile __bit RC7PPS3 __attribute__((address(0xF93B)));


extern volatile __bit RC7PPS4 __attribute__((address(0xF93C)));


extern volatile __bit RC7PPS5 __attribute__((address(0xF93D)));


extern volatile __bit RCEN __attribute__((address(0xC8B)));


extern volatile __bit RCEN1 __attribute__((address(0xC8B)));


extern volatile __bit RD __attribute__((address(0x40F0)));


extern volatile __bit RD16 __attribute__((address(0x1071)));


extern volatile __bit RD161 __attribute__((address(0x1071)));


extern volatile __bit RE3 __attribute__((address(0x83)));


extern volatile __bit READ_WRITE __attribute__((address(0xC7A)));


extern volatile __bit READ_WRITE1 __attribute__((address(0xC7A)));


extern volatile __bit REN __attribute__((address(0x3096)));


extern volatile __bit REPEAT __attribute__((address(0x24CE)));


extern volatile __bit RES0 __attribute__((address(0x4E8)));


extern volatile __bit RES1 __attribute__((address(0x4E9)));


extern volatile __bit RES10 __attribute__((address(0x4F2)));


extern volatile __bit RES11 __attribute__((address(0x4F3)));


extern volatile __bit RES12 __attribute__((address(0x4F4)));


extern volatile __bit RES13 __attribute__((address(0x4F5)));


extern volatile __bit RES14 __attribute__((address(0x4F6)));


extern volatile __bit RES15 __attribute__((address(0x4F7)));


extern volatile __bit RES2 __attribute__((address(0x4EA)));


extern volatile __bit RES3 __attribute__((address(0x4EB)));


extern volatile __bit RES4 __attribute__((address(0x4EC)));


extern volatile __bit RES5 __attribute__((address(0x4ED)));


extern volatile __bit RES6 __attribute__((address(0x4EE)));


extern volatile __bit RES7 __attribute__((address(0x4EF)));


extern volatile __bit RES8 __attribute__((address(0x4F0)));


extern volatile __bit RES9 __attribute__((address(0x4F1)));


extern volatile __bit ROI __attribute__((address(0x4465)));


extern volatile __bit RPT0 __attribute__((address(0x4D0)));


extern volatile __bit RPT1 __attribute__((address(0x4D1)));


extern volatile __bit RPT2 __attribute__((address(0x4D2)));


extern volatile __bit RPT3 __attribute__((address(0x4D3)));


extern volatile __bit RPT4 __attribute__((address(0x4D4)));


extern volatile __bit RPT5 __attribute__((address(0x4D5)));


extern volatile __bit RPT6 __attribute__((address(0x4D6)));


extern volatile __bit RPT7 __attribute__((address(0x4D7)));


extern volatile __bit RSEN __attribute__((address(0xC89)));


extern volatile __bit RSEN1 __attribute__((address(0xC89)));


extern volatile __bit RST __attribute__((address(0x24D5)));


extern volatile __bit RTCCIE __attribute__((address(0x38F6)));


extern volatile __bit RTCCIF __attribute__((address(0x38A6)));


extern volatile __bit RTCCLKSEL0 __attribute__((address(0x6060)));


extern volatile __bit RTCCLKSEL1 __attribute__((address(0x6061)));


extern volatile __bit RTCCMD __attribute__((address(0x3CC7)));


extern volatile __bit RTCEN __attribute__((address(0x6067)));


extern volatile __bit RTCSYNC __attribute__((address(0x6064)));


extern volatile __bit RTCWREN __attribute__((address(0x6065)));


extern volatile __bit RW __attribute__((address(0xC7A)));


extern volatile __bit RW1 __attribute__((address(0xC7A)));


extern volatile __bit RX1PPS0 __attribute__((address(0xF658)));


extern volatile __bit RX1PPS1 __attribute__((address(0xF659)));


extern volatile __bit RX1PPS2 __attribute__((address(0xF65A)));


extern volatile __bit RX1PPS3 __attribute__((address(0xF65B)));


extern volatile __bit RX1PPS4 __attribute__((address(0xF65C)));


extern volatile __bit RX2PPS0 __attribute__((address(0xF668)));


extern volatile __bit RX2PPS1 __attribute__((address(0xF669)));


extern volatile __bit RX3PPS2 __attribute__((address(0xF66A)));


extern volatile __bit RX4PPS3 __attribute__((address(0xF66B)));


extern volatile __bit RX5PPS4 __attribute__((address(0xF66C)));


extern volatile __bit R_W __attribute__((address(0xC7A)));


extern volatile __bit R_W1 __attribute__((address(0xC7A)));


extern volatile __bit R_nW __attribute__((address(0xC7A)));


extern volatile __bit R_nW1 __attribute__((address(0xC7A)));


extern volatile __bit S00C0 __attribute__((address(0xE8C0)));


extern volatile __bit S00C1 __attribute__((address(0xE8F0)));


extern volatile __bit S00C2 __attribute__((address(0xE920)));


extern volatile __bit S00C3 __attribute__((address(0xE950)));


extern volatile __bit S00C4 __attribute__((address(0xE980)));


extern volatile __bit S00C5 __attribute__((address(0xE9B0)));


extern volatile __bit S00C6 __attribute__((address(0xE9E0)));


extern volatile __bit S00C7 __attribute__((address(0xEA10)));


extern volatile __bit S01C0 __attribute__((address(0xE8C1)));


extern volatile __bit S01C1 __attribute__((address(0xE8F1)));


extern volatile __bit S01C2 __attribute__((address(0xE921)));


extern volatile __bit S01C3 __attribute__((address(0xE951)));


extern volatile __bit S01C4 __attribute__((address(0xE981)));


extern volatile __bit S01C5 __attribute__((address(0xE9B1)));


extern volatile __bit S01C6 __attribute__((address(0xE9E1)));


extern volatile __bit S01C7 __attribute__((address(0xEA11)));


extern volatile __bit S02C0 __attribute__((address(0xE8C2)));


extern volatile __bit S02C1 __attribute__((address(0xE8F2)));


extern volatile __bit S02C2 __attribute__((address(0xE922)));


extern volatile __bit S02C3 __attribute__((address(0xE952)));


extern volatile __bit S02C4 __attribute__((address(0xE982)));


extern volatile __bit S02C5 __attribute__((address(0xE9B2)));


extern volatile __bit S02C6 __attribute__((address(0xE9E2)));


extern volatile __bit S02C7 __attribute__((address(0xEA12)));


extern volatile __bit S03C0 __attribute__((address(0xE8C3)));


extern volatile __bit S03C1 __attribute__((address(0xE8F3)));


extern volatile __bit S03C2 __attribute__((address(0xE923)));


extern volatile __bit S03C3 __attribute__((address(0xE953)));


extern volatile __bit S03C4 __attribute__((address(0xE983)));


extern volatile __bit S03C5 __attribute__((address(0xE9B3)));


extern volatile __bit S03C6 __attribute__((address(0xE9E3)));


extern volatile __bit S03C7 __attribute__((address(0xEA13)));


extern volatile __bit S04C0 __attribute__((address(0xE8C4)));


extern volatile __bit S04C1 __attribute__((address(0xE8F4)));


extern volatile __bit S04C2 __attribute__((address(0xE924)));


extern volatile __bit S04C3 __attribute__((address(0xE954)));


extern volatile __bit S04C4 __attribute__((address(0xE984)));


extern volatile __bit S04C5 __attribute__((address(0xE9B4)));


extern volatile __bit S04C6 __attribute__((address(0xE9E4)));


extern volatile __bit S04C7 __attribute__((address(0xEA14)));


extern volatile __bit S06C0 __attribute__((address(0xE8C6)));


extern volatile __bit S06C1 __attribute__((address(0xE8F6)));


extern volatile __bit S06C2 __attribute__((address(0xE926)));


extern volatile __bit S06C3 __attribute__((address(0xE956)));


extern volatile __bit S06C4 __attribute__((address(0xE986)));


extern volatile __bit S06C5 __attribute__((address(0xE9B6)));


extern volatile __bit S06C6 __attribute__((address(0xE9E6)));


extern volatile __bit S06C7 __attribute__((address(0xEA16)));


extern volatile __bit S07C0 __attribute__((address(0xE8C7)));


extern volatile __bit S07C1 __attribute__((address(0xE8F7)));


extern volatile __bit S07C2 __attribute__((address(0xE927)));


extern volatile __bit S07C3 __attribute__((address(0xE957)));


extern volatile __bit S07C4 __attribute__((address(0xE987)));


extern volatile __bit S07C5 __attribute__((address(0xE9B7)));


extern volatile __bit S07C6 __attribute__((address(0xE9E7)));


extern volatile __bit S07C7 __attribute__((address(0xEA17)));


extern volatile __bit S08C0 __attribute__((address(0xE8C8)));


extern volatile __bit S08C1 __attribute__((address(0xE8F8)));


extern volatile __bit S08C2 __attribute__((address(0xE928)));


extern volatile __bit S08C3 __attribute__((address(0xE958)));


extern volatile __bit S08C4 __attribute__((address(0xE988)));


extern volatile __bit S08C5 __attribute__((address(0xE9B8)));


extern volatile __bit S08C6 __attribute__((address(0xE9E8)));


extern volatile __bit S08C7 __attribute__((address(0xEA18)));


extern volatile __bit S09C0 __attribute__((address(0xE8C9)));


extern volatile __bit S09C1 __attribute__((address(0xE8F9)));


extern volatile __bit S09C2 __attribute__((address(0xE929)));


extern volatile __bit S09C3 __attribute__((address(0xE959)));


extern volatile __bit S09C4 __attribute__((address(0xE989)));


extern volatile __bit S09C5 __attribute__((address(0xE9B9)));


extern volatile __bit S09C6 __attribute__((address(0xE9E9)));


extern volatile __bit S09C7 __attribute__((address(0xEA19)));


extern volatile __bit S10C0 __attribute__((address(0xE8CA)));


extern volatile __bit S10C1 __attribute__((address(0xE8FA)));


extern volatile __bit S10C2 __attribute__((address(0xE92A)));


extern volatile __bit S10C3 __attribute__((address(0xE95A)));


extern volatile __bit S10C4 __attribute__((address(0xE98A)));


extern volatile __bit S10C5 __attribute__((address(0xE9BA)));


extern volatile __bit S10C6 __attribute__((address(0xE9EA)));


extern volatile __bit S10C7 __attribute__((address(0xEA1A)));


extern volatile __bit S11C0 __attribute__((address(0xE8CB)));


extern volatile __bit S11C1 __attribute__((address(0xE8FB)));


extern volatile __bit S11C2 __attribute__((address(0xE92B)));


extern volatile __bit S11C3 __attribute__((address(0xE95B)));


extern volatile __bit S11C4 __attribute__((address(0xE98B)));


extern volatile __bit S11C5 __attribute__((address(0xE9BB)));


extern volatile __bit S11C6 __attribute__((address(0xE9EB)));


extern volatile __bit S11C7 __attribute__((address(0xEA1B)));


extern volatile __bit S13C0 __attribute__((address(0xE8CD)));


extern volatile __bit S13C1 __attribute__((address(0xE8FD)));


extern volatile __bit S13C2 __attribute__((address(0xE92D)));


extern volatile __bit S13C3 __attribute__((address(0xE95D)));


extern volatile __bit S13C4 __attribute__((address(0xE98D)));


extern volatile __bit S13C5 __attribute__((address(0xE9BD)));


extern volatile __bit S13C6 __attribute__((address(0xE9ED)));


extern volatile __bit S13C7 __attribute__((address(0xEA1D)));


extern volatile __bit S14C0 __attribute__((address(0xE8CE)));


extern volatile __bit S14C1 __attribute__((address(0xE8FE)));


extern volatile __bit S14C2 __attribute__((address(0xE92E)));


extern volatile __bit S14C3 __attribute__((address(0xE95E)));


extern volatile __bit S14C4 __attribute__((address(0xE98E)));


extern volatile __bit S14C5 __attribute__((address(0xE9BE)));


extern volatile __bit S14C6 __attribute__((address(0xE9EE)));


extern volatile __bit S14C7 __attribute__((address(0xEA1E)));


extern volatile __bit S15C0 __attribute__((address(0xE8CF)));


extern volatile __bit S15C1 __attribute__((address(0xE8FF)));


extern volatile __bit S15C2 __attribute__((address(0xE92F)));


extern volatile __bit S15C3 __attribute__((address(0xE95F)));


extern volatile __bit S15C4 __attribute__((address(0xE98F)));


extern volatile __bit S15C5 __attribute__((address(0xE9BF)));


extern volatile __bit S15C6 __attribute__((address(0xE9EF)));


extern volatile __bit S15C7 __attribute__((address(0xEA1F)));


extern volatile __bit S18C0 __attribute__((address(0xE8D2)));


extern volatile __bit S18C1 __attribute__((address(0xE902)));


extern volatile __bit S18C2 __attribute__((address(0xE932)));


extern volatile __bit S18C3 __attribute__((address(0xE962)));


extern volatile __bit S18C4 __attribute__((address(0xE992)));


extern volatile __bit S18C5 __attribute__((address(0xE9C2)));


extern volatile __bit S18C6 __attribute__((address(0xE9F2)));


extern volatile __bit S18C7 __attribute__((address(0xEA22)));


extern volatile __bit S19C0 __attribute__((address(0xE8D3)));


extern volatile __bit S19C1 __attribute__((address(0xE903)));


extern volatile __bit S19C2 __attribute__((address(0xE933)));


extern volatile __bit S19C3 __attribute__((address(0xE963)));


extern volatile __bit S19C4 __attribute__((address(0xE993)));


extern volatile __bit S19C5 __attribute__((address(0xE9C3)));


extern volatile __bit S19C6 __attribute__((address(0xE9F3)));


extern volatile __bit S19C7 __attribute__((address(0xEA23)));


extern volatile __bit S2 __attribute__((address(0xC7B)));


extern volatile __bit S20C0 __attribute__((address(0xE8D4)));


extern volatile __bit S20C1 __attribute__((address(0xE904)));


extern volatile __bit S20C2 __attribute__((address(0xE934)));


extern volatile __bit S20C3 __attribute__((address(0xE964)));


extern volatile __bit S20C4 __attribute__((address(0xE994)));


extern volatile __bit S20C5 __attribute__((address(0xE9C4)));


extern volatile __bit S20C6 __attribute__((address(0xE9F4)));


extern volatile __bit S20C7 __attribute__((address(0xEA24)));


extern volatile __bit S22C0 __attribute__((address(0xE8D6)));


extern volatile __bit S22C1 __attribute__((address(0xE906)));


extern volatile __bit S22C2 __attribute__((address(0xE936)));


extern volatile __bit S22C3 __attribute__((address(0xE966)));


extern volatile __bit S22C4 __attribute__((address(0xE996)));


extern volatile __bit S22C5 __attribute__((address(0xE9C6)));


extern volatile __bit S22C6 __attribute__((address(0xE9F6)));


extern volatile __bit S22C7 __attribute__((address(0xEA26)));


extern volatile __bit S23C0 __attribute__((address(0xE8D7)));


extern volatile __bit S23C1 __attribute__((address(0xE907)));


extern volatile __bit S23C2 __attribute__((address(0xE937)));


extern volatile __bit S23C3 __attribute__((address(0xE967)));


extern volatile __bit S23C4 __attribute__((address(0xE997)));


extern volatile __bit S23C5 __attribute__((address(0xE9C7)));


extern volatile __bit S23C6 __attribute__((address(0xE9F7)));


extern volatile __bit S23C7 __attribute__((address(0xEA27)));


extern volatile __bit SBCDE __attribute__((address(0xC92)));


extern volatile __bit SBOREN __attribute__((address(0x408F)));


extern volatile __bit SCIE __attribute__((address(0xC95)));


extern volatile __bit SDAHT __attribute__((address(0xC93)));


extern volatile __bit SE00 __attribute__((address(0xE870)));


extern volatile __bit SE01 __attribute__((address(0xE871)));


extern volatile __bit SE02 __attribute__((address(0xE872)));


extern volatile __bit SE03 __attribute__((address(0xE873)));


extern volatile __bit SE04 __attribute__((address(0xE874)));


extern volatile __bit SE05 __attribute__((address(0xE875)));


extern volatile __bit SE06 __attribute__((address(0xE876)));


extern volatile __bit SE07 __attribute__((address(0xE877)));


extern volatile __bit SE08 __attribute__((address(0xE878)));


extern volatile __bit SE09 __attribute__((address(0xE879)));


extern volatile __bit SE10 __attribute__((address(0xE87A)));


extern volatile __bit SE11 __attribute__((address(0xE87B)));


extern volatile __bit SE12 __attribute__((address(0xE87C)));


extern volatile __bit SE13 __attribute__((address(0xE87D)));


extern volatile __bit SE14 __attribute__((address(0xE87E)));


extern volatile __bit SE15 __attribute__((address(0xE87F)));


extern volatile __bit SE16 __attribute__((address(0xE880)));


extern volatile __bit SE17 __attribute__((address(0xE881)));


extern volatile __bit SE18 __attribute__((address(0xE882)));


extern volatile __bit SE19 __attribute__((address(0xE883)));


extern volatile __bit SE20 __attribute__((address(0xE884)));


extern volatile __bit SE21 __attribute__((address(0xE885)));


extern volatile __bit SE22 __attribute__((address(0xE886)));


extern volatile __bit SE23 __attribute__((address(0xE887)));


extern volatile __bit SE24 __attribute__((address(0xE888)));


extern volatile __bit SE25 __attribute__((address(0xE889)));


extern volatile __bit SE26 __attribute__((address(0xE88A)));


extern volatile __bit SE27 __attribute__((address(0xE88B)));


extern volatile __bit SE28 __attribute__((address(0xE88C)));


extern volatile __bit SE29 __attribute__((address(0xE88D)));


extern volatile __bit SE30 __attribute__((address(0xE88E)));


extern volatile __bit SE31 __attribute__((address(0xE88F)));


extern volatile __bit SE32 __attribute__((address(0xE890)));


extern volatile __bit SE33 __attribute__((address(0xE891)));


extern volatile __bit SE34 __attribute__((address(0xE892)));


extern volatile __bit SE35 __attribute__((address(0xE893)));


extern volatile __bit SE36 __attribute__((address(0xE894)));


extern volatile __bit SE37 __attribute__((address(0xE895)));


extern volatile __bit SE38 __attribute__((address(0xE896)));


extern volatile __bit SE39 __attribute__((address(0xE897)));


extern volatile __bit SE40 __attribute__((address(0xE898)));


extern volatile __bit SE41 __attribute__((address(0xE899)));


extern volatile __bit SE42 __attribute__((address(0xE89A)));


extern volatile __bit SE43 __attribute__((address(0xE89B)));


extern volatile __bit SE44 __attribute__((address(0xE89C)));


extern volatile __bit SE45 __attribute__((address(0xE89D)));


extern volatile __bit SE46 __attribute__((address(0xE89E)));


extern volatile __bit SE47 __attribute__((address(0xE89F)));


extern volatile __bit SECALRMH0 __attribute__((address(0x60E4)));


extern volatile __bit SECALRMH1 __attribute__((address(0x60E5)));


extern volatile __bit SECALRMH2 __attribute__((address(0x60E6)));


extern volatile __bit SECALRML0 __attribute__((address(0x60E0)));


extern volatile __bit SECALRML1 __attribute__((address(0x60E1)));


extern volatile __bit SECALRML2 __attribute__((address(0x60E2)));


extern volatile __bit SECALRML3 __attribute__((address(0x60E3)));


extern volatile __bit SECH0 __attribute__((address(0x60B4)));


extern volatile __bit SECH1 __attribute__((address(0x60B5)));


extern volatile __bit SECH2 __attribute__((address(0x60B6)));


extern volatile __bit SECL0 __attribute__((address(0x60B0)));


extern volatile __bit SECL1 __attribute__((address(0x60B1)));


extern volatile __bit SECL2 __attribute__((address(0x60B2)));


extern volatile __bit SECL3 __attribute__((address(0x60B3)));


extern volatile __bit SEN1 __attribute__((address(0xC88)));


extern volatile __bit SHUTDOWN __attribute__((address(0x3097)));


extern volatile __bit SLPEN __attribute__((address(0xE866)));


extern volatile __bit SLRA0 __attribute__((address(0xF9D8)));


extern volatile __bit SLRA1 __attribute__((address(0xF9D9)));


extern volatile __bit SLRA2 __attribute__((address(0xF9DA)));


extern volatile __bit SLRA3 __attribute__((address(0xF9DB)));


extern volatile __bit SLRA4 __attribute__((address(0xF9DC)));


extern volatile __bit SLRA6 __attribute__((address(0xF9DE)));


extern volatile __bit SLRA7 __attribute__((address(0xF9DF)));


extern volatile __bit SLRB0 __attribute__((address(0xFA30)));


extern volatile __bit SLRB1 __attribute__((address(0xFA31)));


extern volatile __bit SLRB2 __attribute__((address(0xFA32)));


extern volatile __bit SLRB3 __attribute__((address(0xFA33)));


extern volatile __bit SLRB4 __attribute__((address(0xFA34)));


extern volatile __bit SLRB5 __attribute__((address(0xFA35)));


extern volatile __bit SLRB6 __attribute__((address(0xFA36)));


extern volatile __bit SLRB7 __attribute__((address(0xFA37)));


extern volatile __bit SLRC0 __attribute__((address(0xFA88)));


extern volatile __bit SLRC1 __attribute__((address(0xFA89)));


extern volatile __bit SLRC2 __attribute__((address(0xFA8A)));


extern volatile __bit SLRC3 __attribute__((address(0xFA8B)));


extern volatile __bit SLRC4 __attribute__((address(0xFA8C)));


extern volatile __bit SLRC6 __attribute__((address(0xFA8E)));


extern volatile __bit SLRC7 __attribute__((address(0xFA8F)));


extern volatile __bit SMP __attribute__((address(0xC7F)));


extern volatile __bit SMP1 __attribute__((address(0xC7F)));


extern volatile __bit SMT1AS __attribute__((address(0x24D0)));


extern volatile __bit SMT1CPR10 __attribute__((address(0x2482)));


extern volatile __bit SMT1CPR11 __attribute__((address(0x2483)));


extern volatile __bit SMT1CPR12 __attribute__((address(0x2484)));


extern volatile __bit SMT1CPR13 __attribute__((address(0x2485)));


extern volatile __bit SMT1CPR14 __attribute__((address(0x2486)));


extern volatile __bit SMT1CPR15 __attribute__((address(0x2487)));


extern volatile __bit SMT1CPR16 __attribute__((address(0x2488)));


extern volatile __bit SMT1CPR17 __attribute__((address(0x2489)));


extern volatile __bit SMT1CPR18 __attribute__((address(0x248A)));


extern volatile __bit SMT1CPR19 __attribute__((address(0x248B)));


extern volatile __bit SMT1CPR20 __attribute__((address(0x248C)));


extern volatile __bit SMT1CPR21 __attribute__((address(0x248D)));


extern volatile __bit SMT1CPR22 __attribute__((address(0x248E)));


extern volatile __bit SMT1CPR23 __attribute__((address(0x248F)));


extern volatile __bit SMT1CPR8 __attribute__((address(0x2480)));


extern volatile __bit SMT1CPR9 __attribute__((address(0x2481)));


extern volatile __bit SMT1CPRUP __attribute__((address(0x24D7)));


extern volatile __bit SMT1CPW0 __attribute__((address(0x2490)));


extern volatile __bit SMT1CPW1 __attribute__((address(0x2491)));


extern volatile __bit SMT1CPW10 __attribute__((address(0x249A)));


extern volatile __bit SMT1CPW11 __attribute__((address(0x249B)));


extern volatile __bit SMT1CPW12 __attribute__((address(0x249C)));


extern volatile __bit SMT1CPW13 __attribute__((address(0x249D)));


extern volatile __bit SMT1CPW14 __attribute__((address(0x249E)));


extern volatile __bit SMT1CPW15 __attribute__((address(0x249F)));


extern volatile __bit SMT1CPW16 __attribute__((address(0x24A0)));


extern volatile __bit SMT1CPW17 __attribute__((address(0x24A1)));


extern volatile __bit SMT1CPW18 __attribute__((address(0x24A2)));


extern volatile __bit SMT1CPW19 __attribute__((address(0x24A3)));


extern volatile __bit SMT1CPW2 __attribute__((address(0x2492)));


extern volatile __bit SMT1CPW20 __attribute__((address(0x24A4)));


extern volatile __bit SMT1CPW21 __attribute__((address(0x24A5)));


extern volatile __bit SMT1CPW22 __attribute__((address(0x24A6)));


extern volatile __bit SMT1CPW23 __attribute__((address(0x24A7)));


extern volatile __bit SMT1CPW3 __attribute__((address(0x2493)));


extern volatile __bit SMT1CPW4 __attribute__((address(0x2494)));


extern volatile __bit SMT1CPW5 __attribute__((address(0x2495)));


extern volatile __bit SMT1CPW6 __attribute__((address(0x2496)));


extern volatile __bit SMT1CPW7 __attribute__((address(0x2497)));


extern volatile __bit SMT1CPW8 __attribute__((address(0x2498)));


extern volatile __bit SMT1CPW9 __attribute__((address(0x2499)));


extern volatile __bit SMT1CPWUP __attribute__((address(0x24D6)));


extern volatile __bit SMT1CSEL0 __attribute__((address(0x24D8)));


extern volatile __bit SMT1CSEL1 __attribute__((address(0x24D9)));


extern volatile __bit SMT1CSEL2 __attribute__((address(0x24DA)));


extern volatile __bit SMT1EN __attribute__((address(0x24C7)));


extern volatile __bit SMT1GO __attribute__((address(0x24CF)));


extern volatile __bit SMT1IE __attribute__((address(0x38F0)));


extern volatile __bit SMT1IF __attribute__((address(0x38A0)));


extern volatile __bit SMT1MD __attribute__((address(0x3CDE)));


extern volatile __bit SMT1PR0 __attribute__((address(0x24A8)));


extern volatile __bit SMT1PR1 __attribute__((address(0x24A9)));


extern volatile __bit SMT1PR10 __attribute__((address(0x24B2)));


extern volatile __bit SMT1PR11 __attribute__((address(0x24B3)));


extern volatile __bit SMT1PR12 __attribute__((address(0x24B4)));


extern volatile __bit SMT1PR13 __attribute__((address(0x24B5)));


extern volatile __bit SMT1PR14 __attribute__((address(0x24B6)));


extern volatile __bit SMT1PR15 __attribute__((address(0x24B7)));


extern volatile __bit SMT1PR16 __attribute__((address(0x24B8)));


extern volatile __bit SMT1PR17 __attribute__((address(0x24B9)));


extern volatile __bit SMT1PR18 __attribute__((address(0x24BA)));


extern volatile __bit SMT1PR19 __attribute__((address(0x24BB)));


extern volatile __bit SMT1PR2 __attribute__((address(0x24AA)));


extern volatile __bit SMT1PR20 __attribute__((address(0x24BC)));


extern volatile __bit SMT1PR21 __attribute__((address(0x24BD)));


extern volatile __bit SMT1PR22 __attribute__((address(0x24BE)));


extern volatile __bit SMT1PR23 __attribute__((address(0x24BF)));


extern volatile __bit SMT1PR3 __attribute__((address(0x24AB)));


extern volatile __bit SMT1PR4 __attribute__((address(0x24AC)));


extern volatile __bit SMT1PR5 __attribute__((address(0x24AD)));


extern volatile __bit SMT1PR6 __attribute__((address(0x24AE)));


extern volatile __bit SMT1PR7 __attribute__((address(0x24AF)));


extern volatile __bit SMT1PR8 __attribute__((address(0x24B0)));


extern volatile __bit SMT1PR9 __attribute__((address(0x24B1)));


extern volatile __bit SMT1PRAIE __attribute__((address(0x38F1)));


extern volatile __bit SMT1PRAIF __attribute__((address(0x38A1)));


extern volatile __bit SMT1PS0 __attribute__((address(0x24C0)));


extern volatile __bit SMT1PS1 __attribute__((address(0x24C1)));


extern volatile __bit SMT1PWAIE __attribute__((address(0x38F2)));


extern volatile __bit SMT1PWAIF __attribute__((address(0x38A2)));


extern volatile __bit SMT1RESET __attribute__((address(0x24D5)));


extern volatile __bit SMT1SSEL0 __attribute__((address(0x24E0)));


extern volatile __bit SMT1SSEL1 __attribute__((address(0x24E1)));


extern volatile __bit SMT1SSEL2 __attribute__((address(0x24E2)));


extern volatile __bit SMT1SSEL3 __attribute__((address(0x24E3)));


extern volatile __bit SMT1SSEL4 __attribute__((address(0x24E4)));


extern volatile __bit SMT1TMR0 __attribute__((address(0x2460)));


extern volatile __bit SMT1TMR1 __attribute__((address(0x2461)));


extern volatile __bit SMT1TMR10 __attribute__((address(0x246A)));


extern volatile __bit SMT1TMR11 __attribute__((address(0x246B)));


extern volatile __bit SMT1TMR12 __attribute__((address(0x246C)));


extern volatile __bit SMT1TMR13 __attribute__((address(0x246D)));


extern volatile __bit SMT1TMR14 __attribute__((address(0x246E)));


extern volatile __bit SMT1TMR15 __attribute__((address(0x246F)));


extern volatile __bit SMT1TMR16 __attribute__((address(0x2470)));


extern volatile __bit SMT1TMR17 __attribute__((address(0x2471)));


extern volatile __bit SMT1TMR18 __attribute__((address(0x2472)));


extern volatile __bit SMT1TMR19 __attribute__((address(0x2473)));


extern volatile __bit SMT1TMR2 __attribute__((address(0x2462)));


extern volatile __bit SMT1TMR20 __attribute__((address(0x2474)));


extern volatile __bit SMT1TMR21 __attribute__((address(0x2475)));


extern volatile __bit SMT1TMR22 __attribute__((address(0x2476)));


extern volatile __bit SMT1TMR23 __attribute__((address(0x2477)));


extern volatile __bit SMT1TMR3 __attribute__((address(0x2463)));


extern volatile __bit SMT1TMR4 __attribute__((address(0x2464)));


extern volatile __bit SMT1TMR5 __attribute__((address(0x2465)));


extern volatile __bit SMT1TMR6 __attribute__((address(0x2466)));


extern volatile __bit SMT1TMR7 __attribute__((address(0x2467)));


extern volatile __bit SMT1TMR8 __attribute__((address(0x2468)));


extern volatile __bit SMT1TMR9 __attribute__((address(0x2469)));


extern volatile __bit SMT1TS __attribute__((address(0x24D2)));


extern volatile __bit SMT1WS __attribute__((address(0x24D1)));


extern volatile __bit SMT1WSEL0 __attribute__((address(0x24E8)));


extern volatile __bit SMT1WSEL1 __attribute__((address(0x24E9)));


extern volatile __bit SMT1WSEL2 __attribute__((address(0x24EA)));


extern volatile __bit SMT1WSEL3 __attribute__((address(0x24EB)));


extern volatile __bit SMT1WSEL4 __attribute__((address(0x24EC)));


extern volatile __bit SMU1SIGPPS0 __attribute__((address(0xF550)));


extern volatile __bit SMU1SIGPPS1 __attribute__((address(0xF551)));


extern volatile __bit SMU1SIGPPS2 __attribute__((address(0xF552)));


extern volatile __bit SMU1SIGPPS3 __attribute__((address(0xF553)));


extern volatile __bit SMU1SIGPPS4 __attribute__((address(0xF554)));


extern volatile __bit SMU1WINPPS0 __attribute__((address(0xF548)));


extern volatile __bit SMU1WINPPS1 __attribute__((address(0xF549)));


extern volatile __bit SMU1WINPPS2 __attribute__((address(0xF54A)));


extern volatile __bit SMU1WINPPS3 __attribute__((address(0xF54B)));


extern volatile __bit SMU1WINPPS4 __attribute__((address(0xF54C)));


extern volatile __bit SOI __attribute__((address(0x8A3)));


extern volatile __bit SOR __attribute__((address(0x4483)));


extern volatile __bit SOSCEN __attribute__((address(0x448B)));


extern volatile __bit SOSCPWR __attribute__((address(0x447E)));


extern volatile __bit SPOL __attribute__((address(0x24C3)));


extern volatile __bit SSEL0 __attribute__((address(0x24E0)));


extern volatile __bit SSEL1 __attribute__((address(0x24E1)));


extern volatile __bit SSEL2 __attribute__((address(0x24E2)));


extern volatile __bit SSEL3 __attribute__((address(0x24E3)));


extern volatile __bit SSEL4 __attribute__((address(0x24E4)));


extern volatile __bit SSP1CLKPPS0 __attribute__((address(0xF628)));


extern volatile __bit SSP1CLKPPS1 __attribute__((address(0xF629)));


extern volatile __bit SSP1CLKPPS2 __attribute__((address(0xF62A)));


extern volatile __bit SSP1CLKPPS3 __attribute__((address(0xF62B)));


extern volatile __bit SSP1CLKPPS4 __attribute__((address(0xF62C)));


extern volatile __bit SSP1DATPPS0 __attribute__((address(0xF630)));


extern volatile __bit SSP1DATPPS1 __attribute__((address(0xF631)));


extern volatile __bit SSP1DATPPS2 __attribute__((address(0xF632)));


extern volatile __bit SSP1DATPPS3 __attribute__((address(0xF633)));


extern volatile __bit SSP1DATPPS4 __attribute__((address(0xF634)));


extern volatile __bit SSP1IE __attribute__((address(0x38C8)));


extern volatile __bit SSP1IF __attribute__((address(0x3878)));


extern volatile __bit SSP1SSPPS0 __attribute__((address(0xF638)));


extern volatile __bit SSP1SSPPS1 __attribute__((address(0xF639)));


extern volatile __bit SSP1SSPPS2 __attribute__((address(0xF63A)));


extern volatile __bit SSP1SSPPS3 __attribute__((address(0xF63B)));


extern volatile __bit SSP1SSPPS4 __attribute__((address(0xF63C)));


extern volatile __bit SSPEN __attribute__((address(0xC85)));


extern volatile __bit SSPEN1 __attribute__((address(0xC85)));


extern volatile __bit SSPM0 __attribute__((address(0xC80)));


extern volatile __bit SSPM01 __attribute__((address(0xC80)));


extern volatile __bit SSPM1 __attribute__((address(0xC81)));


extern volatile __bit SSPM11 __attribute__((address(0xC81)));


extern volatile __bit SSPM2 __attribute__((address(0xC82)));


extern volatile __bit SSPM21 __attribute__((address(0xC82)));


extern volatile __bit SSPM3 __attribute__((address(0xC83)));


extern volatile __bit SSPM31 __attribute__((address(0xC83)));


extern volatile __bit SSPOV __attribute__((address(0xC86)));


extern volatile __bit SSPOV1 __attribute__((address(0xC86)));


extern volatile __bit START __attribute__((address(0xC7B)));


extern volatile __bit START1 __attribute__((address(0xC7B)));


extern volatile __bit STAT0 __attribute__((address(0x8A8)));


extern volatile __bit STAT1 __attribute__((address(0x8A9)));


extern volatile __bit STAT2 __attribute__((address(0x8AA)));


extern volatile __bit STATE __attribute__((address(0x4082)));


extern volatile __bit STKOVF __attribute__((address(0x409F)));


extern volatile __bit STKUNF __attribute__((address(0x409E)));


extern volatile __bit STOP __attribute__((address(0xC7C)));


extern volatile __bit STOP1 __attribute__((address(0xC7C)));


extern volatile __bit STP __attribute__((address(0x24C5)));


extern volatile __bit STPT0 __attribute__((address(0x490)));


extern volatile __bit STPT1 __attribute__((address(0x491)));


extern volatile __bit STPT10 __attribute__((address(0x49A)));


extern volatile __bit STPT11 __attribute__((address(0x49B)));


extern volatile __bit STPT12 __attribute__((address(0x49C)));


extern volatile __bit STPT13 __attribute__((address(0x49D)));


extern volatile __bit STPT15 __attribute__((address(0x49E)));


extern volatile __bit STPT16 __attribute__((address(0x49F)));


extern volatile __bit STPT2 __attribute__((address(0x492)));


extern volatile __bit STPT3 __attribute__((address(0x493)));


extern volatile __bit STPT4 __attribute__((address(0x494)));


extern volatile __bit STPT5 __attribute__((address(0x495)));


extern volatile __bit STPT6 __attribute__((address(0x496)));


extern volatile __bit STPT7 __attribute__((address(0x497)));


extern volatile __bit STPT8 __attribute__((address(0x498)));


extern volatile __bit STPT9 __attribute__((address(0x499)));


extern volatile __bit STRA __attribute__((address(0x30A0)));


extern volatile __bit STRB __attribute__((address(0x30A1)));


extern volatile __bit STRC __attribute__((address(0x30A2)));


extern volatile __bit STRD __attribute__((address(0x30A3)));


extern volatile __bit SWDTEN __attribute__((address(0x4060)));


extern volatile __bit SYSCMD __attribute__((address(0x3CB7)));


extern volatile __bit T016BIT __attribute__((address(0x2CF4)));


extern volatile __bit T0ASYNC __attribute__((address(0x2CFC)));


extern volatile __bit T0CKIPPS0 __attribute__((address(0xF488)));


extern volatile __bit T0CKIPPS1 __attribute__((address(0xF489)));


extern volatile __bit T0CKIPPS2 __attribute__((address(0xF48A)));


extern volatile __bit T0CKIPPS3 __attribute__((address(0xF48B)));


extern volatile __bit T0CKPS0 __attribute__((address(0x2CF8)));


extern volatile __bit T0CKPS1 __attribute__((address(0x2CF9)));


extern volatile __bit T0CKPS2 __attribute__((address(0x2CFA)));


extern volatile __bit T0CKPS3 __attribute__((address(0x2CFB)));


extern volatile __bit T0CS0 __attribute__((address(0x2CFD)));


extern volatile __bit T0CS1 __attribute__((address(0x2CFE)));


extern volatile __bit T0CS2 __attribute__((address(0x2CFF)));


extern volatile __bit T0EN __attribute__((address(0x2CF7)));


extern volatile __bit T0OE __attribute__((address(0x2CF6)));


extern volatile __bit T0OUT __attribute__((address(0x2CF5)));


extern volatile __bit T0OUTPS0 __attribute__((address(0x2CF0)));


extern volatile __bit T0OUTPS1 __attribute__((address(0x2CF1)));


extern volatile __bit T0OUTPS2 __attribute__((address(0x2CF2)));


extern volatile __bit T0OUTPS3 __attribute__((address(0x2CF3)));


extern volatile __bit T0PR0 __attribute__((address(0x2CE8)));


extern volatile __bit T0PR1 __attribute__((address(0x2CE9)));


extern volatile __bit T0PR2 __attribute__((address(0x2CEA)));


extern volatile __bit T0PR3 __attribute__((address(0x2CEB)));


extern volatile __bit T0PR4 __attribute__((address(0x2CEC)));


extern volatile __bit T0PR5 __attribute__((address(0x2CED)));


extern volatile __bit T0PR6 __attribute__((address(0x2CEE)));


extern volatile __bit T0PR7 __attribute__((address(0x2CEF)));


extern volatile __bit T0PS0 __attribute__((address(0x2CF8)));


extern volatile __bit T0PS1 __attribute__((address(0x2CF9)));


extern volatile __bit T0PS2 __attribute__((address(0x2CFA)));


extern volatile __bit T0PS3 __attribute__((address(0x2CFB)));


extern volatile __bit T1CKIPPS0 __attribute__((address(0xF490)));


extern volatile __bit T1CKIPPS1 __attribute__((address(0xF491)));


extern volatile __bit T1CKIPPS2 __attribute__((address(0xF492)));


extern volatile __bit T1CKIPPS3 __attribute__((address(0xF493)));


extern volatile __bit T1CKIPPS4 __attribute__((address(0xF494)));


extern volatile __bit T1CKPS0 __attribute__((address(0x1074)));


extern volatile __bit T1CKPS1 __attribute__((address(0x1075)));


extern volatile __bit T1CS0 __attribute__((address(0x1088)));


extern volatile __bit T1CS1 __attribute__((address(0x1089)));


extern volatile __bit T1CS2 __attribute__((address(0x108A)));


extern volatile __bit T1CS3 __attribute__((address(0x108B)));


extern volatile __bit T1GE __attribute__((address(0x107F)));


extern volatile __bit T1GGO __attribute__((address(0x107B)));


extern volatile __bit T1GGO_nDONE __attribute__((address(0x107B)));


extern volatile __bit T1GPOL __attribute__((address(0x107E)));


extern volatile __bit T1GPPS0 __attribute__((address(0xF498)));


extern volatile __bit T1GPPS1 __attribute__((address(0xF499)));


extern volatile __bit T1GPPS2 __attribute__((address(0xF49A)));


extern volatile __bit T1GPPS3 __attribute__((address(0xF49B)));


extern volatile __bit T1GPPS4 __attribute__((address(0xF49C)));


extern volatile __bit T1GSPM __attribute__((address(0x107C)));


extern volatile __bit T1GSS0 __attribute__((address(0x1080)));


extern volatile __bit T1GSS1 __attribute__((address(0x1081)));


extern volatile __bit T1GSS2 __attribute__((address(0x1082)));


extern volatile __bit T1GSS3 __attribute__((address(0x1083)));


extern volatile __bit T1GSS4 __attribute__((address(0x1084)));


extern volatile __bit T1GTM __attribute__((address(0x107D)));


extern volatile __bit T1GVAL __attribute__((address(0x107A)));


extern volatile __bit T1RD16 __attribute__((address(0x1071)));


extern volatile __bit T2CKPOL __attribute__((address(0x147E)));


extern volatile __bit T2CKPS0 __attribute__((address(0x1474)));


extern volatile __bit T2CKPS1 __attribute__((address(0x1475)));


extern volatile __bit T2CKPS2 __attribute__((address(0x1476)));


extern volatile __bit T2CKSYNC __attribute__((address(0x147D)));


extern volatile __bit T2CS0 __attribute__((address(0x1480)));


extern volatile __bit T2CS1 __attribute__((address(0x1481)));


extern volatile __bit T2CS2 __attribute__((address(0x1482)));


extern volatile __bit T2CS3 __attribute__((address(0x1483)));


extern volatile __bit T2INPPS0 __attribute__((address(0xF4E0)));


extern volatile __bit T2INPPS1 __attribute__((address(0xF4E1)));


extern volatile __bit T2INPPS2 __attribute__((address(0xF4E2)));


extern volatile __bit T2INPPS3 __attribute__((address(0xF4E3)));


extern volatile __bit T2INPPS4 __attribute__((address(0xF4E4)));


extern volatile __bit T2MODE0 __attribute__((address(0x1478)));


extern volatile __bit T2MODE1 __attribute__((address(0x1479)));


extern volatile __bit T2MODE2 __attribute__((address(0x147A)));


extern volatile __bit T2MODE3 __attribute__((address(0x147B)));


extern volatile __bit T2MODE4 __attribute__((address(0x147C)));


extern volatile __bit T2ON __attribute__((address(0x1477)));


extern volatile __bit T2OUTPS0 __attribute__((address(0x1470)));


extern volatile __bit T2OUTPS1 __attribute__((address(0x1471)));


extern volatile __bit T2OUTPS2 __attribute__((address(0x1472)));


extern volatile __bit T2OUTPS3 __attribute__((address(0x1473)));


extern volatile __bit T2PSYNC __attribute__((address(0x147F)));


extern volatile __bit T2RSEL0 __attribute__((address(0x1488)));


extern volatile __bit T2RSEL1 __attribute__((address(0x1489)));


extern volatile __bit T2RSEL2 __attribute__((address(0x148A)));


extern volatile __bit T2RSEL3 __attribute__((address(0x148B)));


extern volatile __bit T2RSEL4 __attribute__((address(0x148C)));


extern volatile __bit T4CKPOL __attribute__((address(0x14AE)));


extern volatile __bit T4CKPS0 __attribute__((address(0x14A4)));


extern volatile __bit T4CKPS1 __attribute__((address(0x14A5)));


extern volatile __bit T4CKPS2 __attribute__((address(0x14A6)));


extern volatile __bit T4CKSYNC __attribute__((address(0x14AD)));


extern volatile __bit T4CS0 __attribute__((address(0x14B0)));


extern volatile __bit T4CS1 __attribute__((address(0x14B1)));


extern volatile __bit T4CS2 __attribute__((address(0x14B2)));


extern volatile __bit T4CS3 __attribute__((address(0x14B3)));


extern volatile __bit T4INPPS0 __attribute__((address(0xF4E8)));


extern volatile __bit T4INPPS1 __attribute__((address(0xF4E9)));


extern volatile __bit T4INPPS2 __attribute__((address(0xF4EA)));


extern volatile __bit T4INPPS3 __attribute__((address(0xF4EB)));


extern volatile __bit T4INPPS4 __attribute__((address(0xF4EC)));


extern volatile __bit T4MODE0 __attribute__((address(0x14A8)));


extern volatile __bit T4MODE1 __attribute__((address(0x14A9)));


extern volatile __bit T4MODE2 __attribute__((address(0x14AA)));


extern volatile __bit T4MODE3 __attribute__((address(0x14AB)));


extern volatile __bit T4MODE4 __attribute__((address(0x14AC)));


extern volatile __bit T4ON __attribute__((address(0x14A7)));


extern volatile __bit T4OUTPS0 __attribute__((address(0x14A0)));


extern volatile __bit T4OUTPS1 __attribute__((address(0x14A1)));


extern volatile __bit T4OUTPS2 __attribute__((address(0x14A2)));


extern volatile __bit T4OUTPS3 __attribute__((address(0x14A3)));


extern volatile __bit T4PSYNC __attribute__((address(0x14AF)));


extern volatile __bit T4RSEL0 __attribute__((address(0x14B8)));


extern volatile __bit T4RSEL1 __attribute__((address(0x14B9)));


extern volatile __bit T4RSEL2 __attribute__((address(0x14BA)));


extern volatile __bit T4RSEL3 __attribute__((address(0x14BB)));


extern volatile __bit T4RSEL4 __attribute__((address(0x14BC)));


extern volatile __bit TMD0 __attribute__((address(0x8A0)));


extern volatile __bit TMD1 __attribute__((address(0x8A1)));


extern volatile __bit TMD2 __attribute__((address(0x8A2)));


extern volatile __bit TMR0H0 __attribute__((address(0x2CE8)));


extern volatile __bit TMR0H1 __attribute__((address(0x2CE9)));


extern volatile __bit TMR0H2 __attribute__((address(0x2CEA)));


extern volatile __bit TMR0H3 __attribute__((address(0x2CEB)));


extern volatile __bit TMR0H4 __attribute__((address(0x2CEC)));


extern volatile __bit TMR0H5 __attribute__((address(0x2CED)));


extern volatile __bit TMR0H6 __attribute__((address(0x2CEE)));


extern volatile __bit TMR0H7 __attribute__((address(0x2CEF)));


extern volatile __bit TMR0IE __attribute__((address(0x38B5)));


extern volatile __bit TMR0IF __attribute__((address(0x3865)));


extern volatile __bit TMR0L0 __attribute__((address(0x2CE0)));


extern volatile __bit TMR0L1 __attribute__((address(0x2CE1)));


extern volatile __bit TMR0L2 __attribute__((address(0x2CE2)));


extern volatile __bit TMR0L3 __attribute__((address(0x2CE3)));


extern volatile __bit TMR0L4 __attribute__((address(0x2CE4)));


extern volatile __bit TMR0L5 __attribute__((address(0x2CE5)));


extern volatile __bit TMR0L6 __attribute__((address(0x2CE6)));


extern volatile __bit TMR0L7 __attribute__((address(0x2CE7)));


extern volatile __bit TMR0MD __attribute__((address(0x3CB8)));


extern volatile __bit TMR10 __attribute__((address(0x1060)));


extern volatile __bit TMR11 __attribute__((address(0x1061)));


extern volatile __bit TMR110 __attribute__((address(0x106A)));


extern volatile __bit TMR111 __attribute__((address(0x106B)));


extern volatile __bit TMR112 __attribute__((address(0x106C)));


extern volatile __bit TMR113 __attribute__((address(0x106D)));


extern volatile __bit TMR114 __attribute__((address(0x106E)));


extern volatile __bit TMR115 __attribute__((address(0x106F)));


extern volatile __bit TMR12 __attribute__((address(0x1062)));


extern volatile __bit TMR13 __attribute__((address(0x1063)));


extern volatile __bit TMR14 __attribute__((address(0x1064)));


extern volatile __bit TMR15 __attribute__((address(0x1065)));


extern volatile __bit TMR16 __attribute__((address(0x1066)));


extern volatile __bit TMR17 __attribute__((address(0x1067)));


extern volatile __bit TMR18 __attribute__((address(0x1068)));


extern volatile __bit TMR19 __attribute__((address(0x1069)));


extern volatile __bit TMR1GIE __attribute__((address(0x38D8)));


extern volatile __bit TMR1GIF __attribute__((address(0x3888)));


extern volatile __bit TMR1H0 __attribute__((address(0x1068)));


extern volatile __bit TMR1H1 __attribute__((address(0x1069)));


extern volatile __bit TMR1H2 __attribute__((address(0x106A)));


extern volatile __bit TMR1H3 __attribute__((address(0x106B)));


extern volatile __bit TMR1H4 __attribute__((address(0x106C)));


extern volatile __bit TMR1H5 __attribute__((address(0x106D)));


extern volatile __bit TMR1H6 __attribute__((address(0x106E)));


extern volatile __bit TMR1H7 __attribute__((address(0x106F)));


extern volatile __bit TMR1IE __attribute__((address(0x38D0)));


extern volatile __bit TMR1IF __attribute__((address(0x3880)));


extern volatile __bit TMR1L0 __attribute__((address(0x1060)));


extern volatile __bit TMR1L1 __attribute__((address(0x1061)));


extern volatile __bit TMR1L2 __attribute__((address(0x1062)));


extern volatile __bit TMR1L3 __attribute__((address(0x1063)));


extern volatile __bit TMR1L4 __attribute__((address(0x1064)));


extern volatile __bit TMR1L5 __attribute__((address(0x1065)));


extern volatile __bit TMR1L6 __attribute__((address(0x1066)));


extern volatile __bit TMR1L7 __attribute__((address(0x1067)));


extern volatile __bit TMR1MD __attribute__((address(0x3CB9)));


extern volatile __bit TMR1ON __attribute__((address(0x1070)));


extern volatile __bit TMR2IE __attribute__((address(0x38D1)));


extern volatile __bit TMR2IF __attribute__((address(0x3881)));


extern volatile __bit TMR2MD __attribute__((address(0x3CBA)));


extern volatile __bit TMR2ON __attribute__((address(0x1477)));


extern volatile __bit TMR4IE __attribute__((address(0x38D3)));


extern volatile __bit TMR4IF __attribute__((address(0x3883)));


extern volatile __bit TMR4MD __attribute__((address(0x3CBC)));


extern volatile __bit TMR4ON __attribute__((address(0x14A7)));


extern volatile __bit TRISA0 __attribute__((address(0x90)));


extern volatile __bit TRISA1 __attribute__((address(0x91)));


extern volatile __bit TRISA2 __attribute__((address(0x92)));


extern volatile __bit TRISA3 __attribute__((address(0x93)));


extern volatile __bit TRISA4 __attribute__((address(0x94)));


extern volatile __bit TRISA6 __attribute__((address(0x96)));


extern volatile __bit TRISA7 __attribute__((address(0x97)));


extern volatile __bit TRISB0 __attribute__((address(0x98)));


extern volatile __bit TRISB1 __attribute__((address(0x99)));


extern volatile __bit TRISB2 __attribute__((address(0x9A)));


extern volatile __bit TRISB3 __attribute__((address(0x9B)));


extern volatile __bit TRISB4 __attribute__((address(0x9C)));


extern volatile __bit TRISB5 __attribute__((address(0x9D)));


extern volatile __bit TRISB6 __attribute__((address(0x9E)));


extern volatile __bit TRISB7 __attribute__((address(0x9F)));


extern volatile __bit TRISC0 __attribute__((address(0xA0)));


extern volatile __bit TRISC1 __attribute__((address(0xA1)));


extern volatile __bit TRISC2 __attribute__((address(0xA2)));


extern volatile __bit TRISC3 __attribute__((address(0xA3)));


extern volatile __bit TRISC4 __attribute__((address(0xA4)));


extern volatile __bit TRISC6 __attribute__((address(0xA6)));


extern volatile __bit TRISC7 __attribute__((address(0xA7)));


extern volatile __bit TS __attribute__((address(0x24D2)));


extern volatile __bit TSEN __attribute__((address(0x4865)));


extern volatile __bit TSRNG __attribute__((address(0x4864)));


extern volatile __bit TUN0 __attribute__((address(0x4490)));


extern volatile __bit TUN1 __attribute__((address(0x4491)));


extern volatile __bit TUN2 __attribute__((address(0x4492)));


extern volatile __bit TUN3 __attribute__((address(0x4493)));


extern volatile __bit TUN4 __attribute__((address(0x4494)));


extern volatile __bit TUN5 __attribute__((address(0x4495)));


extern volatile __bit TX1IE __attribute__((address(0x38CC)));


extern volatile __bit TX1IF __attribute__((address(0x387C)));


extern volatile __bit TX1PPS0 __attribute__((address(0xF660)));


extern volatile __bit TX1PPS1 __attribute__((address(0xF661)));


extern volatile __bit TX1PPS2 __attribute__((address(0xF662)));


extern volatile __bit TX1PPS3 __attribute__((address(0xF663)));


extern volatile __bit TX1PPS4 __attribute__((address(0xF664)));


extern volatile __bit TX2IE __attribute__((address(0x38CE)));


extern volatile __bit TX2IF __attribute__((address(0x387E)));


extern volatile __bit TX2PPS0 __attribute__((address(0xF670)));


extern volatile __bit TX2PPS1 __attribute__((address(0xF671)));


extern volatile __bit TX2PPS2 __attribute__((address(0xF672)));


extern volatile __bit TX2PPS3 __attribute__((address(0xF673)));


extern volatile __bit TX2PPS4 __attribute__((address(0xF674)));


extern volatile __bit UA __attribute__((address(0xC79)));


extern volatile __bit UA1 __attribute__((address(0xC79)));


extern volatile __bit UART1MD __attribute__((address(0x3CD6)));


extern volatile __bit UART2MD __attribute__((address(0x3CD7)));


extern volatile __bit UTH0 __attribute__((address(0x470)));


extern volatile __bit UTH1 __attribute__((address(0x471)));


extern volatile __bit UTH10 __attribute__((address(0x47A)));


extern volatile __bit UTH11 __attribute__((address(0x47B)));


extern volatile __bit UTH12 __attribute__((address(0x47C)));


extern volatile __bit UTH13 __attribute__((address(0x47D)));


extern volatile __bit UTH14 __attribute__((address(0x47E)));


extern volatile __bit UTH15 __attribute__((address(0x47F)));


extern volatile __bit UTH2 __attribute__((address(0x472)));


extern volatile __bit UTH3 __attribute__((address(0x473)));


extern volatile __bit UTH4 __attribute__((address(0x474)));


extern volatile __bit UTH5 __attribute__((address(0x475)));


extern volatile __bit UTH6 __attribute__((address(0x476)));


extern volatile __bit UTH7 __attribute__((address(0x477)));


extern volatile __bit UTH8 __attribute__((address(0x478)));


extern volatile __bit UTH9 __attribute__((address(0x479)));


extern volatile __bit UTHR __attribute__((address(0x8AE)));


extern volatile __bit VB0GPR0 __attribute__((address(0x7460)));


extern volatile __bit VB0GPR1 __attribute__((address(0x7461)));


extern volatile __bit VB0GPR2 __attribute__((address(0x7462)));


extern volatile __bit VB0GPR3 __attribute__((address(0x7463)));


extern volatile __bit VB0GPR4 __attribute__((address(0x7464)));


extern volatile __bit VB0GPR5 __attribute__((address(0x7465)));


extern volatile __bit VB0GPR6 __attribute__((address(0x7466)));


extern volatile __bit VB0GPR7 __attribute__((address(0x7467)));


extern volatile __bit VB1GPR0 __attribute__((address(0x7468)));


extern volatile __bit VB1GPR1 __attribute__((address(0x7469)));


extern volatile __bit VB1GPR2 __attribute__((address(0x746A)));


extern volatile __bit VB1GPR3 __attribute__((address(0x746B)));


extern volatile __bit VB1GPR4 __attribute__((address(0x746C)));


extern volatile __bit VB1GPR5 __attribute__((address(0x746D)));


extern volatile __bit VB1GPR6 __attribute__((address(0x746E)));


extern volatile __bit VB1GPR7 __attribute__((address(0x746F)));


extern volatile __bit VB2GPR0 __attribute__((address(0x7470)));


extern volatile __bit VB2GPR1 __attribute__((address(0x7471)));


extern volatile __bit VB2GPR2 __attribute__((address(0x7472)));


extern volatile __bit VB2GPR3 __attribute__((address(0x7473)));


extern volatile __bit VB2GPR4 __attribute__((address(0x7474)));


extern volatile __bit VB2GPR5 __attribute__((address(0x7475)));


extern volatile __bit VB2GPR6 __attribute__((address(0x7476)));


extern volatile __bit VB2GPR7 __attribute__((address(0x7477)));


extern volatile __bit VB3GPR0 __attribute__((address(0x7478)));


extern volatile __bit VB3GPR1 __attribute__((address(0x7479)));


extern volatile __bit VB3GPR2 __attribute__((address(0x747A)));


extern volatile __bit VB3GPR3 __attribute__((address(0x747B)));


extern volatile __bit VB3GPR4 __attribute__((address(0x747C)));


extern volatile __bit VB3GPR5 __attribute__((address(0x747D)));


extern volatile __bit VB3GPR6 __attribute__((address(0x747E)));


extern volatile __bit VB3GPR7 __attribute__((address(0x747F)));


extern volatile __bit VBATBOR __attribute__((address(0x40A0)));


extern volatile __bit WA __attribute__((address(0xE86C)));


extern volatile __bit WCOL __attribute__((address(0xC87)));


extern volatile __bit WCOL1 __attribute__((address(0xC87)));


extern volatile __bit WDALRM0 __attribute__((address(0x60C0)));


extern volatile __bit WDALRM1 __attribute__((address(0x60C1)));


extern volatile __bit WDALRM2 __attribute__((address(0x60C2)));


extern volatile __bit WDAY0 __attribute__((address(0x6090)));


extern volatile __bit WDAY1 __attribute__((address(0x6091)));


extern volatile __bit WDAY2 __attribute__((address(0x6092)));


extern volatile __bit WDTCS0 __attribute__((address(0x406C)));


extern volatile __bit WDTCS1 __attribute__((address(0x406D)));


extern volatile __bit WDTCS2 __attribute__((address(0x406E)));


extern volatile __bit WDTPS0 __attribute__((address(0x4061)));


extern volatile __bit WDTPS1 __attribute__((address(0x4062)));


extern volatile __bit WDTPS2 __attribute__((address(0x4063)));


extern volatile __bit WDTPS3 __attribute__((address(0x4064)));


extern volatile __bit WDTPS4 __attribute__((address(0x4065)));


extern volatile __bit WDTPSCNT0 __attribute__((address(0x4070)));


extern volatile __bit WDTPSCNT1 __attribute__((address(0x4071)));


extern volatile __bit WDTPSCNT10 __attribute__((address(0x407A)));


extern volatile __bit WDTPSCNT11 __attribute__((address(0x407B)));


extern volatile __bit WDTPSCNT12 __attribute__((address(0x407C)));


extern volatile __bit WDTPSCNT13 __attribute__((address(0x407D)));


extern volatile __bit WDTPSCNT14 __attribute__((address(0x407E)));


extern volatile __bit WDTPSCNT15 __attribute__((address(0x407F)));


extern volatile __bit WDTPSCNT16 __attribute__((address(0x4080)));


extern volatile __bit WDTPSCNT17 __attribute__((address(0x4081)));


extern volatile __bit WDTPSCNT2 __attribute__((address(0x4072)));


extern volatile __bit WDTPSCNT3 __attribute__((address(0x4073)));


extern volatile __bit WDTPSCNT4 __attribute__((address(0x4074)));


extern volatile __bit WDTPSCNT5 __attribute__((address(0x4075)));


extern volatile __bit WDTPSCNT6 __attribute__((address(0x4076)));


extern volatile __bit WDTPSCNT7 __attribute__((address(0x4077)));


extern volatile __bit WDTPSCNT8 __attribute__((address(0x4078)));


extern volatile __bit WDTPSCNT9 __attribute__((address(0x4079)));


extern volatile __bit WDTSEN __attribute__((address(0x4060)));


extern volatile __bit WDTSTATE __attribute__((address(0x4082)));


extern volatile __bit WDTTMR0 __attribute__((address(0x4083)));


extern volatile __bit WDTTMR1 __attribute__((address(0x4084)));


extern volatile __bit WDTTMR2 __attribute__((address(0x4085)));


extern volatile __bit WDTTMR3 __attribute__((address(0x4086)));


extern volatile __bit WDTWINDOW0 __attribute__((address(0x4068)));


extern volatile __bit WDTWINDOW1 __attribute__((address(0x4069)));


extern volatile __bit WDTWINDOW2 __attribute__((address(0x406A)));


extern volatile __bit WERR __attribute__((address(0xE865)));


extern volatile __bit WFT __attribute__((address(0xE86F)));


extern volatile __bit WINDOW0 __attribute__((address(0x4068)));


extern volatile __bit WINDOW1 __attribute__((address(0x4069)));


extern volatile __bit WINDOW2 __attribute__((address(0x406A)));


extern volatile __bit WPOL __attribute__((address(0x24C4)));


extern volatile __bit WPUA0 __attribute__((address(0xF9C8)));


extern volatile __bit WPUA1 __attribute__((address(0xF9C9)));


extern volatile __bit WPUA2 __attribute__((address(0xF9CA)));


extern volatile __bit WPUA3 __attribute__((address(0xF9CB)));


extern volatile __bit WPUA4 __attribute__((address(0xF9CC)));


extern volatile __bit WPUA5 __attribute__((address(0xF9CD)));


extern volatile __bit WPUA6 __attribute__((address(0xF9CE)));


extern volatile __bit WPUA7 __attribute__((address(0xF9CF)));


extern volatile __bit WPUB0 __attribute__((address(0xFA20)));


extern volatile __bit WPUB1 __attribute__((address(0xFA21)));


extern volatile __bit WPUB2 __attribute__((address(0xFA22)));


extern volatile __bit WPUB3 __attribute__((address(0xFA23)));


extern volatile __bit WPUB4 __attribute__((address(0xFA24)));


extern volatile __bit WPUB5 __attribute__((address(0xFA25)));


extern volatile __bit WPUB6 __attribute__((address(0xFA26)));


extern volatile __bit WPUB7 __attribute__((address(0xFA27)));


extern volatile __bit WPUC0 __attribute__((address(0xFA78)));


extern volatile __bit WPUC1 __attribute__((address(0xFA79)));


extern volatile __bit WPUC2 __attribute__((address(0xFA7A)));


extern volatile __bit WPUC3 __attribute__((address(0xFA7B)));


extern volatile __bit WPUC4 __attribute__((address(0xFA7C)));


extern volatile __bit WPUC6 __attribute__((address(0xFA7E)));


extern volatile __bit WPUC7 __attribute__((address(0xFA7F)));


extern volatile __bit WPUE0 __attribute__((address(0xFB28)));


extern volatile __bit WPUE1 __attribute__((address(0xFB29)));


extern volatile __bit WPUE2 __attribute__((address(0xFB2A)));


extern volatile __bit WPUE3 __attribute__((address(0xFB2B)));


extern volatile __bit WR __attribute__((address(0x40F1)));


extern volatile __bit WREN __attribute__((address(0x40F2)));


extern volatile __bit WRERR __attribute__((address(0x40F3)));


extern volatile __bit WS __attribute__((address(0x24D1)));


extern volatile __bit WSEL0 __attribute__((address(0x24E8)));


extern volatile __bit WSEL1 __attribute__((address(0x24E9)));


extern volatile __bit WSEL2 __attribute__((address(0x24EA)));


extern volatile __bit WSEL3 __attribute__((address(0x24EB)));


extern volatile __bit WSEL4 __attribute__((address(0x24EC)));


extern volatile __bit YEARH0 __attribute__((address(0x6084)));


extern volatile __bit YEARH1 __attribute__((address(0x6085)));


extern volatile __bit YEARH2 __attribute__((address(0x6086)));


extern volatile __bit YEARH3 __attribute__((address(0x6087)));


extern volatile __bit YEARL0 __attribute__((address(0x6080)));


extern volatile __bit YEARL1 __attribute__((address(0x6081)));


extern volatile __bit YEARL2 __attribute__((address(0x6082)));


extern volatile __bit YEARL3 __attribute__((address(0x6083)));


extern volatile __bit ZCDIE __attribute__((address(0x38C6)));


extern volatile __bit ZCDIF __attribute__((address(0x3876)));


extern volatile __bit ZCDINTN __attribute__((address(0x48F8)));


extern volatile __bit ZCDINTP __attribute__((address(0x48F9)));


extern volatile __bit ZCDMD __attribute__((address(0x3CC0)));


extern volatile __bit ZCDOUT __attribute__((address(0x48FD)));


extern volatile __bit ZCDPOL __attribute__((address(0x48FC)));


extern volatile __bit ZCDSEN __attribute__((address(0x48FF)));


extern volatile __bit ZERO __attribute__((address(0x1A)));


extern volatile __bit Z_SHAD __attribute__((address(0xFF22)));


extern volatile __bit nA __attribute__((address(0xC7D)));


extern volatile __bit nA2 __attribute__((address(0xC7D)));


extern volatile __bit nADDRESS __attribute__((address(0xC7D)));


extern volatile __bit nADDRESS1 __attribute__((address(0xC7D)));


extern volatile __bit nBOR __attribute__((address(0x4098)));


extern volatile __bit nPD __attribute__((address(0x1B)));


extern volatile __bit nPOR __attribute__((address(0x4099)));


extern volatile __bit nRI __attribute__((address(0x409A)));


extern volatile __bit nRMCLR __attribute__((address(0x409B)));


extern volatile __bit nRWDT __attribute__((address(0x409C)));


extern volatile __bit nSYNC __attribute__((address(0x1072)));


extern volatile __bit nT1SYNC __attribute__((address(0x1072)));


extern volatile __bit nTO __attribute__((address(0x1C)));


extern volatile __bit nW __attribute__((address(0xC7A)));


extern volatile __bit nW2 __attribute__((address(0xC7A)));


extern volatile __bit nWDTWV __attribute__((address(0x409D)));


extern volatile __bit nWRITE __attribute__((address(0xC7A)));


extern volatile __bit nWRITE1 __attribute__((address(0xC7A)));
# 3334 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic_chip_select.h" 2 3
# 13 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 2 3
# 30 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 3
#pragma intrinsic(__nop)
extern void __nop(void);
# 78 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 3
__attribute__((__unsupported__("The " "FLASH_READ" " macro function is no longer supported. Please use the MPLAB X MCC."))) unsigned char __flash_read(unsigned short addr);

__attribute__((__unsupported__("The " "FLASH_WRITE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_write(unsigned short addr, unsigned short data);

__attribute__((__unsupported__("The " "FLASH_ERASE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_erase(unsigned short addr);



# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\eeprom_routines.h" 1 3
# 114 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\eeprom_routines.h" 3
extern void eeprom_write(unsigned char addr, unsigned char value);
extern unsigned char eeprom_read(unsigned char addr);
# 85 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 2 3






#pragma intrinsic(_delay)
extern __attribute__((nonreentrant)) void _delay(unsigned long);
#pragma intrinsic(_delaywdt)
extern __attribute__((nonreentrant)) void _delaywdt(unsigned long);

#pragma intrinsic(_delay3)
extern __attribute__((nonreentrant)) void _delay3(unsigned char);
# 137 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 3
extern __bank0 unsigned char __resetbits;
extern __bank0 __bit __powerdown;
extern __bank0 __bit __timeout;
# 27 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\xc.h" 2 3
# 49 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/device_config.h" 1
# 50 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/pin_manager.h" 1
# 90 "./mcc_generated_files/pin_manager.h"
void PIN_MANAGER_Initialize (void);
# 102 "./mcc_generated_files/pin_manager.h"
void PIN_MANAGER_IOC(void);
# 51 "./mcc_generated_files/mcc.h" 2

# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdint.h" 1 3
# 22 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 127 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uintptr_t;
# 142 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long intptr_t;
# 158 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;
# 173 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int32_t;





typedef long long int64_t;
# 188 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;
# 209 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 229 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 22 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdint.h" 2 3


typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 139 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/stdint.h" 1 3
typedef int32_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint32_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 139 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdint.h" 2 3
# 52 "./mcc_generated_files/mcc.h" 2

# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdbool.h" 1 3
# 53 "./mcc_generated_files/mcc.h" 2

# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\conio.h" 1 3







# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\errno.h" 1 3
# 10 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\errno.h" 3
extern int errno;
# 8 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\conio.h" 2 3

# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\__null.h" 1 3
# 9 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\conio.h" 2 3



extern void init_uart(void);

extern char getch(void);
extern char getche(void);
extern void putch(char);
extern void ungetch(char);

extern __bit kbhit(void);



extern char * cgets(char *);
extern void cputs(const char *);
# 54 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/dac1.h" 1
# 93 "./mcc_generated_files/dac1.h"
void DAC1_Initialize(void);
# 129 "./mcc_generated_files/dac1.h"
void DAC1_SetOutput(uint8_t inputData);
# 163 "./mcc_generated_files/dac1.h"
uint8_t DAC1_GetOutput(void);
# 55 "./mcc_generated_files/mcc.h" 2
# 70 "./mcc_generated_files/mcc.h"
void SYSTEM_Initialize(void);
# 83 "./mcc_generated_files/mcc.h"
void OSCILLATOR_Initialize(void);
# 96 "./mcc_generated_files/mcc.h"
void PMD_Initialize(void);
# 44 "main.c" 2

# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdio.h" 1 3
# 24 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdio.h" 3
# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 1 3





typedef void * va_list[1];




typedef void * __isoc_va_list[1];
# 137 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long ssize_t;
# 246 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long off_t;
# 399 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\bits/alltypes.h" 3
typedef struct _IO_FILE FILE;
# 24 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdio.h" 2 3
# 52 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.10\\pic\\include\\c99\\stdio.h" 3
typedef union _G_fpos64_t {
 char __opaque[16];
 double __align;
} fpos_t;

extern FILE *const stdin;
extern FILE *const stdout;
extern FILE *const stderr;





FILE *fopen(const char *restrict, const char *restrict);
FILE *freopen(const char *restrict, const char *restrict, FILE *restrict);
int fclose(FILE *);

int remove(const char *);
int rename(const char *, const char *);

int feof(FILE *);
int ferror(FILE *);
int fflush(FILE *);
void clearerr(FILE *);

int fseek(FILE *, long, int);
long ftell(FILE *);
void rewind(FILE *);

int fgetpos(FILE *restrict, fpos_t *restrict);
int fsetpos(FILE *, const fpos_t *);

size_t fread(void *restrict, size_t, size_t, FILE *restrict);
size_t fwrite(const void *restrict, size_t, size_t, FILE *restrict);

int fgetc(FILE *);
int getc(FILE *);
int getchar(void);
int ungetc(int, FILE *);

int fputc(int, FILE *);
int putc(int, FILE *);
int putchar(int);

char *fgets(char *restrict, int, FILE *restrict);

char *gets(char *);


int fputs(const char *restrict, FILE *restrict);
int puts(const char *);

#pragma printf_check(printf) const
#pragma printf_check(vprintf) const
#pragma printf_check(sprintf) const
#pragma printf_check(snprintf) const
#pragma printf_check(vsprintf) const
#pragma printf_check(vsnprintf) const

int printf(const char *restrict, ...);
int fprintf(FILE *restrict, const char *restrict, ...);
int sprintf(char *restrict, const char *restrict, ...);
int snprintf(char *restrict, size_t, const char *restrict, ...);

int vprintf(const char *restrict, __isoc_va_list);
int vfprintf(FILE *restrict, const char *restrict, __isoc_va_list);
int vsprintf(char *restrict, const char *restrict, __isoc_va_list);
int vsnprintf(char *restrict, size_t, const char *restrict, __isoc_va_list);

int scanf(const char *restrict, ...);
int fscanf(FILE *restrict, const char *restrict, ...);
int sscanf(const char *restrict, const char *restrict, ...);
int vscanf(const char *restrict, __isoc_va_list);
int vfscanf(FILE *restrict, const char *restrict, __isoc_va_list);
int vsscanf(const char *restrict, const char *restrict, __isoc_va_list);

void perror(const char *);

int setvbuf(FILE *restrict, char *restrict, int, size_t);
void setbuf(FILE *restrict, char *restrict);

char *tmpnam(char *);
FILE *tmpfile(void);




FILE *fmemopen(void *restrict, size_t, const char *restrict);
FILE *open_memstream(char **, size_t *);
FILE *fdopen(int, const char *);
FILE *popen(const char *, const char *);
int pclose(FILE *);
int fileno(FILE *);
int fseeko(FILE *, off_t, int);
off_t ftello(FILE *);
int dprintf(int, const char *restrict, ...);
int vdprintf(int, const char *restrict, __isoc_va_list);
void flockfile(FILE *);
int ftrylockfile(FILE *);
void funlockfile(FILE *);
int getc_unlocked(FILE *);
int getchar_unlocked(void);
int putc_unlocked(int, FILE *);
int putchar_unlocked(int);
ssize_t getdelim(char **restrict, size_t *restrict, int, FILE *restrict);
ssize_t getline(char **restrict, size_t *restrict, FILE *restrict);
int renameat(int, const char *, int, const char *);
char *ctermid(char *);







char *tempnam(const char *, const char *);
# 45 "main.c" 2







void main(void)
{

    SYSTEM_Initialize();
# 71 "main.c"
    uint8_t count=0;

    DAC1_Initialize();

    while (1)
    {

        for(count=0; count<=30; count++)
        {
            DAC1_SetOutput(count);
            _delay((unsigned long)((500)*(1000000/4000.0)));
        }

    }
}
