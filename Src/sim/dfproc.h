#define SPEED_232  9600

// для кварца 16МГц значения для UBRR
#define R115200_H  0x00
#define R115200_L  0x08

#define R19200_H  0x00
#define R19200_L  51

#define R38400_H  0x00
#define R38400_L  25


// для 6 Мгц кварца пересчет 
//#define R9600_H  0x00    для 16 Мгц
//#define R9600_L  103

#define R9600_H  0x00
#define R9600_L  38

//#define R4800_H  0x00
//#define R4800_L  207

#define R4800_H  0x00
#define R4800_L  77

//#define R2400_H  0x01
//#define R2400_L  0x9f

#define R2400_H  0x00
#define R2400_L  155



//для портов
#define RXCIE 0x80
#define TXCIE 0x40
#define UDRIE 0x20
#define RXEN 0x10
#define TXEN 0x08
#define TXC  0x40



//#define SDA      0x08
//#define SCL      0x04




