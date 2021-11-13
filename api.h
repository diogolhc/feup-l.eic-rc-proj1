#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define FLAG 0x7E
#define A 0x03
#define C_SET 0x03
#define C_UA 0X07
#define SET_SIZE 5

#define TIME_OUT_TIME 3


typedef enum type {
    TRANSMITTER,
    RECEIVER
} type_t;


int llopen(int porta, type_t type);

int llclose(int fd);
