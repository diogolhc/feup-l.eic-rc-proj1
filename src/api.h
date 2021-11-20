#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define FLAG 0x7E
#define ESC 0x7D
#define A 0x03
#define C_SET 0x03
#define C_DISC 0x0B
#define C_UA 0X07
#define C_RR(Nr)  ((Nr << 7) | 0b101)
#define C_REJ(Nr) ((Nr << 7) | 0b1)
#define SET_SIZE 5

#define TIME_OUT_TIME 3


typedef enum type {
    TRANSMITTER,
    RECEIVER
} type_t;


int llopen(int porta, type_t type);

int llclose(int fd);

int message_stuffing(unsigned char in_msg[], unsigned int in_msg_size, unsigned char ** out_msg);
