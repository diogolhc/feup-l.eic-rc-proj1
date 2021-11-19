#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define FLAG 0x7E
#define ESC 0x7D
#define A 0x03
#define C_SET 0x03
#define C_UA 0X07
#define C_I(s) (s << 6)
#define SET_SIZE 5

#define TMP_BCC1 0x66 // TODO temp value, not sure what this should be
#define TMP_BCC2 0x77 // TODO same as above

#define STUFFER 0x20

#define TIME_OUT_TIME 3


typedef enum type {
    TRANSMITTER,
    RECEIVER
} type_t;


int llopen(int porta, type_t type);

int llclose(int fd);

int message_stuffing(char in_msg[], unsigned int in_msg_size, char ** out_msg);

char bcc2_builder(char msg[], unsigned int msg_size);

int llwrite(int fd, char * buffer, int length);

int llread(int fd, char * buffer);
