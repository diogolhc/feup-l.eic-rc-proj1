#include <stdint.h>

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

#define C_RR(r) (0x05 | (((r) << 7) & 0x80))
#define C_REJ(r) (0x01 | (((r) << 7) & 0x80))
#define C_I(s) ((s) << 6)

#define CONTROL_SIZE 5

#define TMP_BCC1 0x66 // TODO temp value, not sure what this should be
#define TMP_BCC2 0x77 // TODO same as above

#define STUFFER 0x20

#define TIME_OUT_TIME 3

#define DATA_PACKET_MAX_SIZE 1000 // TODO define a good number 


typedef enum control_frame_type {
    SET,
    DISC,
    UA,
    RR,
    REJ
}  control_frame_type_t;

typedef enum type {
    TRANSMITTER,
    RECEIVER
} type_t;

int llopen(int porta, type_t type);

int llclose(int fd);

int llwrite(int fd, uint8_t *buffer, int length);

int llread(int fd, uint8_t *buffer);
