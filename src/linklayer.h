#include <stdint.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define DATA_PACKET_MAX_SIZE 1000

typedef enum type {
    TRANSMITTER,
    RECEIVER
} type_t;

int llopen(int porta, type_t type);

int llclose(int fd, type_t type);

int llwrite(int fd, uint8_t *buffer, int length);

int llread(int fd, uint8_t *buffer);
