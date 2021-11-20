#include "aplic.h"
#include "api.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <unistd.h>

#define C_DATA 0x1
#define C_START 0x2
#define C_END 0x3

#define N(seq) ((seq) % 255)

#define L1(K) ((K) & 0b11111111)
#define L2(K) (((K) >> 8) & 0b11111111)

#define T_FILE_SIZE 0x0
#define T_FILE_NAME 0x1

#define L(v) (v)

// TODO check if the return with errors should return also or enforce here other attempts


int send_file(int porta, char *path, int path_size) {

    // TODO open file, get name, and size

    int fd_serial_port;
    if ((fd_serial_port = llopen(porta, TRANSMITTER)) < 0) {
        return -1;
    }

    // TODO send file in packets

    if (llclose(fd_serial_port) < 0) {
        return -1;
    }

    // TODO close file

    return 0;
}


int receive_file(int porta) {
    int fd;
    if ((fd = llopen(porta, RECEIVER)) < 0) {
        return -1;
    }

    // TODO

    if (llclose(fd) < 0) {
        return -1;
    }

    return 0;
}
