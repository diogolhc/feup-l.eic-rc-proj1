#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "common.h"


int main(int argc, char** argv) {
    int fd, res;
    //int c;
    struct termios oldtio,newtio;
    //char buf[255];

    if (argc != 2) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS<i>\n");
        exit(1);
    }


  /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
  */
  
    
    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) {
        perror(argv[1]);
        exit(-1);
    }

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */



  /* 
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a 
    leitura do(s) próximo(s) caracter(es)
  */



    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    /// STARTS HERE
    unsigned char UA[5];

    UA[0] = FLAG;
    UA[1] = A;
    UA[2] = C_UA;
    UA[3] = UA[1]^UA[2];
    UA[4] = FLAG;

    // TODO should the receiver also time out?
    state_set_ua_t state = START;
    while (state != STOP) {
        unsigned char byte_read = 0;
        res = read(fd, &byte_read, 1);

        if (res == 1) {
            if (update_state_set_ua(C_SET, &state, byte_read) != 0) {
                return 1;
            }
        } else {
            printf("DEBUG: not supposed to happen\n");
        }
    }
    
    if (write(fd, UA, 5) < 0) {
        perror("");
        return 1;
    }

    printf("ACK\n");


  /* 
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião 
  */



    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
