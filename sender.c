#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include "common.h"

volatile int g_count = 0;

//=======================================
unsigned char SET[SET_SIZE] = {
    FLAG,
    A,
    C_SET,
    A ^ C_SET,
    FLAG
};

//=======================================

void time_out() {
    printf("alarme # %d\n", g_count);
    g_count++;
}


int setup_alarm() {
    struct sigaction new;
    sigset_t smask;

    if (sigemptyset(&smask)==-1) {
        perror ("sigsetfunctions");
        return 1;
    }
        
    new.sa_handler = time_out;
    new.sa_mask = smask;
    new.sa_flags = 0;

    if (sigaction(SIGALRM, &new, NULL) == -1) {
        perror ("sigaction");
        return 1;
    }

    return 0;
}


int main(int argc, char** argv) {
    int fd, res;
    struct termios oldtio,newtio;
    //char buf[255];
    //int i, sum = 0, speed = 0;
    
    if (argc != 2) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS<i>\n");
        exit(1);
    }

    if (setup_alarm() != 0) {
        return 1;
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
    leitura do(s) pr�ximo(s) caracter(es)
  */

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    int ua_received = FALSE;
    while (g_count < 3 && !ua_received) {
        state_set_ua_t state = START; // TODO should the state reset every time? or mantain after sending other SET?

        res = write(fd, SET, SET_SIZE * sizeof(unsigned char));   
        printf("%d bytes written\n", res);

        alarm(TIME_OUT_TIME);

        int timed_out = FALSE;
        while (!timed_out) {
            unsigned char byte_read = 0;

            res = read(fd, &byte_read, 1);
            if (res == 1) {
                if (update_state_set_ua(C_UA, &state, byte_read) != 0) {
                    return 1;
                }
                ua_received = (state==STOP);

            } else if (res == -1) {
                timed_out = TRUE;

            } else {
                printf("DEBUG: not supposed to happen\n");
            }
        }
        
        alarm(0);
    }

    if (ua_received) {
        printf("ACK\n");
    }


    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 0;
}
