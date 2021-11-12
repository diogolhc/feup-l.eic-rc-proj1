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
#include <signal.h>

typedef enum state_set_ua {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} state_set_ua_t;


static struct termios oldtio;
static volatile int g_count = 0;

static unsigned char SET[SET_SIZE] = {
    FLAG,
    A,
    C_SET,
    A ^ C_SET,
    FLAG
};

static unsigned char UA[SET_SIZE] = {
    FLAG,
    A,
    C_UA,
    A ^ C_UA,
    FLAG
};

// c is to pass the C used (SET or UA)
static int update_state_set_ua(unsigned char c, state_set_ua_t *state, unsigned char byte) {
    if (state == NULL) {
        return 1;
    }

    switch (*state) {
    case START:
        if (byte == FLAG) {
            *state = FLAG_RCV;
        }
        break;
    
    case FLAG_RCV:
        if (byte == A) {
            *state = A_RCV;
        } else if (byte != FLAG) {
            *state = START;
        }
        break;
    
    case A_RCV:
        if (byte == c) {
            *state = C_RCV;
        } else if (byte == FLAG) {
            *state = FLAG_RCV;
        } else {
            *state = START;
        }
        break;

    case C_RCV:
        if (byte == (A^c)) {
            *state = BCC_OK;
        } else if (byte == FLAG) {
            *state = FLAG_RCV;
        } else {
            *state = START;
        }
        break;

    case BCC_OK:
        if (byte == FLAG) {
            *state = STOP;
        } else {
            *state = START;
        }
        break;
    
    case STOP:
        break;
    }

    return 0;
}

static void time_out() {
    printf("alarme # %d\n", g_count);
    g_count++;
}

static int setup_alarm() {
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

static int common_open(int porta) {
    int fd = -1;
    struct termios newtio;
    
    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */

    char buffer[20];
    if (sprintf(buffer, "/dev/ttyS%d", porta) < 0) {
        perror("");
        return 1;
    }


    fd = open(buffer, O_RDWR | O_NOCTTY );
    if (fd < 0) {
        perror(buffer);
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

    return fd;
}

int llopen(int porta, type_t type) {
    state_set_ua_t state;
    int fd = common_open(porta);


    // TODO break into 2?
    switch (type) {
    case TRANSMITETR:
        if (setup_alarm() != 0) {
            return 1;
        }

        int ua_received = FALSE;
        int res;
        while (g_count < 3 && !ua_received) {
            state = START; // TODO should the state reset every time? or mantain after sending other SET?

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
        break;
    
    case RECEIVER:
        // TODO should the receiver also time out?
        state = START;
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
        break;
    }

    return fd;
}

int llclose(int fd) {
    // TODO not complete yet...
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    close(fd);

    return 0;
}
