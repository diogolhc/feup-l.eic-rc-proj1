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

typedef enum state_info_rcv {
    START_I,
    FLAG_RCV_I,
    A_RCV_I,
    C_RCV_I,
    DATA_COLLECTION, //Also BB1_OK
    DESTUFFING,
    BCC2_TEST,
    BCC1_INVALID,
    REJ,
    RR,
    STOP_I
} state_info_rcv_t;

static struct termios oldtio;
static volatile int g_count = 0;

static int Ns = 0; // TODO leave (both) global?
static int Nr = 0;

static const unsigned char SET[SET_SIZE] = {
    FLAG,
    A,
    C_SET,
    A ^ C_SET,
    FLAG
};

static const unsigned char UA[SET_SIZE] = {
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

static int update_state_info_rcv(state_info_rcv_t *state, unsigned char byte){
    
    switch (*state){

        case (START_I):
            if (byte == FLAG) *state = FLAG_RCV_I;
            else *state = REJ;
            break;

        case (FLAG_RCV_I):
            if (byte == A) *state = A_RCV_I;
            else *state = REJ;
            break;

        case (A_RCV_I):
            if (C_I(Nr) == byte) *state = C_RCV_I;
            else *state = REJ;
            break;

        case (C_RCV_I):
            if (A ^ C_I(Nr)) *state = DATA_COLLECTION; // TODO the cndition won't be this one, it's just for testing purposes
            else *state = BCC1_INVALID;
            break;

        case (DATA_COLLECTION):
            if (byte == FLAG) *state = BCC2_TEST;
            else if (byte == ESC) *state = DESTUFFING;
            else *state = DATA_COLLECTION;
            break;

        case (DESTUFFING):
            *state = DATA_COLLECTION; // TODO should probably test for FLAG || ESC
            break;

        case (BCC2_TEST): 
            if (byte) *state = RR; // TODO byte acts as bool 1 for valid 0 invalid
            else *state = REJ;
            break;

        case (BCC1_INVALID): // TODO should we rather pass the current message and the old message and compare them here instead of outside?
            if (byte) *state = RR; // TODO if (byte) then it's a repeated msg
            else *state = REJ;
            break;

        case (REJ): // TODO change r 
            *state = STOP_I;
            break;

        case (RR):
            *state = STOP_I;
            break;

        default:
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
    case TRANSMITTER:
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
            while (!timed_out && state != STOP) {
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

int message_stuffing(char in_msg[], unsigned int in_msg_size, char ** out_msg){

    int size_counter = 0;

    *out_msg = malloc(in_msg_size*2);

   char * out_message = * out_msg;

    for (int i = 0; i < in_msg_size; i++){
        switch (in_msg[i]){
            case FLAG:
                out_message[size_counter++] = ESC;
                out_message[size_counter++] = FLAG ^ STUFFER;
                break;
            case ESC:
                out_message[size_counter++] = ESC;
                out_message[size_counter++] = ESC ^ STUFFER;
                break;
            default:
                out_message[size_counter++] = in_msg[i];
                break;
        }
    }
    return size_counter;
}

char bcc2_builder(char msg[], unsigned int msg_size){

    if (msg_size == 1) {
        return msg[0];
    } else if ( msg_size < 0) {
        return 0;
    }

    char ret = msg[0];

    for (int i = 1; i < msg_size; i++){
        ret ^= msg[i];
    }

    return ret;
}

int llwrite(int fd, char * buffer, int length){

    int rcv_nr = 0; // TODO read from the receiver response

    char * stuffed_msg; 
    int stuffed_msg_len = message_stuffing(buffer, length, &stuffed_msg); //memory is allocated formstuffed_msg, dont forget to free it

    int total_msg_len = stuffed_msg_len + 6;
    char * info_msg = malloc(total_msg_len); // 6 gotten from { F, A, C, BCC1, D1...DN, BCC2, F}

    info_msg[0] = FLAG;
    info_msg[1] = A;
    info_msg[2] = C_I(Ns);
    info_msg[3] = A ^ C_I(Ns);   
    memcpy(&(info_msg[4]), stuffed_msg, stuffed_msg_len);
    info_msg[total_msg_len-2] = bcc2_builder(stuffed_msg, stuffed_msg_len);
    info_msg[total_msg_len-1] = FLAG;

    for (int i = 0; i < total_msg_len; i++){
        printf("%d: 0x%x\n", i, info_msg[i]);
    }

    write(fd, info_msg, total_msg_len * sizeof(char));

    free(info_msg);
    free(stuffed_msg);

    return rcv_nr;
}

int llread(int fd, char * buffer) {

    state_info_rcv_t state;
    state = START_I;

    char byte_read = 0;

    char temp_buffer[512]; // TODO what should the size be?

    int res = 0;

    int msg_size = 0;

    while(state != BCC2_TEST && state != REJ){ //READS message
        res = read(fd, &byte_read, 1);

        // TODO guardas e what not


        update_state_info_rcv(&state, byte_read);

        printf("BYTE: 0x%x; STATE: %d\n", byte_read, state);

        temp_buffer[msg_size] = byte_read;

        msg_size++;
    }

    char * stuffed_msg[msg_size - 6];

    while(state != STOP_I){ //TESTS BCC2

        switch(state){

            case (BCC2_TEST):
                memcpy(stuffed_msg, &(temp_buffer[4]), msg_size-6);
                update_state_info_rcv(&state, temp_buffer[msg_size-2] == bcc2_builder(stuffed_msg, msg_size - 6));
                break;

            case (REJ):
                update_state_info_rcv(&state, 0);
                printf("RES: REJ\n");
                break;

            case (RR):
                update_state_info_rcv(&state, 0);
                printf("RES: RR\n");
                break;
        }

    }

    buffer = malloc(msg_size);

    memcpy(buffer, &(temp_buffer[0]), msg_size);

    return msg_size;
}
