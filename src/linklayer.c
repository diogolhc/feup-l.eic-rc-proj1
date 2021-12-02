#include "linklayer.h"
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
#include <stdint.h>

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

#define STUFFER 0x20

#define TIME_OUT_TIME 3
#define MAX_NO_TIMEOUT 3

#define HEADER_AND_TAIL_SIZE 10 // more than enough

typedef enum control_frame_type {
    SET,
    DISC,
    UA,
    RR,
    REJ
}  control_frame_type_t;

typedef enum state_sv_frame {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    RR_RCV,
    REJ_RCV,
    BCC_OK,
    STOP
} state_sv_frame_t;

typedef enum state_info_rcv {
    I_START,
    I_GOT_FLAG,
    I_IGNORE,
    I_GOT_A,
    I_GOT_C,
    I_GOT_BCC1,
    I_DATA_COLLECTION,
    I_GOT_ESC,
    I_GOT_END_FLAG,
    I_TEST_DUP_RR,
    I_TEST_DUP_REJ,
    I_RR_DONT_STORE,
    I_RR_STORE,
    I_REJ,
    I_STOP
} state_info_rcv_t;

static struct termios oldtio;
static volatile int g_count = 0;

static uint8_t S = 0;
static uint8_t next_S = 0;
static uint8_t R = 0;

static void control_frame_builder(control_frame_type_t cft, uint8_t msg[]){
    msg[0] = FLAG; 
    msg[1] = A;
    
    switch (cft) {
    case SET:
        msg[2] = C_SET;
        break;
    
    case DISC:
        msg[2] = C_DISC;
        break;
    
    case UA:
        msg[2] = C_UA;
        break;

    case RR: 
        msg[2] = C_RR(R);
        break;

    case REJ:
        msg[2] = C_REJ(R);
        break;

    default:
        break;
    }

    msg[3] = msg[1] ^ msg[2];
    msg[4] = FLAG;
}

static int update_state_rr_rej(state_sv_frame_t *state, uint8_t byte) {

    if (state == NULL) {
        return 1;
    }
    switch (*state) {

        case START:
            if (byte == FLAG) *state = FLAG_RCV;
            else *state = START;
            break;
        
        case FLAG_RCV:
            if (byte == A) *state = A_RCV;
            else *state = START;
            break;
        
        case A_RCV:
            if ((byte & 0x0F) == 0x05) *state = RR_RCV;
            else if ((byte & 0x0F) == 0x01) *state = REJ_RCV;
            else *state = START;
            next_S = (byte >> 7) & 0x01;
            break;

        case RR_RCV:
            if (byte == (A^C_RR(next_S))) *state = BCC_OK;
            else if (byte == FLAG) *state = FLAG_RCV;
            else *state = START;
            break;

        case REJ_RCV:
            if (byte == (A^C_REJ(next_S))) *state = BCC_OK;
            else if (byte == FLAG) *state = FLAG_RCV;
            else *state = START;
            break;

        case BCC_OK:
            if (byte == FLAG) *state = STOP;
            else *state = START;
            break;
        
        case STOP:
            break;

        default:
            printf("ERROR: not supposed to reach this\n");
            break;
    }

    return 0;
}

static int update_state_set_ua(uint8_t c, state_sv_frame_t *state, uint8_t byte) {
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
            
        default:
            printf("ERROR: not supposed to reach this\n");
            break;
    }

    
    return 0;
}

static int update_state_info_rcv(state_info_rcv_t *state, uint8_t byte){
    
    switch (*state){
        case (I_START):
            if (byte == FLAG) *state = I_GOT_FLAG;
            else *state = I_IGNORE;
            break;
        
        case (I_GOT_FLAG):
            if (byte == FLAG) *state = I_GOT_FLAG;
            else if (byte == A) *state = I_GOT_A;
            else *state = I_IGNORE;
            break;

        case (I_IGNORE):
            if (byte == FLAG) *state = I_GOT_FLAG;
            else *state = I_IGNORE;
            break;

        case (I_GOT_A):
            if (byte == C_I(R)) * state = I_GOT_C;
            else *state = I_IGNORE;
            break;

        case (I_GOT_C):
            if (byte == (C_I(R)^A)) *state = I_GOT_BCC1;
            else *state = I_IGNORE;
            break;

        case (I_GOT_BCC1):
            if (byte == ESC) *state = I_GOT_ESC;
            else *state = I_DATA_COLLECTION;
            break;

        case (I_DATA_COLLECTION):
            if (byte == FLAG) *state = I_GOT_END_FLAG;
            else if (byte == ESC) *state = I_GOT_ESC;
            else *state = I_DATA_COLLECTION;  
            break;
        
        case (I_GOT_ESC): 
            *state = I_DATA_COLLECTION;
            break;
        
        case (I_GOT_END_FLAG):
            if (byte) *state = I_TEST_DUP_RR; // byte = is bcc2 valid ?
            else *state = I_TEST_DUP_REJ;
            break;
        
        case (I_TEST_DUP_RR):
            if (byte) *state = I_RR_DONT_STORE; // byte = is dup ?
            else *state = I_RR_STORE;
            break;

        case (I_TEST_DUP_REJ):
            if (byte) *state = I_RR_DONT_STORE; // byte = is dup ?
            else *state = I_REJ;
            break;

        case (I_RR_DONT_STORE):
            *state = I_START;
            break;

        case (I_RR_STORE):
            *state = I_STOP;
            break;

        case (I_REJ):
            *state = I_START;
            break;
        
        case (I_STOP):
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
        return -1;
    }

    if (tcgetattr(fd, &oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        return -1;
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 1 char received */

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    printf("New termios structure set\n");

    return fd;
}

static int common_close(int fd) {
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return close(fd);
}

static void R_invert(){
    R = ((!R) << 7) >> 7;
}

int llopen(int porta, type_t type) {
    state_sv_frame_t state;
    int fd = common_open(porta);
    if (fd < 0) {
        printf("Failed to open serial port.\n");
        return -1;
    }

    printf("%d opened fd\n", fd);

    uint8_t set[CONTROL_SIZE];
    control_frame_builder(SET, set);

    uint8_t ua[CONTROL_SIZE];
    control_frame_builder(UA, ua);

    if (setup_alarm() != 0) {
        common_close(fd);
        return -1;
    }
    g_count = 0;

    switch (type) {
    case TRANSMITTER:;
        int ua_received = FALSE;
        int res;
        while (g_count < MAX_NO_TIMEOUT && !ua_received) {
            state = START;

            res = write(fd, set, CONTROL_SIZE * sizeof(uint8_t));
            if (res == -1) {
                printf("llopen() -> write() TRANSMITTER error\n");
                common_close(fd);
                return -1;
            }
            printf("SET sent.\n");
            printf("%d bytes written\n", res);

            alarm(TIME_OUT_TIME);

            int timed_out = FALSE;
            while (!timed_out && state != STOP) {
                uint8_t byte_read = 0;

                res = read(fd, &byte_read, 1);
                if (res == 1) {
                    if (update_state_set_ua(C_UA, &state, byte_read) != 0) {
                        common_close(fd);
                        return -1;
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
            printf("UA received.\n");
            printf("ACK\n");
        }
        break;
    
    case RECEIVER:
        alarm(TIME_OUT_TIME * MAX_NO_TIMEOUT);
        state = START;
        while (state != STOP) {
            uint8_t byte_read = 0;
            res = read(fd, &byte_read, 1);

            if (res == 1) {
                if (update_state_set_ua(C_SET, &state, byte_read) != 0) {
                    common_close(fd);
                    return -1;
                }
            } else if (res == -1) {
                if (g_count > 0) {
                    printf("llopen timedout\n");
                } else {
                    printf("llopen() -> read() RECEIVER error\n");
                }
                common_close(fd);
                return -1;
            } else {
                printf("DEBUG: not supposed to happen\n");
            }
        }
        
        printf("SET received.\n");
        if (write(fd, ua, CONTROL_SIZE) < 0) {
            printf("llopen() -> write() RECEIVER error\n");
            common_close(fd);
            return -1;
        }

        printf("UA sent.\n");
        printf("ACK\n");
        break;
    }

    return fd;
}

int llclose(int fd, type_t type) {
    state_sv_frame_t state;
    uint8_t disc[CONTROL_SIZE];
    control_frame_builder(DISC, disc);

    uint8_t ua[CONTROL_SIZE];
    control_frame_builder(UA, ua);

    int res = 0;
    int disc_received = FALSE;
    g_count = 0;

    switch (type) {
    case TRANSMITTER:
        while (g_count < MAX_NO_TIMEOUT && !disc_received) {
            state = START;

            res = write(fd, disc, CONTROL_SIZE * sizeof(uint8_t));
            if (res == -1) {
                printf("llclose() -> write() TRANSMITTER error\n");
                return -1;
            }
            printf("DISC sent.\n");
            printf("%d bytes written\n", res);

            alarm(TIME_OUT_TIME);

            int timed_out = FALSE;
            while (!timed_out && state != STOP) {
                uint8_t byte_read = 0;

                res = read(fd, &byte_read, 1);
                if (res == 1) {
                    if (update_state_set_ua(C_DISC, &state, byte_read) != 0) {
                        close(fd);
                        return -1;
                    }
                    disc_received = (state==STOP);

                } else if (res == -1) {
                    timed_out = TRUE;

                } else {
                    printf("DEBUG: not supposed to happen\n");
                }
            }
            
            alarm(0);
        }

        if (disc_received) {
            printf("DISC received.\n");
            res = write(fd, ua, CONTROL_SIZE * sizeof(uint8_t));
            printf("UA sent.\n");
        } else {
            printf("DISC not received.\n");
        }

        break;
    
    case RECEIVER:
        alarm(TIME_OUT_TIME * MAX_NO_TIMEOUT);
        state = START;
        while (state != STOP) {
            uint8_t byte_read = 0;

            res = read(fd, &byte_read, 1);
            if (res == 1) {
                if (update_state_set_ua(C_DISC, &state, byte_read) != 0) {
                    close(fd);
                    return -1;
                }
                disc_received = (state==STOP);
            } else if (res == -1) {
                if (g_count > 0) {
                    printf("llclose timedout\n");
                } else {
                    printf("llclose() -> read() RECEIVER error\n");
                }
                break;
            } else {
                printf("DEBUG: not supposed to happen\n");
            }
        }

        if (res != -1) {
            printf("DISC received.\n");
            res = write(fd, disc, CONTROL_SIZE * sizeof(uint8_t));
            printf("DISC sent.\n");
        }
        // no need to wait for UA
        break;
    }

    res = common_close(fd);

    return res;
}

int message_stuffing(uint8_t in_msg[], unsigned int in_msg_size, uint8_t ** out_msg){

    int size_counter = 0;
    *out_msg = malloc(in_msg_size*2);

   uint8_t * out_message = * out_msg;

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

int message_destuffer(uint8_t in_msg[], unsigned int in_msg_size, uint8_t ** out_msg){

    int size_counter = 0;
    *out_msg = malloc(in_msg_size);

    uint8_t * out_message = * out_msg;

    for (int i = 0; i < in_msg_size; i++){
        if (in_msg[i] == ESC){
            out_message[size_counter] = (in_msg[++i] ^ STUFFER);
        } else {
            out_message[size_counter] = in_msg[i];
        }
        size_counter++;
    }

    return size_counter;
}

uint8_t bcc2_builder(uint8_t msg[], unsigned int msg_size){

    if (msg_size == 1) {
        return msg[0];
    } else if ( msg_size < 0) {
        return 0;
    }

    uint8_t ret = msg[0];

    for (int i = 1; i < msg_size; i++){
        ret ^= msg[i];
    }

    return ret;
}

int llwrite(int fd, uint8_t * buffer, int length){

    int write_successful = 0;
    int ret = 0;
    uint8_t bcc2 = bcc2_builder(buffer, length);
    uint8_t *unstuffed_msg = malloc((length+1) * sizeof(uint8_t));
    memcpy(unstuffed_msg, buffer, length);
    unstuffed_msg[length] = bcc2;
    uint8_t *stuffed_msg = NULL; 
    int stuffed_msg_len = message_stuffing(unstuffed_msg, length+1, &stuffed_msg);
    free(unstuffed_msg);
    int total_msg_len = stuffed_msg_len + CONTROL_SIZE;
    uint8_t *info_msg = malloc(total_msg_len);

    info_msg[0] = FLAG;
    info_msg[1] = A;
    info_msg[2] = C_I(S);
    info_msg[3] = A ^ C_I(S);
    memcpy(&(info_msg[4]), stuffed_msg, stuffed_msg_len);
    info_msg[total_msg_len-1] = FLAG;

    setup_alarm();
    g_count = 0;

    while(!write_successful && g_count < MAX_NO_TIMEOUT) {

        printf("----- TASK: WRITING MESSAGE\n");

        if (write(fd, info_msg, total_msg_len * sizeof(uint8_t)) == -1) {
            printf("llwrite() -> write() error\n");
            free(info_msg);
            free(stuffed_msg);
            return -1;
        }

        printf("----- TASK: DONE\n");

        uint8_t byte_read = 0;
        int res = 0;
        state_sv_frame_t state = START;

        printf("----- TASK: READING REPLY\n");

        alarm(TIME_OUT_TIME);

        while(state != STOP){
            res = read(fd, &byte_read, 1);

            if (res == -1) {
                write_successful = 0;
                break;
            }

            update_state_rr_rej(&state, byte_read);
            printf("BYTE: 0x%x; STATE: %d\n", byte_read, state);

            if (state == RR_RCV) {
                write_successful = 1;
            } else if (state == REJ_RCV) {
                write_successful = 0;
            }
        }

        printf("----- TASK: DONE\n");
    }

    alarm(0);

    S = next_S;

    free(info_msg);
    free(stuffed_msg);

    if (g_count >= MAX_NO_TIMEOUT) {
        ret = -1;
    } else {
        ret = total_msg_len;
    }

    return ret;
}

int llread(int fd, uint8_t *buffer) {

    state_info_rcv_t state;
    uint8_t byte_read = 0;
    uint8_t data_read[DATA_PACKET_MAX_SIZE * 2 + HEADER_AND_TAIL_SIZE];
    int msg_size = 0;

    uint8_t *unstuffed_msg = NULL;
    int unstuffed_size = 0;

    setup_alarm();
    g_count = 0;

    state = I_START;

    printf("--- NEW READ ---\n");

    while (state != I_STOP){

        printf("--- TRY READ ---\n");

        alarm(TIME_OUT_TIME * MAX_NO_TIMEOUT);

        msg_size = 0;
        int rcv_s = -1;

        while (state != I_GOT_BCC1){
            printf("PHASE 1 ; START_STATE : %d ; ", state);
            if (read(fd, &byte_read, 1) == -1) {
                printf("llread() -> read() 1. error.\n");
                alarm(0);
                free(unstuffed_msg);
                return -1;
            }

            if (g_count) {
                alarm(0);
                free(unstuffed_msg);
                return -1;
            }
            if (state == I_GOT_C) rcv_s = byte_read >> 6;
            update_state_info_rcv(&state, byte_read);
            printf("END_STATE : %d\n", state);
        }

        while(state != I_GOT_END_FLAG) {
            printf("PHASE 2 ; START_STATE : %d ; ", state);
            if (read(fd, &byte_read, 1) == -1) {
                printf("llread() -> read() 2. error.\n");
                alarm(0);
                free(unstuffed_msg);
                return -1;
            }

            if (g_count) {
                alarm(0);
                free(unstuffed_msg);
                return -1;
            }
            update_state_info_rcv(&state, byte_read);
            data_read[msg_size] = byte_read;
            msg_size++;
            printf("BYTE : 0x%x ; END_STATE : %d\n", byte_read, state);
        }

        unstuffed_size = 0;
        uint8_t rej_msg[CONTROL_SIZE];
        uint8_t rr_msg[CONTROL_SIZE];

        free(unstuffed_msg);
        unstuffed_size = message_destuffer(data_read, msg_size-1, &unstuffed_msg);

        while (state != I_STOP && state != I_START){

            printf("PHASE 3 ; START_STATE : %d ; ", state);

            uint8_t res = 0;

            switch(state){
                case (I_GOT_END_FLAG):
                    res = unstuffed_msg[unstuffed_size-1] == bcc2_builder(unstuffed_msg, unstuffed_size-1);
                    break;

                case (I_TEST_DUP_REJ):
                    res = rcv_s != R;
                    break;

                case (I_TEST_DUP_RR):
                    res = rcv_s != R;
                    break;
                
                case (I_RR_DONT_STORE):
                    R_invert();
                    control_frame_builder(RR, rr_msg);
                    if (write(fd, rr_msg, CONTROL_SIZE) == -1) {
                        printf("llread() -> write() 1. error.\n");
                        alarm(0);
                        free(unstuffed_msg);
                        return -1;
                    }
                    break;

                case (I_RR_STORE):
                    R_invert();
                    control_frame_builder(RR, rr_msg);
                    memcpy(buffer, unstuffed_msg, unstuffed_size-1);
                    if (write(fd, rr_msg, CONTROL_SIZE) == -1) {
                        printf("llread() -> write() 2. error.\n");
                        alarm(0);
                        free(unstuffed_msg);
                        return -1;
                    }
                    break;

                case (I_REJ):
                    control_frame_builder(REJ, rej_msg);
                    if (write(fd, rej_msg, CONTROL_SIZE) == -1) {
                        printf("llread() -> write() 3. error.\n");
                        alarm(0);
                        free(unstuffed_msg);
                        return -1;
                    }
                    break;

                case (I_STOP):
                    break;

                case (I_START):
                    break;

                default:
                    printf("NOT SUPPOSED TO REACH THIS\n");
                    break;
            }
            update_state_info_rcv(&state, res);
            printf("RES : %d ; END_STATE : %d\n", res, state);
        }
    }

    alarm(0);

    free(unstuffed_msg);

    return msg_size;
}
