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

#define CONTROL_PACKET_MAX_SIZE 500

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


static off_t get_file_size(int fd) {
    struct stat s;
    if (fstat(fd, &s) == -1) {
        return -1;
    }

    return s.st_size;
}

// file_name must be a small string (like <100 bytes) define later
static unsigned char* get_control_packet(off_t file_size, char *file_name, int *length) {
    unsigned char* control_packet = malloc(CONTROL_PACKET_MAX_SIZE);
    if (control_packet == NULL) {
        return NULL;
    }

    size_t i = 0;

    control_packet[i++]  = C_START;
    control_packet[i++]  = T_FILE_SIZE; // T1
    control_packet[i++]  = sizeof(off_t); // L1
    
    memcpy(&control_packet[i], &file_size, sizeof(off_t)); // V1
    i += sizeof(off_t);

    control_packet[i++] = T_FILE_NAME; // T2

    size_t file_name_size = strlen(file_name);
    control_packet[i++] = (unsigned char)file_name_size; // L2

    memcpy(&control_packet[i], file_name, file_name_size); // V2

    *length = i + file_name_size;
    return control_packet;
}

static int send_packaged_file(int fd_serial_port, int fd_file) {
    unsigned char *data_packet = malloc(DATA_PACKET_MAX_SIZE);
    if (data_packet == NULL) {
        return -1;
    }

    unsigned char sequence_number = 0;
    data_packet[0] = C_DATA;

    while (1) {
        data_packet[1] = sequence_number;
        sequence_number = (sequence_number+1) % 255;
        
        ssize_t num = read(fd_file, &data_packet[4], DATA_PACKET_MAX_SIZE-4);

        if (num == -1) {
            free(data_packet);
            return -1;
        } else if (num == 0) {
            break;
        } else {
            data_packet[2] = L2(num);
            data_packet[3] = L1(num);

            if (llwrite(fd_serial_port, data_packet, num+4) < 0) {
                free(data_packet);
                return -1;
            }
        }
    }
    
    free(data_packet);
    return 0;
}

// name_to_give must be a small string (like <100 bytes) define later
int send_file(int porta, char *path, int path_size, char *name_to_give) {
    int fd_file;
    if ((fd_file = open(path, O_RDONLY)) < 0) {
        return -1;
    }

    off_t file_size = 0;
    if ((file_size = get_file_size(fd_file)) < 0) {
        close(fd_file);
        return -1;
    }

    int fd_serial_port;
    if ((fd_serial_port = llopen(porta, TRANSMITTER)) < 0) {
        close(fd_file);
        return -1;
    }

    unsigned char* control_packet = NULL;
    int control_packet_size = 0;
    if ((control_packet = get_control_packet(file_size, name_to_give, &control_packet_size)) == NULL) {
        close(fd_file);
        llclose(fd_serial_port);
        return -1;
    }

    // Control packet start
    if (llwrite(fd_serial_port, control_packet, control_packet_size) < 0) {
        free(control_packet);
        close(fd_file);
        llclose(fd_serial_port);
        return -1;
    }

    if (send_packaged_file(fd_serial_port, fd_file) != 0) {
        free(control_packet);
        close(fd_file);
        llclose(fd_serial_port);
        return -1;
    }

    // Control packet end
    control_packet[0] = C_END;
    if (llwrite(fd_serial_port, control_packet, control_packet_size) < 0) {
        free(control_packet);
        close(fd_file);
        llclose(fd_serial_port);
        return -1;
    }

    free(control_packet);
    llclose(fd_serial_port);
    close(fd_file);
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
