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
#include <stdint.h>

#define CONTROL_PACKET_MAX_SIZE 500 // TODO check if good
#define PACKET_MAX_SIZE (CONTROL_PACKET_MAX_SIZE > DATA_PACKET_MAX_SIZE ? CONTROL_PACKET_MAX_SIZE : DATA_PACKET_MAX_SIZE)
#define FILE_NAME_MAX_SIZE 255

#define C_DATA 0x1
#define C_START 0x2
#define C_END 0x3

#define N(seq) ((seq) % 255)

#define L1(K) ((K) & 0b11111111)
#define L2(K) (((K) >> 8) & 0b11111111)
#define K(L1,L2) (256*(L2)+(L1))

#define T_FILE_SIZE 0x0
#define T_FILE_NAME 0x1

// TODO check if the return with errors should return also or enforce here other attempts


static off_t get_file_size(int fd) {
    struct stat s;
    if (fstat(fd, &s) == -1) {
        return -1;
    }

    return s.st_size;
}

static uint8_t* get_control_packet(off_t file_size, char *file_name, int file_name_size, int *length) {
    uint8_t* control_packet = malloc(CONTROL_PACKET_MAX_SIZE);
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

    control_packet[i++] = (uint8_t)file_name_size; // L2

    memcpy(&control_packet[i], file_name, file_name_size); // V2

    *length = i + file_name_size;
    return control_packet;
}

static int send_packaged_file(int fd_serial_port, int fd_file) {
    uint8_t *data_packet = malloc(DATA_PACKET_MAX_SIZE);
    if (data_packet == NULL) {
        return -1;
    }

    uint8_t sequence_number = 0;
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

int send_file(int porta, char *path, int path_size, char *file_name) {
    int file_name_size = strlen(file_name);
    if (file_name_size > FILE_NAME_MAX_SIZE) {
        fprintf(stderr, "File name to big.\n");
        return -1;
    }

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

    uint8_t* control_packet = NULL;
    int control_packet_size = 0;
    if ((control_packet = get_control_packet(file_size, file_name, file_name_size, &control_packet_size)) == NULL) {
        close(fd_file);
        llclose(fd_serial_port, TRANSMITTER);
        return -1;
    }

    // Control packet start
    if (llwrite(fd_serial_port, control_packet, control_packet_size) < 0) {
        free(control_packet);
        close(fd_file);
        llclose(fd_serial_port, TRANSMITTER);
        return -1;
    }

    if (send_packaged_file(fd_serial_port, fd_file) != 0) {
        free(control_packet);
        close(fd_file);
        llclose(fd_serial_port, TRANSMITTER);
        return -1;
    }

    // Control packet end
    control_packet[0] = C_END;
    if (llwrite(fd_serial_port, control_packet, control_packet_size) < 0) {
        free(control_packet);
        close(fd_file);
        llclose(fd_serial_port, TRANSMITTER);
        return -1;
    }

    free(control_packet);
    llclose(fd_serial_port, TRANSMITTER);
    close(fd_file);
    return 0;
}

int receive_file(int porta) {
    int fd_serial_port;
    if ((fd_serial_port = llopen(porta, RECEIVER)) < 0) {
        return -1;
    }

    uint8_t *packet = malloc(PACKET_MAX_SIZE);
    if (packet == NULL) {
        llclose(fd_serial_port, RECEIVER);
        return -1;
    }

    int fd_file_to_write = -1;
    //int sequence_number = 0; TODO use this
    off_t file_size = 0;

    int not_end_packet = TRUE;
    while (not_end_packet) {
        int packet_size = 0;
        if ((packet_size = llread(fd_serial_port, packet)) < 0) {
            free(packet);
            llclose(fd_serial_port, RECEIVER);
            return -1;
        }

        uint8_t control_field = packet[0];
        switch (control_field) {
        case C_DATA:
            if (fd_file_to_write == -1) { // Control start packet didn't arrive yet
                break;
            }

            // int N = packet[1]; // TODO what to do if N doesn't check out with previous+1? since it's not supposed to check for errors in application layer...

            // if N == sequence_number before, ignores current packet,
            // but what if there is like a sequence_number before + 2?
            // no way here to ask for the missing packet...


            int num_octets = K(packet[3], packet[2]);
            if (write(fd_file_to_write, &packet[4], num_octets) == -1) {
                free(packet);
                llclose(fd_serial_port, RECEIVER);
                return -1;
            }

            break;
        
        case C_START:;
            int i = 1;
            while (i < packet_size) {
                uint8_t T = packet[i++];
                uint8_t L = packet[i++];

                if (T == T_FILE_SIZE) {
                    memcpy(&file_size, &packet[i], L);

                } else if (T == T_FILE_NAME) {
                    char file_name[FILE_NAME_MAX_SIZE];
                    memcpy(file_name, &packet[i], L);

                    fd_file_to_write = open(file_name, O_WRONLY | O_APPEND | O_CREAT, 0644);
                    if (fd_file_to_write == -1) {
                        free(packet);
                        llclose(fd_serial_port, RECEIVER);
                        return -1;
                    }

                } else {
                    // TODO error, should it do something?
                }

                i += L;
            }

            break;

        case C_END:
            // TODO should it check equality in all the fields?
            // what to do if they are different?
            // data-link layer should've handled that


            if (fd_file_to_write != -1) { 
                close(fd_file_to_write);
                not_end_packet = FALSE;
            }
            // else Control start packet didn't arrive yet, so wait for it

            break;
        
        default:
            break; // invalid control field (ignore packet)
        }
    }
    
    if (llclose(fd_serial_port, RECEIVER) < 0) {
        free(packet);
        return -1;
    }

    return 0;
}
