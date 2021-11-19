#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include "api.h"

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial <i>\n");
        exit(1);
    }

    int porta = atoi(argv[1]);
    int fd = llopen(porta, TRANSMITTER);

    /// TEST

    sleep(1);

    char in_msg[] = {0x7e, 0x7d};
    unsigned int in_msg_size = 2;

    llwrite(fd, in_msg, in_msg_size);

    sleep(2);

    /// END

    llclose(fd);
    
    return 0;
}
