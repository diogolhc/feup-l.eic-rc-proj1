#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "api.h"


int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial <i>\n");
        exit(1);
    }

    int porta = atoi(argv[1]);
    int fd = llopen(porta, RECEIVER);
    
    /// TEST

    char *in_msg;

    llread(fd, in_msg);

    //free(in_msg);

    /// END

    llclose(fd);

    return 0;
}
