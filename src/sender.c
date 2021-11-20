#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "aplic.h"


int main(int argc, char** argv) {
    if (argc != 4) {
        printf("Usage:\tsender SerialPort Path NameToGive\n\tex: nserial <i> <path> <name>\n");
        return -1;
    }

    int porta = atoi(argv[1]);
    
    if (send_file(porta, argv[2], strlen(argv[2]), argv[3]) < 0) {
        return -1;
    }
    
    return 0;
}
