#include <stdio.h>
#include <stdlib.h>

#include "aplic.h"


int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage:\treceiver SerialPort\n\tex: receiver <i>\n");
        return -1;
    }

    int porta = atoi(argv[1]);
    
    if (receive_file(porta) < 0 ) {
        return -1;
    }

    return 0;
}
