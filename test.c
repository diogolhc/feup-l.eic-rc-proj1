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

void main(){

    unsigned char in_msg[] = {0x7e, 0x7d};
    unsigned int in_msg_size = 2;

    llwrite(NULL, in_msg, in_msg_size);
}