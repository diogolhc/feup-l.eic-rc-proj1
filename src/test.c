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

int main(){

    unsigned char in_msg[] = {0x7e, 0x7d};
    unsigned int in_msg_size = 2;
    unsigned char * out_msg;

    int out_msg_size = message_stuffing(in_msg, in_msg_size, &out_msg);

    for (int i = 0; i < out_msg_size; i++){
        printf("%d: 0x%x\n", i, out_msg[i]);
    }

}