/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

int flag=1, conta=1;

//=======================================
#define FLAG 0x7E
#define A 0x03
#define C_SET 0x03
#define C_UA 0X07
#define SET_SIZE 5

unsigned char SET[SET_SIZE] = {
  FLAG,
  A,
  C_SET,
  A ^ C_SET,
  FLAG
};

unsigned char UA[SET_SIZE] = {
  FLAG,
  A,
  C_UA,
  A ^ C_UA,
  FLAG
};
//=======================================

volatile int STOP=FALSE;

int compare_sets(unsigned char set1[], unsigned char set2[], int len){
    int res = 1;
    for (int i = 0; i < len; i++){
        res = res && (set1[i] == set2[i]);
    }
    return res;
}

void time_out() {
    printf("alarme # %d\n", conta);
    flag=1;
    conta++;
}


int main(int argc, char** argv) {
    struct sigaction new, old;
    sigset_t smask;

    if (sigemptyset(&smask)==-1)	
        perror ("sigsetfunctions");

    new.sa_handler = time_out;
    new.sa_mask = smask;
    new.sa_flags = 0;	// usually works

    if(sigaction(SIGALRM, &new, &old) == -1)
        perror ("sigaction");
    
    
    int fd,c, res;
    struct termios oldtio,newtio;
    char buf[255];
    int i, sum = 0, speed = 0;
    
    if ( (argc < 2) || 
  	     ((strcmp("/dev/ttyS4", argv[1])!=0) && 
  	      (strcmp("/dev/ttyS1", argv[1])!=0) )) {
      printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
      exit(1);
    }


  /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
  */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd <0) {
      perror(argv[1]); exit(-1);
    }

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
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

    if ( tcsetattr(fd,TCSANOW,&newtio) == -1) {
      perror("tcsetattr");
      exit(-1);
    }

    printf("New termios structure set\n");

    //printf("> ");
    //gets(buf);

    unsigned int breakk = 0;
    while (conta < 4 && flag) {
          breakk = 0;

          res = write(fd, SET, SET_SIZE * sizeof(unsigned char));   
          printf("%d bytes written\n", res);

          alarm(3);                 // activa alarme de 3s

          unsigned char received_ua[5];

          do {
              bzero(received_ua, 5);
              for (int i = 0; i < 5; i++){
                  res = read(fd, &received_ua[i], 1);

                  printf("Debug: %x %x %d\n", received_ua[i], UA[i], res);
                  if (res < 0) {
                     // printf("alarme # %d\n", conta);
                     // flag=1;
                     // conta++;
                      breakk = 1;
                      break;
                  }
  
              }
              
              if (breakk)
                break;
          } while (!compare_sets(UA, received_ua, 5));

          if (breakk)
            continue;

          alarm(0);
          flag=0;
          printf("ACK\n");
    }


    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
      perror("tcsetattr");
      exit(-1);
    }




    close(fd);
    return 0;
}
