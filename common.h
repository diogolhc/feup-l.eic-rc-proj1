#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define FLAG 0x7E
#define A 0x03
#define C_SET 0x03
#define C_UA 0X07
#define SET_SIZE 5

#define TIME_OUT_TIME 3

typedef enum state_set_ua {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} state_set_ua_t;

int update_state_set_ua(unsigned char c, state_set_ua_t *state, unsigned char byte);
