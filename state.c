#include <stdio.h>

#define FLAG 0x7E
#define A 0x03
#define C_SET 0x03
#define C_UA 0X07
#define SET_SIZE 5

typedef enum state_set_ua {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} state_set_ua_t;

// c is to pass the C used (SET or UA)
int update_state_set_ua(unsigned char c, state_set_ua_t *state, unsigned char byte) {
    if (state == NULL) {
        return -1;
    }

    switch (*state) {
    case START:
        if (byte == FLAG) {
            *state = FLAG_RCV;
        }
        break;
    
    case FLAG_RCV:
        if (byte == A) {
            *state = A_RCV;
        } else if (byte != FLAG) {
            *state = START;
        }
        break;
    
    case A_RCV:
        if (byte == c) {
            *state = C_RCV;
        } else if (byte == FLAG) {
            *state = FLAG_RCV;
        } else {
            *state = START;
        }
        break;

    case C_RCV:
        if (byte == (A^c)) {
            *state = BCC_OK;
        } else if (byte == FLAG) {
            *state = FLAG_RCV;
        } else {
            *state = START;
        }
        break;

    case BCC_OK:
        if (byte == FLAG) {
            *state = STOP;
        } else {
            *state = START;
        }
        break;
    
    case STOP:
        break;
    }

    return 0;
}

int main() {
    state_set_ua_t state = START;

    printf("%d\n", state);
    update_state_set_ua(C_SET, &state, FLAG);
    printf("%d\n", state);
    update_state_set_ua(C_SET, &state, A);
    printf("%d\n", state);
    update_state_set_ua(C_SET, &state, C_SET);
    printf("%d\n", state);
    update_state_set_ua(C_SET, &state, A^C_SET);
    printf("%d\n", state);
    update_state_set_ua(C_SET, &state, FLAG);
    printf("%d\n", state);

    return 0;
}
