CC=gcc
CFLAGS=-Wall

all: receiver sender test

receiver: receiver.c api.c
	$(CC) $(CFLAGS) -o receiver receiver.c api.c

sender: sender.c api.c
	$(CC) $(CFLAGS) -o sender sender.c api.c

test: test.c api.c
	$(CC) $(CFLAGS) -o test test.c api.c

clean:
	rm -rf receiver sender test
