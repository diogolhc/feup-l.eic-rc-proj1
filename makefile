CC=gcc
CFLAGS=-Wall -g -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer 

all: receiver sender test

receiver: receiver.c api.c aplic.c
	$(CC) $(CFLAGS) -o receiver receiver.c api.c aplic.c

sender: sender.c api.c aplic.c
	$(CC) $(CFLAGS) -o sender sender.c api.c aplic.c

test: test.c api.c
	$(CC) $(CFLAGS) -o test test.c api.c

clean:
	rm -rf receiver sender test
