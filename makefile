CC=gcc
CFLAGS=-Wall -g -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer 

all: receiver sender test

notest: receiver sender

receiver: src/receiver.c src/api.c src/aplic.c
	$(CC) $(CFLAGS) -o receiver src/receiver.c src/api.c src/aplic.c

sender: src/sender.c src/api.c src/aplic.c
	$(CC) $(CFLAGS) -o sender src/sender.c src/api.c src/aplic.c

test: src/test.c src/api.c
	$(CC) $(CFLAGS) -o test src/test.c src/api.c

clean:
	rm -rf receiver sender test
