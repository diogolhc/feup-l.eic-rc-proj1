CC=gcc
CFLAGS=-Wall -g -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer 

all: receiver sender

receiver: src/receiver.c src/linklayer.c src/aplic.c
	$(CC) $(CFLAGS) -o receiver src/receiver.c src/linklayer.c src/aplic.c

sender: src/sender.c src/linklayer.c src/aplic.c
	$(CC) $(CFLAGS) -o sender src/sender.c src/linklayer.c src/aplic.c

clean:
	rm -rf receiver sender
