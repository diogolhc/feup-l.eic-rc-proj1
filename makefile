CC=gcc
CFLAGS=-Wall -g -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer 
CFLAGS2=-Wall

all: receiver sender
debug: receiverd senderd

receiverd: src/receiver.c src/linklayer.c src/aplic.c
	$(CC) $(CFLAGS) -o receiver src/receiver.c src/linklayer.c src/aplic.c

senderd: src/sender.c src/linklayer.c src/aplic.c
	$(CC) $(CFLAGS) -o sender src/sender.c src/linklayer.c src/aplic.c


receiver: src/receiver.c src/linklayer.c src/aplic.c
	$(CC) $(CFLAGS2) -o receiver src/receiver.c src/linklayer.c src/aplic.c

sender: src/sender.c src/linklayer.c src/aplic.c
	$(CC) $(CFLAGS2) -o sender src/sender.c src/linklayer.c src/aplic.c

clean:
	rm -rf receiver sender
