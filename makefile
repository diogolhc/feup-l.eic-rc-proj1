CC=gcc
CFLAGS=-Wall -g -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer 

all: receiver sender

receiver: receiver.c
	$(CC) $(CFLAGS) -o receiver receiver.c

sender: sender.c
	$(CC) $(CFLAGS) -o sender sender.c
