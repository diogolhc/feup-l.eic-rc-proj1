CC=gcc
CFLAGS=-Wall -g -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer 

all: receiver sender

receiver: receiver.c common.c
	$(CC) $(CFLAGS) -o receiver receiver.c common.c

sender: sender.c common.c
	$(CC) $(CFLAGS) -o sender sender.c common.c

clean:
	rm -rf receiver sender
