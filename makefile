CC=gcc
CFLAGS=-Wall

all: receiver sender

receiver: receiver.c api.c
	$(CC) $(CFLAGS) -o receiver receiver.c api.c

sender: sender.c api.c
	$(CC) $(CFLAGS) -o sender sender.c api.c

clean:
	rm -rf receiver sender

