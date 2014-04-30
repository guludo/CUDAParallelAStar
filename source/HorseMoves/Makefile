
CC = g++ -O3 -g

TARGETS=serial

all: $(TARGETS)

r:
	make -C ../serial
	make

serial: serial.o ../serial/AStarSerial.o
	$(CC) -o $@ $< ../serial/AStarSerial.o

serial.o: serial.cpp
	$(CC) -c $<
	
clean:
	rm -f *.o $(TARGETS) *.exe.*
