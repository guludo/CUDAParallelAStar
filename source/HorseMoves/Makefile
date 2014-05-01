
CC = g++ -O3 -g

TARGETS=serial pre-parallel

all: $(TARGETS)

serial: serial.o ../serial/AStarSerial.o
	$(CC) -o $@ $< ../serial/AStarSerial.o

pre-parallel: pre-parallel.o ../pre-parallel/AStarPP.o
	$(CC) -o $@ $< ../pre-parallel/AStarPP.o
	
serial.o: serial.cpp
	$(CC) -c $<

pre-parallel.o: pre-parallel.cpp
	$(CC) -c $<
	
clean:
	rm -f *.o $(TARGETS) *.exe.*
