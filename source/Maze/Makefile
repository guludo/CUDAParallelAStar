
CC = g++ -O3 -g

TARGETS=serial

all: $(TARGETS)

openmp: openmp.o ../openmp/AStarMP.o
	$(CC) -fopenmp -o $@ $< ../openmp/AStarMP.o
	
serial: serial.o ../serial/AStarSerial.o
	$(CC) -o $@ $< ../serial/AStarSerial.o

pre-parallel: pre-parallel.o ../pre-parallel/AStarPP.o
	$(CC) -o $@ $< ../pre-parallel/AStarPP.o

openmp.o: openmp.cpp
	$(CC) -fopenmp -c $<
	
serial.o: serial.cpp
	$(CC) -c $<

pre-parallel.o: pre-parallel.cpp
	$(CC) -c $<
	
clean:
	rm -f *.o $(TARGETS) *.exe.*
