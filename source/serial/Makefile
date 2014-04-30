
CC = g++ -O3 -g

TARGETS=AStarSerial.o

all: $(TARGETS)

AStarSerial.o: AStarSerial.cpp AStarSerial.h ClosedSet.h Queue.h
	$(CC) -c $<
	
clean:
	rm -f *.o $(TARGETS) *.exe.*
