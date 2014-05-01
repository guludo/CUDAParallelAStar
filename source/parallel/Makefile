
CC = nvcc -O3 -g -arch sm_20

TARGETS=AStarSerial.o

all: $(TARGETS)

AStarSerial.o: AStarSerial.cu AStarSerial.h ClosedSet.h Queue.h
	$(CC) -c $<

clean:
	rm -f *.o $(TARGETS) *.exe.*
