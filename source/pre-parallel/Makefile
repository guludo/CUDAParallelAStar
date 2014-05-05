
CC = g++ -O3 -g

TARGETS=AStarPP.o

all: $(TARGETS)

AStarPP.o: AStarPP.cpp AStarPP.h ClosedSet.h Queue.h
	$(CC) -c $<
	
clean:
	rm -f *.o $(TARGETS) *.exe.*
