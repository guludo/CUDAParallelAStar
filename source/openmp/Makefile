
CC = g++ -O3 -g
OPENMP = -fopenmp
TARGETS=AStarMP.o

all: $(TARGETS)

AStarMP.o: AStarMP.cpp AStarMP.h ClosedSet.h Queue.h
	$(CC) -c $(OPENMP) $<
	
clean:
	rm -f *.o $(TARGETS) *.exe.*
