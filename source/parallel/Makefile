
CC = nvcc -O3 -g -arch sm_20

TARGETS=AStarCUDA.o

all: $(TARGETS)

AStarCUDA.o: AStarCUDA.cu AStarCUDA.h Queue.h
	$(CC) -c $<

clean:
	rm -f *.o $(TARGETS) *.exe.*
