# Load CUDA using the following command
# module load cuda

#
# Dirac
#
CC = nvcc
MPCC = nvcc
OPENMP = 
CFLAGS = -O3 -arch sm_20
NVCCFLAGS = -O3 -arch sm_20
LIBS = -lm

TARGETS = AStarCUDA.o

all: $(TARGETS)

AStarCUDA.o: AStarCUDA.cu AStarCUDA.h ClosedSet.h Queue.h
	$(CC) -c $(NVCCFLAGS) AStarCUDA.cu
	
clean:
	rm -f *.o $(TARGETS) *.exe.*
	