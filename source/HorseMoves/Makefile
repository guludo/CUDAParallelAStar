
CC = g++ -O3 -g

MPCC = nvcc -O3 -arch sm_20

TARGETS=serial pre-parallel openmp

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

gpu.o: gpu.cu
	$(MPCC) -c gpu.cu

gpu: gpu.o ../gpu/AStarCUDA.o
	$(MPCC) -o $@ gpu.o ../gpu/AStarCUDA.o

clean:
	rm -f *.o $(TARGETS) *.exe.*
