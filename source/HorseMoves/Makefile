
CC = g++ -O3 -g

NVCC = nvcc -O3 -g -arch sm_20

TARGETS=serial pre-parallel parallel 

r:
	make -C ../serial
	make -C ../pre-parallel
	make -C ../parallel
	make

all: $(TARGETS)

parallel: parallel.o ../parallel/AStarCUDA.o
	$(NVCC) -o $@ $< ../parallel/AStarCUDA.o

parallel.o: parallel.cu
	$(NVCC) -c $<

serial: serial.o ../serial/AStarSerial.o
	$(CC) -o $@ $< ../serial/AStarSerial.o
	
serial.o: serial.cpp
	$(CC) -c $<

pre-parallel: pre-parallel.o ../pre-parallel/AStarPP.o
	$(CC) -o $@ $< ../pre-parallel/AStarPP.o

pre-parallel.o: pre-parallel.cpp
	$(CC) -c $<

gpu.o: gpu.cu
	$(NVCC) -c gpu.cu

gpu: gpu.o ../gpu/AStarCUDA.o
	$(NVCC) -o $@ gpu.o ../gpu/AStarCUDA.o

clean:
	rm -f *.o $(TARGETS) *.exe.* ../parallel/*.o
