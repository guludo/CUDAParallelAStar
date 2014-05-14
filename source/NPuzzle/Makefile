
CC = g++ -O3 -g

NVCC = nvcc -O3 -g -arch sm_20

TARGETS=serial pre-parallel parallel

r:
	make -C ../serial
	make -C ../pre-parallel
	make -C ../parallel
	make

all: $(TARGETS)

parallel: parallel.o ../parallel2/AStarCUDA.o
	$(NVCC) -o $@ $< ../parallel2/AStarCUDA.o

parallel.o: parallel.cu
	$(NVCC) -c $<

serial: serial.o ../serial/AStarSerial.o
	$(CC) -o $@ $< ../serial/AStarSerial.o

pre-parallel: pre-parallel.o ../pre-parallel/AStarPP.o
	$(CC) -o $@ $< ../pre-parallel/AStarPP.o

serial.o: serial.cpp
	$(CC) -c $<

pre-parallel.o: pre-parallel.cpp
	$(CC) -c $<

clean:
	rm -f *.o $(TARGETS) *.exe.* ../serial/*.o ../parallel2/*.o ../pre-parallel/*.o
