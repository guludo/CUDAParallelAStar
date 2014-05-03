
CC = g++ -O3 -g
NVCC = nvcc -O3 -g -arch sm_20

TARGETS=serial parallel

all: $(TARGETS)

r:
	make -C ../serial
	make -C ../parallel
	make

parallel: parallel.o ../parallel/AStarCUDA.o
	$(NVCC) -o $@ $< ../parallel/AStarCUDA.o

parallel.o: parallel.cu
	$(NVCC) -c $<

serial: serial.o ../serial/AStarSerial.o
	$(CC) -o $@ $< ../serial/AStarSerial.o

serial.o: serial.cpp
	$(CC) -c $<

clean:
	rm -f *.o $(TARGETS) *.exe.* ../parallel/*.o
