CC=gcc
CPPFLAGS=-O3 -fopenmp
LDFLAGS=-lz -lm -lstdc++ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_nonfree -lopencv_features2d
INCLUDES=-I/usr/local/include


.PHONY:all
all: CvToolDemo

%.o : %.cpp
	$(CC) $(CPPFLAGS) -c $< -o $@ $(INCLUDES)

CvToolDemo: main.o cvtool.o
	$(CC) $^ $(LDFLAGS) -o $@  $(INCLUDES)

.PHONY:clean
clean:
	rm -fr *.o

