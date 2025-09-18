CC=g++
CFLAGS=-c -Wall
LDFLAGS=-lm
SOURCES=research.cpp kren_model.cpp  kren_ctrl.cpp # hello.cpp factorial.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=rsrch

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o rsrch

