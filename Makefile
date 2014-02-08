# Makefile for sensord
#Some compiler stuff and flags
CC = gcc
CFLAGS = -g -Wall
LDFLAGS =
EXECUTABLE = sensord
_OBJ = ms5611.o ams5915.o main.o nmea.o timer.o KalmanFilter1d.o cmdline_parser.o configfile_parser.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
LIBS = -lrt -lm
ODIR = obj

#targets

$(ODIR)/%.o: %.c
	mkdir -p $(ODIR)
	$(CC) -c -o $@ $< $(CFLAGS)

all: sensord doc
	
doc: 
	@echo Running doxygen to create documentation
	doxygen
	
sensord: $(OBJ)
	$(CC) $(LIBS) -g -o $@ $^

test: test.o obj/nmea.o
	$(CC) $(LIBS) -g -o $@ $^

sensord_fastsample: sensord_fastsample.o
	$(CC) $(LIBS) -g -o $@ $^
	
clean:
	rm -f $(ODIR)/*.o *~ core $(EXECUTABLE)
	rm -fr doc

.PHONY: clean all doc	
