# Makefile for sensord
#Some compiler stuff and flags
CFLAGS += -g -Wall
EXECUTABLE = sensord sensorcal
_OBJ = ms5611.o ams5915.o ads1110.o main.o nmea.o timer.o KalmanFilter1d.o cmdline_parser.o configfile_parser.o vario.o AirDensity.o 24c16.o
_OBJ_CAL = 24c16.o ams5915.o sensorcal.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
OBJ_CAL = $(patsubst %,$(ODIR)/%,$(_OBJ_CAL))
LIBS = -lrt -lm
ODIR = obj
BINDIR = /opt/bin/
GIT_VERSION := $(shell git describe --dirty)

#targets

$(ODIR)/%.o: %.c
	mkdir -p $(ODIR)
	$(CC) -DVERSION_GIT=\"$(GIT_VERSION)\" -c -o $@ $< $(CFLAGS)
	
all: sensord sensorcal

version.h: 
	@echo Git version $(GIT_VERSION)
	
doc: 
	@echo Running doxygen to create documentation
	doxygen
	
sensord: $(OBJ)
	$(CC) $(LIBS) -g -o $@ $^
	
sensorcal: $(OBJ_CAL)
	$(CC) -g -o $@ $^

install: sensord sensorcal
	install -D sensord $(BINDIR)/$(EXECUTABLE)
	
test: test.o obj/nmea.o
	$(CC) $(LIBS) -g -o $@ $^

sensord_fastsample: sensord_fastsample.o
	$(CC) $(LIBS) -g -o $@ $^

i2c_test: i2c_test.o ms5611.o
	$(CC) $(LIBS) -g -o $@ $^
	
clean:
	rm -f $(ODIR)/*.o *~ core $(EXECUTABLE)
	rm -fr doc

.PHONY: clean all doc
