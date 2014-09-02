# Makefile for sensord
#Some compiler stuff and flags
CFLAGS += -g -Wall
EXECUTABLE = sensord
_OBJ = ms5611.o ams5915.o main.o nmea.o timer.o KalmanFilter1d.o cmdline_parser.o configfile_parser.o vario.o AirDensity.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
LIBS = -lrt -lm
ODIR = obj
BINDIR = /opt/bin/

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
	
install: sensord
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
