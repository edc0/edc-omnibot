#Makefile for “textr” C++ application
#Created by Dan Wilson 1/29/06

PROG = teste
CC = g++
CPPFLAGS = –Wall –pthread
LDFLAGS = -lpigpio -lrt
OBJS = main.o kinematics.o rpi_interface.o rotary_encoder.o

$(PROG) : $(OBJS)
	$(CC) $(CPPFLAGS) -o $(PROG) $(OBJS) $(LDFLAGS)
main.o :
	$(CC) $(CPPFLAGS) -c main.cpp
kinematics.o : kinematics.h
	$(CC) $(CPPFLAGS) -c kinematics.cpp $(LDFLAGS)
rotary_encoder.o : rotary_encoder.h
	$(CC) $(CPPFLAGS) -c rotary_encoder.cpp $(LDFLAGS)
rpi_interface.o : rpi_interface.h
	$(CC) $(CPPFLAGS) -c rpi_interface.cpp $(LDFLAGS)
clean:
	rm -f core $(PROG) $(OBJS)
