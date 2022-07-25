#
# makefile for boltek lightning SDK
#


SRC= libboltek.c demo.c
OBJ= libboltek.o libboltek.a libboltek.so demo

.PHONY: all
all: $(OBJ)

$(OBJ): $(SRC) Makefile
	gcc  -g -Wall -fPIC -c  libboltek.c
	gcc -shared -Wl,-soname,libboltek.so -o libboltek.so libboltek.o 
	ar r libboltek.a libboltek.o
	gcc -g -o demo demo.c libboltek.a -lm

.PHONY: clean
clean:
	rm  -f $(OBJ)
