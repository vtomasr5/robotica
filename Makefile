# variables
CPP = g++
IDIR = /usr/local/Aria/include
LDIR = /usr/local/Aria/lib
LDFLAGS = -L$(LDIR)
CFLAGS = -Wall -O2 -I$(IDIR)
LIBS = -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic
OBJ_LIB = INIReader.o ini.o
EXEC = practica
OB = practica.o

all: practica

# biblioteques
INIReader.o: INIReader.cpp INIReader.h
	$(CPP) $(CFLAGS) -c INIReader.cpp

ini.o: ini.c ini.h
	$(CPP) $(CFLAGS) -c ini.c

# executables
practica: practica.cpp $(OBJ_LIB)
	$(CPP) $(CFLAGS) -o practica practica.cpp $(OBJ_LIB) $(LDFLAGS) $(LIBS)

clean:
	rm -f $(OBJ_LIB)
	rm -f $(EXEC)
	rm -f $(OB)

.PHONY : clean

