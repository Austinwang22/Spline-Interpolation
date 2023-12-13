CC = g++
FLAGS = -g

INCLUDE = -I/opt/homebrew/include -I/usr/X11R6/include -I/usr/include/GL -I/usr/include -I../..
LIBDIR = -L/opt/homebrew/lib -L/usr/X11R6/lib -L/usr/local/lib
SOURCES = *.cpp
LIBS = -lGLEW -framework OpenGL -lGLU -framework glut -lm

EXENAME = keyframe

all: $(SOURCES)
	$(CC) $(FLAGS) -o $(EXENAME) $(INCLUDE) $(LIBDIR) $(SOURCES) $(LIBS)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean

