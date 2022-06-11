TARGET = main
CC = g++
CFLAGS = -O3
OBJS = main.o ga.o

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS)

#.cpp.o:
#	$(CC) $(CFLAGS) -c $<

main.o: main.cpp problemdescription.h
	$(CC) $(CFLAGS) -c main.cpp

ga.o: ga.cpp
	$(CC) $(CFLAGS) -c ga.cpp

clean:
	-rm -f core $(TARGET) $(OBJS)
