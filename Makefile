CC = gcc
CFLAGS = -O2
LDLIBS = -lSDL2

SRCS = $(wildcard *.c)
OBJS = $(SRCS:%.c=build/%.o)
TARGET = vihtorizx

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDLIBS)

build/%.o: %.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build:
	mkdir -p $@

clean:
	rm -rf build $(TARGET)
