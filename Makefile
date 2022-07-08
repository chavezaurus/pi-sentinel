OBJS=pi-sentinel.o
BIN=pi-sentinel.bin

#CPPFLAGS+= -g -std=c++17 -Wall -Wno-psabi
CPPFLAGS+= -O2 -std=c++17 -Wall -Wno-psabi
LDFLAGS+= -lm -lpthread -lstdc++fs -lv4l1 -lv4l2 -ljpeg
INCLUDES+=

CC = g++

all: $(BIN) $(LIB)

%.o: %.cpp
	@rm -f $@ 
	$(CC) $(CPPFLAGS) $(INCLUDES) -c $< -o $@ -Wno-deprecated-declarations

%.bin: $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS) -rdynamic

clean:
	for i in $(OBJS); do (if test -e "$$i"; then ( rm $$i ); fi ); done
	@rm -f $(BIN) $(LIB)
