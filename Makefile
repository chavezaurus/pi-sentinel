OBJS=sentinel.o
BIN=sentinel.bin

CFLAGS+= -g -DOMX_SKIP64BIT
#CFLAGS+= -O2 -DOMX_SKIP64BIT
LDFLAGS+=-L$(SDKSTAGE)/opt/vc/lib/ -lbcm_host -lopenmaxil -lm -lpthread
INCLUDES+=-I$(SDKSTAGE)/opt/vc/include/

all: $(BIN) $(LIB)

%.o: %.c
	@rm -f $@ 
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@ -Wno-deprecated-declarations

%.bin: $(OBJS)
	$(CC) -o $@ -Wl,--whole-archive $(OBJS) $(LDFLAGS) -Wl,--no-whole-archive -rdynamic

clean:
	for i in $(OBJS); do (if test -e "$$i"; then ( rm $$i ); fi ); done
	@rm -f $(BIN) $(LIB)
