NAME            = softmodem

SOURCES		= $(shell ls *.c)
OBJECTS         = $(subst .c,.o,$(SOURCES))
# make DEBUG=1
ifdef DEBUG
$(warning DEBUG: SOURCES=$(SOURCES))
$(warning DEBUG: OBJECTS=$(OBJECTS))
endif

CPU             = msp430f149
CFLAGS          = -mmcu=${CPU} -O2 -Wall -g
CC              = msp430-gcc

#-------------------------------------------------------------

.PHONY: all FORCE clean download download-gdb dist cscope

all: ${NAME}.elf ${NAME}.ihex ${NAME}.lst

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

${NAME}.elf: ${OBJECTS}
	${CC} -mmcu=${CPU} -o $@ ${OBJECTS}

${NAME}.ihex: ${NAME}.elf
	msp430-objcopy -O ihex $^ $@

${NAME}.lst: ${NAME}.elf
	msp430-objdump -dSt $^ >$@

#FIXME:
#download: download-gdb

cscope:
	find $${PWD}/ -name "*.[ch]" > cscope.files
	cscope -b

clean:
	rm -f ${NAME}.elf ${NAME}.ihex ${NAME}.lst ${OBJECTS} cscope.*

dist: clean
	tar czf dist.tgz *.c *.h *.txt Makefile

#dummy target as dependecy if something has to be build everytime
FORCE:


