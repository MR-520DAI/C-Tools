CC	= @echo " GCC	$@"; $(CROSS)gcc

CROSS = arm-xmv2-linux-
CFLAGS += -mcpu=cortex-a9 -mfloat-abi=softfp  -mfpu=neon -mno-unaligned-access -fno-aggressive-loop-optimizations -flax-vector-conversions -fsigned-char -fopenmp
CFLAGS += -std=gnu99 -Wall -O2

TESTSOURCE = $(wildcard ./main.c)
TESTTARGET = main.out

TARGET = $(TESTTARGET) 

all:$(TARGET)

$(TESTTARGET):
	$(CC) $(CFLAGS) -save-temps -o $(TESTTARGET) $(TESTSOURCE) -lstdc++ -lm

clean:
	rm -f $(TESTTARGET) 