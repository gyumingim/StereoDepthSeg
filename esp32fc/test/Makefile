# PC unit tests for the hardware-independent firmware modules (run in WSL / Linux):  make -C esp32fc/test
CC ?= gcc
CFLAGS = -std=c99 -O1 -g -Wall -Wextra -Werror -Wshadow \
         -fsanitize=address,undefined -fno-sanitize-recover=all
INC = -I../components/fc_safety/include -I../components/fc_core/include -I../components/viop/include
SRC = test_safety.c ../components/fc_safety/fc_safety.c ../components/fc_core/fc_core.c

all: run

test_safety: $(SRC) $(wildcard ../components/*/include/*.h)
	$(CC) $(CFLAGS) $(INC) $(SRC) -o $@ -lm

run: test_safety
	./test_safety

clean:
	rm -f test_safety

.PHONY: all run clean
