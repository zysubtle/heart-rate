CC       = cc
CFLAGS   = -std=c11 -Wall -Wextra -Wpedantic
INCLUDES = -I../include -I../src

SRCS_M1  = ../src/hr_sampling.c

.PHONY: all clean run

all: test_hr_sampling

test_hr_sampling: test_hr_sampling.c $(SRCS_M1) test_framework.h ../src/hr_sampling.h
	$(CC) $(CFLAGS) $(INCLUDES) test_hr_sampling.c $(SRCS_M1) -o $@

run: test_hr_sampling
	@./test_hr_sampling

clean:
	rm -f test_hr_sampling
