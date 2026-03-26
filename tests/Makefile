CC       = cc
CFLAGS   = -std=c11 -Wall -Wextra -Wpedantic
INCLUDES = -I../include -I../src

SRCS_M1  = ../src/hr_sampling.c
SRCS_M2  = ../src/hr_preproc.c
SRCS_M3  = ../src/hr_sqi.c
SRCS_M4  = ../src/hr_motion.c
SRCS_M5  = ../src/hr_mac.c
SRCS_M6  = ../src/hr_candidate.c

.PHONY: all clean run

all: test_hr_sampling test_hr_preproc test_hr_sqi test_hr_motion test_hr_mac test_hr_candidate

test_hr_sampling: test_hr_sampling.c $(SRCS_M1) test_framework.h ../src/hr_sampling.h
	$(CC) $(CFLAGS) $(INCLUDES) test_hr_sampling.c $(SRCS_M1) -o $@

test_hr_preproc: test_hr_preproc.c $(SRCS_M2) $(SRCS_M1) test_framework.h ../src/hr_preproc.h ../src/hr_sampling.h
	$(CC) $(CFLAGS) $(INCLUDES) test_hr_preproc.c $(SRCS_M2) $(SRCS_M1) -lm -o $@

test_hr_sqi: test_hr_sqi.c $(SRCS_M3) test_framework.h ../src/hr_sqi.h ../src/hr_preproc.h ../src/hr_sampling.h
	$(CC) $(CFLAGS) $(INCLUDES) test_hr_sqi.c $(SRCS_M3) -lm -o $@

test_hr_motion: test_hr_motion.c $(SRCS_M4) test_framework.h ../src/hr_motion.h ../src/hr_preproc.h ../src/hr_sampling.h
	$(CC) $(CFLAGS) $(INCLUDES) test_hr_motion.c $(SRCS_M4) -lm -o $@

test_hr_mac: test_hr_mac.c $(SRCS_M5) test_framework.h ../src/hr_mac.h ../src/hr_preproc.h ../src/hr_sqi.h ../src/hr_motion.h ../src/hr_sampling.h
	$(CC) $(CFLAGS) $(INCLUDES) test_hr_mac.c $(SRCS_M5) -lm -o $@

test_hr_candidate: test_hr_candidate.c $(SRCS_M6) test_framework.h ../src/hr_candidate.h ../src/hr_mac.h ../src/hr_preproc.h ../src/hr_sqi.h ../src/hr_motion.h ../src/hr_sampling.h
	$(CC) $(CFLAGS) $(INCLUDES) test_hr_candidate.c $(SRCS_M6) -lm -o $@

run: test_hr_sampling test_hr_preproc test_hr_sqi test_hr_motion test_hr_mac test_hr_candidate
	@echo ""
	@./test_hr_sampling
	@echo ""
	@./test_hr_preproc
	@echo ""
	@./test_hr_sqi
	@echo ""
	@./test_hr_motion
	@echo ""
	@./test_hr_mac
	@echo ""
	@./test_hr_candidate

clean:
	rm -f test_hr_sampling test_hr_preproc test_hr_sqi test_hr_motion test_hr_mac test_hr_candidate
