# Copyright (C) 2018 Amarisoft/LimeMicro
# TRX Makefile for Lime MicroSystems


CC=gcc #-m64
CXX=g++ -m64
AR=ar
CFLAGS=-O3 -fno-strict-aliasing -Werror-implicit-function-declaration 
CFLAGS+=-march=native
CFLAGS+=-D_GNU_SOURCE -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE
CFLAGS+=-MMD -g -Wno-unused-but-set-variable  -Wno-unused-result -Wl,--no-undefined
CFLAGS+=-mfpmath=sse --param max-inline-insns-single=10000 --param large-function-growth=10000 --param inline-unit-growth=10000
CXXFLAGS:=-std=c++11
LIBS:=-lLimeSuite

PROGS=trx_lms7002m.so

all: $(PROGS)

clean:
	rm -f $(PROGS) *.lo *~ *.d *.so

trx_lms7002m.so: trx_lms7002m.cpp
	@$(CXX) $(CPPFLAGS) $(CFLAGS) $(CXXFLAGS) $(LDFLAGS) -fPIC -shared -o $@ $^ $(LIBS) -Wl,-z,defs








