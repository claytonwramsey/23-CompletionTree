BASE = ../rai

OPTIM = fast_debug

DEPEND = Core Algo Search

# Defaults assume the standard sibling-checkout layout (hopscotch/ and
# 23-CompletionTree/ both directly under the same parent directory, as in
# hopscotch/examples/pddlstream/README.md); override on the command line
# (e.g. `make HOP_BENCH_CXX_DIR=/path/to/hopscotch/crates/hop-bench-cxx
# HOP_BENCH_CXX_LIB=/path/to/hopscotch/target/release all_bench`) if your
# checkout doesn't follow that layout. $(abspath ...) resolves relative to
# the directory `make` is invoked from (this one), so this works for any
# user/home directory without editing the Makefile.
HOP_BENCH_CXX_DIR ?= $(abspath ../../hopscotch/crates/hop-bench-cxx)
HOP_BENCH_CXX_LIB ?= $(abspath ../../hopscotch/target/release)

CXXFLAGS += -I$(HOP_BENCH_CXX_DIR)
LIBS += -L$(HOP_BENCH_CXX_LIB) -lhop_bench_cxx -Wl,-rpath,$(HOP_BENCH_CXX_LIB) -lpthread -ldl

# One executable per bench_*.cpp driver (each has its own `main`), sharing the
# domain-node implementations below. Build with e.g.
#   make MAIN=bench_hopscotch OUTPUT=x_hopscotch.exe
#   make MAIN=bench_stacking OUTPUT=x_stacking.exe
# `make all_bench` builds every driver at once.
COMMON_SRCS = HopCommon.cpp HopPickPlaceNode.cpp HopStackingNode.cpp HopMobileNode.cpp HopCoffeeNode.cpp
MAIN ?= bench_hopscotch
OUTPUT ?= x_hopscotch.exe

SRCS = $(COMMON_SRCS) $(MAIN).cpp
OBJS = $(SRCS:%.cpp=%.o)

# NOTE: this must come *after* `include`, since make's default goal (used
# whenever `make` is invoked with no explicit target, as the recursive calls
# below do) is the *first* target in the fully-expanded makefile -- defining
# `all_bench` before `include` pulls in generic.mk's `default: $(OUTPUT)`
# would make `all_bench` the default goal instead, which recurses into
# itself forever (each recursive `make MAIN=... OUTPUT=...` again runs
# `all_bench` by default rather than building just that one driver).
include $(BASE)/makeutils/generic.mk

DRIVERS = bench_hopscotch:x_hopscotch.exe bench_stacking:x_stacking.exe bench_mobile:x_mobile.exe bench_coffee:x_coffee.exe

all_bench: force
	@for d in $(DRIVERS); do \
	  main=$${d%%:*}; out=$${d##*:}; \
	  $(MAKE) MAIN=$$main OUTPUT=$$out default; \
	done
