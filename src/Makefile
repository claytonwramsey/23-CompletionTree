BASE = ../rai

OPTIM = fast_debug

#OBJS = main.o completionBandits.o solver_uct.o

DEPEND = KOMO Core Geo Kin Gui Optim PathAlgos Logic Search LGP

LIBS += -lpthread

include $(BASE)/makeutils/generic.mk

# rai.cfg is generated from rai.cfg.in (see ../generate-rai-cfg.sh) so no
# one user's absolute path is hardcoded into a tracked file; regenerate
# it whenever the template changes.
rai.cfg: rai.cfg.in
	../generate-rai-cfg.sh

default: rai.cfg
