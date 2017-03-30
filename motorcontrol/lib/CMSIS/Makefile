DSPBASE = DSP

include $(DSPBASE)/dsp.mk

CROSS_COMPILE = arm-none-eabi-
AR = $(CROSS_COMPILE)ar
CC = $(CROSS_COMPILE)gcc

rm = rm -f
MKDIR_P = mkdir -p

OBJDIR = ../../buil/obj

DSPOBJ  := $(DSPSRC:%.c=%.o)
OBJECTS := $(foreach file,${DSPOBJ},$(OBJDIR)/$(notdir ${file}))

default: all
all: libmath_cm4f.a

libmath_cm4f.a: $(OBJECTS)
        $(AR) rcs $@ $(OBJECTS)

$(OBJECTS): 
        $(error $@ is missing, please compile the project without the static lib first)

showobj:
        @echo $(OBJECTS)

clean:
        @$(rm) libmath_cm4f.a
        @echo "Cleanup complete!"