
# List base directories
DSPBASE = $(CMSIS)/DSP

# Includes from the core
COREINC = $(CMSIS)/Core/Include

# Required include
include $(DSPBASE)/dsp.mk

# Add all together
CMSISSRC = $(DSPSRC)
CMSISINC = $(DSPINC) $(COREINC)
