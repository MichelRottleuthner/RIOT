MODULE = efm32_gclk

# The calibration code is based on a basic example from silicon labs and is currently not used.
# It was only briefly tested. To enable it comment out the line below and call
# _calibrate_HFRCO(uint32_t freq) to start the calibration procedure for the HFRCO oscillator
#SRC += gclk_efm32pg12b_hfrco_calibration_example.c
SRC += gclk_efm32pg12b_all.c
SRC += gclk_efm32pg12b.c

include $(RIOTBASE)/Makefile.base
