SRC = \
    kw2xrf_getset.c \
    kw2xrf_intern.c \
	kw2xrf_spi.c \
	kw2xrf_tm.c \
	kw2xrf.c \
    #

ifneq (,$(filter ieee802154_radio_hal,$(USEMODULE)))
  SRC += kw2xrf_radio_hal.c
else
  SRC += kw2xrf_netdev.c
endif

include $(RIOTBASE)/Makefile.base
