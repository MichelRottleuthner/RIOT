include ../Makefile.tests_common

# only include the cmsis-dsp lib for boards that support cmsis (and gclk)
ifneq (,$(filter nucleo-l476rg slstk3402a,$(BOARD)))
USEPKG += cmsis-dsp
endif

FEATURES_REQUIRED = periph_adc

USEPKG += heatshrink
# ####################### CRYPTO STUFF
USEMODULE += cipher_modes
# By using this pseudomodule, T tables will be precalculated.
USEMODULE += crypto_aes_precalculated
# This pseudomodule causes a loop in AES to be unrolled (more flash, less CPU)
USEMODULE += crypto_aes_unroll
# ####################### RADIO STUFF
# Include packages that pull up and auto-init the link layer.
# NOTE: 6LoWPAN will be included if IEEE802.15.4 devices are present
USEMODULE += gnrc_netif
USEMODULE += auto_init_gnrc_netif
# Activate ICMPv6 error messages
USEMODULE += gnrc_icmpv6_error
# Specify the mandatory networking modules for IPv6 and UDP
USEMODULE += gnrc_ipv6_router_default
USEMODULE += gnrc_udp
# Add a routing protocol
#USEMODULE += gnrc_rpl
#USEMODULE += auto_init_gnrc_rpl
# This application dumps received packets to STDIO using the pktdump module
#USEMODULE += gnrc_pktdump
# Additional networking modules that can be dropped if not needed
USEMODULE += gnrc_icmpv6_echo
USEMODULE += ps
USEMODULE += netstats_l2
USEMODULE += netstats_ipv6
USEMODULE += netstats_rpl
USEMODULE += gnrc_neterr
CFLAGS+=-DCONFIG_GNRC_IPV6_NIB_NO_RTR_SOL=1
CFLAGS+=-DCONFIG_GNRC_IPV6_NIB_ADV_ROUTER=1
USEMODULE += at86rf233

############### Higher-layer Networking stuff
USEMODULE += gcoap
# required by gcoap example code
USEMODULE += od

############### utensor stuff
USEPKG += utensor

# Include input data as blob
BLOBS += digit

# Include C++ model and training weights code as an external module
USEMODULE += models
EXTERNAL_MODULE_DIRS += external_modules

USEMODULE += mlwrapper
##############

FEATURES_REQUIRED += periph_spi

USEMODULE += gclk

USEMODULE += shell
USEMODULE += shell_cmds_default

# RTT is used for idle_timer so we must avoid that it is also used by ztimer
USEMODULE += ztimer_no_periph_rtt
USEMODULE += ztimer64_xtimer_compat
USEMODULE += xtimer

USEMODULE += ps
USEMODULE += schedstatistics

FEATURES_OPTIONAL += periph_flash_opt
FEATURES_OPTIONAL += periph_core_voltage
FEATURES_REQUIRED += periph_rtt

#CFLAGS += -DGCLK_USE_SEPARATE_CONF_REG_ARRAYS
#CFLAGS += -DGCLK_USE_TINY_REG_REF

CFLAGS += -DGCLK_GATE_USE_TINY_REG_REF
#CFLAGS += -DGCLK_GATE_USE_SEPARATE_CONF_REG_VALUES
# not implemented:
#CFLAGS += -DGCLK_GATE_USE_SEPARATE_CONF_REG_ARRAYS

ifneq (esp32-wroom-32,$(BOARD))
CFLAGS += -DISR_STACKSIZE=1024
endif

CFLAGS += -DTHREAD_STACKSIZE_MAIN=\(3*THREAD_STACKSIZE_DEFAULT+THREAD_EXTRA_STACKSIZE_PRINTF\)

# The wakeup pin must be one that is able to wakeup the MCU from the lowest power mode to allow for
# evaluation of low power mode consumption.
# To ensure there is no leakage whatsoever this pin should best be interfaced to the instrumentation
# circuit (i.e. the DMM) via an optocoupler.

ifeq (nucleo-l476rg,$(BOARD))
CFLAGS += -DDMM_TRIGGER_PIN_START=GPIO_PIN\(0,0\)
CFLAGS += -DDMM_TRIGGER_PIN_STOP=GPIO_PIN\(0,1\)
CFLAGS += -DDMM_SYNC_PIN=GPIO_PIN\(2,0\)
CFLAGS += -DTRANSITION_DEBUG_PIN=GPIO_PIN\(2,0\)
CFLAGS += -DOSZI_DEBUG_PIN=GPIO_PIN\(2,4\)

# PC1 (A4)
CFLAGS += -DLOGIC_ANALYZER_PIN=GPIO_PIN\(2,1\)

# PB5 (D4) to notify when leaving WFI
CFLAGS += -DDBG_GPIO_WFI=GPIO_PIN\(1,5\)

# PC5 WKUP5
CFLAGS += -DWAKEUP_GPIO=GPIO_PIN\(2,5\)
# on l476rg the additional wakeup_pin module is needed to allow wakeup also
# from the lowest of all power modes. This wakeup functionality is only available on
# some selected pins. Check the wakeup pin module for further details.
USEMODULE += gpio_wakeup
# MISO 	PA6  / CN5[5] D12
# MOSI 	PA7  / CN5[4] D11
# CS		PB6  / CN5[3] D10
# INT 	PA8  / CN9[8] D7
# WAKE  PB10 / CN9[7] D6
# RESET PB4  / CN9[6] D5
# SCK 	PA5  / CN5[6] D13
# FET   PA4  / CN8[3] A2

CFLAGS += -DAT86RF2XX_PARAM_SPI=SPI_DEV\(0\)
# The aboce impicilty allocates the following pins:
# MISO 	PA6  / CN5[5] D12 / GPIO_PIN\(0,6\)
# MOSI 	PA7  / CN5[4] D11 / GPIO_PIN\(0,7\)
# CS		PB6  / CN5[3] D10 / GPIO_PIN\(1,6\)
# SCK 	PA5  / CN5[6] D13 / GPIO_PIN\(0,5\)
CFLAGS += -DAT86RF2XX_PARAM_SPI_CLK=SPI_CLK_5MHZ
CFLAGS += -DAT86RF2XX_PARAM_CS=GPIO_PIN\(1,6\)
CFLAGS += -DAT86RF2XX_PARAM_INT=GPIO_PIN\(0,8\)
CFLAGS += -DAT86RF2XX_PARAM_SLEEP=GPIO_PIN\(1,10\)
CFLAGS += -DAT86RF2XX_PARAM_RESET=GPIO_PIN\(1,4\)
CFLAGS += -DRADIO_SUPPLY_SWITCH_PIN=GPIO_PIN\(0,4\)
CFLAGS += -DUNUSED_SPI_SC_PIN=GPIO_PIN\(1,13\)

# These pins are used to bitbang a command to the debug interface that switches it
# from JTAG to SWD mode (needed for SWO output of DWT events, that signal profiling counter overflows (LSU/CPI etc.))
# Since the ST-Link can also drive these pins it is best to connect the pins through appropriately sized resistors
# to avoid any conflicts/overcurrents.
# i.e.: (PB13/CN10.30)---[1kOhm]---(SWCLK/CN4.2)
#       (PB14/CN10.28)---[1kOhm]---(SWDIO/CN4.4)
CFLAGS += -DDEBUG_INTERFACE_SWDCLK_STIM_PIN=GPIO_PIN\(1,13\)
CFLAGS += -DDEBUG_INTERFACE_SWDIO_STIM_PIN=GPIO_PIN\(1,14\)
else ifeq (slstk3402a,$(BOARD))
CFLAGS += -DDMM_TRIGGER_PIN_START=GPIO_PIN\(2,4\)
CFLAGS += -DDMM_TRIGGER_PIN_STOP=GPIO_PIN\(2,5\)
CFLAGS += -DDMM_SYNC_PIN=GPIO_PIN\(1,9\)
#CFLAGS += -DTRANSITION_DEBUG_PIN=GPIO_PIN\(2,0\)

# on slstk3402a we use the pin that is used for connected to Button 0, pin F6
CFLAGS += -DWAKEUP_GPIO=PB0_PIN

CFLAGS += -DLOGIC_ANALYZER_PIN=GPIO_PIN\(1,11\)

CFLAGS += -DDBG_GPIO_WFI=GPIO_PIN\(1,5\)

# untested defines to silence the compiler
CFLAGS += -DOSZI_DEBUG_PIN=GPIO_UNDEF
CFLAGS += -DRADIO_SUPPLY_SWITCH_PIN=GPIO_UNDEF
CFLAGS += -DAT86RF2XX_PARAM_INT=GPIO_UNDEF
CFLAGS += -DAT86RF2XX_PARAM_SPI_CLK=SPI_CLK_5MHZ
CFLAGS += -DAT86RF2XX_PARAM_SPI=SPI_DEV\(0\)
CFLAGS += -DUNUSED_SPI_SC_PIN=GPIO_UNDEF
CFLAGS += -DDEBUG_INTERFACE_SWDCLK_STIM_PIN=GPIO_UNDEF
CFLAGS += -DDEBUG_INTERFACE_SWDIO_STIM_PIN=GPIO_UNDEF
else ifeq (esp32-wroom-32,$(BOARD))
CFLAGS += -DDEBUG_INTERFACE_SWDCLK_STIM_PIN=GPIO_UNDEF
CFLAGS += -DDEBUG_INTERFACE_SWDIO_STIM_PIN=GPIO_UNDEF
CFLAGS += -DDMM_TRIGGER_PIN_START=GPIO_UNDEF
CFLAGS += -DDMM_TRIGGER_PIN_STOP=GPIO_UNDEF
CFLAGS += -DDMM_SYNC_PIN=GPIO_UNDEF
CFLAGS += -DLOGIC_ANALYZER_PIN=GPIO_UNDEF
CFLAGS += -DDBG_GPIO_WFI=GPIO_UNDEF
# CAUTION: the LED0 pin is used for UART TX
#          (which is also the reason why the pin is not provided via the board.h config)
CFLAGS += -DLED0_PIN=GPIO_PIN\(0,1\)
# LED is low active
CFLAGS += -DLED0_ACTIVE=\(0\)
CFLAGS += -DUNUSED_SPI_SC_PIN=GPIO_UNDEF
CFLAGS += -DOSZI_DEBUG_PIN=GPIO_UNDEF
CFLAGS += -DRADIO_SUPPLY_SWITCH_PIN=GPIO_UNDEF
endif

FEATURES_REQUIRED += periph_gpio_irq

include Makefile.dep
include $(RIOTBASE)/Makefile.include
