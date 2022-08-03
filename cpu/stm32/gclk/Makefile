MODULE = stm32_gclk

ifneq (,$(filter $(CPU_FAM),l0))
  SRC += gclk_stm32l0.c
  SRC += gclk_stm32l0_gate.c
  SRC += gclk_stm32l0_mux.c
  SRC += gclk_stm32l0_scaler.c
else ifneq (,$(filter $(CPU_FAM),l4))
  SRC += gclk_stm32l4.c
endif

include $(RIOTBASE)/Makefile.base
