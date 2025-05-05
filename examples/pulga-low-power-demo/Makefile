# name of your application
APPLICATION = pulga-low-power-demo

# If no BOARD is found in the environment, use this default:
BOARD ?= native

FEATURES_OPTIONAL += periph_gpio_irq
FEATURES_OPTIONAL += periph_rtc

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ztimer
USEMODULE += ztimer_msec
USEMODULE += ztimer_usec
USEMODULE += ztimer_sec

# avoid running Kconfig by default
SHOULD_RUN_KCONFIG ?=

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include
