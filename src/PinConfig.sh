#!/bin/sh
# Fitaque pins - Kernel 4.14

sleep 5

config-pin P9_17 spi_cs         # CS
config-pin P9_21 spi            # DO
config-pin P9_18 spi            # DI
config-pin P9_22 spi_sclk       # CLK

config-pin P9_14 out            # Relay
config-pin P8_11 in_pd          # Switch

config-pin P9_24 out            # ADC 1
config-pin P9_26 out            # ADC2
config-pin P9_23 out            # DAC

sleep 10
