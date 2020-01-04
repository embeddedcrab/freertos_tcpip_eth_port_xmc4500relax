
# Includes for project
INCLUDES_DIR += $(PROJECT_DIR)/Libraries/CMSIS/Include/
INCLUDES_DIR += $(PROJECT_DIR)/Libraries/CMSIS/Infineon/XMC4500_series/Include/
INCLUDES_DIR += $(PROJECT_DIR)/Libraries/XMCLib/inc/


# Startup files
SOURCES += $(PROJECT_DIR)/Startup/startup_XMC4500.s
SOURCES += $(PROJECT_DIR)/Startup/system_XMC4500.c


# Library files
SOURCES += $(PROJECT_DIR)/Libraries/Newlib/syscalls.c


SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc4_eru.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc4_flash.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc4_gpio.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc4_rtc.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc4_scu.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_can.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_ccu4.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_ccu8.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_common.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_dac.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_dma.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_dsd.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_ebu.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_ecat.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_eru.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_eth_mac.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_eth_phy_dp83848.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_eth_phy_ksz8031rnl.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_eth_phy_ksz8081rnb.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_fce.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_gpio.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_hrpwm.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_i2c.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_i2s.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_ledts.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_posif.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_rtc.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_sdmmc.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_spi.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_uart.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_usbd.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_usbh.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_usic.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_vadc.c
SOURCES += $(PROJECT_DIR)/Libraries/XMCLib/src/xmc_wdt.c


# Main source files
SOURCES += $(PROJECT_DIR)/main.cpp


