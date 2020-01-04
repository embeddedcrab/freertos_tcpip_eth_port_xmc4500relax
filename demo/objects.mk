

# Startup Files
OBJECTS += $(BUILD_DIR)/Startup/startup_XMC4500.o
OBJECTS += $(BUILD_DIR)/Startup/system_XMC4500.o


# Library Files
OBJECTS += $(BUILD_DIR)/Libraries/Newlib/syscalls.o

OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc4_eru.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc4_flash.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc4_gpio.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc4_rtc.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc4_scu.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_can.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_ccu4.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_ccu8.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_common.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_dac.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_dma.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_dsd.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_ebu.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_ecat.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_eru.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_eth_mac.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_eth_phy_dp83848.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_eth_phy_ksz8031rnl.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_eth_phy_ksz8081rnb.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_fce.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_gpio.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_hrpwm.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_i2c.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_i2s.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_ledts.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_posif.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_rtc.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_sdmmc.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_spi.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_uart.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_usbd.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_usbh.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_usic.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_vadc.o
OBJECTS += $(BUILD_DIR)/Libraries/XMCLib/src/xmc_wdt.o


# Main Source Files
OBJECTS += $(BUILD_DIR)/main.o


