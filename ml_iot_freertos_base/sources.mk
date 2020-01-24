
# Includes for project
INCLUDES_DIR += $(PROJECT_DIR)/Libraries/CMSIS/Include/
INCLUDES_DIR += $(PROJECT_DIR)/Libraries/CMSIS/Infineon/XMC4500_series/Include/
INCLUDES_DIR += $(PROJECT_DIR)/Libraries/XMCLib/inc/


# App headers
INCLUDES_DIR += $(PROJECT_DIR)/App/inc/


# Drivers headers
INCLUDES_DIR += $(PROJECT_DIR)/Drivers/inc/
INCLUDES_DIR += $(PROJECT_DIR)/Drivers/MEM/inc/
INCLUDES_DIR += $(PROJECT_DIR)/Drivers/GPIO/inc/
INCLUDES_DIR += $(PROJECT_DIR)/Drivers/UART/cfg/inc/
INCLUDES_DIR += $(PROJECT_DIR)/Drivers/UART/inc/


# Trace headers
INCLUDES_DIR += $(PROJECT_DIR)/Trace/inc/


# RTOS Wrappers
INCLUDES_DIR += $(PROJECT_DIR)/Wrappers/inc/


# DSP Lib Headers
ifeq ($(USE_DSP_LIB), 1)
INCLUDES_DIR += $(PROJECT_DIR)/Libraries/CMSIS/DSP/Include/
endif

# CMSIS NN Lib Headers
ifeq ($(USE_CMSIS_NN), 1)
INCLUDES_DIR += $(PROJECT_DIR)/Libraries/CMSIS/NN/Include/
endif


# FreeRTOS Kernel headers
INCLUDES_DIR += $(PROJECT_DIR)/FreeRTOS/config/
INCLUDES_DIR += $(PROJECT_DIR)/FreeRTOS/Source/include/
INCLUDES_DIR += $(PROJECT_DIR)/FreeRTOS/Source/portable/GCC/ARM_CM4F/


ifeq ($(USE_FREERTOS_TCP), 1)
INCLUDES_DIR += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/include/
INCLUDES_DIR += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/portable/Compiler/GCC/

# May need to change {if block} on basis of add-ons in FreeRTOS-Plus functionalities
INCLUDES_DIR += $(PROJECT_DIR)/FreeRTOS-Plus/config/
endif


# ARM NN Test
INCLUDES_DIR += $(PROJECT_DIR)/ML_Test/



# Startup files
SOURCES += $(PROJECT_DIR)/Startup/startup_XMC4500.s
SOURCES += $(PROJECT_DIR)/Startup/system_XMC4500.c


# Library files
SOURCES += $(PROJECT_DIR)/Libraries/Newlib/syscalls.c


# XMC4500 Hardware Source Library
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


# Application sources
SOURCES += $(PROJECT_DIR)/App/src/led_task.cpp
SOURCES += $(PROJECT_DIR)/App/src/client_task.cpp
SOURCES += $(PROJECT_DIR)/App/src/server_task.cpp


# Drivers sources
SOURCES += $(PROJECT_DIR)/Drivers/GPIO/src/dri_gpio_pins.cpp
SOURCES += $(PROJECT_DIR)/Drivers/GPIO/src/dri_gpio.cpp

SOURCES += $(PROJECT_DIR)/Drivers/MEM/src/dri_mem.cpp

SOURCES += $(PROJECT_DIR)/Drivers/UART/cfg/src/dri_uart_conf.cpp
SOURCES += $(PROJECT_DIR)/Drivers/UART/src/dri_uart.cpp


# Trace sources
SOURCES += $(PROJECT_DIR)/Trace/src/trace.c

# RTOS Wrappers
SOURCES +=$(PROJECT_DIR)/Wrappers/src/rtos_wrappers.cpp
SOURCES +=$(PROJECT_DIR)/Wrappers/src/wrappers.cpp


# Main source files
SOURCES += $(PROJECT_DIR)/main.cpp


# CMSIS NN library source inclusion
ifeq ($(USE_CMSIS_NN), 1)

SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ActivationFunctions/arm_relu6_s8.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.c

SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_add_s8.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_mul_s8.c

SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_w.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_x.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_y.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_z.c

SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_HWC_q7_fast_nonsquare.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_basic_nonsquare.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8_opt.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15_reordered.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_s8_fast.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_fast.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_u8_basic_ver1.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_basic.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_fast_nonsquare.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16_reordered.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_fast.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_RGB.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7_nonsquare.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_s8.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_fast_nonsquare.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_s8.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_depthwise_conv_s8_core.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_basic.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15.c

SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q15.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q7.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_s8.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15_opt.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q15_opt.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q7_opt.c

SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_nn_accumulate_q7_to_q15.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mult_q15.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_nntables.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_reordered_no_shift.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_with_offset.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_nn_add_q7.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mult_q7.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_no_shift.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_reordered_with_offset.c


SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/PoolingFunctions/arm_avgpool_s8.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/PoolingFunctions/arm_max_pool_s8.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/PoolingFunctions/arm_max_pool_s8_opt.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/PoolingFunctions/arm_pool_q7_HWC.c

SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/ReshapeFunctions/arm_reshape_s8.c

SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_q15.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_q7.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_s8.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_u8.c
SOURCES += $(PROJECT_DIR)/Libraries/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_with_batch_q7.c

endif


# FreeRTOS Kernel Source
SOURCES += $(PROJECT_DIR)/FreeRTOS/Source/MemMang/heap_4.c

SOURCES += $(PROJECT_DIR)/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c

SOURCES += $(PROJECT_DIR)/FreeRTOS/Source/source/croutine.c
SOURCES += $(PROJECT_DIR)/FreeRTOS/Source/source/event_groups.c
SOURCES += $(PROJECT_DIR)/FreeRTOS/Source/source/list.c
SOURCES += $(PROJECT_DIR)/FreeRTOS/Source/source/queue.c
SOURCES += $(PROJECT_DIR)/FreeRTOS/Source/source/stream_buffer.c
SOURCES += $(PROJECT_DIR)/FreeRTOS/Source/source/tasks.c
SOURCES += $(PROJECT_DIR)/FreeRTOS/Source/source/timers.c


ifeq ($(USE_FREERTOS_TCP), 1)
# Source files
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_ARP.c
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_DHCP.c
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_DNS.c
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_IP.c
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_Sockets.c
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_Stream_Buffer.c
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_IP.c
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_WIN.c
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_UDP_IP.c

# port files
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/portable/BufferManagement/BufferAllocation_2.c
SOURCES += $(PROJECT_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/portable/NetworkInterface/XMC4500/NetworkInterface.c
endif



