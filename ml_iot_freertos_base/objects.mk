

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


# Application objects
OBJECTS += $(BUILD_DIR)/App/src/led_task.o
OBJECTS += $(BUILD_DIR)/App/src/client_task.o
OBJECTS += $(BUILD_DIR)/App/src/server_task.o


# Driver objects
OBJECTS += $(BUILD_DIR)/Drivers/GPIO/src/dri_gpio_pins.o
OBJECTS += $(BUILD_DIR)/Drivers/GPIO/src/dri_gpio.o

OBJECTS += $(BUILD_DIR)/Drivers/MEM/src/dri_mem.o

OBJECTS += $(BUILD_DIR)/Drivers/UART/cfg/src/dri_uart_conf.o
OBJECTS += $(BUILD_DIR)/Drivers/UART/src/dri_uart.o


# Trace objects
OBJECTS += $(BUILD_DIR)/Trace/src/trace.o

# RTOS Wrappers objects
OBJECTS += $(BUILD_DIR)/Wrappers/src/rtos_wrappers.o
OBJECTS += $(BUILD_DIR)/Wrappers/src/wrappers.o


# Main Source Files
OBJECTS += $(BUILD_DIR)/main.o


# CMSIS NN Lib Objects
ifeq ($(USE_CMSIS_NN), 1)

OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ActivationFunctions/arm_relu6_s8.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.o

OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_add_s8.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_mul_s8.o

OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_w.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_x.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_y.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_z.o

OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_HWC_q7_fast_nonsquare.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_basic_nonsquare.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8_opt.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15_reordered.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_s8_fast.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_fast.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_u8_basic_ver1.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_basic.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_fast_nonsquare.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16_reordered.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_fast.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_RGB.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7_nonsquare.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_s8.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_fast_nonsquare.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_s8.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_depthwise_conv_s8_core.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_basic.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15.o

OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q15.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q7.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_s8.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15_opt.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q15_opt.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q7_opt.o

OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_nn_accumulate_q7_to_q15.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mult_q15.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_nntables.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_reordered_no_shift.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_with_offset.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_nn_add_q7.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mult_q7.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_no_shift.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_reordered_with_offset.o


OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/PoolingFunctions/arm_avgpool_s8.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/PoolingFunctions/arm_max_pool_s8.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/PoolingFunctions/arm_max_pool_s8_opt.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/PoolingFunctions/arm_pool_q7_HWC.o

OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/ReshapeFunctions/arm_reshape_s8.o

OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_q15.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_q7.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_s8.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_u8.o
OBJECTS += $(BUILD_DIR)/Libraries/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_with_batch_q7.o

endif


# FreeRTOS Kernel Objects
OBJECTS += $(BUILD_DIR)/FreeRTOS/Source/MemMang/heap_4.o

OBJECTS += $(BUILD_DIR)/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o

OBJECTS += $(BUILD_DIR)/FreeRTOS/Source/source/croutine.o
OBJECTS += $(BUILD_DIR)/FreeRTOS/Source/source/event_groups.o
OBJECTS += $(BUILD_DIR)/FreeRTOS/Source/source/list.o
OBJECTS += $(BUILD_DIR)/FreeRTOS/Source/source/queue.o
OBJECTS += $(BUILD_DIR)/FreeRTOS/Source/source/stream_buffer.o
OBJECTS += $(BUILD_DIR)/FreeRTOS/Source/source/tasks.o
OBJECTS += $(BUILD_DIR)/FreeRTOS/Source/source/timers.o


ifeq ($(USE_FREERTOS_TCP), 1)
# Source files
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_ARP.o
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_DHCP.o
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_DNS.o
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_IP.o
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_Sockets.o
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_Stream_Buffer.o
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_IP.o
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_WIN.o
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/source/FreeRTOS_UDP_IP.o

# port files
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/portable/BufferManagement/BufferAllocation_2.o
OBJECTS += $(BUILD_DIR)/FreeRTOS-Plus/FreeRTOS-Plus-TCP/portable/NetworkInterface/XMC4500/NetworkInterface.o
endif


