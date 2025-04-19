################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Third_Party/FreeRTOS/portable/GCC/ARM_CM3/port.c 

OBJS += \
./Third_Party/FreeRTOS/portable/GCC/ARM_CM3/port.o 

C_DEPS += \
./Third_Party/FreeRTOS/portable/GCC/ARM_CM3/port.d 


# Each subdirectory must supply rules for building sources it contributes
Third_Party/FreeRTOS/portable/GCC/ARM_CM3/%.o Third_Party/FreeRTOS/portable/GCC/ARM_CM3/%.su Third_Party/FreeRTOS/portable/GCC/ARM_CM3/%.cyclo: ../Third_Party/FreeRTOS/portable/GCC/ARM_CM3/%.c Third_Party/FreeRTOS/portable/GCC/ARM_CM3/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"D:/Embedded_C/FreeRTOS_STM32f103/002_Free_RTOS_Interuupts/Drivers" -I"D:/Embedded_C/FreeRTOS_STM32f103/002_Free_RTOS_Interuupts/Third_Party/FreeRTOS" -I"D:/Embedded_C/FreeRTOS_STM32f103/002_Free_RTOS_Interuupts/Third_Party/FreeRTOS/include" -I"D:/Embedded_C/FreeRTOS_STM32f103/002_Free_RTOS_Interuupts/Third_Party/FreeRTOS/portable/GCC/ARM_CM3" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Third_Party-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM3

clean-Third_Party-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM3:
	-$(RM) ./Third_Party/FreeRTOS/portable/GCC/ARM_CM3/port.cyclo ./Third_Party/FreeRTOS/portable/GCC/ARM_CM3/port.d ./Third_Party/FreeRTOS/portable/GCC/ARM_CM3/port.o ./Third_Party/FreeRTOS/portable/GCC/ARM_CM3/port.su

.PHONY: clean-Third_Party-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM3

