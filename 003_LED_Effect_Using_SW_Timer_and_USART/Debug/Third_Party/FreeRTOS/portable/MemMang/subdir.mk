################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Third_Party/FreeRTOS/portable/MemMang/heap_4.c 

OBJS += \
./Third_Party/FreeRTOS/portable/MemMang/heap_4.o 

C_DEPS += \
./Third_Party/FreeRTOS/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
Third_Party/FreeRTOS/portable/MemMang/%.o Third_Party/FreeRTOS/portable/MemMang/%.su Third_Party/FreeRTOS/portable/MemMang/%.cyclo: ../Third_Party/FreeRTOS/portable/MemMang/%.c Third_Party/FreeRTOS/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Src" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Drivers" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Third_Party/FreeRTOS" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Third_Party/FreeRTOS/include" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Third_Party/FreeRTOS/portable/GCC/ARM_CM3" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Third_Party-2f-FreeRTOS-2f-portable-2f-MemMang

clean-Third_Party-2f-FreeRTOS-2f-portable-2f-MemMang:
	-$(RM) ./Third_Party/FreeRTOS/portable/MemMang/heap_4.cyclo ./Third_Party/FreeRTOS/portable/MemMang/heap_4.d ./Third_Party/FreeRTOS/portable/MemMang/heap_4.o ./Third_Party/FreeRTOS/portable/MemMang/heap_4.su

.PHONY: clean-Third_Party-2f-FreeRTOS-2f-portable-2f-MemMang

