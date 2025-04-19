################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/System_Config.c \
../Drivers/stm32f103Driver.c 

OBJS += \
./Drivers/System_Config.o \
./Drivers/stm32f103Driver.o 

C_DEPS += \
./Drivers/System_Config.d \
./Drivers/stm32f103Driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/%.o Drivers/%.su Drivers/%.cyclo: ../Drivers/%.c Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Src" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Drivers" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Third_Party/FreeRTOS" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Third_Party/FreeRTOS/include" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Third_Party/FreeRTOS/portable/GCC/ARM_CM3" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers

clean-Drivers:
	-$(RM) ./Drivers/System_Config.cyclo ./Drivers/System_Config.d ./Drivers/System_Config.o ./Drivers/System_Config.su ./Drivers/stm32f103Driver.cyclo ./Drivers/stm32f103Driver.d ./Drivers/stm32f103Driver.o ./Drivers/stm32f103Driver.su

.PHONY: clean-Drivers

