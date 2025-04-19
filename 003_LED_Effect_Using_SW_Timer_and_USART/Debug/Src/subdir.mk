################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/LED_Effects.c \
../Src/Task_Handler.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/LED_Effects.o \
./Src/Task_Handler.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/LED_Effects.d \
./Src/Task_Handler.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Src" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Drivers" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Third_Party/FreeRTOS" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Third_Party/FreeRTOS/include" -I"D:/Embedded_C/FreeRTOS_STM32f103/003_LED_Effect_Using_SW_Timer_and_USART/Third_Party/FreeRTOS/portable/GCC/ARM_CM3" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/LED_Effects.cyclo ./Src/LED_Effects.d ./Src/LED_Effects.o ./Src/LED_Effects.su ./Src/Task_Handler.cyclo ./Src/Task_Handler.d ./Src/Task_Handler.o ./Src/Task_Handler.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

