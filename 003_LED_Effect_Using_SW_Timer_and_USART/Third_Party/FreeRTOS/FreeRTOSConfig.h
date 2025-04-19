#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>  // Required for fixed-width integer types like uint32_t

/*-----------------------------------------------------------
 * Application specific definitions.
 * These settings configure core FreeRTOS behavior.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION                    1
// Enable preemptive multitasking (vs. cooperative)

#define configUSE_IDLE_HOOK                     0
// 1 = Call vApplicationIdleHook() each time the idle task runs (optional user code)

#define configUSE_TICK_HOOK                     0
// 1 = Call vApplicationTickHook() each tick interrupt (for periodic tasks)

#define configCPU_CLOCK_HZ                      ( ( uint32_t ) 72000000 )
// Your MCU clock speed in Hz (STM32F103 runs at 72 MHz)

#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
// Tick rate of 1 kHz = 1 ms tick interval

#define configMAX_PRIORITIES                    5
// Number of task priorities you want to support (0 to 4)

#define configMINIMAL_STACK_SIZE                ( ( uint16_t ) 200 )
// Stack size for the idle task (words, not bytes)

#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 12 * 1024 ) )
// Total heap size used for dynamic allocation (in bytes)

#define configMAX_TASK_NAME_LEN                 16
// Max length of task names (including null terminator)

#define configUSE_16_BIT_TICKS                  0
// 0 = Use 32-bit tick counter (default), 1 = Use 16-bit (saves memory)

#define configIDLE_SHOULD_YIELD                 1
// Idle task should yield CPU if higher priority tasks are ready

#define configUSE_TIME_SLICING                  1
// Enable round-robin time-slicing between tasks at the same priority

/*-----------------------------------------------------------
 * Synchronization primitives (Mutexes and Semaphores)
 *----------------------------------------------------------*/

#define configUSE_MUTEXES                       1
// Enable binary and recursive mutex support

#define configUSE_RECURSIVE_MUTEXES             1
// Allow recursive mutexes (a task can re-acquire the same mutex)

#define configUSE_COUNTING_SEMAPHORES           1
// Enable counting semaphores (not just binary)

/*-----------------------------------------------------------
 * Memory allocation and stack overflow
 *----------------------------------------------------------*/

#define configSUPPORT_DYNAMIC_ALLOCATION        1
// Enable dynamic memory (using malloc-style functions)

#define configSUPPORT_STATIC_ALLOCATION         0
// Disable static allocation (if you don't use it)

#define configCHECK_FOR_STACK_OVERFLOW          0
// 1 or 2 = Enable stack overflow checks (2 is more thorough)

#define configUSE_MALLOC_FAILED_HOOK            0
// 1 = Call vApplicationMallocFailedHook() on malloc fail

/*-----------------------------------------------------------
 * Run-time stats and trace
 *----------------------------------------------------------*/

#define configUSE_TRACE_FACILITY                0
// 1 = Enable tracing (used for monitoring tools)

#define configUSE_STATS_FORMATTING_FUNCTIONS    0
// 1 = Include vTaskList() and vTaskGetRunTimeStats() functions

#define configGENERATE_RUN_TIME_STATS           0
// 1 = Track task run-times (requires timer setup)

/*-----------------------------------------------------------
 *  API functions to include
 *----------------------------------------------------------*/

#define INCLUDE_vTaskPrioritySet                1
// Enable vTaskPrioritySet()

#define INCLUDE_uxTaskPriorityGet               1
// Enable uxTaskPriorityGet()

#define INCLUDE_vTaskDelete                     1
// Enable vTaskDelete()

#define INCLUDE_vTaskCleanUpResources           1
// Enable vTaskCleanUpResources() (not usually needed)

#define INCLUDE_vTaskSuspend                    1
// Enable vTaskSuspend()/vTaskResume()

#define INCLUDE_vTaskDelayUntil                 1
// Enable vTaskDelayUntil() (useful for periodic tasks)

#define INCLUDE_vTaskDelay                      1
// Enable vTaskDelay()

// #define INCLUDE_xTaskGetIdleTaskHandle        1
// Uncomment if you want to access the idle task handle

// #define INCLUDE_pxTaskGetStackStart           1
// Uncomment to access a task's stack start address

/*-----------------------------------------------------------
 * Software Timer Definitions
 *----------------------------------------------------------*/
#define configUSE_TIMERS 						1
#define configTIMER_TASK_PRIORITY 				1
#define configTIMER_QUEUE_LENGTH 				10
#define configTIMER_TASK_STACK_DEPTH         (configMINIMAL_STACK_SIZE*2)

/*-----------------------------------------------------------
 * Cortex-M interrupt priority configuration
 *----------------------------------------------------------*/
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   0xF  // 4-bit priority space (0x0 to 0xF)
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configPRIO_BITS         4  //4 bits priority
//Interrupt priorities are used by the kernel port layer itself
//these are generic to all cortex-m ports and do not relay on any particular priority function
// Priority values are shifted left by 5 bits (to match ARM NVIC)
#define configKERNEL_INTERRUPT_PRIORITY  (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
// Lowest priority for the kernel's internal interrupts (0xE0)

#define configMAX_SYSCALL_INTERRUPT_PRIORITY     (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
// Max priority (numerically lower = higher) from which FreeRTOS API can be safely called from ISRs

//#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY 15
// STM32 HAL/CMSIS uses this to set priority groupings

/*-----------------------------------------------------------
 * Assertion macro
 *----------------------------------------------------------*/

#define configASSERT( x ) if (( x ) == 0) { taskDISABLE_INTERRUPTS(); for( ;; ); }
// Assertion for catching config or logic errors at runtime

/*-----------------------------------------------------------
 * Handler function mapping for Cortex-M port
 *----------------------------------------------------------*/

#define xPortPendSVHandler     PendSV_Handler
// Maps FreeRTOS's PendSV handler to the CMSIS name

#define vPortSVCHandler        SVC_Handler
// Maps FreeRTOS's SVC handler to CMSIS name

#define xPortSysTickHandler    SysTick_Handler
// Maps FreeRTOS's SysTick handler to CMSIS name

/*-----------------------------------------------------------
 *  UART buffer sizes (only used with FreeRTOS+IO)
 *----------------------------------------------------------*/

#define configCOM0_RX_BUFFER_LENGTH             128
#define configCOM0_TX_BUFFER_LENGTH             128
#define configCOM1_RX_BUFFER_LENGTH             128
#define configCOM1_TX_BUFFER_LENGTH             128

/*-----------------------------------------------------------
 *  SEGGER SystemView tracing
 *----------------------------------------------------------*/

// #include "SEGGER_SYSVIEW_FreeRTOS.h"
// Uncomment if you're using SEGGER SystemView for real-time analysis

#endif /* FREERTOS_CONFIG_H */
