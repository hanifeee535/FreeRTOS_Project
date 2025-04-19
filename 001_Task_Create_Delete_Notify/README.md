## LED Blinking with FreeRTOS

This project demonstrates a sequential control of three LEDs (Green, Red, Blue) using FreeRTOS Tasks. On startup, All of  the  task is active and toggling its corresponding LED. When the user presses the button, the "Green LED" task turns its LED on then signals the "RED_LED" task, and then deletes itself. The "RED_LED" task then starts toggling its LED. Another button press will cause the "RED_LED" task to turn its LED on, signal the "Blue_LED" task, and delete itself. Finally, the "Blue_LED" task will toggle its LED, and upon the next button press, it will turn on its LED and delete both itself and the button monitoring task.

#This project is based on basic FreeRTOS concepts such as:
				-- task creation
				-- task scheduling 
				-- inter-task communication using task notifications
				-- dynamic task management.

##Folder Structure: 

			│   └── Src/
			│       ├── main.c
			│       └── syscalls.c
			│       └── system.c
			│
			│   └── Startup/
			│       ├── startup_stm32f103rbtx.s
			│
			│   └── Third_Party/
			│   	└── FreeRTOS/
			│       	├── FreeRTOS files
			│
			│   └── Drivers/
			│       ├── stm32f103Driver.c
			│       └── stm32f103Driver.h
			│       └── System_Config.c



## Hardware:

* STM32 NUCLEO-F103RB
* LED

## Hardware Connections:

* **Green LED:** Connected to Port C, Pin 0.
* **Red LED:** Connected to Port C, Pin 1.
* **Blue LED:** Connected to Port B, Pin 0.
* **User Button:** Connected to Port C, Pin 13 (configure pull-up/pull-down resistor as needed).

