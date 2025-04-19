/*
 * Task_Handler.c
 *
 *  Created on: Apr 19, 2025
 *      Author: hanif
 */

#include "main.h"
#include <stdint.h>
#include "stm32f103Driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "String.h"

uint8_t message_valid = 1 ;

void Command_Task_Handler(void* parameters){
	//BaseType_t returnValue;
	uint8_t command;

	while (1){
		//initially it is blocked using notify wait. If notification is sent, it will unblock and ready to run
		 xTaskNotifyWait(0,0, NULL,portMAX_DELAY );

		 if (xQueueReceive(q_data, &command, 0) == pdPASS){
			 if (command >= '0' && command <= '5'){
				 uint8_t cmd_num = command - '0';
				 effect = cmd_num;

				 if (cmd_num <= 4) {
				      xTaskNotify(led_task_Handle, 0, eNoAction);
				   }
				 else {
					 led_stop ();
					 message_valid = 1;
					 xTaskNotify(menu_task_Handle, 0, eNoAction);
				 }
			 }
			 else {
				 led_stop ();
				 message_valid = 0;
				 xTaskNotify(menu_task_Handle, 0, eNoAction);
			 }

		 }




	}
}

void LED_Task_Handler(void* parameters){

	while (1){
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		if (effect == 0){
			xTimerStop(handle_led_timer, 0);
			led_stop ();
		}

		else {
			xTimerStart(handle_led_timer, 0);
		}
	}

}


void Menu_Task_Handler(void* parameters){
	char* menu_msg =
	        "\r\n--- LED Effects ---\r\n"
	        "0: Stop LED effect\r\n"
	        "1: LED Effect 1\r\n"
	        "2: LED Effect 2\r\n"
	        "3: LED Effect 3\r\n"
	        "4: LED Effect 4\r\n"
	        "Enter your choice: ";
	char* invalid_msg = "\r\n--- Command Invalid ---\r\n"
						"Enter 5 to back: ";

	while (1){
		// Send menu to print task
		//size_t menu_msg_len = strlen(menu_msg);
		//size_t invalid_msg_len = strlen(invalid_msg);
		if (message_valid){

			xQueueSend(q_print, &menu_msg, portMAX_DELAY);

			}
		else {

			xQueueSend(q_print, &invalid_msg, portMAX_DELAY);

		}
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY );

	}

}


void Print_Task_Handler(void* parameters) {
	char *msg_to_print;

    while (1) {
    	//USART_transmit(1, 'h');
        if (xQueueReceive(q_print, &msg_to_print, portMAX_DELAY) == pdPASS) {
        	USART_send_string(1, msg_to_print);
        }

        //xTaskNotify(commmand_task_Handle, 0, eNoAction);
    }
}




