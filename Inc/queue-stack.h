/**
  ******************************************************************************
  * @file           : queue-stack.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 
  * All rights reserved.
  *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QUEUE_STACK_H__
#define __QUEUE_STACK_H__

#include <stdio.h>   
#include <stdlib.h> 
#include <stdint.h> 

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

struct sBufferStruct {
	uint32_t address;
	volatile uint32_t pos;
	volatile uint32_t length;
	volatile uint32_t frame_id;
	uint32_t maxlen;
};

/* structure of a stack node */
struct sNode {
	struct sBufferStruct* data;
	struct sNode* next;
};

/* Function to push an item to stack*/
void push(struct sNode** top_ref, struct sBufferStruct* new_data);

/* Function to pop an item from stack*/
struct sBufferStruct* pop(struct sNode** top_ref);


/* structure of queue having two stacks */
struct queue {
	struct sNode* stack1;
	struct sNode* stack2;
	unsigned char queuename[15];
};


void enQueue(struct queue* q, struct sBufferStruct* x);
struct sBufferStruct* deQueue(struct queue* q);

#endif /* __QUEUE_STACK_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
