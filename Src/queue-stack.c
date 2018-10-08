#include "queue-stack.h"
#include <string.h>

/* Function to enqueue an item to queue */
void enQueue(struct queue* q, struct sBufferStruct* x)
{
	push(&q->stack1, x);
}

/* Function to deQueue an item from queue */
struct sBufferStruct*  deQueue(struct queue* q)
{
	struct sBufferStruct* x;

	/* If both stacks are empty then error */
	if (q->stack1 == NULL && q->stack2 == NULL) {		
		return 0;
	}

	/* Move elements from stack1 to stack 2 only if
	   stack2 is empty */
	if (q->stack2 == NULL) {
		while (q->stack1 != NULL) {
			x = pop(&q->stack1);
			push(&q->stack2, x);
		}
	}

	x = pop(&q->stack2);
	return x;
}

/* Function to push an item to stack*/
void push(struct sNode** top_ref, struct sBufferStruct* new_data)
{
	/* allocate node */
	struct sNode* new_node = (struct sNode*)malloc(sizeof(struct sNode));
	if (new_node == NULL) {
		printf("Stack Overflow\r\n");
	}

	/* put in the data */
	new_node->data = new_data;

	/* link the old list off the new node */
	new_node->next = (*top_ref);

	/* move the head to point to the new node */
	(*top_ref) = new_node;
}

/* Function to pop an item from stack*/
struct sBufferStruct* pop(struct sNode** top_ref)
{
	struct sBufferStruct* res;
	struct sNode* top;

	/*If stack is empty then error */
	if (*top_ref == NULL) {
		printf("Stack Underflow\r\n");
		return NULL;
	}
	else {
		top = *top_ref;
		res = top->data;
		*top_ref = top->next;
		free(top);
		return res;
	}
}
