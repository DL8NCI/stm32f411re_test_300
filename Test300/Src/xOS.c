/*
 * xOS.c
 *
 *  Created on: 08.08.2017
 *      Author: DL8NCI
 */


#include "xOS.h"

void xOS_init() {
	xOS_status.in = 0;
	xOS_status.out = 0;
	xOS_status.len = 0;
	}



void xOS_execute() {
	struct TxOS_task task;
	for (;;) {
		task = getNextTask();
		if (task.execute!=NULL) {
			task.execute();
			}
		}
	}



int xOS_queue(struct TxOS_task task) {
	if (xOS_status.len<XOS_MAX_TASK) {
		xOS_status.queue[xOS_status.in++] = task;
		xOS_status.len++;
		if (xOS_status.in==XOS_MAX_TASK) xOS_status.in = 0;
		return 0;
		}
	else
		return -1;
	}


static struct TxOS_task getNextTask() {
	struct TxOS_task task;
	if (xOS_status.len>0) {
		task = xOS_status.queue[xOS_status.out++];
		xOS_status.len--;
		if (xOS_status.out==XOS_MAX_TASK) xOS_status.out = 0;

		if (task.taskMode == tmForever) xOS_queue(task);
		}
	else {
		task.execute = NULL;
		task.taskMode = tmNone;
		return task;
		}
	}
