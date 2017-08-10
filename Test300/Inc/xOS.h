/*
 * xOS.h
 *
 *  Created on: 08.08.2017
 *      Author: DL8NCI
 */

#ifndef XOS_H_
#define XOS_H_

#include  <sys/unistd.h>
#include "xOS_config.h"

static enum E_xOS_taskMode { tmNone, tmOnce, tmForever };


struct TxOS_task {
	long (*execute) (void);
	enum E_xOS_taskMode taskMode;
	};


struct TxOS_status {
	struct TxOS_task queue[XOS_MAX_TASK];
	uint8_t in, out, len;
	} xOS_status;



void xOS_init();
void xOS_execute();
int xOS_queue(struct TxOS_task task);

static struct TxOS_task getNextTask();


#endif /* XOS_H_ */
