/*
 * Statistics.h
 *
 *  Created on: 16.06.2017
 *      Author: DL8NCI
 */

#ifndef STATISTICS_H_
#define STATISTICS_H_

#include  <sys/unistd.h>


typedef struct {
	uint32_t sx;
	double sx2;
	uint16_t n;
	uint16_t min;
	uint16_t max;
	} TStat;


void STAT_init(TStat *st);
void STAT_add(TStat *st, uint16_t x);
double STAT_meanValue(TStat *st);
double STAT_stdDev(TStat *st);
uint16_t STAT_interval(TStat *st);




#endif /* STATISTICS_H_ */
