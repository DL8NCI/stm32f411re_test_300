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
	uint32_t sx;			// sum x
	double sx2;				// sum x^2
	uint16_t n;				// number of samples
	uint16_t min;			// min of all x
	uint16_t max;			// max of all x
	} TStat;

void STAT_init(TStat *st);
void STAT_add(TStat *st, uint16_t x);
double STAT_meanValue(TStat *st);
double STAT_stdDev(TStat *st);
uint16_t STAT_interval(TStat *st);
void STAT_print(TStat *st);
double STAT_printVolt(TStat *st, double uRef, uint16_t fullScale);
void STAT_printRH(TStat *st, double uRef, double uSupp, double T);
void STAT_printLux(TStat *st, double uSupp);


#endif /* STATISTICS_H_ */
