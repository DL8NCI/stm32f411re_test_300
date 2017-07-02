/*
 * Statistics.c
 *
 *  Created on: 16.06.2017
 *      Author: DL8NCI
 */


#include "Statistics.h"
#include "math.h"

void STAT_init(TStat *st) {
	st->sx = 0;
	st->sx2 = 0.0;
	st->n = 0;
	st->max = 0;
	st->min = 0xffff;
	}

void STAT_add(TStat *st, uint16_t x) {
	st->sx += x;
	st->sx2 += (double)x * (double)x;
	st->n++;

	if (x>st->max) st->max = x;
	if (x<st->min) st->min = x;
	}

double STAT_meanValue(TStat *st) {
	return (double)st->sx/(double)st->n;
	}

double STAT_stdDev(TStat *st) {
	double h = (double)st->sx/(double)st->n;
	return sqrt(st->sx2/(double)st->n - h*h);
	}

uint16_t STAT_interval(TStat *st) {
	return (st->max - st->min);
	}

void STAT_print(TStat *st) {

	double sd = STAT_stdDev(st);
	uint16_t dy = STAT_interval(st);

	if (sd>=100) sd = 99.99;
	if (dy>=100) dy = 99;
	printf("%7.1f (%5.2f)[%2d]  ",STAT_meanValue(st), sd, dy);
	}

void STAT_printVolt(TStat *st, double uRef, uint16_t fullScale) {
	double u = STAT_meanValue(st)*uRef/(double)fullScale;
	double du = STAT_stdDev(st)*uRef/(double)fullScale;
	printf("%7.4f V \302\261 %5.1f mV   ",u, du*1000.0);
	}

void STAT_printRH(TStat *st, double uSupp, double T) {
	const double a1 = 0.0062;
	const double b1 = 0.16;
	const double a2 = 0.00216;
	const double b2 = 1.0546;
	const double cnt_max = 4096.0;
	const double uRef = 3.0;
	const double R1 = 47.0;
	const double R2 = 33.0;


	double RH_c = (b1*cnt_max*R1*uSupp-STAT_meanValue(st)*(R2+R1)*uRef)/(a1*(a2*cnt_max*R1*T-b2*cnt_max*R1)*uSupp);

	double dRH_c = ((-R2-R1)*uRef)/(a1*(a2*cnt_max*R1*T-b2*cnt_max*R1)*uSupp)*STAT_stdDev(st);

	printf("%7.1f %% \302\261 %5.1f %%    ",RH_c, dRH_c);

	}


