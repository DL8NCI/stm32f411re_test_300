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

double STAT_printVolt(TStat *st, double uRef, uint16_t fullScale) {
	double u = STAT_meanValue(st)*uRef/(double)fullScale;
	double du = STAT_stdDev(st)*uRef/(double)fullScale;
	printf("%7.4f V \302\261 %5.1f mV   ",u, du*1000.0);
	return u;
	}

void STAT_printRH(TStat *st, double uRef, double uSupp, double T) {
	const double a1 = 0.0062;
	const double b1 = 0.16;
	const double a2 = 0.00216;
	const double b2 = 1.0546;
	const double cnt_max = 4096.0;
	const double R1 = 47.0;
	const double R2 = 33.0;


	double RH_c = (b1*cnt_max*R1*uSupp-STAT_meanValue(st)*(R2+R1)*uRef)/(a1*(a2*cnt_max*R1*T-b2*cnt_max*R1)*uSupp);

	double dRH_c = ((-R2-R1)*uRef)/(a1*(a2*cnt_max*R1*T-b2*cnt_max*R1)*uSupp)*STAT_stdDev(st);

	printf("%7.1f %% \302\261 %5.1f %%    ",RH_c, dRH_c);

	}

void STAT_printLux(TStat *st, double uSupp) {
	const double rr1 = 10000.0;
	const double rr2 = 10000.0;
	const double u33 = 3.3;
	const double c4096 = 4096.0;
	const double a = -1.382618914854915;
	const double b = 14.66930934325008;

	double lux=exp(a*log((exp(b/a)*STAT_meanValue(st)*rr1*rr2*u33)/(c4096*rr2*uSupp-STAT_meanValue(st)*rr2*u33-STAT_meanValue(st)*rr1*u33)));

	printf("%7.1f lux",lux);
	}


