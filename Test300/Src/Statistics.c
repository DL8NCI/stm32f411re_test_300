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

