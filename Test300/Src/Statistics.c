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

double STAT_printCpuTemperature(TStat *st) {
	uint16_t *cal1 = (uint16_t*) (0x1FFF7A2C);
	uint16_t *cal2 = (uint16_t*) (0x1FFF7A2E);

	double a = 80.0/((double)*cal2-(double)*cal1);
	double b = (110.0*(double)*cal1-30.0*(double)*cal2)/((double)*cal1-(double)*cal2);

	double t = STAT_meanValue(st)*a+b;
	double dt = STAT_stdDev(st)*a;

	printf("%7.1f deg C \302\261 %3.1f deg C",t,dt);
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

void STAT_printLux(TStat *st, double uSupp) { // https://unterricht.educa.ch/sites/default/files/20101215/ldr.pdf
	const double rr1 = 10000.0;
	const double rr2 = 10000.0;
	const double u33 = 3.3;
	const double c4096 = 4096.0;
	const double a = -1.382618914854915;
	const double b = 14.66930934325008;
	const double lA=log(40530.0683987197);
	const double alpha = 0.72326509442042;

	double cnt = STAT_meanValue(st);

// (%o13) [alpha=0.72326509442042,A=40530.0683987197]
// E(R):=exp((log(A)-log(R))/alpha)


/*
 *
 *                   o uSupp (ca. 5V)
 *                   |
 *                   |
 *                 +-+-+
 *                 |   |
 *                 | R |
 *                 | 1 |
 *                 |   |
 *                 +-+-+
 *                   |
 *                   |
 *               +---o---+-----o ux
 *               |       |
 *               |       |
 *             +-+-+   +-+-+
 *             |   |   |   | LDR
 *             | R |   | R |
 *             | 2 |   | x |
 *             |   |   |   |
 *             +-+-+   +-+-+
 *               |       |
 *               |       |
 *              ===     ===
 *
 *
 *
 */


	double rx=(cnt*rr1*rr2*u33)/(c4096*rr2*uSupp+(-cnt*rr2-cnt*rr1)*u33);
	double lux = exp((lA-log(rx))/alpha);

//	double lux = exp(a*log((cnt*rr1*rr2*u33)/(c4096*rr2*uSupp+(-cnt*rr2-cnt*rr1)*u33))+b);
//	double dlux = STAT_stdDev(st)*lux*a*c4096*rr2*uSupp/(cnt*c4096*rr2*uSupp-cnt*cnt*u33*(rr1+rr2));
//	printf("%7.1f lux \302\261 %5.2f",lux,dlux);
	printf("%7.1f Lx",lux);
	}


double STAT_printPT1000Temperature(TStat *st, double uSupp, double uRef) {
	const double r1 = 1000.0;
	const double c4096 = 4096.0;

	double cnt = STAT_meanValue(st);
	double rx = (cnt*r1*uRef)/(c4096*uSupp-cnt*uRef);
	double r = rx/1000.0 - 1.0;
	double t = r*(255.8723+r*(9.6+r*0.878));

	printf("%7.1f deg C",t);
	}


