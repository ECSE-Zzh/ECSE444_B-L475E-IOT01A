/*
 * lab1math.h
 *
 *  Created on: Sep 11, 2023
 *      Author: Zoey
 */

#ifndef INC_LAB1MATH_H_
#define INC_LAB1MATH_H_

void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);
extern void asmMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);
extern void squareRootM4(float x, float *output);
void squareRootNewton (float x, float *output);
void cosRoot(float w, float y, float *x);
extern void cosAsm (float w, float y, float *x);

#endif /* INC_LAB1MATH_H_ */
