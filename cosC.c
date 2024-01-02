/*
 * transcendentalFunc.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Zoey
 */
#include "main.h"
#include "arm_math.h"
//#include <math.h>

void cosRoot (float w, float y, float *x){
//	if(w == 0){
//		无解时？？？break 运行次数
//	}
	float guess = 0.5;
	float updatedGuess = 0;
	int count = 0;
	while(1){
		if(count >= 100){
			updatedGuess = infinityf();
			break;
		}
//		updatedGuess = guess-(cos(w*guess+y)-pow(guess,2))/(-w*sin(w*guess+y)-2*guess);
		updatedGuess = guess-(arm_cos_f32(w*guess+y)-powf(guess,2))/(-w*arm_sin_f32(w*guess+y)-2*guess);
		if(fabsf(guess - updatedGuess) < 0.0001){
			break;
		}
		guess = updatedGuess;
		count++;
	}
	(*x) = updatedGuess;
}
