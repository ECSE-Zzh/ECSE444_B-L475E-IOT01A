/*
 * squareRootNewton.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Zoey
 */
#include "main.h"
#include "math.h"
#include <stdio.h>

void squareRootNewton (float x, float *output){
	float guess = x;
	float updatedGuess = 0;
	while(1){
		if(x < 0){
			updatedGuess = infinityf();
			break;
		}
		updatedGuess = guess - ((powf(guess,2))-x)/(2*guess);
		if(fabsf(guess - updatedGuess) < 0.0001){
			break;
		}
		guess = updatedGuess;
	}
	(*output) = updatedGuess;
}
