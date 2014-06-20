#ifndef BUSSOLA_H_
#define BUSSOLA_H_

#include <consts.h>
#include <HMC5883L.h>
#include <math.h>
#include <Wire.h>

#define TAMANHO_FILA 5


class Bussola{

public:
	Bussola(void);
	~Bussola(void);
	
	float getHeading();

private :
	int pointer, pointerSize, filaCheia;//ponteiro da fila circular, fila size, fila está totalmente preenchida 
	float filacirc[TAMANHO_FILA];     //fila circular
	HMC5883L compass;

	void add(float v);
	void calcHeading();
};

#endif