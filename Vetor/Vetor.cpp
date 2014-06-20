#include "Vetor.h"
#include <math.h>
#include <Arduino.h>
#include <Wire.h>

Vetor::Vetor(){
	this->a = 0;
	this->bj = 0;
	this->magnitude = 0;
	this->angulo = 0;
}

Vetor::Vetor(float magnitude, float angulo){
	this->a = cos(angulo)*magnitude;
	this->bj = sin(angulo)*magnitude;
	this->magnitude = magnitude;
	this->angulo = angulo;
}

Vetor::~Vetor(void){
}

void Vetor::somar(float magnitude, float angulo){
	//Soma-se as partes reais.
	this->a += cos(angulo)*magnitude;
	//Soma-se a parte Imaginária.
	this->bj += sin(angulo)*magnitude;
	// |R| = raiz( a² + b²)
	this->magnitude = sqrt(pow(a,2)+pow(bj,2));
	// Fase = atang(b/a)
	this->angulo = atan2(this->bj,this->a);
}
