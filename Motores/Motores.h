/*
 * Motores.h
 *
 *  Created on: 12/12/2013
 *      Author: Apolo Marton
 */

#ifndef MOTORES_H_
#define MOTORES_H_

#include <Math.h>
#include <consts.h>
#include "Arduino.h"

#define DISTANCIA_RODAS 9.0f //cm
#define RAIO_RODA 1.5f //cm

//Portas logicas
#define MOTOR_DIR_FORWARD 35
#define MOTOR_ESQ_FORWARD 34
#define MOTOR_DIR_BACKWARD 37
#define MOTOR_ESQ_BACKWARD 36

#define Q1_DIR 25 
#define Q2_DIR 27 
#define Q3_DIR 29
#define Q4_DIR 31
#define RESET_DIR 33
#define ODOM_GND_DIR 23

#define Q1_ESQ 24 
#define Q2_ESQ 26 
#define Q3_ESQ 28
#define Q4_ESQ 30
#define RESET_ESQ 32
#define ODOM_GND_ESQ 22


//Portas pwm
#define MOTOR_ESQ_PWM 12
#define MOTOR_DIR_PWM 13


#define VLINEAR_MAX 10
#define VLINEAR_MIN 5

#define RPM_MAX 60
#define RPM_MIN 30
//const float KpEsq = 0.05, KdEsq = 0.006, KiEsq = 0.0005, KpDir = 0.03, KdDir = 0.001, KiDir = 0.0001;
//KpDir = 1.0, KdDir = 0.03, KiDir = 0.005

//KpEsq = 0.5f, KdEsq = 0.5f, KiEsq = 0.005f,
const float KpEsq = 0.5f, KdEsq = 0.5f, KiEsq = 0.005f,
			KpDir = 1.0f, KdDir = 1.0f, KiDir = 0.01f,
			KpAngulo = 1.4f, KdAngulo = 1.0f, KiAngulo = 0.016f;
const float maximoErroInt = 50;
const float maximoRPMInit = 100;


class Motores {
private:
	float sumErroEsq, sumErroDir, sumErroAngulo;
	
	float hRpmDir[9],hRpmEsq[9];
	float pwmEsq, pwmDir;
	unsigned long tempoAnteriorEsq, tempoAnteriorDir, tempoAnteriorAngulo;
	float erroAnteriorEsq, erroAnteriorDir, erroAnteriorAngulo;
	float filtroMediana(float amostras[9]);
	
public:
	float rpmDesejadoEsq, rpmDesejadoDir, rpmEsq, rpmDir, 
	anguloAtual, anguloAlvo, velLinear, posicaoX, posicaoY;
	
	bool desligado;
	Motores();
	virtual ~Motores();
	void pid();
	void atualizarPWM();
	void desligarSensores();
	void ligarSensores();
	void calculaWdWe(float W);
	float pidAngulo(float anguloAtual, float anguloAlvo);
};

#endif /* MOTORES_H_ */
