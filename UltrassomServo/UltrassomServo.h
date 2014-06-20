/*
 * UltrassomServo.h
 *
 *  Created on: 30/11/2013
 *      Author: Apolo Marton
 */


#ifndef ULTRASOMSERVO_H_
#define ULTRASOMSERVO_H_

#include <Math.h>
#include "Arduino.h"
#include <Matriz2Bits.h>
#include <Servo.h>
#include <windows.h>


#define PIN_VCC 41//Vcc setado para pino 41
#define PIN_TRIGGER 43//Trigger setado para pino 43
#define PIN_ECHO  45//Echo setado para pino 45
#define PIN_GND 47
#define PIN_SERVO 9


const float ALTURA = 24.0; //constante: altura do chao ao ultrassom
const float DISTANCIA = 40.0;//constante: distancia da projecao do ultrassom no chao ao ponto fixo onde o ultrassom "ve"; Este valor muda se mudar a angulacao do ultrassom e altura.
const float HIPOTENUSA = 50.0;//constante: distancia medida pelo ultrassom, distancia do ultrassom ate no ponto fixo do chao; Este valor muda se mudar a angulacao do ultrassom e altura.
const int SERVO_POS_MAX = 160;
const int SERVO_POS_MIN = 20;

class UltrassomServo {
private:
	Servo* myservo;//declara servo motor
public:
	UltrassomServo();
	float medir(int numeroMedidas);
	void atualizarMapa(Matriz2Bits* mapa, float xAtual, float yAtual, int actualAngle, float tamanhoCelula);
	int degreesToServoPosition(int angle);
	int servoPositionToDegrees(int position);
	void atualizarPosicao(int posicao, Matriz2Bits* mapa, float xAtual, float yAtual, int anguloAtual, float tamanhoCelula);
	virtual ~UltrassomServo();
};

#endif /* ULTRASOMSERVO_H_ */
