#pragma once
#include <consts.h>
#include <Math.h>
#include <Arduino.h>
#include <Matriz2Bits.h>

#define numretas 1

class Cenario{

public:

	Cenario();
	void novaReta(int x1, int y1, int x2, int y2); // Adiciona nova reta ao cenário.
	void pontoReta(int posX, int posY); // Calcula os parâmetros da reta mais próxima da posição especificada
	int pX;  // coordenada X da reta mais próxima ao robô.
	int pY;  // coordenada Y da reta mais próxima ao robô.
	int dist; // Distância da reta mais próxima ao robô.
	int reta; // Indíce da reta mais próxima ao robô.
	int getYreta(); //Retorna o ponto Y1 da reta mais próxima.
	int getXreta(); //Retorna o ponto X1 da reta mais próxima.
	static float distanciaPontos(int x1, int y1, int x2, int y2); //Retorna a distância entre os pontos
private:
	int retas[numretas][4];
	int numReta;

};