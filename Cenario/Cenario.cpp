#include "Cenario.h"

Cenario::Cenario(){
	for(int i = 0; i < numretas; i++){
		this->retas[i][0] = -1;
	} 
	this->numReta = -1;
}

void Cenario::novaReta(int x1, int y1, int x2, int y2){
	this->numReta++;
	this->retas[numReta][0] = x1;
	this->retas[numReta][1] = y1;
	this->retas[numReta][2] = x2;
	this->retas[numReta][3] = y2;
}

void Cenario::pontoReta(int posX, int posY){
	int X1,Y1,X2,Y2;
	float menorDist = 1000;
//	Ponto p;
	for (int i = 0; i <= this->numReta; i++){
		if(this->retas[i][0] == -1)
			break;
		X1 = this->retas[i][0]; // x1
		Y1 = this->retas[i][1]; // y1

		X2 = this->retas[i][2]; // x2
		Y2 = this->retas[i][3]; // y2

		float bOrt, mPX, mPY, mDist;

		float m = (float)(Y2-Y1)/(X2-X1);
		float b = Y1 - m*X1;
		float mOrt = -1/m;

		bOrt = -mOrt*posX + posY;
		mPX = (bOrt-b)/(m-mOrt);
		mPY = m*mPX + b;
		
		if(mPX > X1 && mPX > X2)
			mPX = (float) ((X1 > X2) ? X1 : X2);
		else if(mPX < X1 && mPX < X2)
			mPX = (float) ((X1 < X2) ? X1 : X2);

		if(mPY > Y1 && mPY > Y2)
			mPY = (float) ((Y1 > Y2) ? Y1 : Y2);
		else if(mPY < Y1 && mPY < Y2)
			mPY = (float) ((Y1 < Y2) ? Y1 : Y2);
	
		mDist = sqrt(pow((mPX-posX), 2) + pow((mPY-posY), 2));
			
		if(mDist < menorDist){
			menorDist = mDist;
			this->pX = mPX;
			this->pY = mPY;
			this->dist = mDist;
			this->reta = i;
		}
	}
	//return p;
}

//Retorna o ponto X1 da reta mais próxima.
int Cenario::getXreta(){
	return this->retas[this->reta][0];
}

//Retorna o ponto Y1 da reta mais próxima.
int Cenario::getYreta(){
	return this->retas[this->reta][1];
}

float Cenario::distanciaPontos(int x1, int y1, int x2, int y2){
	return sqrt(pow(x1-x2,2)+pow(y1-y2, 2));
}

