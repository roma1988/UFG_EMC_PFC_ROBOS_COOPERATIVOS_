#pragma once
class Vetor
{
public:
	Vetor();
	Vetor(float magnitude, float angulo);
	~Vetor(void);
	void somar(float magnitude, float angulo);

	//Atributos de um vetor
	float magnitude;
	float angulo;
	float a;
	float bj;
};

