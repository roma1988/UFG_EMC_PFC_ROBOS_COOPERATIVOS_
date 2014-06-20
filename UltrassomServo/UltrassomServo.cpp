/*
 * UltrassomServo.cpp
 *
 *  Created on: 30/11/2013
 *      Author: Apolo Marton
 */

#include "UltrassomServo.h"

UltrassomServo::UltrassomServo() {
	myservo = new Servo();
	myservo->attach(PIN_SERVO); //setar o servo motor no pino 9
	myservo->write(degreesToServoPosition(90));//girar o motor para a posicao inicial pos
	
	pinMode(PIN_VCC,OUTPUT);//Vcc do sensor
	pinMode(PIN_GND,OUTPUT);//Gnd do sensor
	
	// set pins
	pinMode(PIN_TRIGGER, OUTPUT);
	pinMode(PIN_ECHO, INPUT);
	
	digitalWrite(PIN_VCC,HIGH);//Liga o sonar
	digitalWrite(PIN_GND,LOW);
}
UltrassomServo::~UltrassomServo() {

}
float UltrassomServo::medir(int numeroMedidas){
	float diferencaHipotenusa=0, distanciaMedida=0, x;
	float hipotenusaMedida=0;
	int medidasFeitas = 0;
	
	long amostras[numeroMedidas];
	for(int medida=0; medida<numeroMedidas; medida++){
		 // send a 10us+ pulse
		digitalWrite(PIN_TRIGGER, LOW);
		delayMicroseconds(2);
		digitalWrite(PIN_TRIGGER, HIGH);
		delayMicroseconds(10);
		digitalWrite(PIN_TRIGGER, LOW);
		//delayMicroseconds(20);
		//  read duration of echo 
		long duration = pulseIn(PIN_ECHO, HIGH, 5000);
		//Esse calculo é baseado em s = v . t, lembrando que o tempo vem dobrado  
		//porque é o tempo de ida e volta do ultrassom  
		long distancia = duration /29 / 2 ;  
		
		if(distancia==0)
			distancia=HIPOTENUSA;

		amostras[medida]=distancia;
	}
	
	//Ordena o vetor de amostras
	for(int x = 0; x < numeroMedidas; x++ )
	{
		for(int y = x + 1; y < numeroMedidas; y++ ) // sempre 1 elemento à frente
		{
			// se o (x > (x+1)) então o x passa pra frente (ordem crescente)
			if ( amostras[x] > amostras[y] )
			{
			long aux = amostras[x];
			amostras[x] = amostras[y];
			amostras[y] = aux;
			}
		}
	} //fim da ordenação

	//Mediana das amostras realizadas
	hipotenusaMedida = (float)(amostras[((int)(numeroMedidas/2))]);
	
	//Serial.println(hipotenusaMedida);
	
	if(hipotenusaMedida > 0) {

		if(hipotenusaMedida>HIPOTENUSA){
		
			diferencaHipotenusa = hipotenusaMedida-HIPOTENUSA;//distancia real medida pelo ultrassom
			x = DISTANCIA*diferencaHipotenusa/HIPOTENUSA;//distancia do ponto fixo do chao(onde o ponto que o ultrassom "ve" diretamente) ao obstaculo.
			distanciaMedida = DISTANCIA + x;//distancia da projeção do ultrassom no chao ao obstaculo
		} 
		if(hipotenusaMedida<HIPOTENUSA){
			diferencaHipotenusa = HIPOTENUSA-hipotenusaMedida;//distancia real medida pelo ultrassom
			//hl = h*pl/p;//altura calculada do obstaculo. Se hl for 0 entao é plano ou sem obstaculo.
			x = DISTANCIA*diferencaHipotenusa/HIPOTENUSA;//distancia do ponto fixo do chao(onde o ponto que o ultrassom "ve" diretamente) ao obstaculo.
			distanciaMedida = DISTANCIA - x;//distancia da projeção do ultrassom no chao ao obstaculo
		}
	}
	return distanciaMedida;
}

void UltrassomServo::atualizarMapa(Matriz2Bits* mapa, float xAtual, float yAtual, int anguloAtual, float tamanhoCelula){
	if(!myservo->attached())
		myservo->attach(PIN_SERVO);
		
	for(int posicao = SERVO_POS_MIN; posicao < SERVO_POS_MAX; posicao += 4){
		atualizarPosicao(posicao, mapa, xAtual, yAtual, anguloAtual, tamanhoCelula);
		delay(5);
	}
	for(int posicao = SERVO_POS_MAX; posicao >= SERVO_POS_MIN; posicao -= 4){
		atualizarPosicao(posicao, mapa, xAtual, yAtual, anguloAtual, tamanhoCelula);
		delay(5);
	}
	//Vira o sensor para frente
	myservo->write(degreesToServoPosition(90));
	delay(15);
}

int UltrassomServo::degreesToServoPosition(int angle){
	return 90*angle/90;
}

int UltrassomServo::servoPositionToDegrees(int position){
	return 90*position/90;
}

void UltrassomServo::atualizarPosicao(int posicao, Matriz2Bits* mapa, float xAtual, float yAtual, int anguloAtual, float tamanhoCelula){
	myservo->write(posicao);
	float distanciaMedida = medir(3);

	if(distanciaMedida>0 && distanciaMedida <= DISTANCIA){
		//o obstáculo fica distante de distanciaMedida
		int coordX=(int)((distanciaMedida*cos((90-servoPositionToDegrees(posicao))*PI/180 + anguloAtual)+xAtual)/tamanhoCelula);//coordenada global x  do obstaculo
		int coordY=(int)((distanciaMedida*sin((90-servoPositionToDegrees(posicao))*PI/180 + anguloAtual)+yAtual)/tamanhoCelula);//coordenada global y  do obstaculo
		//Determina a probabilidade de existir um obstaculo
		float probabilidadeOcupado = (1-pow((distanciaMedida/DISTANCIA),2))*4;//Risco = (1-(d'/d)^2)*4
		
	/*	Serial.print("Aclive: ");
		Serial.print(probabilidadeOcupado);
		Serial.print("\t");
		Serial.println((int)(probabilidadeOcupado));
	*/
		//(linha, coluna)
		mapa->setValor(coordY,coordX,(uint8_t)(probabilidadeOcupado));
	}else if(distanciaMedida>0 && distanciaMedida > DISTANCIA){
	
		//O buraco fica distante de DISTANCIA
		int coordX=(int)((DISTANCIA*cos((90-servoPositionToDegrees(posicao))*PI/180 +anguloAtual)+xAtual)/tamanhoCelula);//coordenada global x  da lata calculada
		int coordY=(int)((DISTANCIA*sin((90-servoPositionToDegrees(posicao))*PI/180 +anguloAtual)+yAtual)/tamanhoCelula);//coordenada global y  da lata calculada
		//Determina a probabilidade de existir um obstaculo
		float probabilidadeOcupado = 0.67*(pow(((distanciaMedida-DISTANCIA)/DISTANCIA),2))*4; //Risco = taxaAprendizagem*(((d'-d)/d)^2)*4
		probabilidadeOcupado+= 0.33*mapa->getValor(coordY,coordX);
		/*
		Serial.print("Declive: ");
		Serial.print(probabilidadeOcupado);
		Serial.print("\t");
		Serial.println((int)(probabilidadeOcupado));
		*/
		//(linha, coluna)
		mapa->setValor(coordY,coordX,(uint8_t)(probabilidadeOcupado));
	}
}



