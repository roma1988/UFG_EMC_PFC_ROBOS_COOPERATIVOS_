/*
 * Motores.cpp
 *
 *  Created on: 12/12/2013
 *      Author: Apolo Marton
 */

#include "Motores.h"

Motores::Motores() {
	pinMode(MOTOR_ESQ_FORWARD, OUTPUT);
	pinMode(MOTOR_ESQ_BACKWARD, OUTPUT);
	pinMode(MOTOR_DIR_FORWARD, OUTPUT);
	pinMode(MOTOR_DIR_BACKWARD, OUTPUT);
	pinMode(MOTOR_ESQ_PWM, OUTPUT);
	pinMode(MOTOR_DIR_PWM, OUTPUT);
	
	pinMode(Q1_ESQ, INPUT);
	pinMode(Q2_ESQ, INPUT);
	pinMode(Q3_ESQ, INPUT);
	pinMode(Q4_ESQ, INPUT);
	pinMode(RESET_ESQ, OUTPUT);
	pinMode(ODOM_GND_ESQ, OUTPUT);
	
	pinMode(Q1_DIR, INPUT);
	pinMode(Q2_DIR, INPUT);
	pinMode(Q3_DIR, INPUT);
	pinMode(Q4_DIR, INPUT);
	pinMode(RESET_DIR, OUTPUT);
	pinMode(ODOM_GND_DIR, OUTPUT);
	
	ligarSensores();

	tempoAnteriorEsq = millis();
	tempoAnteriorDir = millis();

	sumErroEsq=0;
	sumErroDir=0;
	sumErroAngulo=0;
	
	rpmEsq=0;
	rpmDir=0;
	rpmDesejadoDir=0;
	rpmDesejadoEsq=0;
	
	pwmEsq=PWM_MIN_ESQ;
	pwmDir=PWM_MIN_DIR;
	
	erroAnteriorEsq=0;
	erroAnteriorDir=0;
	erroAnteriorAngulo=0;
	
	hRpmDir[8] = 10000000;
	hRpmEsq[8] = 10000000;
	this->anguloAtual=0;
}

Motores::~Motores() {
	// TODO Auto-generated destructor stub
}

//Seta o novo pwm nos motores
void Motores::atualizarPWM(){

	//Limita a tensao preservando o sinal
  if(rpmDesejadoEsq != 0 && abs(pwmEsq) < PWM_MIN_ESQ)
    pwmEsq = abs(pwmEsq)*PWM_MIN_ESQ/pwmEsq;
  else if(rpmDesejadoEsq == 0)
	pwmEsq = 0;
	
  if(rpmDesejadoDir != 0 && abs(pwmDir) < PWM_MIN_DIR)
    pwmDir = abs(pwmDir)*PWM_MIN_DIR/pwmDir;
  else if(rpmDesejadoDir==0)
	pwmDir = 0;

  //Limita a tensao preservando o sinal
  if(abs(pwmEsq) > PWM_MAX_ESQ)
    pwmEsq = abs(pwmEsq)*PWM_MAX_ESQ/pwmEsq;
  if(abs(pwmDir) > PWM_MAX_DIR)
    pwmDir = abs(pwmDir)*PWM_MAX_DIR/pwmDir;

  if(pwmEsq >=0){
    digitalWrite(MOTOR_ESQ_BACKWARD, LOW);
    digitalWrite(MOTOR_ESQ_FORWARD, HIGH);
    analogWrite(MOTOR_ESQ_PWM, abs(((int)pwmEsq)));
  }

  if(pwmDir>=0){
    digitalWrite(MOTOR_DIR_BACKWARD, LOW);
    digitalWrite(MOTOR_DIR_FORWARD, HIGH);
    analogWrite(MOTOR_DIR_PWM, abs(((int)pwmDir)));
  }
  if(pwmEsq <0){
    digitalWrite(MOTOR_ESQ_FORWARD, LOW);
    digitalWrite(MOTOR_ESQ_BACKWARD, HIGH);
    analogWrite(MOTOR_ESQ_PWM, abs(((int)pwmEsq)));
  }
  if(pwmDir<0){
    digitalWrite(MOTOR_DIR_FORWARD, LOW);
    digitalWrite(MOTOR_DIR_BACKWARD, HIGH);
    analogWrite(MOTOR_DIR_PWM, abs(((int)pwmDir)));
  }
}

/**
  * Executa o controlador PID para a velocidade
  */
void Motores::pid(){

	if(!desligado){
		uint8_t retorno = 0;
		int odomEsq = digitalRead(Q1_ESQ) + digitalRead(Q2_ESQ)*2 + digitalRead(Q3_ESQ)*4 + digitalRead(Q4_ESQ)*8;
				 
		if(odomEsq>0){
			
			digitalWrite(RESET_ESQ, HIGH);
			digitalWrite(RESET_ESQ, LOW);
			unsigned long tempoAtual = millis();
			int deltaTempo = tempoAtual - tempoAnteriorEsq;
			tempoAnteriorEsq = tempoAtual;

			
			rpmEsq = (60000*odomEsq)/(8*deltaTempo);
			odomEsq=0;
			
			//Verifica se nao ocorreu erro de medicao
			if((rpmEsq<abs(hRpmEsq[0])+70 && rpmEsq>0) || hRpmEsq[8]>=1000000){
				if(pwmEsq<0)
					rpmEsq = -rpmEsq;
					
				//Flag para saber caso for a primeira medida
				if(hRpmEsq[0]>=1000000){
				  rpmEsq = 0;
				  hRpmEsq[8] = rpmEsq;
				}else{
				  hRpmEsq[8] = hRpmEsq[7];
				  hRpmEsq[7] = hRpmEsq[6];
				  hRpmEsq[6] = hRpmEsq[5];
				  hRpmEsq[5] = hRpmEsq[4];
				  hRpmEsq[4] = hRpmEsq[3];
				  hRpmEsq[3] = hRpmEsq[2];
				  hRpmEsq[2] = hRpmEsq[1];
				  hRpmEsq[1] = hRpmEsq[0];
				  hRpmEsq[0] = rpmEsq;
				}
					
				float erro = rpmDesejadoEsq - rpmEsq;
				
				sumErroEsq = sumErroEsq + erro;

				if(abs(sumErroEsq)>maximoErroInt)
					sumErroEsq=maximoErroInt*sumErroEsq/abs(sumErroEsq);

				float w = KpEsq*erro + KdEsq*(erro-erroAnteriorEsq) + KiEsq*sumErroEsq;
				 
				// Serial.print("\t RpmDesejado Esq: ");
				// Serial.print(rpmDesejadoEsq);
				
				//Amostragem de dados
				//rpmEsq = //filtroMediana(hRpmEsq);
				//(hRpmEsq[0]+hRpmEsq[1]+hRpmEsq[2]+hRpmEsq[3]+hRpmEsq[4]+hRpmEsq[5]+hRpmEsq[6]+hRpmEsq[7]+hRpmEsq[8])/9;
				//Serial.print(rpmEsq);
				//Serial.print(pwmEsq);
				//Serial.println();
				
				pwmEsq = pwmEsq + w;

				//Aplica o pwm minimo se for menor que o minimo
				if(pwmEsq < 0 && rpmDesejadoEsq > 0){
					pwmEsq=PWM_MIN_ESQ;
				}
				if(pwmEsq > 0 && rpmDesejadoEsq < 0){
					pwmEsq=-PWM_MIN_ESQ;
				}
				
				//Fortalece a re
				if(pwmEsq<0){
					pwmEsq-=15;
				}
				
				float W = this->pidAngulo(this->anguloAtual,this->anguloAlvo);
				this->rpmDesejadoEsq = (60.0f/(2.0f*PI))*(2.0f*this->velLinear + DISTANCIA_RODAS*W)/(2.0f*RAIO_RODA);
				// calculaWdWe(W);
				
			 	this->posicaoX += COEF_REAL*(2.0f*PI/60.0f)*(rpmEsq)*cos(this->anguloAtual)*RAIO_RODA*deltaTempo/2000;
				this->posicaoY += COEF_REAL*(2.0f*PI/60.0f)*(rpmEsq)*sin(this->anguloAtual)*RAIO_RODA*deltaTempo/2000; 
				
				erroAnteriorEsq = erro;
			 
			}
			
		}
		int odomDir = digitalRead(Q1_DIR) + digitalRead(Q2_DIR)*2+digitalRead(Q3_DIR)*4 + digitalRead(Q4_DIR)*8;
		
		if(odomDir>0){
			digitalWrite(RESET_DIR, HIGH);
			digitalWrite(RESET_DIR, LOW);

			unsigned long tempoAtual = millis();
			int deltaTempo = tempoAtual - tempoAnteriorDir;
			tempoAnteriorDir = tempoAtual;

			//Serial.print("OdomDir: ");
			//Serial.println(odomDir);
			//Serial.print(" TempoDir: ");
			//Serial.print(deltaTempo);
			
			rpmDir = 60000*odomDir/(8*deltaTempo);
			odomDir=0;
			//Serial.println(rpmDir);
			//Verifica se nao ocorreu erro de medicao
			if((rpmDir<abs(hRpmDir[0])+70 && rpmDir>0)|| hRpmDir[8]>=1000000 ){
				
				if(pwmDir<0)
					rpmDir = -rpmDir;

				//Flag para saber caso for a primeira medida
				if(hRpmDir[8]>=1000000){  
				  rpmDir = 0;
					
				  hRpmDir[8] = rpmDir;
				}else{
				  hRpmDir[8] = hRpmDir[7];
				  hRpmDir[7] = hRpmDir[6];
				  hRpmDir[6] = hRpmDir[5];
				  hRpmDir[5] = hRpmDir[4];
				  hRpmDir[4] = hRpmDir[3];
				  hRpmDir[3] = hRpmDir[2];
				  hRpmDir[2] = hRpmDir[1];
				  hRpmDir[1] = hRpmDir[0];
				  hRpmDir[0] = rpmDir;
				}


				float erro = rpmDesejadoDir - rpmDir;

				sumErroDir = sumErroDir + erro;

				if(abs(sumErroDir)>maximoErroInt)
					sumErroDir=maximoErroInt*sumErroDir/abs(sumErroDir);

				float w = KpDir*erro + KdDir*(erro-erroAnteriorDir) + KiDir*sumErroDir;
				
				//Amostragem de dados
				//rpmDir = //filtroMediana(hRpmDir);
				//(hRpmDir[0]+hRpmDir[1]+hRpmDir[2]+hRpmDir[3]+hRpmDir[4]+hRpmDir[5]+hRpmDir[6]+hRpmDir[7]+hRpmDir[8])/9;
				//Serial.print(rpmDir);
				//Serial.print(pwmDir);
				//Serial.println();
				
				pwmDir = pwmDir + w;

				//Aplica o pwm minimo se for menor que o minimo
				if(pwmDir < 0 && rpmDesejadoDir > 0){
					pwmDir=PWM_MIN_DIR;
				}
				if(pwmDir > 0 && rpmDesejadoDir < 0){
					pwmDir=-PWM_MIN_DIR;
				}
				
				//Fortalece a re
				if(pwmDir<0){
					pwmDir-=15;
				}

				float W = pidAngulo(this->anguloAtual,this->anguloAlvo);
				 this->rpmDesejadoDir = (60.0f/(2.0f*PI))*(2.0f*this->velLinear - DISTANCIA_RODAS*W)/(2.0f*RAIO_RODA);
				// calculaWdWe(W);
				
				this->posicaoX += COEF_REAL*(2.0f*PI/60.0f)*(rpmDir)*cos(this->anguloAtual)*RAIO_RODA*deltaTempo/2000;
				this->posicaoY += COEF_REAL*(2.0f*PI/60.0f)*(rpmDir)*sin(this->anguloAtual)*RAIO_RODA*deltaTempo/2000; 
				
				erroAnteriorDir = erro;
			}
		}
	}
}

/**
*	Liga os sensores do tacometro
*/
void Motores::ligarSensores(){
	// digitalWrite(ODOM_GND_DIR, LOW);
	// digitalWrite(ODOM_GND_ESQ, LOW);
	sumErroAngulo=0;
	erroAnteriorAngulo=0;
	
	rpmDir = 0;
	rpmEsq = 0;
				
	//Flag para sinalizar que estava parado 
	hRpmDir[8] = 10000000;
	hRpmEsq[8] = 10000000;
	
	hRpmDir[7] = rpmDir;
	hRpmDir[6] = rpmDir;
	hRpmDir[5] = rpmDir;
	hRpmDir[4] = rpmDir;
	hRpmDir[3] = rpmDir;
	hRpmDir[2] = rpmDir;
	hRpmDir[1] = rpmDir;
	hRpmDir[0] = rpmDir;
	
	hRpmEsq[7] = rpmEsq;
	hRpmEsq[6] = rpmEsq;
	hRpmEsq[5] = rpmEsq;
	hRpmEsq[4] = rpmEsq;
	hRpmEsq[3] = rpmEsq;
	hRpmEsq[2] = rpmEsq;
	hRpmEsq[1] = rpmEsq;
	hRpmEsq[0] = rpmEsq;
	
	digitalWrite(RESET_ESQ, HIGH);
	digitalWrite(RESET_ESQ, LOW);
	
	digitalWrite(RESET_DIR, HIGH);
	digitalWrite(RESET_DIR, LOW);
	
	pwmEsq=50;
	pwmDir=50;
	
	desligado = false;
}

/**
*	Desliga os sensores do tacometro
*/
void Motores::desligarSensores(){
	// digitalWrite(ODOM_GND_DIR, HIGH);
	// digitalWrite(ODOM_GND_ESQ, HIGH);
	
	desligado = true;
}

/**
*	Controla a orientacao do robo
*/
float Motores::pidAngulo(float anguloAtual, float anguloAlvo){
	
	/*unsigned long tempoAtual = millis();
    int deltaTempo = tempoAtual - tempoAnteriorAngulo;
	tempoAnteriorAngulo = tempoAtual;
	*/
	float erro = atan2(sin(anguloAlvo-anguloAtual), cos(anguloAlvo-anguloAtual));
	
	
	sumErroAngulo = sumErroAngulo + erro;
		
	if(sumErroAngulo>50)
		sumErroAngulo=50;

	if(sumErroAngulo<-50)
		sumErroAngulo=-50;		

	float W = KpAngulo * erro + KiAngulo * sumErroAngulo + KdAngulo * (erro - erroAnteriorAngulo);
	
	erroAnteriorAngulo = erro;
	
	return W;
}

void Motores::calculaWdWe(float W){ 
	float V_lim = max((min(abs(this->velLinear), VLINEAR_MAX)), VLINEAR_MIN); 
	float W_lim = max((min(abs(W), (VLINEAR_MAX - VLINEAR_MIN)/DISTANCIA_RODAS)), 0); 

	float rpmToSI = (2*PI/60);
	rpmDesejadoDir = (2*V_lim + W_lim*DISTANCIA_RODAS)/(2*RAIO_RODA*rpmToSI); //rad/s 
	rpmDesejadoEsq = (2*V_lim - W_lim*DISTANCIA_RODAS)/(2*RAIO_RODA*rpmToSI); //rad/s 

	float W_Maior = max(rpmDesejadoDir, rpmDesejadoEsq); 
	double W_Menor = min(rpmDesejadoDir, rpmDesejadoEsq); 

	if(W_Maior > RPM_MAX){ 
		rpmDesejadoDir -= (W_Maior - RPM_MAX); 
		rpmDesejadoEsq -= (W_Maior - RPM_MAX); 
	} 

	if(W_Menor < RPM_MIN){ 
		rpmDesejadoDir += (RPM_MIN - W_Menor); 
		rpmDesejadoEsq += (RPM_MIN - W_Menor); 
	} 

	velLinear = (abs(velLinear)/velLinear)* RAIO_RODA*(rpmDesejadoDir + rpmDesejadoEsq)/2; 
	W = (abs(W)/W) * RAIO_RODA*(rpmDesejadoEsq - rpmDesejadoDir)/DISTANCIA_RODAS; 

	rpmDesejadoEsq = (2*velLinear + W*DISTANCIA_RODAS)/(2*RAIO_RODA); //rad/s 
	rpmDesejadoDir = (2*velLinear - W*DISTANCIA_RODAS)/(2*RAIO_RODA); //rad/s 
}

float Motores::filtroMediana(float amostras[9]){
	float amostrasOrndenadas[9];
	for(int x = 0; x < 9; x++ ){
		amostrasOrndenadas[x] =  amostras[x];
	}
	for(int x = 0; x < 9; x++ )
	{
		for(int y = x + 1; y < 9; y++ ) // sempre 1 elemento à frente
		{
			// se o (x > (x+1)) então o x passa pra frente (ordem crescente)
			if ( amostrasOrndenadas[x] > amostrasOrndenadas[y] )
			{
				float aux = amostrasOrndenadas[x];
				amostrasOrndenadas[x] = amostrasOrndenadas[y];
				amostrasOrndenadas[y] = aux;
			}
		}
	} //fim da ordenação

	//Mediana das amostras realizadas
	return amostrasOrndenadas[4];			
}

