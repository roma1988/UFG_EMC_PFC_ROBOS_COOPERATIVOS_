#include <consts.h>
#include <Wire.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <Matriz2Bits.h>
#include <Vetor.h>
#include <Servo.h>
#include <UltrassomServo.h>
#include <Motores.h>
#include <XBeeCommunication.h>
#include <HMC5883L.h>
#include <Bussola.h>

//Constantes metricas
#define TAMANHO_CELULA 10 //cm

#define NUM_ROBOS 4

#define VELOCIDADE_LINEAR 5.0f  //cm/s

#define TEMPO_DE_ENVIO 20 //segundos
#define BATERIA_NIVEL_BAIXO 5100 //tensao lida no Vin

uint16_t NUM_LINHAS = 100;
uint16_t NUM_COLUNAS = 100;
int limiteLinha = TAMANHO_CELULA*NUM_LINHAS-10*TAMANHO_CELULA, 
limiteColuna = TAMANHO_CELULA*NUM_COLUNAS-10*TAMANHO_CELULA;

int linhaAtual=Y_INICIAL, colunaAtual=X_INICIAL, 
linhaAnterior=7, colunaAnterior=20, 
mapaLinha=0, mapaColuna=0, numeroMapa, offsetLinha=0, offsetColuna=0;

float xAtual=200, yAtual=100, //cm
xDestino = 500, yDestino = 500, //cm
anguloAtual = 0, anguloAnterior, anguloInicial=PI/2; //rad

bool bateriaBaixa = false;

Matriz2Bits* mapa;
UltrassomServo* ultrassom;
Motores* motores;
Bussola* compass;
XBeeCommunication *xbee;
Robo robos[NUM_ROBOS];
Vetor vetorAtual;


//Campo
#define TAM_MATRIZ 15
float erroMinimo = 10E-5;
double campo[TAM_MATRIZ][TAM_MATRIZ];

//tempo do ultimo envio da posicao
long ultimoEnvio=0;

/**
 * Setup
 */
void setup()
{
  delay(10000);
  Serial.begin(9600);
  inicializarRobo();
  numeroMapa = (uint8_t)(mapaLinha*10+mapaColuna);
  mapa = new Matriz2Bits(NUM_LINHAS, NUM_COLUNAS, numeroMapa);
  xbee = new XBeeCommunication(&Serial3, 9600);
  ultrassom = new UltrassomServo();
  compass = new Bussola();
 
  calcularCampo();
  //Força verificar ambiente
  linhaAnterior = linhaAtual-3;
  
  xAtual = colunaAtual*TAMANHO_CELULA + limiteColuna*mapaColuna;
  yAtual = linhaAtual*TAMANHO_CELULA + limiteLinha*mapaLinha;
  
  motores = new Motores();
  motores->posicaoX = xAtual;
  motores->posicaoY = yAtual;
  
  ligarMotores();
  anguloInicial = getAngulo();
  anguloAtual = anguloInicial;
  anguloAnterior=anguloAtual;
  
  ultimoEnvio = millis();
  xbee->sendPosition(colunaAtual, linhaAtual, numeroMapa);
  xbee->receivePositions(robos);
}

/**
 * Main program loop
 */
void loop()
{
  if(!bateriaBaixa){
     if(readVcc()<BATERIA_NIVEL_BAIXO){
      bateriaBaixa = true;
      return;
     }
      
    atualizarOrientacao();
    atualizarAlvo();
    motores->pid();
    
    if(motores->velLinear < VELOCIDADE_LINEAR){
      //Serial.println("Vel Linear");
      desligarMotores();
//      //Nao deixa se mover enquanto estiver no alvo desejado
      while(motores->velLinear < VELOCIDADE_LINEAR){
        xbee->sendPosition(colunaAtual, linhaAtual, numeroMapa);
        xbee->receivePositions(robos);
        delay(5000);
        atualizarAlvo();
        atualizarOrientacao();
      }
      //Força atualizacao do mapa
      linhaAnterior = linhaAtual-3;
      ligarMotores();
    }else{
      motores->atualizarPWM();
    }
      
    atualizarPosicao();
    
    verificarMapa();        
  }else{
      desligarMotores();
      xbee->sendPosition(colunaAtual, linhaAtual, numeroMapa);
      xbee->receivePositions(robos);
      //mapa->imprimirMatriz();      
      delay(10000);
  }
}

void inicializarRobo(){
    for(int i=0; i<NUM_ROBOS; i++){
      robos[i].x=0;
      robos[i].y=0;
      robos[i].mapNumber=0;
    }
}

void verificarMapa(){
  if((abs(linhaAtual-linhaAnterior)+abs(colunaAtual-colunaAnterior))>=2 || abs(anguloAtual-anguloAnterior)>PI/3){
    desligarMotores();
    delay(10);
    ultrassom->atualizarMapa(mapa, xAtual-mapaColuna*limiteColuna, yAtual-mapaLinha*limiteLinha, anguloAtual, TAMANHO_CELULA);
    xbee->sendPosition(colunaAtual, linhaAtual, numeroMapa);
    xbee->receivePositions(robos); 
    calcularCampo();
    linhaAnterior = linhaAtual;
    colunaAnterior = colunaAtual;
    anguloAnterior = anguloAtual;
    ligarMotores();
  }else{
      int posicao = ultrassom->degreesToServoPosition(90);
      ultrassom->atualizarPosicao(posicao, mapa, xAtual, yAtual, anguloAtual, TAMANHO_CELULA);    
  }
}
void atualizarAlvo(){
  
  int col = colunaAtual-offsetColuna-1;
  int lin = linhaAtual-offsetLinha-1;
  
  double mTop = (col-1) >=0 ? campo[col-1][lin] : 1;
  double mBot = (col+1) < TAM_MATRIZ ? campo[col+1][lin] : 1;
  double mEsq = (lin-1) >=0 ? campo[col][lin-1] : 1;
  double mDir = (lin+1) < TAM_MATRIZ ? campo[col][lin+1] : 1;

  float dMax = 50, magnitude, angulo;
  float distancia = sqrt(pow(xDestino - xAtual,2) + pow(yDestino - yAtual,2));
  if(distancia < dMax)
    magnitude = distancia/dMax;
  else     
    magnitude = 1;  
  
  angulo = atan2((mEsq-mDir),(mTop-mBot));
  vetorAtual.a=0;
  vetorAtual.bj=0;
  vetorAtual.somar(magnitude, angulo);
  
  float lMax = 100;
  for(int i = 0; i < NUM_ROBOS; i++){
    if((i+1)!=ROBOT_NUMBER){
      int nMapa = robos[i].mapNumber;
      int mapLin = ((int)(nMapa%10));
      int mapCol = ((int)(nMapa/10));
      float roboX = robos[i].x*TAMANHO_CELULA + mapCol*limiteColuna;
      float roboY = robos[i].y*TAMANHO_CELULA + mapLin*limiteLinha;
      
      //cálculo dos componentes do Vetor;
      angulo = atan2(yAtual-(roboY),xAtual-roboX);
      float dist = sqrt(pow(xAtual-roboX,2)+pow(yAtual-roboY,2));
      
      if(dist > lMax)
        magnitude = 0;
      else
        magnitude = 1 - dist/lMax;
        
      vetorAtual.somar(magnitude, angulo);
    }
  }

  motores->anguloAlvo  = vetorAtual.angulo;
  
if(vetorAtual.magnitude > 0.4){
    motores->velLinear = VELOCIDADE_LINEAR;
  }else{
    motores->velLinear=0;
  }
}

void atualizarOrientacao(){
  anguloAtual = getAngulo();
  motores->anguloAtual = anguloAtual;
}
void atualizarPosicao(){
  xAtual = motores->posicaoX;
  yAtual = motores->posicaoY;
 
  colunaAtual = (xAtual)/TAMANHO_CELULA;
  linhaAtual = (yAtual)/TAMANHO_CELULA;
  
  Serial.print(xAtual);
  Serial.print("\t");
  Serial.print(yAtual);
  Serial.print("\t");
  Serial.println(anguloAtual*180/PI);
}

void desligarMotores(){
   motores->rpmDesejadoDir = 0;
   motores->rpmDesejadoEsq = 0;
   motores->atualizarPWM();
   motores->desligarSensores();
}
void ligarMotores(){
  motores->ligarSensores();
  motores->rpmDesejadoDir = 40;
  motores->rpmDesejadoEsq = 40;
}

float getAngulo(){
  
  float heading = compass->getHeading();
  heading-=anguloInicial;
  
  return atan2(sin(heading), cos(heading));
}

void inicializarCampo(int x, int y){
    // Inicializa a matriz.
    offsetColuna = (colunaAtual - TAM_MATRIZ/2);
    offsetLinha = (linhaAtual - TAM_MATRIZ/2);
    for (int i = 1; i < TAM_MATRIZ-1; i++){
      int idxI = offsetColuna +i;
      for (int j = 1; j < TAM_MATRIZ-1; j++){
        int idxJ = offsetLinha +j;
        uint8_t valor = mapa->getValor(idxJ,idxI);
        
        if(valor>1)
          campo[i][j] = 1;
        else
          campo[i][j] = 0.5;
          
      }
    }
    // Preenche as bordas com obstáculos
    for (int k = 0; k < TAM_MATRIZ; k++){
      campo[0][k] = 1;
      campo[TAM_MATRIZ-1][k] = 1;
      campo[k][0] = 1;
      campo[k][TAM_MATRIZ-1] = 1;
    }
    
    int xAlvo = x-offsetColuna;
    if(xAlvo<1)
      xAlvo=1;
    if(xAlvo>TAM_MATRIZ-2)
      xAlvo=TAM_MATRIZ-2;
     
    int yAlvo = y-offsetLinha;
    if(yAlvo<1)
      yAlvo=1;
    if(yAlvo>TAM_MATRIZ-2)
      yAlvo=TAM_MATRIZ-2;
      
    //Meta
    campo[xAlvo][yAlvo] = 0;
}

void calcularCampo() {
  inicializarCampo((int)(xDestino/TAMANHO_CELULA),(int)(yDestino/TAMANHO_CELULA));

  double erro;
  double campoIJ = 0;
  float p = 0.5*(cos(3.14159265/TAM_MATRIZ)*2);
  float w  = 2/(1+sqrt(1-p*p));
    
  for(int t = 0; t < 1500; t++){
    erro = 0;
    for(int i = 1; i < TAM_MATRIZ-1; i++){
      for(int j = 1; j < TAM_MATRIZ-1; j++){
       //Obstáculo ou Meta -> continua. 
        if(campo[i][j] == 0.0 || campo[i][j] == 1.0)
          continue;  
          
        campoIJ = campo[i][j];
        // Método SOR
        campo[i][j] = campoIJ+w*(campo[i][j-1]+campo[i][j+1]+campo[i-1][j]+campo[i+1][j]-4*campoIJ)/4;
        erro += abs(campoIJ - campo[i][j]);
      }
    }
    //Critério de convergência
    if(erro < erroMinimo){
      break;
    }
  }

}

//Função para calcular o nível de carga da bateria
long readVcc()
{
  // Ler referência interna 1.1V
  // Ajusta a referência ao Vcc e a medição de referência interna para 1.1V
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(2); // Aguarde
  ADCSRA |= _BV(ADSC); // Inicia a conversão
  while (bit_is_set(ADCSRA,ADSC)); // Medindo
  uint8_t low  = ADCL; // Vai ler ADCL primeiro - Então trava ADCH
  uint8_t high = ADCH; // Desbloqueia
  long result = (high<<8) | low;
  result = 1125300L / result; // Calcular Vcc em milivolts; 1125300 = 1.1*1023*1000
  return result; // Vcc em milivolts
}

