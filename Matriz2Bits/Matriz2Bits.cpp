/*
 * MatrizBits.cpp
 *
 *  Created on: 06/11/2013
 *      Author: Apolo Marton
 */

#include "Matriz2Bits.h"

Matriz2Bits::Matriz2Bits(uint16_t linhas, uint16_t colunas, uint8_t numeroMapa) {
	numeroLinhas = linhas;
	numeroColunas = colunas;

	//Numero de bits por linha
	int bitsLinha = (2*numeroColunas);

	//Numero de bytes cheios
	bytesPorLinha = (int)(bitsLinha/8);

	//Um byte meio cheio por linha
	if(bitsLinha%8 > 0){
		bytesPorLinha++;
	}

	//Numero de bytes
	numeroBytes = bytesPorLinha*numeroLinhas;

	//Inicializa a matriz
	matriz = new uint8_t[numeroBytes];

	for(int i=0; i<numeroBytes; i++){
		matriz[i] =0;
	}

	pinMode(CS_PIN, OUTPUT);
	sdFailed=false;
	if (!SD.begin(CS_PIN)) {
		Serial.println("SD initialization failed!");
		sdFailed = true;
	}
  
	root = SD.open("/");
	
	mapArchiveName = "";
	mapArchiveName+=numeroMapa;
	mapArchiveName+=".txt";
	mapArchiveName.toCharArray(nomeArquivo, 7);
	numeroMapaAtual = numeroMapa;
	Serial.println("SD initialization done.");
}

Matriz2Bits::~Matriz2Bits() {
	//Deleta a matriz
	for(int i =0; i<numeroBytes; i++)
		delete(&matriz[i]);

	delete(matriz);
}

uint8_t Matriz2Bits::getValor(uint16_t linha, uint16_t coluna){
	if(linha>=numeroLinhas || coluna>=numeroColunas || linha<0 || coluna<0)
		return 3;

	int bitColuna = coluna*2;

	int byteColuna = bitColuna/8;

	int offsetColuna = bitColuna%8;

	uint8_t valor = matriz[byteColuna+linha*bytesPorLinha];

	//1100 0000 shift ate a posicao de offset da coluna
	uint8_t retorno = 0xC0>>offsetColuna;
	retorno = valor & retorno;

	//Shift ate a posicao menos significativa
	retorno = retorno>>(6-offsetColuna);

	//retorno de 0 a 3;
	return retorno;
}

void Matriz2Bits::setValor(uint16_t linha, uint16_t coluna, uint8_t valor){
	if(valor >3)
		valor =3;

	//Coluna e linha invalida
	if(linha>=numeroLinhas || coluna>=numeroColunas)
		return;

	int bitColuna = coluna*2;

	int byteColuna = bitColuna/8;

	int offsetColuna = bitColuna%8;

	//Shift ate a posicao de offset da coluna
	valor = valor << (6-offsetColuna);

	//Zera a posicao em que vao ser colocados os bits (Ex.: se offsetColuna = 4 entao posicao &= 1111 0011)
	matriz[byteColuna+linha*bytesPorLinha] &= ~(0xC0 >> offsetColuna);

	//Coloca os bits na posicao que foi zerada
	matriz[byteColuna+linha*bytesPorLinha] |= valor;
}

void Matriz2Bits::imprimirMatriz(){
	for(int i=0; i<numeroLinhas; i++){
		for(int j=0; j<numeroColunas; j++){
			Serial.print(((int)getValor(i, j)));
			Serial.print("\t");
		}
		Serial.print("\n");
	}
}

void Matriz2Bits::carregarMapa(uint8_t numeroMapa){

	if(!sdFailed){
		
		int linhaInicial = ((((int)(numeroMapa/MAXIMO_MAPAS))-((int)(numeroMapaAtual/MAXIMO_MAPAS))) == 1 ? CELULAS_SOBREPOSTAS  : 0);
		int linhaFinal = numeroLinhas - ((((int)(numeroMapa/MAXIMO_MAPAS))-((int)(numeroMapaAtual/MAXIMO_MAPAS))) == -1 ? CELULAS_SOBREPOSTAS  : 0);
		int colunaInicial = (((numeroMapa%MAXIMO_MAPAS)-(numeroMapaAtual%MAXIMO_MAPAS)) == 1 ? CELULAS_SOBREPOSTAS  : 0);
		int colunaFinal = numeroColunas - (((numeroMapa%MAXIMO_MAPAS)-(numeroMapaAtual%MAXIMO_MAPAS)) == -1 ? CELULAS_SOBREPOSTAS  : 0);
		
		numeroMapaAtual = numeroMapa;
		mapArchiveName = "";
		mapArchiveName += (int)(numeroMapa);
		mapArchiveName += ".txt";
		mapArchiveName.toCharArray(nomeArquivo,7);
		
		//Sobrepoe os mapas verificando se houve sobreposicao da linha ou da coluna
		if(linhaInicial>0){
			for(int i=0; i<linhaInicial; i++){
				for(int j=colunaInicial; j<colunaFinal; j++){
					setValor(i,j,getValor(numeroLinhas-1-i,j));
				}
			}
		}else if(linhaFinal<numeroLinhas){
			for(int i=linhaFinal; i<numeroLinhas; i++){
				for(int j=colunaInicial; j<colunaFinal; j++){
					setValor(i,j,getValor(i-linhaFinal,j));
				}
			}
		}
		if(colunaInicial>0){
			for(int i=linhaInicial; i<linhaFinal; i++){
				for(int j=0; j<colunaInicial; j++){
					setValor(i,j,getValor(i,numeroColunas-1-j));
				}
			}
		}else if(colunaFinal< numeroColunas){
			for(int i=linhaInicial; i<linhaFinal; i++){
				for(int j=colunaFinal; j<numeroColunas; j++){
					setValor(i,j,getValor(i,j-colunaFinal));
				}
			}
		}
		
		if (!SD.exists(nomeArquivo)) {
			
			for(int i=linhaInicial; i<linhaFinal; i++){
				for(int j=colunaInicial; j<colunaFinal; j++){
					setValor(i,j,0);
				}
			}

		}else {
			//Cria arquivo
			File myFile = SD.open(nomeArquivo);
			int linha = linhaInicial, coluna=colunaInicial;
			//carrega o existente
			String valor="0";
			while(myFile.available()>0){
				char nextChar = myFile.read();
				if(nextChar =='\t'){
				
					//Verifica se esta na regiao de sobreposicao do mapa
					if(linha>=linhaInicial && linha<linhaFinal && coluna>=colunaInicial && coluna<colunaFinal)
						setValor(linha, coluna, (uint8_t)(valor.toInt()));
						
					coluna++;
					valor="0";
				}else if(nextChar =='\n'){
				
					//Verifica se esta na regiao de sobreposicao do mapa
					if(linha>=linhaInicial && linha<linhaFinal && coluna>=colunaInicial && coluna<colunaFinal)
						setValor(linha, coluna, (uint8_t)(valor.toInt()));
						
					linha++;
					coluna=0;
					valor="0";
				}else if(nextChar==-1){
					// Final de arquivo 
					break;
				}else{
					valor += nextChar;
				}
			}
			
			myFile.close();
		}
		
	}
}

void Matriz2Bits::salvarMapa(){
	if(!sdFailed){
		if (SD.exists(nomeArquivo)) {
			SD.remove(nomeArquivo);
		}
		File myFile = SD.open(nomeArquivo, FILE_WRITE);
		
		if(myFile){
			for(int i=0; i<numeroLinhas; i++){
				for(int j=0; j<numeroColunas; j++){
					myFile.print(((int)getValor(i, j)));
					myFile.print("\t");
				}
				myFile.print("\n");
			}
			
			Serial.println("Salvando mapa.");

			myFile.close();
		}
		
		Serial.print("Arquivo ");
		Serial.print(mapArchiveName);
		Serial.println(" criado.");
	}
}

