/*
 * MatrizBits.h
 *
 *  Created on: 06/11/2013
 *      Author: Apolo Marton
 */

#ifndef MATRIZ2BITS_H_
#define MATRIZ2BITS_H_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define MISO_PIN 50
#define MOSI_PIN 51
#define SCK_PIN 52
#define CS_PIN 53

#define CELULAS_SOBREPOSTAS 10
#define MAXIMO_MAPAS 10

using namespace std;

class Matriz2Bits {
private:
	uint8_t *matriz, numeroMapaAtual;
	uint16_t numeroLinhas, numeroColunas, numeroBytes, bytesPorLinha;
	char nomeArquivo[7];
	String mapArchiveName;
	File root;
	bool sdFailed;
public:
	Matriz2Bits(uint16_t linhas, uint16_t colunas, uint8_t numeroMapa);
	virtual ~Matriz2Bits();
	void setValor(uint16_t linha, uint16_t coluna, uint8_t valor);
	uint8_t getValor(uint16_t linha, uint16_t coluna);
	void imprimirMatriz();
	void carregarMapa(uint8_t numeroMapa);
	void salvarMapa();
};

#endif /* MATRIZ2BITS_H_ */
