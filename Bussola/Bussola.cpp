#include "Bussola.h"

Bussola::Bussola(){
 
	pointer=0;
	pointerSize = TAMANHO_FILA;
	filaCheia=0;
	
	Serial.println("Starting the I2C interface.");
	Wire.begin(); // Start the I2C interface.

	Serial.println("Constructing new HMC5883L");
	compass = HMC5883L(); // Construct a new HMC5883 compass.
	int error=0;

	Serial.println("Setting scale to +/- 1.3 Ga");
	error = compass.SetScale(1.3); // Set the scale of the compass.
	if(error != 0) // If there is an error, print it out.
		Serial.println(compass.GetErrorText(error));

	Serial.println("Setting measurement mode to continous.");
	error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
	if(error != 0) // If there is an error, print it out.
		Serial.println(compass.GetErrorText(error));

	delay(10);
	
	//Preenche a fila
	for(int i=0; i<=TAMANHO_FILA; i++){
		calcHeading();
		delay(1);
	}
}

Bussola::~Bussola(){
}

/**Adicionar um valor na fila circular**/
void Bussola::add(float v){
  filacirc[pointer] = v;
  pointer++; filaCheia++;
  if(pointer>=TAMANHO_FILA-1)pointer = 0;
}

	/**Retorna o valor mediano apos a ordenacao**/
float Bussola::getHeading(){
	
	calcHeading();
	if(filaCheia>=TAMANHO_FILA){
		int x, y;
		float aux;
		for( x = 0; x < pointerSize; x++ ){
			for( y = x + 1; y < pointerSize; y++ ){ // sempre 1 elemento à frente
			  // se o (x > (x+1)) então o x passa pra frente (ordem crescente)
				if ( filacirc[x] > filacirc[y] ){
					aux = filacirc[x];
					filacirc[x] = filacirc[y];
					filacirc[y] = aux;
				}
			}
		}
		return filacirc[(int)(TAMANHO_FILA/2)];
	}else{
		return 0;
	}
}

void Bussola::calcHeading(){
 //Calibracao
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  
  float compass_x_scalled = scaled.XAxis,
  compass_y_scalled = scaled.YAxis;
  
  compass_x_scalled=compass_x_scalled*compass_gain_fact*compass_x_gainError+compass_x_offset;
  compass_y_scalled=compass_y_scalled*compass_gain_fact*compass_y_gainError+compass_y_offset;

  add(atan2(compass_y_scalled, compass_x_scalled));
 }