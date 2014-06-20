#ifndef BUSSOLA2_H_
#define BUSSOLA2_H_

#include <consts.h>
#include "I2Cdev.h"
#include <HMC5883LV2.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <math.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define TAMANHO_FILA 3

	bool dmpReady = false;  // set true if DMP init was successful
	bool stability = false;      //estabilidade do sensor: 0 instavel 1 estavel
	int counter = 0;        //contador do estabilizador 
	int pointer=0, pointerSize = TAMANHO_FILA, aux, filaCheia=0;//ponteiro da fila circular, fila size, fila está totalmente preenchida 
	int error = 0;          //erro de declinacao da Terra (local onde está o magnetometro)
	volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
	float difAngle = 0;     //diferença de angulo do MPU6050 com HMC5883L
	
	


class MotionSensor{
	
	public:
	// MPU control/status vars
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// orientation/motion vars
	Quaternion hq, q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float 	ypr[3],// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
			compassHeading,   //Ponteiro de norte do magnetometro	
			v,             //somatoria de todos angulos do sensor MPU6050 para estabilização
			filacirc[TAMANHO_FILA];     //fila circular

	// Record any errors that may occur in the compass.
	MPU6050 mpu;
	HMC5883LV2 compass;

	MotionSensor(void){}

	~MotionSensor(void){}

	bool initialize(){
	Serial.println("wire initializing");
			// join I2C bus (I2Cdev library doesn't do this automatically)
		Wire.begin();
		// initialize serial communication
	 
		Serial.println("Initializing I2C devices...");
	 
		mpu.initialize();
		
		if (mpu.testConnection()){
		  Serial.println("MPU6050 connection successful");
		}
		else {
		  Serial.println("MPU6050 connection failed");
		}
		
		// load and configure the DMP
		Serial.println(F("Initializing DMP..."));
		devStatus = mpu.dmpInitialize();
	 
	 			// supply your own gyro offsets here, scaled for min sensitivity
		mpu.setXGyroOffset(220);
		mpu.setYGyroOffset(76);
		mpu.setZGyroOffset(-85);
		mpu.setZAccelOffset(1688); // 1688 factory default for my test chip
		
		// configuration of the compass module
		// activate the I2C bypass to directly access the Sub I2C
		mpu.setI2CMasterModeEnabled(0);
		mpu.setI2CBypassEnabled(1);
	 
		if (compass.testConnection()) {
			Serial.println("HMC5883l connection successful");
			compass.initialize();
	 
			// unfourtunally
			// hmc5883l.setMode(HMC5883L_MODE_CONTINUOUS);
			// does not work correctly. I used the following command to
			// "manually" switch on continouse measurements
			I2Cdev::writeByte(HMC5883L_DEFAULT_ADDRESS,
							  HMC5883L_RA_MODE,
							  HMC5883L_MODE_SINGLE);
	 
			// the HMC5883l is configured now, we switch back to the MPU 6050
			mpu.setI2CBypassEnabled(0);
	 
			// X axis word
			mpu.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80);
			mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
			setSlaveControl(0);
	 
			// Y axis word
			mpu.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
			mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
			setSlaveControl(1);
	 
			// Z axis word
			mpu.setSlaveAddress(2, HMC5883L_DEFAULT_ADDRESS | 0x80);
			mpu.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
			setSlaveControl(2);
	 
			mpu.setI2CMasterModeEnabled(1);

		} else {
			Serial.println("HMC5883l connection failed");
		}
		
		 // make sure it worked (returns 0 if so)
		if (devStatus == 0) {//////////////////////////////////////ativado a interrupcao
			// turn on the DMP, now that it's ready
			Serial.println(F("Enabling DMP..."));
			mpu.setDMPEnabled(true);

			// set our DMP Ready flag so the main loop() function knows it's okay to use it
			Serial.println(F("DMP ready!"));
			dmpReady = true;

			// get expected DMP packet size for later comparison
			packetSize = mpu.dmpGetFIFOPacketSize();
			stability = false;
			return regulateSensor();
		} else {
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
			Serial.print(F("DMP Initialization failed (code "));
			Serial.print(devStatus);
			Serial.println(F(")"));
			return false;
		}
	}

	/**Coloca este metodo no loop para poder atualizar os valores dos sensores**/
	bool calcGyroAngle(){
		//===============================================================
	  //===          Tratamento de MPU6050                          ===
	  //===============================================================
		// if programming failed, don't try to do anything
		if (!dmpReady) return false;
		
		
		if(!mpu.resetFIFO()) return false;
		
		delay(1);
		mpuIntStatus = mpu.getIntStatus();
		
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if (mpuIntStatus & 0x02) {
			
				
			fifoCount = mpu.getFIFOCount();	
			int n_error = 0;
					
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize && n_error < 5) {
				n_error++;
				fifoCount = mpu.getFIFOCount();	
			}
			
			if(fifoCount < packetSize)
				return false;
			
			// read a packet from FIFO
			if(!mpu.getFIFOBytes(fifoBuffer, packetSize))
				return false;
			
			fifoCount -= packetSize;
			
			// display Euler angles in degrees
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			if(stability){
				mpu.dmpGetYawPitchRoll(ypr, &hq.getProduct(q), &gravity);
			}else{
				mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			}
			
			checkStability();
			
			float gyroAngle = 0;
			gyroAngle = ypr[0]*180/M_PI+difAngle;
			gyroAngle = atan2(sin(gyroAngle*M_PI/180), cos(gyroAngle*M_PI/180))*180/M_PI;
			add(gyroAngle);//////////////adiciona o angulo na fila circular
			
			if(!stability)
				Serial.println(gyroAngle);
			
			return true;
		}else
			return false;
	}

	void checkStability(){
		
		if(!stability){
			float sum = abs((ypr[0]*180/M_PI)+(ypr[1]*180/M_PI)+(ypr[2]*180/M_PI));
			v += sum;
			if(counter>=100 && !stability)
				stability = isStabilizable((v/(counter+1)), sum, ((v/(counter+1))*0.01));
			if(counter>=100 && !stability){
				counter = 0;v=0;
			}
			counter++;
		}
	}
	
	bool isStabilizable(float med, float first, float th){
		if(med>(first-th) && med<(first+th)){
			Serial.println("Sensor estavel!");
			delay(1000);
			hq = q.getConjugate();
			calcCompassAngle();
			difAngle = compassHeading;
			return true;
		}else{
		return false;
	  }
	}
	
	void calcCompassAngle(){
		//I2Cdev::writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));

		// read raw heading measurements from device
 		int16_t compass_x_scalled = mpu.getExternalSensorWord(0);
		int16_t compass_y_scalled = mpu.getExternalSensorWord(2);
		
		compass_x_scalled=compass_x_scalled*compass_gain_fact*compass_x_gainError+compass_x_offset;
		compass_y_scalled=compass_y_scalled*compass_gain_fact*compass_y_gainError+compass_y_offset;

		compassHeading = atan2(compass_y_scalled, compass_x_scalled)*180/M_PI; 
		
		//int16_t mx, my, mz;
		//compass.getHeading(&mx, &my, &mz);
	}
	
	bool regulateSensor(){
		Serial.println("Calibrando Sensor...");
		int n_error= 0;
		while(!stability && n_error<100){
			if(!calcGyroAngle())
				n_error++;
			else
				n_error=0;
		}
		return stability;
	}

		/**Adicionar um valor na fila circular**/
	void add(float v){
	  filacirc[pointer] = v;
	  pointer++; filaCheia++;
	  if(pointer>=TAMANHO_FILA-1)pointer = 0;
	}

		/**Retorna o valor mediano apos a ordenacao**/
	float getHeading(){
	  if(filaCheia>TAMANHO_FILA){
	  int x, y;
	  for( x = 0; x < pointerSize; x++ ){
		for( y = x + 1; y < pointerSize; y++ ){ // sempre 1 elemento à frente
		  // se o (x > (x+1)) então o x passa pra frente (ordem crescente)
		  if ( filacirc[x] > filacirc[y] ){
			 aux = filacirc[x];
			 filacirc[x] = filacirc[y];
			 filacirc[y] = aux;
		  }
		}
	  }int i=0;
	  return filacirc[(int)(TAMANHO_FILA/2)]*M_PI/180;
	  }else{
		return -1;
	  }
	}
	
	void setSlaveControl(uint8_t slaveID){
		mpu.setSlaveEnabled(slaveID, true);
		mpu.setSlaveWordByteSwap(slaveID, false);
		mpu.setSlaveWriteMode(slaveID, false);
		mpu.setSlaveWordGroupOffset(slaveID, false);
		mpu.setSlaveDataLength(slaveID, 2);
	}
	
	void resetSensors(){
		Wire.endTransmission(true);
		mpu.resetSensors();
	}
};
#endif /*BUSSOLA2_H_*/