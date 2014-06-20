#include "XBeeCommunication.h"

XBeeCommunication::XBeeCommunication(HardwareSerial *serial, int baud)
{
	xbeeSerial = serial;
	xbeeSerial->begin(baud);
}


XBeeCommunication::~XBeeCommunication(void)
{

}

void XBeeCommunication::sendPosition(int positionX, int positionY, uint8_t mapNumber){
  
  String data = "";
  data+=ROBOT_NUMBER;
  data+=";";
  data+=(int)(mapNumber);
  data+=";";
  data+=positionX;
  data+=";";
  data+=positionY;

  xbeeSerial->println(data);
  //Wait send complete
  xbeeSerial->flush();
}

void XBeeCommunication::receivePositions(Robo* robos){
  
    if(xbeeSerial->available() > 0){
      uint8_t iData = 0;
      int numRobo, numMapa, posX, posY;

      while (xbeeSerial->available() > 0) {
        switch(iData){
          case 0:
            numRobo = xbeeSerial->parseInt();
            iData++;
            break;
          case 1:
            numMapa = xbeeSerial->parseInt();
            robos[numRobo-1].mapNumber=numMapa;
            iData++;
            break;
          case 2:
            posX = xbeeSerial->parseInt();
            robos[numRobo-1].x=posX;
            iData++;
            break;
          case 3:
            posY = xbeeSerial->parseInt();
            iData=0;
            robos[numRobo-1].y=posY;
            break;
        }
      }
    }
}