#pragma once
#include <Arduino.h>
#include <consts.h>

struct Robo
{
	int x;
	int y;
	int mapNumber;
};

class XBeeCommunication
{
public:
	XBeeCommunication(HardwareSerial *serial, int baud);
	~XBeeCommunication(void);
	void sendPosition(int positionX, int positionY, uint8_t mapNumber);
	void receivePositions(Robo* robos);

	HardwareSerial *xbeeSerial;
};

