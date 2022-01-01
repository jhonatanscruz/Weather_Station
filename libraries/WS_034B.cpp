/**
	Library for managing the Campbell Scientific 034B Wind Set with Arduino

	This library was made by: 
	1. Made by SWoto, June 2020.

	This library was modified by: 
	1. Modified by Jhonatan Cruz, December 2021 - v1.0.0

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.

	Version:		1.0.0
 */

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <math.h>
#include "WS_034B.h"

/******************************************************************************
 * Constructors
 ******************************************************************************/

WS_034B::WS_034B(uint8_t wSpeedPin, uint8_t wDirecPin)
{
    this->_wSpeedPin = wSpeedPin;
	this->_wDirecPin = wDirecPin;

	_wdMin = 5; // It never reachs the 0 to avoid short
	_wdMax = 1023;
	freq = 0;
	codedWindDirec = 0;
    windDirec = 0;
	windDirecDegrees = 0;

    pinMode(this->_wSpeedPin, INPUT);
	pinMode(this->_wDirecPin, INPUT);
}

/******************************************************************************
 * PRIVATE FUNCTIONS
 ******************************************************************************/

/**
  *	@brief Check if the readings changed from noisy to flat or flat to noisy.
  * S1 = true -> encontrou primeiro zero
  * S2 = true -> ruido apos S1 true
  */

void WS_034B::windSpeedDetectaMudanca(void)
{
	uint8_t i = 1; 
	uint8_t countZeros = 0; 
	float _freq = 0.;
	unsigned long timePulse = 0;

    while (i <= READINGS) {

		if(DEBUG_1){
			Serial.print(i);
			Serial.print(" - timePulse: ");
		} else delay(250);

		// Calcula a duração do pulso no pino _wSpeedPin esperando _timeOut us para o pulso iniciar
		//_timeOut = 5000000 microseconds = 5 seconds
		timePulse = pulseIn(_wSpeedPin, HIGH, _timeOut);

		if(DEBUG_1){
			Serial.print(timePulse);
			Serial.print(" - Freq: ");
		} else delay(250);

		if(timePulse > 0 && timePulse <= 5000000)
		{
			freq = 1000000.0/(2*(timePulse));
			if(DEBUG_1)	Serial.println(freq);
			else delay(250);

			if(freq > 100 || freq < 0.1){
				freq = 0;
				++countZeros;
				if(DEBUG_1)	Serial.println("freq > 100 || freq < 0.1 >>> ");
				else delay(250);
			}
		}
		else
		{
			freq = 0;
			++countZeros;
			if(DEBUG_1)	Serial.println("first else");
			else delay(250);
		}
		_freq += freq;

		i++;
    }

	if(READINGS!=countZeros){
		freq = _freq/(READINGS-countZeros);
	}else{
		freq = 0;
		if(DEBUG_1)	Serial.println("i!=countZeros");
		else delay(250);
	}

	//Evaluate velocity in Km/h
	//The expression for wind speed (freqKmph) is:
	//freqKmph = (MULT_WIND*freq + OFFSET_WIND) * 3.6
	//where
	//MULT_WIND = multiplier
	//freq = number of pulses per second (Hertz)
	//OFFSET_WIND = offset
	// 3.6 is the multiplier to transform from m/s to km/h
	if(freq > 0) freqKmph = (freq*MULT_WIND + OFFSET_WIND) * 3.6;
	else freqKmph = 0;
}

/**
 *	@brief Call codeWindDirection with object's value
*/
void WS_034B::codeWindDirection(void)
{ 	
	this->codedWindDirec = codeWindDirection(this->windDirecDegrees);
}

/**
 *	@brief Call windDirectionDegrees with object's value
*/
void WS_034B::windDirectionDegrees(void)
{
	this->windDirecDegrees = windDirectionDegrees(this->windDirec);
}
/******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

/**
 *	@brief Code the degrees between 0-7.
 * 	0 -> North
 * 	1 -> NorthEast
 * 	2 -> East
 * 	3 -> SouthEast
 * 	4 -> South
 * 	5 -> SouthWest
 * 	6 -> West
 *  7 -> NortWest
*/
uint8_t WS_034B::codeWindDirection(int windDirec)
{
	uint8_t WD = 99;
	String message = "";

	if(windDirec > _l8 || windDirec <= _l1)
	{
		message = "N - Norte";
		WD = 0;
	}
	else if(windDirec > _l1 && windDirec <= _l2)
	{
		message = "NE - Nordeste";
		WD = 1;
	}
	else if(windDirec > _l2 && windDirec <= _l3)
	{
		message = "L - Leste";
		WD = 2;
	}
	else if(windDirec > _l3 && windDirec <= _l4)
	{
		message = "SE - Sudeste";
		WD = 3;
	}
	else if(windDirec > _l4 && windDirec <= _l5)
	{
		message = "S - Sul";
		WD = 4;
	}
	else if(windDirec > _l5 && windDirec <= _l6)
	{
		message = "SO - Sudoeste";
		WD = 5;
	}
	else if(windDirec > _l6 && windDirec <= _l7)
	{
		message = "O - Oeste";
		WD = 6;
	}
	else if(windDirec > _l7 && windDirec <= _l8)
	{
		message = "NO - Noroeste";
		WD = 7;
	}
	//Serial.println(message);
	return WD;
}

/**
 *	@brief calculate the degrees from the analog reading. Also
 * updates the min and max values calculate it.
 * 
 * @warning IMPORTANT to move give the WD a full 360 turn
*/
float WS_034B::windDirectionDegrees(int windDirec)
{
	if(windDirec < _wdMin){
      	_wdMin = windDirec;
  	}
	if(windDirec > _wdMax){
		_wdMax = windDirec;
	}
  	return - (_wdMax - windDirec) * 360. / (_wdMax - _wdMin) + 360.;
}

/**
 *	@brief reads the WD and WS.
*/
void WS_034B::readWindSet(void)
{
	windSpeedDetectaMudanca();

	windDirec = analogRead(_wDirecPin);
	windDirectionDegrees();
	codeWindDirection();
}