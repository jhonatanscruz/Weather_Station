/*! @file opcn2.h
    @brief Library for managing the Alphasense OPC_N2 Particle counter.

	This library is a mix between: 
	1. Davic H Hagan, March 2016  Modified by Marcelo Yungaicela, May 2017
	2. Joseph Habeck (habec021@umn.edu) on June 2018
	3. David Gascón, Marcos Yarza, Alejandro Gállego from Libelium Comunicaciones Distribuidas
	The joint and some other modifications were made by SWoto, June 2020.

    This library was modified by Jhonatan Cruz, December 2021.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    Version:		1.0.0

*/

#ifndef WS_034B_h
#define WS_034B_h

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <Arduino.h>

/******************************************************************************
 * Definitions & Declarations
 ******************************************************************************/

#define DEBUG_1 true
#define READINGS 10
#define MULT_WIND 0.7989
#define OFFSET_WIND 0.28

/******************************************************************************
 * Variables
 ******************************************************************************/
class WS_034B
{
    private:
        uint8_t _wSpeedPin;
        uint8_t _wDirecPin;
        
        // Wind Speed Variables
        unsigned long _timeOut = 5000000; //timeout in us

        // Wind Direction Variables
        int _wdMin = 5;
        int _wdMax = 1023;
        int _l1 = 70;
        int _l2 = 200;
        int _l3 = 325;
        int _l4 = 460;
        int _l5 = 585;
        int _l6 = 710;
        int _l7 = 840;
        int _l8 = 960;

        //Functions
        void windSpeedDetectaMudanca(void);
    public:
        // Wind Speed Variables
        float freq; //Hz
        float freqKmph; //Km/h

        // Wind Direction Variables
        uint8_t codedWindDirec;
        int windDirec; //analog reading
        float windDirecDegrees;
        

        //Functions
        WS_034B(uint8_t wSpeedPin, uint8_t wDirecPin);
        uint8_t codeWindDirection(int windDirec);
        void readWindSet(void);
        void codeWindDirection(void);
        void windDirectionDegrees(void);
        float windDirectionDegrees(int windDirec);
};

#endif