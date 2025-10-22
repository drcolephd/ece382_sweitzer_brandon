// IRDistance.c
// Runs on MSP432
// Provide mid-level functions that convert raw ADC
// values from the GP2Y0A21YK0F infrared distance sensors to
// distances in mm.
// Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// 5V  connected to all three Pololu #136 GP2Y0A21YK0F Vcc's (+5V)
// ground connected to all three Pololu #136 GP2Y0A21YK0F grounds
// MSP432 P9.0 (J5) (analog input A17 to MSP432) connected to right GP2Y0A21YK0F Vout
// MSP432 P6.1 (J3.23) (analog input A14 to MSP432) connected to center GP2Y0A21YK0F Vout
// MSP432 P9.1 (J5) (analog input A16 to MSP432) connected to left GP2Y0A21YK0F Vout


#include <stdint.h>
#include "msp.h"

#define MAX_DIST            800         // Max distance in mm.

// Update the following coefficients for Lab 15
#define ADCMAX_LEFT			2000	// Maximum IR ADC value
#define IRSLOPE_LEFT 		1177404	// Calibration coefficient, m
#define IROFFSET_LEFT 		610	// Calibration coefficient, c
#define DIST_OFFSET_LEFT 	-60	// Distance from common spot on robot to IR sensor

// LeftConvert
// Calculate the distance in mm given the 14-bit ADC value
// d = m/(n-c) + r, where n is the adc_value
// The maximum measurement distance for the sensor is 800 mm,
// so if the ADC value is less than ADCMAX_LEFT,
// your function should return MAX_DIST
// Input adc_value: 14-bit ADC data
// Output: Distance in mm
uint16_t LeftConvert(uint32_t adc_value){        // returns left distance in mm
    // write this for Lab 15
    if (adc_value < ADCMAX_LEFT) {
        return MAX_DIST;
    }
    uint16_t leftDist = IRSLOPE_LEFT/(adc_value-IROFFSET_LEFT) - DIST_OFFSET_LEFT;

}


// Update the following coefficients for Lab 15
// Ensure ADCMAX must be greater than IROFFSET
#define ADCMAX_CENTER         2000   // Maximum IR ADC value
#define IRSLOPE_CENTER        1123412   // Calibration coefficient, m
#define IROFFSET_CENTER       475   // Calibration coefficient, c
#define DIST_OFFSET_CENTER    70   // Distance from common spot on robot to IR sensor.

// CenterConvert
// Calculate the distance in mm given the 14-bit ADC value
// d = m/(n-c) + r, where n is the adc_value
// The maximum measurement distance for the sensor is 800 mm,
// so if the ADC value is less than ADCMAX_CENTER,
// your function should return MAX_DIST
// Input adc_value: 14-bit ADC data
// Output: Distance in mm
uint16_t CenterConvert(uint32_t adc_value){   // returns center distance in mm
    // write this for Lab 15
    if (adc_value < ADCMAX_CENTER) {
            return MAX_DIST;
        }
        uint16_t leftDist = IRSLOPE_CENTER/(adc_value-IROFFSET_CENTER) + DIST_OFFSET_CENTER;
}


// Update the following coefficients for Lab 15
// Ensure ADCMAX must be greater than IROFFSET
#define ADCMAX_RIGHT        1800     // Maximum IR ADC value
#define IRSLOPE_RIGHT       1248465     // Calibration coefficient, m
#define IROFFSET_RIGHT      336     // Calibration coefficient, c
#define DIST_OFFSET_RIGHT   -60     // Distance from common spot on robot to IR sensor.

// RightConvert
// Calculate the distance in mm given the 14-bit ADC value
// d = m/(n-c) + r, where n is the adc_value
// The maximum measurement distance for the sensor is 800 mm,
// so if the ADC value is less than ADCMAX_RIGHT,
// your function should return MAX_DIST
// Input adc_value: 14-bit ADC data
// Output: Distance in mm
uint16_t RightConvert(uint32_t adc_value){      // returns right distance in mm
    // write this for Lab 15
    if (adc_value < ADCMAX_RIGHT) {
            return MAX_DIST;
        }
        uint16_t leftDist = IRSLOPE_RIGHT/(adc_value-IROFFSET_RIGHT) - DIST_OFFSET_RIGHT;
}
