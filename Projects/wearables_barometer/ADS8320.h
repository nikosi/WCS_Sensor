/* Nikolas Simon
 * University of Freiburg 
 * Ocotber 2014
 * 
 * Functions for initializing and reading 
 * analog-digital converter ADS8320
 *
 */


#include <stdbool.h>
#include <stdint.h>


#ifndef ADS8320_H
#define ADS8320_H


typedef enum {ADC1 = 0, ADC2}selector;

uint16_t ads8320_read(selector select);

#endif //ADS8320_H
