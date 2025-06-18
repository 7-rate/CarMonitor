#include <M5Stack.h>
/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/
#define SEALEVELPRESSURE_HPA ( 1013.25 )

/***********************************/
/* Class                           */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
void bmp_init();

/***********************************/
/* Global Variables                */
/***********************************/
extern float temp, pressure, altitude;
extern float sealevel_pressure_offset;

extern bool is_temperature_from_sensord;
