#include <M5Stack.h>
/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/
#define SEALEVELPRESSURE_HPA (1013.25)

/***********************************/
/* Class                           */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
void accelgyro_init();
void accelgyro_exec();
void bmp_init();
void bmp_exec();

/***********************************/
/* Global Variables                */
/***********************************/
extern int16_t ax, ay, az;
extern int32_t ax_filterd, ay_filterd, az_filterd;
extern int16_t gx, gy, gz;
extern float roll, pitch, yaw;

extern float temp, pressure, altitude;
extern float sealevel_pressure_offset;
