#include <M5Stack.h>
/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/
#define PM_MAX (6.0)

/***********************************/
/* Class                           */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
void car_param_init();
void car_param_exec();

/***********************************/
/* Global Variables                */
/***********************************/
extern uint32_t rpm;
extern uint32_t kph;
extern float dpf_pm_accum;
extern float dpf_pm_gen;
extern int dpf_reg_count;
extern int dpf_reg_dist;
extern int dpf_reg_status;
extern float car_outside_temperature;
