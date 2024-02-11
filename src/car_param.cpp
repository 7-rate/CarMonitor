#include "car_param.h"
#include <BluetoothSerial.h>
#include <ELMduino.h>
#include "common.h"
/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
#define OBD_TIMEOUT ( 1000 )

// for Mazda2 PIDs
const uint16_t DPF_PM_ACCUMULATION = 0x042C;
const uint16_t DPF_PM_GENERATION = 0x042D;
const uint16_t DPF_REGENERATION_COUNT = 0x0432;
const uint16_t DPF_REGENERATION_DISTANCE = 0x0434;
const uint16_t DPF_REGENERATION_STATUS = 0x0380;

/***********************************/
/* Local Variables                 */
/***********************************/
uint8_t ELM327_MACADDRESS[] = { 0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33 };
ELM327 elm;
BluetoothSerial SerialBT;
static unsigned long tmr_obd_timeout;

/***********************************/
/* Global Variables                */
/***********************************/
uint32_t rpm;
uint32_t kph;
float dpf_pm_accum;
float dpf_pm_gen;
int dpf_reg_count;
int dpf_reg_dist;
int dpf_reg_status;

float car_outside_temperature = 0.0f;

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
static void update_dpf_pm_accumulation() {
    float temp = elm.processPID( 0x22, DPF_PM_ACCUMULATION, 1, 2, 100.0 / 65535.0, 0.0 ); //(A*256+B)*100/65535
    tmr_obd_timeout = millis();
    while ( elm.nb_rx_state != ELM_SUCCESS && millis() - tmr_obd_timeout < OBD_TIMEOUT ) {
        temp = elm.processPID( 0x22, DPF_PM_ACCUMULATION, 1, 2, 100.0 / 65535.0, 0.0 );
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    if ( elm.nb_rx_state == ELM_SUCCESS ) {
        dpf_pm_accum = temp;
    }
}

static void update_dpf_pm_generation() {
    float temp = elm.processPID( 0x22, DPF_PM_GENERATION, 1, 2, 0.00153, 0.0 ); //(A*256+B)*100/65535
    tmr_obd_timeout = millis();
    while ( elm.nb_rx_state != ELM_SUCCESS && (long)( millis() - tmr_obd_timeout ) < OBD_TIMEOUT ) {
        temp = elm.processPID( 0x22, DPF_PM_GENERATION, 1, 2, 0.00153, 0.0 );
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    if ( elm.nb_rx_state == ELM_SUCCESS ) {
        dpf_pm_gen = temp;
    }
}

static void update_dpf_regeneration_count() {
    int temp = (int)elm.processPID( 0x22, DPF_REGENERATION_COUNT, 1, 2, 1.0, 0.0 ); // A*256+B
    tmr_obd_timeout = millis();
    while ( elm.nb_rx_state != ELM_SUCCESS && (long)( millis() - tmr_obd_timeout ) < OBD_TIMEOUT ) {
        temp = (int)elm.processPID( 0x22, DPF_REGENERATION_COUNT, 1, 2, 1.0, 0.0 );
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    if ( elm.nb_rx_state == ELM_SUCCESS ) {
        dpf_reg_count = temp;
    }
}

static void update_dpf_regeneration_distance() {
    int temp = (int)elm.processPID( 0x22, DPF_REGENERATION_DISTANCE, 1, 4, 0.00156, 0.0 ); //((B<<16)+(C<<8)+D)/640
    tmr_obd_timeout = millis();
    while ( elm.nb_rx_state != ELM_SUCCESS && (long)( millis() - tmr_obd_timeout ) < OBD_TIMEOUT ) {
        temp = (int)elm.processPID( 0x22, DPF_REGENERATION_DISTANCE, 1, 4, 0.00156, 0.0 );
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    if ( elm.nb_rx_state == ELM_SUCCESS ) {
        dpf_reg_dist = temp;
    }
}

static void update_dpf_regeneration_status() {
    int temp = (int)elm.processPID( 0x22, DPF_REGENERATION_STATUS, 1, 1, 1.0, 0.0 ); //(A)
    tmr_obd_timeout = millis();
    while ( elm.nb_rx_state != ELM_SUCCESS && (long)( millis() - tmr_obd_timeout ) < OBD_TIMEOUT ) {
        temp = (int)elm.processPID( 0x22, DPF_REGENERATION_STATUS, 1, 1, 1.0, 0.0 );
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    if ( elm.nb_rx_state == ELM_SUCCESS ) {
        dpf_reg_status = temp;
    }
}

static void update_car_outside_temperature() {
    float temp = elm.ambientAirTemp();
    tmr_obd_timeout = millis();
    while ( elm.nb_rx_state != ELM_SUCCESS && (long)( millis() - tmr_obd_timeout ) < OBD_TIMEOUT ) {
        temp = elm.ambientAirTemp();
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    if ( elm.nb_rx_state == ELM_SUCCESS ) {
        car_outside_temperature = temp;
    }
}

void task_update_obd( void* arg ) {
    while ( 1 ) {
        update_dpf_regeneration_status();
        update_dpf_pm_accumulation();
        update_dpf_pm_generation();
        update_dpf_regeneration_count();
        update_dpf_regeneration_distance();
        update_car_outside_temperature();
        vTaskDelay( pdMS_TO_TICKS( 100 ) );
    }
}

/***********************************/
/* Class implementions             */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
void car_param_init() {
    bool error;

    // Connecting ELM327
    sprite.setCursor( 10, 110 );
    sprite.printf( "Init ELM327 phase1..." );
    sprite.pushSprite( 0, 0 );
    SerialBT.begin( "ArduHUD", true );
    error = !SerialBT.connect( ELM327_MACADDRESS );
    sprite.setCursor( 240, 110 );
    sprite.print( error ? "err!!" : "done!" );
    sprite.pushSprite( 0, 0 );

    if ( error ) {
        return;
    }

    sprite.setCursor( 10, 140 );
    sprite.printf( "Init ELM327 phase2..." );
    sprite.pushSprite( 0, 0 );
    error = !elm.begin( SerialBT, true, 2000 );
    sprite.setCursor( 240, 140 );
    sprite.print( error ? "err!!" : "done!" );
    sprite.pushSprite( 0, 0 );

    sprite.setTextSize( 4 );
    sprite.setCursor( 10, 200 );
    sprite.print( error ? "Error!" : "Complete!" );
    sprite.pushSprite( 0, 0 );

    if ( error ) {
        return;
    }
    xTaskCreatePinnedToCore( task_update_obd, "task_update_obd", 4096, NULL, 1, NULL, 0 );
}

void car_param_exec() {
}