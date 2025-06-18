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
#define UPDATE( dest, expr )                                                                                           \
    do {                                                                                                               \
        auto tmp = ( expr );                                                                                           \
        tmr_obd_timeout = millis();                                                                                    \
        while ( elm.nb_rx_state != ELM_SUCCESS && (long)( millis() - tmr_obd_timeout ) < OBD_TIMEOUT ) {               \
            tmp = ( expr );                                                                                            \
            vTaskDelay( pdMS_TO_TICKS( 10 ) );                                                                         \
        }                                                                                                              \
        if ( elm.nb_rx_state == ELM_SUCCESS )                                                                          \
            dest = tmp;                                                                                                \
    } while ( 0 )

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

float car_outside_temperature = 25.0f;
bool temp_initialized = false;
float engine_coolant_temp;
float engine_oil_temp;
uint8_t manifold_pressure;
uint8_t abs_baro_pressure;
float boost_pressure;
float fuel_level;

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
static void update_dpf_pm_accumulation() {
    UPDATE( dpf_pm_accum, elm.processPID( 0x22, DPF_PM_ACCUMULATION, 1, 2, 100.0 / 65535.0, 0.0 ) );
}

static void update_dpf_pm_generation() {
    UPDATE( dpf_pm_gen, elm.processPID( 0x22, DPF_PM_GENERATION, 1, 2, 0.00153, 0.0 ) );
}

static void update_dpf_regeneration_count() {
    UPDATE( dpf_reg_count, (int)elm.processPID( 0x22, DPF_REGENERATION_COUNT, 1, 2, 1.0, 0.0 ) );
}

static void update_dpf_regeneration_distance() {
    UPDATE( dpf_reg_dist, (int)elm.processPID( 0x22, DPF_REGENERATION_DISTANCE, 1, 4, 0.00156, 0.0 ) );
}

static void update_dpf_regeneration_status() {
    UPDATE( dpf_reg_status, (int)elm.processPID( 0x22, DPF_REGENERATION_STATUS, 1, 1, 1.0, 0.0 ) );
}

static void update_car_outside_temperature() {
    UPDATE( car_outside_temperature, elm.ambientAirTemp() );
    if ( elm.nb_rx_state == ELM_SUCCESS )
        temp_initialized = true;
}

static void update_engine_coolant_temp() {
    UPDATE( engine_coolant_temp, elm.engineCoolantTemp() );
}

static void update_engine_oil_temp() {
    UPDATE( engine_oil_temp, elm.oilTemp() );
}

static void update_manifold_pressure() {
    UPDATE( manifold_pressure, elm.manifoldPressure() );
}

static void update_abs_baro_pressure() {
    UPDATE( abs_baro_pressure, elm.absBaroPressure() );
}

static void update_boost_pressure() {
    boost_pressure = manifold_pressure * 10 - abs_baro_pressure;
}

static void update_fuel_level() {
    UPDATE( fuel_level, elm.fuelLevel() );
}

void task_update_obd( void* arg ) {
    while ( 1 ) {
        update_dpf_regeneration_status();
        update_dpf_pm_accumulation();
        update_dpf_pm_generation();
        update_dpf_regeneration_count();
        update_dpf_regeneration_distance();
        update_car_outside_temperature();
        update_engine_coolant_temp();
        update_engine_oil_temp();
        update_manifold_pressure();
        update_abs_baro_pressure();
        update_boost_pressure();
        update_fuel_level();
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
    sprite.setCursor( 10, 50 );
    sprite.printf( "Init ELM327 phase1..." );
    sprite.pushSprite( 0, 0 );
    SerialBT.begin( "ArduHUD", true );
    error = !SerialBT.connect( ELM327_MACADDRESS );
    sprite.setCursor( 240, 50 );
    sprite.print( error ? "err!!" : "done!" );
    sprite.pushSprite( 0, 0 );

    if ( error ) {
        return;
    }

    sprite.setCursor( 10, 80 );
    sprite.printf( "Init ELM327 phase2..." );
    sprite.pushSprite( 0, 0 );
    error = !elm.begin( SerialBT, true, 2000 );
    sprite.setCursor( 240, 80 );
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