#include <Arduino.h>
#include "sensors.h"
#include "common.h"
#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include "car_param.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
// MPU6050 offsets
#define MPU6050_XA_OFFSET ( -3164 )
#define MPU6050_YA_OFFSET ( 348 )
#define MPU6050_ZA_OFFSET ( 2009 )
#define MPU6050_XG_OFFSET ( -49 )
#define MPU6050_YG_OFFSET ( -15 )
#define MPU6050_ZG_OFFSET ( -61 )

#define BMP_UPDATE_CYCLE ( 100 )
#define MPU_UPDATE_CYCLE ( 100 )

#define TEMP_OFFSET ( -2.5 )

/***********************************/
/* Local Variables                 */
/***********************************/
static MPU6050 mpu( 0x68 );
static Madgwick madgwickfilter;
static Adafruit_BMP280 bmp;
static unsigned long tmr_bmp;
static unsigned long tmr_mpu;

/***********************************/
/* Global Variables                */
/***********************************/
int16_t ax, ay, az;
int32_t ax_filterd, ay_filterd, az_filterd;
int16_t gx, gy, gz;
float roll, pitch, yaw;
float temp, pressure, altitude;
float sealevel_pressure_offset;

bool is_temperature_from_sensord;

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
#define STANDARD_PRESSURE 101325.0
#define STANDARD_TEMPERATURE 15.0
#define STANDARD_GRAVITY 9.80665

// 標準気圧に対する海抜0mの高度（m）
#define ALTITUDE_AT_STANDARD_PRESSURE 0.0

// 以下より、現在地の高度を計算する関数
// 海面気圧:(SEALEVELPRESSURE_HPA+sealevel_pressure_offset)
// 現在地気圧：pressure
float pressure2altitude( float pressure_hPa, float temp_degree ) {
    float pressure_Pa = pressure_hPa * 100.0; // パスカルに変換
    float temp_K = temp_degree + 273.15;      // ケルビンに変換

    // 標準気圧、標準気温の条件での海抜0mの標準大気圧（パスカル）
    float standard_pressure_at_altitude_0m =
        ( ( SEALEVELPRESSURE_HPA + sealevel_pressure_offset ) * 100 ) *
        exp( -( STANDARD_GRAVITY / ( 287.05 * STANDARD_TEMPERATURE ) ) * ALTITUDE_AT_STANDARD_PRESSURE );

    // 高度の計算
    float altitude_m = -( 287.05 * temp_K / STANDARD_GRAVITY ) * log( pressure_Pa / standard_pressure_at_altitude_0m );

    return altitude_m;
}

static void accelgyro_exec() {
    mpu.getMotion6( &ax, &ay, &az, &gx, &gy, &gz );
    madgwickfilter.updateIMU( gx / 131.0, gy / 131.0, gz / 131.0, ax / 16384.0, ay / 16384.0, az / 16384.0 );
    roll = madgwickfilter.getRoll();
    pitch = madgwickfilter.getPitch();
    yaw = madgwickfilter.getYaw();
}

static void accelgyro_task( void* pvParameters ) {
    const TickType_t xFrequency = ( MPU_UPDATE_CYCLE ) / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while ( 1 ) {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        accelgyro_exec();
    }
}

static void bmp_exec() {
    if ( bmp.takeForcedMeasurement() ) {
        tmr_bmp = millis();
        temp = bmp.readTemperature() + TEMP_OFFSET;
        pressure = bmp.readPressure();
        // altitude = bmp.readAltitude( SEALEVELPRESSURE_HPA + sealevel_pressure_offset );
        if ( is_temperature_from_sensord ) {
            altitude = pressure2altitude( pressure / 100.0, temp );
        } else {
            // 気温ソースがOBD2の場合、OBD2から気温が取得できるまで待つ
            while ( !temp_initialized ) {
                vTaskDelay( pdMS_TO_TICKS( 100 ) );
            }
            altitude = pressure2altitude( pressure / 100.0, car_outside_temperature );
        }
        Serial.printf( "------BMP----------\n" );
        Serial.printf( "Temp: %.2f\nPressure: %.2f\nAltitude: %.2f\noffset: %.2f", temp, pressure, altitude,
                       sealevel_pressure_offset );
        Serial.printf( "-------------------\n" );
    }
}

static void bmp_task( void* pvParameters ) {
    const TickType_t xFrequency = ( BMP_UPDATE_CYCLE ) / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while ( 1 ) {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        bmp_exec();
    }
}

/***********************************/
/* Class implementions             */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
void accelgyro_init() {
    sprite.setCursor( 10, 50 );
    sprite.printf( "Init MPU6050..." );
    sprite.pushSprite( 0, 0 );
    mpu.initialize();
    mpu.setXAccelOffset( MPU6050_XA_OFFSET );
    mpu.setYAccelOffset( MPU6050_YA_OFFSET );
    mpu.setZAccelOffset( MPU6050_ZA_OFFSET );
    mpu.setXGyroOffset( MPU6050_XG_OFFSET );
    mpu.setYGyroOffset( MPU6050_YG_OFFSET );
    mpu.setZGyroOffset( MPU6050_ZG_OFFSET );
    madgwickfilter.begin( 100 ); // sampling 100Hz
    sprite.setCursor( 240, 50 );
    sprite.print( "done!" );
    sprite.pushSprite( 0, 0 );

    xTaskCreatePinnedToCore( accelgyro_task, "accelgyro_task", 4096, NULL, 1, NULL, 0 );
}

void bmp_init() {
    sprite.setCursor( 10, 20 );
    sprite.print( "Init BMP280..." );
    sprite.pushSprite( 0, 0 );

    bmp.begin( 0x76 );
    bmp.setSampling( Adafruit_BMP280::MODE_FORCED,      /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,      /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16,     /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X16,       /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500 ); /* Standby time. */
    delay( 50 );
    altitude = preferences.getFloat( "altitude", 0.0 );
    pressure = bmp.readPressure();
    float sealevel_pressure = bmp.seaLevelForAltitude( altitude, pressure );
    Serial.print( "Sealevel pressure: " );
    Serial.println( sealevel_pressure );
    Serial.print( "Pressure: " );
    Serial.println( pressure );
    Serial.print( "Altitude: " );
    Serial.println( altitude );

    sealevel_pressure_offset = ( ( sealevel_pressure / 100.0 ) - SEALEVELPRESSURE_HPA );
    Serial.print( "Pressure offset: " );
    Serial.println( sealevel_pressure_offset );

    sprite.setCursor( 240, 20 );
    sprite.print( "done!" );
    sprite.pushSprite( 0, 0 );

    xTaskCreatePinnedToCore( bmp_task, "bmp_task", 4096, NULL, 1, NULL, 0 );
}
