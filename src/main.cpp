#include "car_param.h"
#include "sensors.h"
#include "common.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
static const unsigned long WAIT = 10;
static const unsigned long WAIT_MONI = 10;

// clang-format off
enum { 
    SCREEN_OVERVIEW, 
    // SCREEN_ALTITUDE,
    SCREEN_DPF_STATUS,
    SCREEN_ADDITIONAL_METER,
    SCREEN_SETTING,
    SCREEN_NUM
};
// clang-format on

#define SCREEN_TITLE_BG_RED( x )                                                                                       \
    sprite.fillRect( 0, 0, 320, 20, RED );                                                                             \
    sprite.setTextColor( BLACK );                                                                                      \
    sprite.setFreeFont( &FreeMonoBold9pt7b );                                                                          \
    sprite.setTextSize( 1 );                                                                                           \
    sprite.setCursor( 10, 10 );                                                                                        \
    sprite.printf( x );

#define SCREEN_TITLE( x )                                                                                              \
    sprite.fillRect( 0, 0, 320, 20, WHITE );                                                                           \
    sprite.setTextColor( BLACK );                                                                                      \
    sprite.setFreeFont( &FreeMonoBold9pt7b );                                                                          \
    sprite.setTextSize( 1 );                                                                                           \
    sprite.setCursor( 10, 12 );                                                                                        \
    sprite.printf( x );

#define SET_FONT_AND_SIZE( x, y )                                                                                      \
    sprite.setFreeFont( &x );                                                                                          \
    sprite.setTextSize( y );

TinyGPSPlus gps;
SoftwareSerial ss( 26, 25 );

/***********************************/
/* Local Variables                 */
/***********************************/
static unsigned long tmr;
static unsigned long tmr_button[3]; // button A,B,C

static unsigned long tmr_save_prefarences;
static const unsigned long SAVE_PREFARENCES_CYCLE = 1000; // 1sec

static int screen;
static float altitude_old;

/***********************************/
/* Global Variables                */
/***********************************/
Preferences preferences;
TFT_eSprite sprite = TFT_eSprite( &M5.Lcd );

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
static void btn_process( int button, void func( void ) ) {
    Button* btn;
    switch ( button ) {
    case BTN_A:
        btn = &M5.BtnA;
        break;
    case BTN_B:
        btn = &M5.BtnB;
        break;
    case BTN_C:
        btn = &M5.BtnC;
        break;
    default:
        btn = NULL; // kill
        return;
    }
    if ( btn->wasPressed() ) {
        func();
    }
    // long press increase/decrease
    if ( btn->pressedFor( 750 ) && tmr_button[button] + 10 < millis() ) {
        tmr_button[button] = millis();
        func();
    }
}

// 進捗バー描画ヘルパー
static void drawProgressBar( int x, int y, int w, int h, float val, float maxVal, uint16_t color ) {
    int per = (int)( val / maxVal * 100 );
    per = min( per, 100 );
    sprite.drawRect( x, y, w, h, color );
    sprite.fillRect( x, y, w * per / 100, h, color );
}

/* ステータス盛り盛り表示
 * ・煤の堆積量
 * ・DPF再生からのTrip距離
 * ・水温
 * ・油温
 * ・燃料量
 * ・高度
 * ・地点気圧
 * ・海面気圧
 */
static void display_overview() {
    int percent = 0;
    if ( tmr + WAIT < millis() ) {
        tmr = millis();

        // display
        sprite.fillScreen( BLACK );
        SCREEN_TITLE( "Overview" );

        // 煤の堆積量
        uint16_t dpf_color = dpf_reg_status ? RED : WHITE;
        sprite.setTextColor( dpf_color );
        SET_FONT_AND_SIZE( FreeMono12pt7b, 1 );
        sprite.setCursor( 10, 38 );
        sprite.printf( "DPF" );
        float soot = max( dpf_pm_accum, dpf_pm_gen );
        sprite.setCursor( 220, 38 );
        sprite.printf( "%.2fg/L", soot );
        drawProgressBar( 85, 25, 125, 18, soot, PM_MAX, dpf_color );

        // DPF再生からのTrip距離
        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono12pt7b, 1 );
        sprite.setCursor( 10, 68 );
        sprite.printf( "Trip" );
        sprite.setCursor( 85, 68 );
        sprite.printf( "%dkm", dpf_reg_dist );

        // 水温
        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono12pt7b, 1 );
        sprite.setCursor( 10, 98 );
        sprite.printf( "Water" );
        sprite.setCursor( 220, 98 );
        sprite.printf( "%ddeg", (int)engine_coolant_temp );
        drawProgressBar( 85, 82, 125, 18, engine_coolant_temp, WARTER_TEMP_MAX, WHITE );

        // 油温
        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono12pt7b, 1 );
        sprite.setCursor( 10, 128 );
        sprite.printf( "Oil" );
        sprite.setCursor( 220, 128 );
        sprite.printf( "%ddeg", (int)engine_oil_temp );
        drawProgressBar( 85, 112, 125, 18, engine_oil_temp, OIL_TEMP_MAX, WHITE );

        // 燃料量
        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono12pt7b, 1 );
        sprite.setCursor( 10, 158 );
        sprite.printf( "Fuel" );
        sprite.setCursor( 220, 158 );
        sprite.printf( "%dL",
                       (int)( fuel_level * FUEL_TANK_CAPACITY ) /
                           100 ); // fuel_levelはパーセンテージなので、タンク容量を掛ける
        drawProgressBar( 85, 142, 125, 18, fuel_level, FUEL_LEVEL_MAX, WHITE );

        // 高度
        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono12pt7b, 1 );
        sprite.setCursor( 10, 198 );
        sprite.printf( "Altitude" );
        SET_FONT_AND_SIZE( FreeMono12pt7b, 2 );
        sprite.setCursor( 130, 210 );
        // sprite.printf( "%dm", (int)altitude );
        sprite.printf( "%dm", (int)gps.altitude.meters() );

        // GPS情報
        uint32_t gps_color = gps.location.isValid() ? GREEN : RED;
        sprite.fillCircle( 300, 220, 8, gps_color );

        // 地点気圧、海面気圧
        // sprite.setTextColor( WHITE );
        // SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        // sprite.setCursor( 120, 215 );
        // sprite.printf( "%.2fhPa", ( pressure / 100.0 ) );
        // sprite.setCursor( 120, 235 );
        // sprite.printf( "sea:%.2fhPa", SEALEVELPRESSURE_HPA + sealevel_pressure_offset );

        sprite.pushSprite( 0, 0 );
    }
}

static void display_altitude() {
    if ( tmr + WAIT < millis() ) {
        tmr = millis();

        // display
        sprite.fillScreen( BLACK );
        SCREEN_TITLE( "Altitude" );

        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMonoBold12pt7b, 2 );
        sprite.setCursor( 10, 110 );
        sprite.printf( "%4.2fm", altitude );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 20, 190 );
        sprite.printf( "sea:%.2fhPa", SEALEVELPRESSURE_HPA + sealevel_pressure_offset );
        sprite.setCursor( 20, 210 );
        sprite.printf( "cur:%.2fhPa", ( pressure / 100.0 ) );
        sprite.setCursor( 20, 230 );
        sprite.printf( "temp:%.2fdeg", ( temp ) );

        sprite.pushSprite( 0, 0 );
    }
}

static void display_dpf_status() {
    int percent = 0;
    if ( tmr + WAIT < millis() ) {
        tmr = millis();

        sprite.fillScreen( BLACK );
        if ( dpf_reg_status ) {
            SCREEN_TITLE_BG_RED( "DPF status" );
        } else {
            SCREEN_TITLE( "DPF status" );
        }

        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 10, 75 );
        sprite.printf( "Acum" );
        sprite.setCursor( 230, 75 );
        sprite.printf( "%.2fg/L", dpf_pm_accum );
        drawProgressBar( 70, 60, 150, 30, dpf_pm_accum, PM_MAX, WHITE );

        sprite.setCursor( 10, 115 );
        sprite.printf( "Gene" );
        sprite.setCursor( 230, 115 );
        sprite.printf( "%.2fg/L", dpf_pm_gen );
        drawProgressBar( 70, 100, 150, 30, dpf_pm_gen, PM_MAX, WHITE );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 2 );
        sprite.setCursor( 10, 170 );
        sprite.printf( "Count" );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 3 );
        sprite.setCursor( 10, 220 );
        sprite.printf( "%d", dpf_reg_count );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 2 );
        sprite.setCursor( 140, 170 );
        sprite.printf( "Dist" );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 3 );
        sprite.setCursor( 140, 220 );
        sprite.printf( "%dkm", dpf_reg_dist );

        sprite.pushSprite( 0, 0 );
    }
}

static void display_additional_meter() {
    int percent = 0;
    if ( tmr + WAIT < millis() ) {
        tmr = millis();

        // display
        sprite.fillScreen( BLACK );
        SCREEN_TITLE( "Meter" );

        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMonoBold9pt7b, 1 );
        sprite.setCursor( 10, 60 );
        if ( dpf_reg_status ) {
            sprite.setTextColor( RED );
        }
        sprite.printf( "DPF" );

        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 15, 80 );
        sprite.printf( "Accum" );
        drawProgressBar( 75, 70, 90, 15, dpf_pm_accum, PM_MAX, WHITE );

        sprite.setCursor( 15, 100 );
        sprite.printf( "Gen" );
        drawProgressBar( 75, 90, 90, 15, dpf_pm_gen, PM_MAX, WHITE );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 2 );
        sprite.setCursor( 200, 70 );
        sprite.printf( "Dist" );
        sprite.setCursor( 200, 100 );
        sprite.printf( "%dkm", dpf_reg_dist );

        SET_FONT_AND_SIZE( FreeMonoBold9pt7b, 1 );
        sprite.setCursor( 10, 140 );
        sprite.printf( "Temperature" );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 2 );
        sprite.setCursor( 15, 170 );
        sprite.printf( "Wtr" );
        sprite.setCursor( 100, 170 );
        sprite.printf( "%d", (int)engine_coolant_temp );

        sprite.setCursor( 15, 200 );
        sprite.printf( "Oil" );
        sprite.setCursor( 100, 200 );
        sprite.printf( "%d", (int)engine_oil_temp );

        sprite.setCursor( 180, 150 );
        sprite.printf( "Boost" );
        sprite.setCursor( 180, 180 );
        sprite.printf( "%dkpa", (int)boost_pressure );

        sprite.pushSprite( 0, 0 );
    }
}

static void display_setting() {
    if ( tmr + WAIT < millis() ) {
        tmr = millis();

        sprite.fillScreen( BLACK );
        SCREEN_TITLE( "Setting" );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 10, 60 );
        sprite.setTextColor( WHITE );
        sprite.printf( "Altitude temp from: " );
        sprite.printf( "%s", is_temperature_from_sensord ? "Sensord" : "OBD2" );

        sprite.setCursor( 10, 90 );
        sprite.setTextColor( WHITE );
        sprite.printf( "Ambient temp: " );
        sprite.printf( "%2.2f", car_outside_temperature );

        sprite.setCursor( 10, 200 );
        sprite.printf( "A:sensor" );
        sprite.setCursor( 10, 220 );
        sprite.printf( "C:Clear" );
    }

    sprite.pushSprite( 0, 0 );
}

/***********************************/
/* Global functions                */
/***********************************/
void setup() {
    bool error = false;

    M5.begin();
    M5.Speaker.begin();
    M5.Speaker.mute();
    M5.Lcd.setRotation( 1 );
    preferences.begin( "myApp", false );
    screen = preferences.getInt( "screen", SCREEN_OVERVIEW );
    is_temperature_from_sensord = preferences.getBool( "sensor_type", true );

    sprite.setColorDepth( 8 );
    sprite.setTextSize( 2 );
    sprite.createSprite( M5.Lcd.width(), M5.Lcd.height() );
    sprite.fillScreen( BLACK );

    // Initializing Serial
    Serial.begin( 115200 );
    while ( !Serial )
        ;

    // Initializing BMP280
    bmp_init();

    // Initializing ELM327
    car_param_init();

    // Initializing GPS
    Serial.begin( 115200 );
    ss.begin( 9600 );

    delay( 1000 ); // wait for sensor init
}

void loop() {
    M5.update();

    switch ( screen ) {
    case SCREEN_OVERVIEW:
        display_overview();
        btn_process( BTN_A, []() { sealevel_pressure_offset += 0.5; } );
        btn_process( BTN_C, []() { sealevel_pressure_offset += -0.5; } );
        break;
    // case SCREEN_ALTITUDE:
    //     display_altitude();
    //     btn_process( BTN_A, []() { sealevel_pressure_offset += 0.5; } );
    //     btn_process( BTN_C, []() { sealevel_pressure_offset += -0.5; } );
    //     break;
    case SCREEN_DPF_STATUS:
        display_dpf_status();
        break;
    case SCREEN_ADDITIONAL_METER:
        display_additional_meter();
        break;
    case SCREEN_SETTING:
        display_setting();
        btn_process( BTN_A, []() {
            is_temperature_from_sensord = !is_temperature_from_sensord;
            preferences.putBool( "sensor_type", is_temperature_from_sensord );
        } );
        btn_process( BTN_C, []() {
            // preferenceをクリアしたらソフトリセットする
            preferences.clear();
            ESP.restart();
        } );
        break;
    default:
        break;
    }

    btn_process( BTN_B, []() {
        screen++;
        if ( screen >= SCREEN_NUM ) {
            screen = SCREEN_OVERVIEW;
        }
        preferences.putInt( "screen", screen );
    } );

    if ( tmr_save_prefarences + SAVE_PREFARENCES_CYCLE < millis() ) {
        tmr_save_prefarences = millis();
        if ( (int)altitude_old != (int)altitude ) { // To reduce the number of write acceses to flash memory
            altitude_old = altitude;
            preferences.putFloat( "altitude", altitude );
        }
    }

    while ( ss.available() > 0 ) {
        char c = ss.read();
        gps.encode( c );
    }
}
